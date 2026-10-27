// ROS 2 + Gazebo Sim port of local_map/src/local_map_pub.cpp from
// jianzhuozhuTHU/MPC-D-CBF. The processing pipeline (PassThrough/VoxelGrid
// filtering, elevation grid, NaN interpolation, inflation, Sobel gradient,
// DBSCAN clustering, CGAL min-area ellipse fitting, Hungarian tracking) is
// unchanged. The unused grid_map_octomap dependency was dropped.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <chrono>
#include <vector>

#include "ellipse.hpp"
#include "DBSCAN.hpp"
#include "KM.hpp"

using namespace grid_map;
using namespace std;

class LocalMapPub : public rclcpp::Node
{
public:
  LocalMapPub() : Node("local_map_pub")
  {
    // Parameters, same names/defaults/order as the original for_simulation.launch.
    // `localmap_x_size`, `localmap_y_size`, `resolution` and `_inv_resolution`
    // are the file-scope globals declared in ellipse.hpp; the upstream node
    // wrote the ROS params straight into them, so this port does the same
    // (a same-named member would shadow the globals and leave them at 0, which
    // filters out every point in get_ellipse_array()).
    ::localmap_x_size = declare_parameter<float>("localmap_x_size", 10.0F);
    ::localmap_y_size = declare_parameter<float>("localmap_y_size", 10.0F);
    ::resolution = declare_parameter<float>("resolution", 0.1F);
    obs_height = declare_parameter<float>("obs_height", 0.4F);
    step_height = declare_parameter<float>("step_height", 0.5F);
    DBSCAN_R = declare_parameter<float>("DBSCAN_R", 5.0F);
    DBSCAN_N = declare_parameter<int>("DBSCAN_N", 5);
    // The upstream node read this default *before* recomputing _inv_resolution,
    // so it saw the header's static initialiser (1/0.05 = 20) -- reproduced here.
    block_size = declare_parameter<int>(
        "block_size", static_cast<int>(::localmap_x_size * ::_inv_resolution * 0.2F));
    block_num = declare_parameter<int>("block_num", 5);
    ::_inv_resolution = 1.0F / ::resolution;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    gridmap_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
        "/local_map_pub/gridmap", rclcpp::QoS(1).reliable().transient_local());
    local_pcd_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/local_map_pub/local_pcd", 1);
    ellipse_vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/local_map_pub/ellipse_vis", 1);
    for_obs_track_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/local_map_pub/for_obs_track", 1);

    velodyne_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", 1,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
          pcl::fromROSMsg(*msg, velodyne_cloud_);
          updateTF();
        });

    map_.setFrameId("world");
    map_.setGeometry(Length(localmap_x_size, localmap_y_size), resolution);
    lidar_pcd_matrix_.resize(map_index_len(), map_index_len());
    map_interpolate_.resize(map_index_len(), map_index_len());
    gradient_map_.resize(map_index_len(), map_index_len());

    pass_x_.setFilterFieldName("x");
    pass_x_.setFilterLimits(-localmap_x_size / 2, localmap_x_size / 2);
    pass_y_.setFilterFieldName("y");
    pass_y_.setFilterLimits(-localmap_y_size / 2, localmap_y_size / 2);

    sor_.setLeafSize(0.05f, 0.05f, 0.05f);

    loop_timer_ = create_wall_timer(
        std::chrono::milliseconds(91),  // 11 Hz, as ros::Rate(11) in the original
        std::bind(&LocalMapPub::mainLoop, this));
  }

private:
  int map_index_len() const { return static_cast<int>(localmap_x_size * _inv_resolution); }

  // Blocking until both transforms are available, as updateTF() did.
  void updateTF()
  {
    while (rclcpp::ok())
    {
      if (tf_buffer_->canTransform("world", "base_link", tf2::TimePointZero) &&
          tf_buffer_->canTransform("world", "velodyne", tf2::TimePointZero))
      {
        base_link_tf_ = tf2::transformToEigen(
            tf_buffer_->lookupTransform("world", "base_link", tf2::TimePointZero));
        lidar_tf_ = tf2::transformToEigen(
            tf_buffer_->lookupTransform("world", "velodyne", tf2::TimePointZero));
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void pcd_transform()
  {
    pass_x_.setInputCloud(velodyne_cloud_.makeShared());
    pass_x_.filter(velodyne_cloud_);

    pass_y_.setInputCloud(velodyne_cloud_.makeShared());
    pass_y_.filter(velodyne_cloud_);

    velodyne_cloud_filter_.clear();
    sor_.setInputCloud(velodyne_cloud_.makeShared());
    sor_.filter(velodyne_cloud_filter_);

    velodyne_cloud_global_ = velodyne_cloud_filter_;
    pcl::transformPointCloud(velodyne_cloud_filter_, velodyne_cloud_global_,
                             lidar_tf_.cast<float>());
  }

  void lidar2gridmap(Eigen::MatrixXf &lidar_data_matrix)
  {
    int col = lidar_data_matrix.cols();
    int row = lidar_data_matrix.rows();
    for (const auto &pt : velodyne_cloud_global_)
    {
      int j = (pt.x - robot_position2d_.x()) * _inv_resolution + col * 0.5;
      j = min(max(j, 0), row - 1);
      int k = (pt.y - robot_position2d_.y()) * _inv_resolution + row * 0.5;
      k = min(max(k, 0), col - 1);

      if (std::isnan(lidar_data_matrix(row - 1 - j, col - 1 - k)))
        lidar_data_matrix(row - 1 - j, col - 1 - k) = pt.z;
      if (lidar_data_matrix(row - 1 - j, col - 1 - k) < pt.z)
        lidar_data_matrix(row - 1 - j, col - 1 - k) = pt.z;
    }
  }

  Eigen::MatrixXf map_interpolation(const Eigen::MatrixXf &map_data)
  {
    int col = map_data.cols(), row = map_data.rows();
    Eigen::MatrixXf map_interpolation(map_data);
    for (int i = 1; i < row - 1; i++)
    {
      for (int j = 1; j < col - 1; j++)
      {
        if (isnan(map_data(i, j)))
        {
          int count = 0;
          float height = 0;
          for (int k = 0; k <= 2; k++)
          {
            for (int q = 0; q <= 2; q++)
            {
              if (!isnan(map_data(i - 1 + k, j - 1 + q)))
              {
                count++;
                height += map_data(i - 1 + k, j - 1 + q);
              }
            }
          }
          map_interpolation(i, j) = (count > 0) ? height / count : NAN;
        }
      }
    }
    return map_interpolation;
  }

  void map_inflate_block(Eigen::MatrixXf &dst, const Eigen::MatrixXf &src, int startRow, int startCol, int radius)
  {
    for (int k = 0; k <= 2 * radius; k++)
    {
      for (int q = 0; q <= 2 * radius; q++)
      {
        if (isnan(src(startRow - radius + k, startCol - radius + q)))
        {
          dst(startRow - radius + k, startCol - radius + q) = src(startRow, startCol);
        }
      }
    }
  }

  Eigen::MatrixXf map_inflate(const Eigen::MatrixXf &map_data)
  {
    int col = map_data.cols(), row = map_data.rows();
    Eigen::MatrixXf map_inflated(map_data);
    for (int i = 3; i < row - 3; i++)
    {
      for (int j = 3; j < col - 3; j++)
      {
        if (isnan(map_data(i, j)))
          continue;

        double dis = sqrt((i - col / 2) * (i - col / 2) + (j - col / 2) * (j - col / 2));
        int radius;
        if (dis < col / 3)
          radius = 1;
        else if (dis < col * 0.45)
          radius = 2;
        else
          radius = 3;
        map_inflate_block(map_inflated, map_data, i, j, radius);
      }
    }
    return map_inflated;
  }

  Eigen::MatrixXf gradient_map_processing(Eigen::MatrixXf &map_data, vector<DBSCAN::Point> &dataset)
  {
    const float threshold = -1.25;
    int col = map_data.cols(), row = map_data.rows();
    Eigen::MatrixXf gradient_map(row, col);
    gradient_map.setOnes();
    gradient_map *= threshold;
    DBSCAN::Point obs;

    for (int i = 1; i < row - 1; i++)
    {
      for (int j = 1; j < col - 1; j++)
      {
        bool has_nan_value = false;
        for (int p = -1; p <= 1; p++)
        {
          for (int q = -1; q <= 1; q++)
          {
            if (isnan(map_data(i + p, j + q)))
            {
              gradient_map(i, j) = threshold;
              has_nan_value = true;
            }
          }
        }
        if (!has_nan_value)
        {
          float sobel_x = map_data(i + 1, j + 1) - map_data(i - 1, j + 1) + map_data(i + 1, j) - map_data(i - 1, j) +
                          map_data(i + 1, j - 1) - map_data(i - 1, j - 1);
          float sobel_y = map_data(i - 1, j + 1) - map_data(i - 1, j - 1) + map_data(i, j + 1) - map_data(i, j - 1) +
                          map_data(i + 1, j + 1) - map_data(i + 1, j - 1);
          gradient_map(i, j) = sqrt(sobel_x * sobel_x + sobel_y * sobel_y) + threshold;
          if (gradient_map(i, j) > obs_height + threshold)
          {
            obs.x = i;
            obs.y = j;
            dataset.push_back(obs);
          }
          else
          {
            Eigen::MatrixXf::Index minRow, minCol;
            Eigen::MatrixXf block = map_data.block(i / block_size, j / block_size, block_size, block_size);
            float min = block.minCoeff(&minRow, &minCol);
            if (map_data(i, j) - min > step_height)
            {
              obs.x = i;
              obs.y = j;
              dataset.push_back(obs);
            }
          }
        }
      }
    }

    return gradient_map;
  }

  void visEllipse(const std::vector<Ellipse> &obs_ellipses)
  {
    visualization_msgs::msg::MarkerArray ellipse_vis;

    visualization_msgs::msg::Marker shape_vis;
    shape_vis.header.frame_id = "world";
    shape_vis.header.stamp = now();
    shape_vis.ns = "ellipse";
    shape_vis.type = visualization_msgs::msg::Marker::CYLINDER;
    shape_vis.action = visualization_msgs::msg::Marker::ADD;
    shape_vis.lifetime = rclcpp::Duration::from_seconds(0.15);

    visualization_msgs::msg::Marker text_vis;
    text_vis.header.frame_id = "world";
    text_vis.header.stamp = now();
    text_vis.ns = "text";
    text_vis.action = visualization_msgs::msg::Marker::ADD;
    text_vis.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_vis.lifetime = rclcpp::Duration::from_seconds(0.1);
    text_vis.scale.z = 0.3;
    text_vis.color.r = 1;
    text_vis.color.g = 1;
    text_vis.color.b = 1;
    text_vis.color.a = 1;

    for (size_t i = 0; i < obs_ellipses.size(); i++)
    {
      shape_vis.color.a = 0.9;
      if (obs_ellipses[i].variance < 0.0007)
      {
        shape_vis.color.r = 0;
        shape_vis.color.g = 0;
        shape_vis.color.b = 1;
      }
      else if (obs_ellipses[i].variance > 0.005)
      {
        shape_vis.color.r = 1;
        shape_vis.color.g = 0;
        shape_vis.color.b = 0;
      }
      else
      {
        float dt = (obs_ellipses[i].variance - 0.0007) / (0.005 - 0.0007);
        if (dt > 0.5)
        {
          shape_vis.color.r = 1;
          shape_vis.color.g = 1 - dt;
          shape_vis.color.b = 1 - dt;
        }
        else
        {
          shape_vis.color.r = dt;
          shape_vis.color.g = dt;
          shape_vis.color.b = 1;
        }
      }

      shape_vis.id = i;
      shape_vis.pose.orientation.x = 0.0;
      shape_vis.pose.orientation.y = 0.0;
      shape_vis.pose.orientation.z = sin(obs_ellipses[i].theta / 2);
      shape_vis.pose.orientation.w = cos(obs_ellipses[i].theta / 2);
      shape_vis.pose.position.x = obs_ellipses[i].cx;
      shape_vis.pose.position.y = obs_ellipses[i].cy;
      shape_vis.pose.position.z = -0.0;
      shape_vis.scale.x = 2 * obs_ellipses[i].semimajor;
      shape_vis.scale.y = 2 * obs_ellipses[i].semiminor;
      shape_vis.scale.z = 0.75;

      text_vis.id = obs_ellipses[i].label;

      text_vis.text = to_string(obs_ellipses[i].label);
      text_vis.pose.position.x = obs_ellipses[i].cx - 0.3;
      text_vis.pose.position.y = obs_ellipses[i].cy;
      text_vis.pose.position.z = 0.2;

      ellipse_vis.markers.push_back(shape_vis);
      ellipse_vis.markers.push_back(text_vis);
    }

    ellipse_vis_pub_->publish(ellipse_vis);
  }

  void mainLoop()
  {
    robot_position2d_ << base_link_tf_.translation().x(), base_link_tf_.translation().y();
    pcd_transform();
    map_.setPosition({robot_position2d_.x(), robot_position2d_.y()});
    map_.clear("local_lidar");
    lidar_pcd_matrix_ = map_.get("local_lidar");
    lidar2gridmap(lidar_pcd_matrix_);
    map_.add("local_lidar", lidar_pcd_matrix_);

    map_interpolate_ = map_interpolation(lidar_pcd_matrix_);
    map_interpolate_ = map_inflate(map_interpolate_);
    map_.add("elevation", map_interpolate_);

    vector<DBSCAN::Point> non_clustered_obs;
    gradient_map_ = gradient_map_processing(map_interpolate_, non_clustered_obs);

    DBSCAN DS(DBSCAN_R, DBSCAN_N, non_clustered_obs);
    vector<Obstacle> clustered_obs(DS.cluster_num);
    for (const auto &obs : non_clustered_obs)
    {
      if (obs.obsID > 0)
      {
        gradient_map_(static_cast<int>(obs.x), static_cast<int>(obs.y)) = -0.3;
        clustered_obs[obs.obsID - 1].emplace_back(obs.x, obs.y);
      }
    }

    vector<Ellipse> ellipses_array = get_ellipse_array(clustered_obs, map_);
    KM_.tracking(ellipses_array);
    ab_variance_calculation(ellipses_array);

    std_msgs::msg::Float32MultiArray for_obs_track;
    for (const auto &ellipse : ellipses_array)
    {
      if (ellipse.label == 0)
        continue;

      for_obs_track.data.push_back(ellipse.cx);
      for_obs_track.data.push_back(ellipse.cy);
      for_obs_track.data.push_back(ellipse.semimajor);
      for_obs_track.data.push_back(ellipse.semiminor);
      for_obs_track.data.push_back(ellipse.theta);
      for_obs_track.data.push_back(ellipse.label);
      for_obs_track.data.push_back(ellipse.variance);
    }
    for_obs_track_pub_->publish(for_obs_track);

    sensor_msgs::msg::PointCloud2 local_velodyne_msg;
    pcl::toROSMsg(velodyne_cloud_global_, local_velodyne_msg);
    local_velodyne_msg.header.stamp = now();
    local_velodyne_msg.header.frame_id = "world";
    local_pcd_pub_->publish(local_velodyne_msg);

    auto gridMapMessage = grid_map::GridMapRosConverter::toMessage(map_);
    gridmap_pub_->publish(std::move(gridMapMessage));

    visEllipse(ellipses_array);
  }

  grid_map::GridMap map_ = grid_map::GridMap(std::vector<std::string>{"elevation", "local_lidar", "gradient_map"});

  pcl::PointCloud<pcl::PointXYZ> velodyne_cloud_, velodyne_cloud_filter_, velodyne_cloud_global_;
  pcl::PassThrough<pcl::PointXYZ> pass_x_, pass_y_;
  pcl::VoxelGrid<pcl::PointXYZ> sor_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  Eigen::Isometry3d base_link_tf_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d lidar_tf_ = Eigen::Isometry3d::Identity();
  Eigen::Vector2d robot_position2d_ = Eigen::Vector2d::Zero();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ellipse_vis_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pcd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr for_obs_track_pub_;

  KMAlgorithm KM_;

  float DBSCAN_R;
  int DBSCAN_N;
  float step_height, obs_height;
  int block_size, block_num;

  Eigen::MatrixXf lidar_pcd_matrix_, map_interpolate_, gradient_map_;

  rclcpp::TimerBase::SharedPtr loop_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalMapPub>());
  rclcpp::shutdown();
  return 0;
}
