// ROS 2 port of global_path_publisher from jianzhuozhuTHU/MPC-D-CBF.
// Publishes a straight-line global path at 10 Hz. Topic names and waypoint
// spacing (0.1 m) are identical to the original.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <eigen3/Eigen/Eigen>

class GlobalPathPublisher : public rclcpp::Node
{
public:
  GlobalPathPublisher() : Node("global_path_pub")
  {
    start_x_ = declare_parameter<double>("start_x", -10.0);
    start_y_ = declare_parameter<double>("start_y", 0.0);
    end_x_ = declare_parameter<double>("end_x", 9.0);
    end_y_ = declare_parameter<double>("end_y", 0.0);

    pub_ = create_publisher<nav_msgs::msg::Path>("/global_path", 1);

    const double step = 0.1;
    dist_ = std::sqrt(std::pow(end_x_ - start_x_, 2) + std::pow(end_y_ - start_y_, 2));
    dx_ = (end_x_ - start_x_) / dist_;
    dy_ = (end_y_ - start_y_) / dist_;
    (void)step;

    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&GlobalPathPublisher::publishPath, this));
  }

private:
  void publishPath()
  {
    nav_msgs::msg::Path global_path;
    global_path.header.stamp = now();
    global_path.header.frame_id = "world";

    int idx = 0;
    for (double i = 0.0; i < dist_; i += 0.1)
    {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = now();
      pose_stamped.header.frame_id = "world";
      pose_stamped.pose.orientation.x = 0;
      pose_stamped.pose.orientation.y = 0;
      pose_stamped.pose.orientation.z = 0;
      pose_stamped.pose.orientation.w = 1;

      pose_stamped.pose.position.x = start_x_ + i * dx_;
      pose_stamped.pose.position.y = start_y_ + i * dy_;
      pose_stamped.pose.position.z = 0;
      (void)idx++;

      global_path.poses.push_back(pose_stamped);
    }
    pub_->publish(global_path);
  }

  double start_x_, start_y_, end_x_, end_y_, dist_, dx_, dy_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPathPublisher>());
  rclcpp::shutdown();
  return 0;
}
