// ROS 2 port of scene/src/getrobot_pose.cpp from jianzhuozhuTHU/MPC-D-CBF.
// Broadcasts the TF world -> base_link from the ground-truth odometry.
//
// Difference with respect to the Gazebo Classic original: gazebo_ros' p3d
// plugin published the *absolute world* pose, while gz-sim's DiffDrive system
// integrates odometry starting from the pose at which the model was spawned.
// The Jackal is spawned at (-10, 0) with yaw 0, so the spawn pose has to be
// composed back in to recover the world pose the planner expects.
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdomTF : public rclcpp::Node
{
public:
  OdomTF() : Node("odom_to_tf_node")
  {
    parent_frame_ = declare_parameter<std::string>("parent_frame", "world");
    child_frame_ = declare_parameter<std::string>("child_frame", "base_link");
    spawn_x_ = declare_parameter<double>("spawn_x", -10.0);
    spawn_y_ = declare_parameter<double>("spawn_y", 0.0);
    spawn_yaw_ = declare_parameter<double>("spawn_yaw", 0.0);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/base_pose_ground_truth", 1,
        std::bind(&OdomTF::odomCallback, this, std::placeholders::_1));
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;

    // yaw from the quaternion of the (relative) odometry pose
    const double yaw_odom = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                       1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    const double c = std::cos(spawn_yaw_);
    const double s = std::sin(spawn_yaw_);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = parent_frame_;
    t.child_frame_id = child_frame_;
    t.transform.translation.x = spawn_x_ + c * p.x - s * p.y;
    t.transform.translation.y = spawn_y_ + s * p.x + c * p.y;
    t.transform.translation.z = p.z;

    // world yaw = spawn yaw + odometry yaw (planar motion only)
    const double yaw = spawn_yaw_ + yaw_odom;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = std::sin(yaw / 2.0);
    t.transform.rotation.w = std::cos(yaw / 2.0);

    broadcaster_->sendTransform(t);
  }

  std::string parent_frame_, child_frame_;
  double spawn_x_, spawn_y_, spawn_yaw_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTF>());
  rclcpp::shutdown();
  return 0;
}
