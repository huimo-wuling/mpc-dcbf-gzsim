// ROS 2 port of scene/src/getrobot_pose.cpp from jianzhuozhuTHU/MPC-D-CBF.
//
// Publishes the robot pose and broadcasts TF world -> base_link.
//
// The ROS 1 original got the pose from the gazebo_ros `p3d` plugin, i.e. the
// *ground-truth* world pose (no wheel slip, no drift). gz-sim's DiffDrive
// system instead integrates the wheel encoders, which drifts on a skid-steer
// robot: measured here, after one 60 s run the wheel odometry reported
// (8.90, 0.00) while the true pose was (8.27, -1.13) -- 1.3 m of error, enough
// for the planner to believe it had reached the goal and stop early.
//
// So this node takes the pose straight from the simulator over gz-transport
// (/world/<world>/dynamic_pose/info), exactly like p3d did, and
//   * broadcasts TF world -> base_link,
//   * republishes it as nav_msgs/Odometry on /base_pose_ground_truth.
// The gz DiffDrive odometry is not used any more (its bridge entry was removed).
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

class OdomTF : public rclcpp::Node
{
public:
  OdomTF() : Node("odom_to_tf_node")
  {
    parent_frame_ = declare_parameter<std::string>("parent_frame", "world");
    child_frame_ = declare_parameter<std::string>("child_frame", "base_link");
    const std::string world = declare_parameter<std::string>("world_name", "mpc_dcbf");
    model_ = declare_parameter<std::string>("model_name", "jackal");

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/base_pose_ground_truth", 10);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const std::string topic = "/world/" + world + "/dynamic_pose/info";
    gz_node_.Subscribe(topic, &OdomTF::onPose, this);
    RCLCPP_INFO(get_logger(), "ground-truth pose of '%s' from %s",
                model_.c_str(), topic.c_str());
  }

private:
  void onPose(const gz::msgs::Pose_V & msg)
  {
    // the topic carries every dynamic entity; pick our robot
    for (int i = 0; i < msg.pose_size(); ++i)
    {
      const auto & p = msg.pose(i);
      if (p.name() != model_) {
        continue;
      }
      const auto & q = p.orientation();

      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now();
      t.header.frame_id = parent_frame_;
      t.child_frame_id = child_frame_;
      t.transform.translation.x = p.position().x();
      t.transform.translation.y = p.position().y();
      t.transform.translation.z = p.position().z();
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      broadcaster_->sendTransform(t);

      nav_msgs::msg::Odometry odom;
      odom.header = t.header;
      odom.child_frame_id = child_frame_;
      odom.pose.pose.position.x = t.transform.translation.x;
      odom.pose.pose.position.y = t.transform.translation.y;
      odom.pose.pose.position.z = t.transform.translation.z;
      odom.pose.pose.orientation = t.transform.rotation;
      odom_pub_->publish(odom);
      return;
    }
  }

  std::string parent_frame_, child_frame_, model_;
  gz::transport::Node gz_node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTF>());
  rclcpp::shutdown();
  return 0;
}
