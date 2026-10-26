// ROS 2 + Gazebo Sim port of scene/src/movetest.cpp (+ moving_cylinder.hpp)
// from jianzhuozhuTHU/MPC-D-CBF.
//
// The original teleported 8 cylinders through the Gazebo Classic service
// /gazebo/set_model_state at 100 Hz. Here the same behaviour is achieved by
// calling the Gazebo Sim world service /world/<name>/set_pose directly over
// gz-transport, so the obstacle motion semantics are unchanged.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>

#include <string>
#include <vector>

static constexpr uint8_t kCylinderNum = 8;

struct MovingCylinder
{
  double x = 5.0, y = 5.0, z = 0.25;
  double vx = 0.0, vy = 0.0;
};

class Movetest : public rclcpp::Node
{
public:
  Movetest() : Node("move_gazebo_model")
  {
    world_name_ = declare_parameter<std::string>("world_name", "mpc_dcbf");

    for (uint8_t i = 0; i < kCylinderNum; ++i)
    {
      cylinders_[i].x = declare_parameter<double>("x" + std::to_string(i), 5.0);
      cylinders_[i].y = declare_parameter<double>("y" + std::to_string(i), 5.0);
      cylinders_[i].z = declare_parameter<double>("z" + std::to_string(i), 0.25);
      cylinders_[i].vx = declare_parameter<double>("vx" + std::to_string(i), 0.0);
      cylinders_[i].vy = declare_parameter<double>("vy" + std::to_string(i), 0.0);
    }

    cmd_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/cmd_move", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) { is_move_ = msg->data; });

    // Ground-truth obstacle poses for the reproduction study. The obstacles are
    // teleported with absolute world poses, so the commanded value *is* the
    // simulated pose; publishing it here avoids relying on a gz transport
    // bridge that does not preserve entity names.
    true_poses_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/obstacles/true_poses", 10);
    true_poses_timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                          std::bind(&Movetest::publishTruePoses, this));

    // Publish the initial poses once (the original set them through
    // set_model_state before the robot started moving as well).
    sendAllPoses();

    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               std::bind(&Movetest::update, this));
  }

private:
  void sendPose(const MovingCylinder &c, int idx)
  {
    gz::msgs::Pose pose_msg;
    pose_msg.set_name("cylinder_" + std::to_string(idx));
    pose_msg.mutable_position()->set_x(c.x);
    pose_msg.mutable_position()->set_y(c.y);
    pose_msg.mutable_position()->set_z(c.z);
    pose_msg.mutable_orientation()->set_w(1.0);
    pose_msg.mutable_orientation()->set_x(0.0);
    pose_msg.mutable_orientation()->set_y(0.0);
    pose_msg.mutable_orientation()->set_z(0.0);
    gz_node_.Request<gz::msgs::Pose>("/world/" + world_name_ + "/set_pose", pose_msg);
  }

  void sendAllPoses()
  {
    for (uint8_t i = 0; i < kCylinderNum; ++i)
      sendPose(cylinders_[i], i);
  }

  void publishTruePoses()
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(2 * kCylinderNum);
    for (const auto &c : cylinders_)
    {
      msg.data.push_back(static_cast<float>(c.x));
      msg.data.push_back(static_cast<float>(c.y));
    }
    true_poses_pub_->publish(msg);
  }

  void update()
  {
    if (is_move_)
    {
      for (auto &c : cylinders_)
      {
        c.x += c.vx;
        c.y += c.vy;
      }
    }
    sendAllPoses();
  }

  std::string world_name_;
  bool is_move_ = false;
  std::vector<MovingCylinder> cylinders_{kCylinderNum};
  gz::transport::Node gz_node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr true_poses_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr true_poses_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Movetest>());
  rclcpp::shutdown();
  return 0;
}
