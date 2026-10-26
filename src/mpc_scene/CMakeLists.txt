cmake_minimum_required(VERSION 3.16)
project(mpc_scene)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz-msgs REQUIRED)

add_executable(odom_tf src/odom_tf.cpp)
target_link_libraries(odom_tf
  rclcpp::rclcpp
  std_msgs::std_msgs
  nav_msgs::nav_msgs
  geometry_msgs::geometry_msgs
  tf2_ros::tf2_ros)

add_executable(movetest src/movetest.cpp)
target_link_libraries(movetest
  rclcpp::rclcpp
  std_msgs::std_msgs
  gz-transport::gz-transport
  gz-msgs::gz-msgs)

install(TARGETS odom_tf movetest DESTINATION lib/${PROJECT_NAME})
ament_package()
