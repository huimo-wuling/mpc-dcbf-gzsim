cmake_minimum_required(VERSION 3.16)
project(global_path_publisher)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(global_path_pub src/global_path_publisher.cpp)
target_link_libraries(global_path_pub
  rclcpp::rclcpp
  nav_msgs::nav_msgs
  geometry_msgs::geometry_msgs)
install(TARGETS global_path_pub DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
