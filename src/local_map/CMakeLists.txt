cmake_minimum_required(VERSION 3.16)
project(local_map)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(CGAL REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(PCL REQUIRED)

include_directories(SYSTEM ${grid_map_ros_INCLUDE_DIRS} ${grid_map_msgs_INCLUDE_DIRS})

add_executable(local_map_pub src/local_map_pub.cpp)
target_include_directories(local_map_pub PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS})
target_link_libraries(local_map_pub
  rclcpp::rclcpp
  std_msgs::std_msgs
  sensor_msgs::sensor_msgs
  visualization_msgs::visualization_msgs
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  pcl_conversions::pcl_conversions
  grid_map_ros::grid_map_ros
  grid_map_msgs::grid_map_msgs
  ${PCL_LIBRARIES}
  CGAL::CGAL)

install(TARGETS local_map_pub DESTINATION lib/${PROJECT_NAME})
ament_package()
