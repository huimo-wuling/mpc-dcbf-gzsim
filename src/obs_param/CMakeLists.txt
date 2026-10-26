cmake_minimum_required(VERSION 3.16)
project(obs_param)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(obs_kf src/obs_kf.cpp src/kalman.cpp)
target_include_directories(obs_kf PRIVATE include ${EIGEN3_INCLUDE_DIR})
target_link_libraries(obs_kf
  rclcpp::rclcpp
  std_msgs::std_msgs
  visualization_msgs::visualization_msgs)

install(TARGETS obs_kf DESTINATION lib/${PROJECT_NAME})
ament_package()
