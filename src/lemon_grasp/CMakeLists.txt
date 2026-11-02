cmake_minimum_required(VERSION 3.8)
project(lemon_grasp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(lemon_grasp_node src/lemon_grasp_node.cpp)
ament_target_dependencies(lemon_grasp_node
  rclcpp
  rclcpp_action
  moveit_servo
  moveit_ros_planning
  moveit_msgs
  control_msgs
  trajectory_msgs
  geometry_msgs
  shape_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  Eigen3
)

install(TARGETS lemon_grasp_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
