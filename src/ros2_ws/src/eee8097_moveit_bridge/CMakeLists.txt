cmake_minimum_required(VERSION 3.8)
project(eee8097_moveit_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eee8097_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(moveit_bridge_node src/moveit_bridge_node.cpp)
target_compile_features(moveit_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(moveit_bridge_node
  eee8097_interfaces
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
  std_srvs
)

install(TARGETS moveit_bridge_node DESTINATION lib/${PROJECT_NAME})
ament_package()
