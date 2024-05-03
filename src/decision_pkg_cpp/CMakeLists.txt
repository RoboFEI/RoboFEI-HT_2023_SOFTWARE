cmake_minimum_required(VERSION 3.8)
project(decision_pkg_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(decision src/decision_node.cpp)
ament_target_dependencies(decision
  rclcpp
  rclcpp_action
  custom_interfaces
  geometry_msgs
  sensor_msgs
)

add_executable(robot_behavior src/robot_behavior.cpp)
ament_target_dependencies(robot_behavior
  rclcpp
  rclcpp_action
  custom_interfaces
  geometry_msgs
  sensor_msgs
)

install(TARGETS
  robot_behavior
  DESTINATION lib/${PROJECT_NAME})

ament_package()
