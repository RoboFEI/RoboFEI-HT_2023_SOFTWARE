cmake_minimum_required(VERSION 3.5)
project(vision_pkg)

# Avisos de compilador
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependências C++
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Nó goal_aim
add_executable(goal_aim_node src/goal_aim/goal_aim_node.cpp)
ament_target_dependencies(goal_aim_node rclcpp std_msgs sensor_msgs)

install(TARGETS
  goal_aim_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
