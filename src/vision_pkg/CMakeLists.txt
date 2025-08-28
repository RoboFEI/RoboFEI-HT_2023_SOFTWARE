cmake_minimum_required(VERSION 3.8)
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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Dependência para integrar Python

# ========================
# C++ node
# ========================
add_executable(goal_aim_node src/goal_aim/goal_aim_node.cpp)
ament_target_dependencies(goal_aim_node rclcpp std_msgs sensor_msgs cv_bridge image_transport)

install(TARGETS
  goal_aim_node
  DESTINATION lib/${PROJECT_NAME})

# ========================
# Python package
# ========================

# Instala scripts definidos no setup.py (ex.: detect.py)
install(PROGRAMS
  vision_pkg/detect.py
  DESTINATION lib/${PROJECT_NAME}
)

find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# ========================
# Finalização
# ========================
ament_package()
