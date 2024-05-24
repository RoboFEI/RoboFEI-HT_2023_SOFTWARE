cmake_minimum_required(VERSION 3.8)
project(motors_pkg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(motors_communication src/motors_communication.cpp)
ament_target_dependencies(motors_communication
  custom_interfaces
  dynamixel_sdk
  rclcpp
)

target_link_directories(motors_communication PUBLIC dynamixel_sdk::dynamixel_sdk)

# Install
install(TARGETS
  motors_communication
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
