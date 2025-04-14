cmake_minimum_required(VERSION 3.5)
project(control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(${CURSES_INCLUDE_DIR})

find_library(LIBRT rt) 
set(CURSES_NEED_NCURSES TRUE)
find_package(Curses)
    
include_directories(include)
include_directories(src/minIni)
include_directories(include/json/include/nlohmann)

set(CMAKE_C_FLAGS " -lncurses")

# robot_joy_control
add_executable(robot_joy_control src/robot_joy_control.cpp)
ament_target_dependencies(robot_joy_control
  rclcpp
  rclcpp_action
  sensor_msgs
  custom_interfaces
)

install(TARGETS
  robot_joy_control
  DESTINATION lib/${PROJECT_NAME})

# Neck control definitions
add_executable(neck_control src/neck_control.cpp)
ament_target_dependencies(neck_control std_msgs vision_msgs rclcpp custom_interfaces)

install(TARGETS
  neck_control
  DESTINATION lib/${PROJECT_NAME})
  

# Control
add_executable(control src/control.cpp)
ament_target_dependencies(control
  custom_interfaces
  rclcpp
  sensor_msgs
  std_msgs
  rclcpp_action
)

install(TARGETS
  control
  DESTINATION lib/${PROJECT_NAME}
)

# gait publisher
add_executable(gait_publisher
  src/motion/MotionManager.cpp
  src/motion/AngleEstimator.cpp
  src/motion/JointData.cpp
  src/motion/MotionStatus.cpp

  src/motion/modules/Walking.cpp
  src/motion/modules/Action.cpp
  
  src/math/MotionState.cpp
  src/math/Matrix.cpp
  src/math/Vector.cpp
  src/math/Point.cpp
  src/math/QuadraticStateTransform.cpp
  
  src/minIni/minIni.cpp
)


ament_target_dependencies(gait_publisher
  custom_interfaces
  rclcpp
  sensor_msgs
  std_msgs
  rclcpp_action
)

install(TARGETS
  gait_publisher
  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()