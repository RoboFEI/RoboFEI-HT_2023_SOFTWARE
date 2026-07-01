cmake_minimum_required(VERSION 3.5)
project(control)

# =========================
# Padrões de linguagem
# =========================
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Dependências
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

find_package(custom_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(nlohmann_json REQUIRED)

find_package(dynamixel_sdk REQUIRED)


# ncurses
set(CURSES_NEED_NCURSES TRUE)

# =========================
# Includes
# =========================
include_directories(
  include
  src/minIni
  include/json/include/nlohmann
  ${CURSES_INCLUDE_DIR}
)

# =========================
# robot_joy_control
# =========================
add_executable(robot_joy_control src/robot_joy_control.cpp)
ament_target_dependencies(robot_joy_control
  rclcpp
  rclcpp_action
  sensor_msgs
  custom_interfaces
)

install(TARGETS robot_joy_control
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# neck_control
# =========================
add_executable(neck_control src/neck_control.cpp)
ament_target_dependencies(neck_control
  rclcpp
  std_msgs
  vision_msgs
  custom_interfaces
)

install(TARGETS neck_control
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# control
# =========================
add_executable(control src/control.cpp)
ament_target_dependencies(control
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  custom_interfaces
)

install(TARGETS control
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# gait_publisher
# =========================
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
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  custom_interfaces
)

install(TARGETS gait_publisher
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(current src/current.cpp)

ament_target_dependencies(
  current
  rclcpp
  custom_interfaces
  dynamixel_sdk
)

install(TARGETS
  current
  DESTINATION lib/${PROJECT_NAME}
)


# =========================
# Instala includes
# =========================
install(
  DIRECTORY include/
  DESTINATION include/
)

# =========================
# Testes
# =========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()