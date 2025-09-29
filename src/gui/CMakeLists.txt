cmake_minimum_required(VERSION 3.8)
project(gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

include_directories(include)

ament_auto_add_executable(
  robot_gui
    ./src/main.cpp
    ./src/main_window.cpp
    ./src/json.cpp)
target_include_directories(
  robot_gui PRIVATE include ${Qt5Widgets_INCLUDE_DIRS})
target_link_libraries(robot_gui Qt5::Widgets)
set_property(TARGET robot_gui PROPERTY AUTOMOC ON)
set_property(TARGET robot_gui PROPERTY AUTOUIC ON)
set_property(TARGET robot_gui PROPERTY AUTORCC ON)

install(TARGETS
  robot_gui
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
