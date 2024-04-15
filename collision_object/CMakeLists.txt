cmake_minimum_required(VERSION 3.8)
project(collision_object)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(collision_object
  src/collision_object.cpp)
ament_target_dependencies(collision_object
  moveit_core
  moveit_visual_tools
  rclcpp
  moveit_ros_move_group
  moveit_ros_planning_interface
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()