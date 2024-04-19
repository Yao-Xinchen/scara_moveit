cmake_minimum_required(VERSION 3.8)
project(target_pose)

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

add_executable(target_pose
  src/target_pose.cpp)
ament_target_dependencies(target_pose
  moveit_core
  moveit_visual_tools
  rclcpp
  moveit_ros_move_group
  moveit_ros_planning_interface
)

install(TARGETS target_pose
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
