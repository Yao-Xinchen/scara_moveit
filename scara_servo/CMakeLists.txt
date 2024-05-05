cmake_minimum_required(VERSION 3.8)
project(scara_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

find_package(behavior_interface REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  moveit_core
  moveit_servo
  control_msgs
  moveit_ros_planning
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  Boost
)

include_directories(include)

add_executable(scara_servo
  src/scara_servo.cpp)
ament_target_dependencies(scara_servo
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  behavior_interface
)

install(TARGETS scara_servo
  DESTINATION lib/${PROJECT_NAME})

# Add executable for using a controller
add_library(scara_servo SHARED src/joystick_servo_example.cpp)
rclcpp_components_register_nodes(scara_servo
  PLUGIN "JoyToServoPub"
  EXECUTABLE example_node)
ament_target_dependencies(scara_servo ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS scara_servo
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS scara_servo
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
