cmake_minimum_required(VERSION 3.8)
project(scara_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(device_interface REQUIRED)

# COMPILE
add_library(
  scara_hardware
  SHARED
  src/scara_hardware.cpp
  src/motor_joint.cpp
  src/mj_map.cpp
)

target_compile_features(scara_hardware PUBLIC cxx_std_20)
target_include_directories(scara_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/scara_hardware>
)
ament_target_dependencies(
  scara_hardware PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
  device_interface
)

target_compile_definitions(scara_hardware PRIVATE "SCARA_HARDWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface scara_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/scara_hardware
)

install(
  TARGETS scara_hardware
  EXPORT export_scara_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

# Export
ament_export_targets(export_scara_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})

ament_package()
