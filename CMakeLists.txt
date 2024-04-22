cmake_minimum_required(VERSION 3.16)
project(diffdrive_roboteq_sbl LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  diffdrive_roboteq_sbl
  SHARED
  hardware/diffbot_system.cpp
  hardware/RoboteqDevice.cpp
)
target_compile_features(diffdrive_roboteq_sbl PUBLIC cxx_std_17)
target_include_directories(diffdrive_roboteq_sbl PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/diffdrive_roboteq_sbl>
)
ament_target_dependencies(
  diffdrive_roboteq_sbl PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFFDRIVE_ROBOTEQ_SBL_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface diffdrive_roboteq_sbl.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/diffdrive_roboteq_sbl
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/diffdrive_roboteq_sbl
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/diffdrive_roboteq_sbl
)
install(
  DIRECTORY rviz
  DESTINATION share/diffdrive_roboteq_sbl
)

install(TARGETS diffdrive_roboteq_sbl
  EXPORT export_diffdrive_roboteq_sbl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(odom_filter src/odom_filter.cpp)
ament_target_dependencies(odom_filter
  rclcpp
  nav_msgs
)
install(TARGETS 
  odom_filter
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_diffdrive_roboteq_sbl HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
