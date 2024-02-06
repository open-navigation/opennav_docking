cmake_minimum_required(VERSION 3.5)
project(nova_carter_dock)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(opennav_docking_core REQUIRED)

# potentially replace with nav2_common, nav2_package()
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)

include_directories(
  include
)

set(executable_name nova_carter_dock)
set(library_name ${executable_name}_core)

set(dependencies
  angles
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  pluginlib
  opennav_docking_core
)

add_library(${library_name} SHARED
  src/nova_carter_dock.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

pluginlib_export_plugin_description_file(opennav_docking_core plugin.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
