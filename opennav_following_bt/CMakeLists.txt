cmake_minimum_required(VERSION 3.5)
project(opennav_following_bt)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(opennav_following_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# potentially replace with nav2_common, nav2_package()
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_action
  nav2_msgs
  nav2_core
  nav2_behavior_tree
  nav_msgs
  geometry_msgs
  opennav_following_msgs
  behaviortree_cpp_v3
)

add_library(opennav_follow_action_bt_node SHARED src/follow_object_action.cpp)
list(APPEND plugin_libs opennav_follow_action_bt_node)
add_library(opennav_follow_cancel_bt_node SHARED src/follow_object_cancel_node.cpp)
list(APPEND plugin_libs opennav_follow_cancel_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_export_dependencies(${dependencies})
ament_package()
