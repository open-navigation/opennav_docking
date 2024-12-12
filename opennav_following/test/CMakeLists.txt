# Test following server (unit)
ament_add_gtest(test_following_server_unit
  test_following_server_unit.cpp
)
target_link_libraries(test_following_server_unit
  ${geometry_msgs_TARGETS}
  ${library_name}
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Test following server (smoke)
ament_add_pytest_test("test_following_server" test_following_server.py)
