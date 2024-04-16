# Test Controller
ament_add_gtest(test_controller
  test_controller.cpp
)
ament_target_dependencies(test_controller
  ${dependencies}
)
target_link_libraries(test_controller
  ${library_name}
)

# Test following server (unit)
ament_add_gtest(test_following_server_unit
  test_following_server_unit.cpp
)
ament_target_dependencies(test_following_server_unit
  ${dependencies}
)
target_link_libraries(test_following_server_unit
  ${library_name}
)
