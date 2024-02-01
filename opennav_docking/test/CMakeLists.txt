# Test utils
ament_add_gtest(test_utils
  test_utils.cpp
)
ament_target_dependencies(test_utils
  ${dependencies}
)
target_link_libraries(test_utils
  ${library_name}
)

# Test dock database
ament_add_gtest(test_dock_database
  test_dock_database.cpp
)
ament_target_dependencies(test_dock_database
  ${dependencies}
)
target_link_libraries(test_dock_database
  ${library_name}
)

# Test navigator
ament_add_gtest(test_navigator
  test_navigator.cpp
)
ament_target_dependencies(test_navigator
  ${dependencies}
)
target_link_libraries(test_navigator
  ${library_name}
)

# Test docking server
ament_add_pytest_test("test_docking_server" test_docking_server.py)
