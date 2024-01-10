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
