cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_path_side_shift_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_behavior_path_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/scene.cpp
  src/utils.cpp
  src/manager.cpp
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gmock(test_${PROJECT_NAME}
    test/test_behavior_path_planner_node_interface.cpp
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE src)

  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_utils
    test/test_side_shift_utils.cpp
  )

  target_include_directories(test_${PROJECT_NAME}_utils PRIVATE src)

  ament_target_dependencies(test_${PROJECT_NAME}_utils
    tf2
    tf2_geometry_msgs
  )

  target_link_libraries(test_${PROJECT_NAME}_utils
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(INSTALL_TO_SHARE config)
