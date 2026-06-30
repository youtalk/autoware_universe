cmake_minimum_required(VERSION 3.14)
project(autoware_obstacle_grid_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test_obstacle_grid_utils.cpp
  )
endif()

ament_auto_package()
