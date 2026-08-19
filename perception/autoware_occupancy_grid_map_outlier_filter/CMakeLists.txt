cmake_minimum_required(VERSION 3.14)
project(autoware_occupancy_grid_map_outlier_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
ament_auto_find_build_dependencies()

include_directories(
  include
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GRID_MAP_INCLUDE_DIR}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/occupancy_grid_map_outlier_filter_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(OPENMP_FOUND)
  set_target_properties(${PROJECT_NAME} PROPERTIES
    COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
    LINK_FLAGS ${OpenMP_CXX_FLAGS}
  )
endif()

# -- Occupancy Grid Map Outlier Filter --
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::occupancy_grid_map_outlier_filter::OccupancyGridMapOutlierFilterComponent"
  EXECUTABLE ${PROJECT_NAME}_node)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_auto_find_test_dependencies()

  ament_add_gtest(test_obstacle_cloud_registration
    test/test_obstacle_cloud_registration.cpp
  )
  target_include_directories(test_obstacle_cloud_registration PRIVATE "src")
  target_link_libraries(test_obstacle_cloud_registration
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
)
