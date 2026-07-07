cmake_minimum_required(VERSION 3.14)
project(autoware_collision_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
  src/grid_query.cpp
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::collision_detector::CollisionDetectorNode"
  EXECUTABLE ${PROJECT_NAME}_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

# The gtest links the agnocast-registered node library, which needs LD_PRELOAD at run time, so it is
# built only in the plain rclcpp configuration (same guard as the sibling agnocast packages).
if(BUILD_TESTING AND NOT "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test_grid_query.cpp
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
