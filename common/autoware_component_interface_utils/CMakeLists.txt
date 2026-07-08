cmake_minimum_required(VERSION 3.14)
project(autoware_component_interface_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  include
  SYSTEM
  ${rclcpp_INCLUDE_DIRS}
  ${rosidl_runtime_cpp_INCLUDE_DIRS}
  ${rcl_INCLUDE_DIRS}
  ${autoware_adapi_v1_msgs_INCLUDE_DIRS}
  ${autoware_component_interface_admission_INCLUDE_DIRS}
  ${autoware_component_interface_specs_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_component_interface_utils
    test/test_component_interface_utils.cpp
  )

  target_include_directories(test_component_interface_utils
    PRIVATE include
  )

  ament_add_ros_isolated_gtest(test_node_adaptor_registration
    test/test_node_adaptor_registration.cpp
  )

  target_include_directories(test_node_adaptor_registration
    PRIVATE include
  )

  ament_target_dependencies(test_node_adaptor_registration
    autoware_component_interface_admission
    autoware_component_interface_specs
    rclcpp
    rosidl_runtime_cpp
    tier4_system_msgs
  )
endif()

ament_auto_package()
