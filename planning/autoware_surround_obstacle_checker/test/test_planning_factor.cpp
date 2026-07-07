// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../src/node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::surround_obstacle_checker
{

using autoware::planning_test_manager::PlanningInterfaceTestManager;
using autoware_internal_planning_msgs::msg::SafetyFactor;

class SurroundObstacleCheckerPlanningFactorTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    test_node_ = std::make_shared<rclcpp::Node>("planning_interface_test_node");
    test_target_node_ = generateTestTargetNode();

    const std::string output_planning_factors_topic =
      "planning/planning_factors/surround_obstacle_checker";
    sub_planning_factor_ =
      test_node_->create_subscription<autoware_internal_planning_msgs::msg::PlanningFactorArray>(
        output_planning_factors_topic, rclcpp::QoS{1},
        [this](autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr msg) {
          planning_factor_msg_ = msg;
        });

    pub_odometry_ = test_node_->create_publisher<nav_msgs::msg::Odometry>(
      "/surround_obstacle_checker_node/input/odometry", 1);
    pub_kinematic_state_ =
      test_node_->create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", 1);
    pub_dynamic_objects_ = test_node_->create_publisher<PredictedObjects>(
      "/surround_obstacle_checker_node/input/objects", 1);
    pub_obstacle_grid_ = test_node_->create_publisher<grid_map_msgs::msg::GridMap>(
      "/surround_obstacle_checker_node/input/obstacle_grid", 1);
  }

  void setEnableCheck(const std::string & type, const bool enable)
  {
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(test_target_node_);
    while (!param_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          test_target_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(test_target_node_->get_logger(), "service not available, waiting again...");
    }

    std::vector<rclcpp::Parameter> new_parameters;
    new_parameters.push_back(rclcpp::Parameter(type + ".enable_check", enable));
    param_client->set_parameters(new_parameters);
  }

  std::shared_ptr<SurroundObstacleCheckerNode> generateTestTargetNode()
  {
    auto node_options = rclcpp::NodeOptions{};
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");

    autoware::test_utils::updateNodeOptions(
      node_options,
      {autoware_test_utils_dir + "/config/test_common.param.yaml",
       autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
       autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
       ament_index_cpp::get_package_share_directory("autoware_surround_obstacle_checker") +
         "/config/surround_obstacle_checker.param.yaml"});

    return std::make_shared<SurroundObstacleCheckerNode>(node_options);
  }

  void publishMandatoryTopics()
  {
    auto odometry = autoware::test_utils::makeInitialPose();
    odometry.header.stamp = test_target_node_->now();
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "base_link";

    pub_odometry_->publish(odometry);
    pub_kinematic_state_->publish(odometry);
  }

  void publishDynamicObject(const unique_identifier_msgs::msg::UUID & object_id)
  {
    autoware_perception_msgs::msg::PredictedObject object;
    object.object_id = object_id;
    object.existence_probability = 1.0f;
    object.classification.resize(1);
    object.classification[0].label =
      autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
    object.kinematics.initial_pose_with_covariance.pose.position.x = 3722.16015625 + 0.5,
    object.kinematics.initial_pose_with_covariance.pose.position.y = 73723.515625;
    object.kinematics.initial_pose_with_covariance.pose.position.z = 0.0;
    object.kinematics.initial_pose_with_covariance.pose.orientation =
      autoware_utils_geometry::create_quaternion_from_yaw(0.0);

    autoware_perception_msgs::msg::PredictedObjects dynamic_objects;
    dynamic_objects.header.stamp = test_target_node_->now();
    dynamic_objects.header.frame_id = "map";
    dynamic_objects.objects.push_back(object);

    pub_dynamic_objects_->publish(dynamic_objects);
  }

  // Publish a base_link obstacle grid with a single occupied cell at the base_link origin. The grid
  // is stamped fresh so the staleness watchdog accepts it. Because the cell center is the base_link
  // origin, the node's reported nearest point in map frame equals the ego position exactly.
  void publishObstacleGrid()
  {
    grid_map::GridMap grid(std::vector<std::string>{"point_count", "max_height"});
    grid.setFrameId("base_link");
    // Odd cell count (3.8 / 0.2 = 19) so a cell center lands exactly on the base_link origin.
    grid.setGeometry(grid_map::Length(3.8, 3.8), 0.2, grid_map::Position(0.0, 0.0));
    grid["point_count"].setConstant(std::numeric_limits<float>::quiet_NaN());
    grid["max_height"].setConstant(std::numeric_limits<float>::quiet_NaN());
    grid_map::Index idx;
    grid.getIndex(grid_map::Position(0.0, 0.0), idx);
    grid.at("point_count", idx) = 5.0f;
    grid.at("max_height", idx) = 0.5f;

    auto msg = grid_map::GridMapRosConverter::toMessage(grid);
    msg->header.stamp = test_target_node_->now();
    pub_obstacle_grid_->publish(*msg);
  }

  void validatePlanningFactor(
    const unique_identifier_msgs::msg::UUID & validate_object_id,
    const uint16_t expected_object_type, const double expected_x, const double expected_y,
    const double position_tolerance = 1e-6)
  {
    // make sure planning_factor_msg_ is received
    EXPECT_NE(planning_factor_msg_, nullptr);

    // make sure planning_factor_msg_ is not empty
    EXPECT_EQ(planning_factor_msg_->factors.size(), 1);

    for (const auto & factor : planning_factor_msg_->factors) {
      EXPECT_FALSE(factor.safety_factors.is_safe);

      // make sure control_points is not empty
      EXPECT_GE(factor.control_points.size(), 1);
      for (auto & control_point : factor.control_points) {
        EXPECT_LT(control_point.distance, 200.0);
      }

      // make sure safety_factors is not empty
      EXPECT_EQ(factor.safety_factors.factors.size(), 1);

      const auto & safety_factor = factor.safety_factors.factors.at(0);

      EXPECT_EQ(safety_factor.type, expected_object_type);
      EXPECT_FALSE(safety_factor.is_safe);
      EXPECT_EQ(safety_factor.points.size(), 1);

      Point2d validate_point_2d(expected_x, expected_y);
      Point2d safety_factor_point_2d(safety_factor.points.at(0).x, safety_factor.points.at(0).y);
      EXPECT_NEAR(bg::distance(validate_point_2d, safety_factor_point_2d), 0.0, position_tolerance);

      EXPECT_EQ(safety_factor.object_id, validate_object_id);
    }
  }
  void TearDown() override { rclcpp::shutdown(); }

  void spinSome()
  {
    rclcpp::spin_some(test_target_node_);
    rclcpp::spin_some(test_node_);
  }

private:
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_kinematic_state_;
  rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr
    pub_dynamic_objects_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_obstacle_grid_;
  std::shared_ptr<SurroundObstacleCheckerNode> test_target_node_;
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::PlanningFactorArray>::SharedPtr
    sub_planning_factor_;
  autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr planning_factor_msg_;
};

TEST_F(SurroundObstacleCheckerPlanningFactorTest, TestByDynamicObject)
{
  const auto object_id = autoware_utils_uuid::generate_uuid();
  for (size_t i = 0; i < 5; i++) {
    publishMandatoryTopics();
    publishDynamicObject(object_id);
    spinSome();
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  validatePlanningFactor(object_id, SafetyFactor::OBJECT, 3722.16015625 + 0.5, 73723.515625);
}

TEST_F(SurroundObstacleCheckerPlanningFactorTest, TestByPointCloud)
{
  setEnableCheck("pointcloud", true);
  const auto default_id = unique_identifier_msgs::msg::UUID{};
  for (size_t i = 0; i < 5; i++) {
    publishMandatoryTopics();
    publishObstacleGrid();
    spinSome();
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  // The occupied cell is centered on the base_link origin and is reported by its corners, so the
  // map-frame nearest point sits within half a cell diagonal (0.2 m cells) of the ego position
  // (independent oracle: the sample initial pose), with the default UUID and POINTCLOUD.
  validatePlanningFactor(
    default_id, SafetyFactor::POINTCLOUD, 3722.16015625, 73723.515625,
    0.5 * std::sqrt(2.0) * 0.2 + 1e-6);
}

}  // namespace autoware::surround_obstacle_checker
