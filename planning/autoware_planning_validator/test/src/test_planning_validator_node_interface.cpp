// Copyright 2023 TIER IV, Inc.
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

#include "autoware/planning_validator/planning_validator.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using autoware::planning_test_manager::PlanningInterfaceTestManager;
using autoware::planning_validator::PlanningValidator;

std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<PlanningInterfaceTestManager>();

  // set subscriber with topic name: planning_validator → test_node_
  test_manager->subscribeOutput<autoware_planning_msgs::msg::Trajectory>(
    "planning_validator/output/trajectory");

  return test_manager;
}

std::shared_ptr<PlanningValidator> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};
  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto planning_validator_dir =
    ament_index_cpp::get_package_share_directory("autoware_planning_validator");
  node_options.arguments(
    {"--ros-args", "--params-file",
     autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml", "--params-file",
     planning_validator_dir + "/config/planning_validator.param.yaml"});
  return std::make_shared<PlanningValidator>(node_options);
}

void publishMandatoryTopics(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  std::shared_ptr<PlanningValidator> test_target_node)
{
  // publish necessary topics from test_manager
  test_manager->publishInput(
    test_target_node, "planning_validator/input/kinematics", autoware::test_utils::makeOdometry());
  test_manager->publishInput(
    test_target_node, "planning_validator/input/acceleration",
    geometry_msgs::msg::AccelWithCovarianceStamped{});
}

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectory)
{
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_trajectory_topic = "planning_validator/input/trajectory";

  // test for normal trajectory
  ASSERT_NO_THROW(test_manager->testWithNormalTrajectory(test_target_node, input_trajectory_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  ASSERT_NO_THROW(
    test_manager->testWithAbnormalTrajectory(test_target_node, input_trajectory_topic));
}

TEST(PlanningModuleInterfaceTest, NodeTestWithOffTrackEgoPose)
{
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_trajectory_topic = "planning_validator/input/trajectory";
  const std::string input_odometry_topic = "planning_validator/input/kinematics";

  // test for normal trajectory
  ASSERT_NO_THROW(test_manager->testWithNormalTrajectory(test_target_node, input_trajectory_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  ASSERT_NO_THROW(test_manager->testWithOffTrackOdometry(test_target_node, input_odometry_topic));
}
