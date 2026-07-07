// Copyright 2026 TIER IV, Inc.
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

#include "autoware/trajectory_processor/trajectory_modifier_plugins/obstacle_stop.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_trajectory_processor/trajectory_modifier_param.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
using autoware::trajectory_modifier::TrajectoryModifierContext;
using autoware::trajectory_modifier::plugin::InputData;
using autoware::trajectory_modifier::plugin::ObstacleStop;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

TrajectoryPoint create_trajectory_point(double x, double y, double velocity)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = 0.0;
  point.pose.orientation.x = 0.0;
  point.pose.orientation.y = 0.0;
  point.pose.orientation.z = 0.0;
  point.pose.orientation.w = 1.0;
  point.longitudinal_velocity_mps = static_cast<float>(velocity);
  point.acceleration_mps2 = 0.0F;
  return point;
}

TrajectoryPoints create_straight_trajectory(double length, double velocity, double spacing = 1.0)
{
  TrajectoryPoints trajectory;
  for (double x = 0.0; x <= length + 1e-6; x += spacing) {
    trajectory.push_back(create_trajectory_point(x, 0.0, velocity));
  }
  return trajectory;
}

Odometry::ConstSharedPtr make_odometry(double x, double y, double velocity)
{
  Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.position.y = y;
  odometry.pose.pose.position.z = 0.0;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = velocity;
  return std::make_shared<const Odometry>(odometry);
}

AccelWithCovarianceStamped::ConstSharedPtr make_acceleration(double accel_x)
{
  AccelWithCovarianceStamped acceleration;
  acceleration.accel.accel.linear.x = accel_x;
  return std::make_shared<const AccelWithCovarianceStamped>(acceleration);
}

PredictedObject create_box_object(
  double x, double y, double size_x, double size_y, double size_z, uint8_t classification_label)
{
  PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose.position.x = x;
  object.kinematics.initial_pose_with_covariance.pose.position.y = y;
  object.kinematics.initial_pose_with_covariance.pose.position.z = 0.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.kinematics.initial_twist_with_covariance.twist.linear.x = 0.0;

  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = size_x;
  object.shape.dimensions.y = size_y;
  object.shape.dimensions.z = size_z;

  ObjectClassification classification;
  classification.label = classification_label;
  classification.probability = 1.0;
  object.classification.push_back(classification);

  return object;
}

PredictedObjects::ConstSharedPtr make_blocking_car(double x, double y)
{
  constexpr double car_size_x = 2.0;
  constexpr double car_size_y = 2.0;
  constexpr double car_size_z = 1.5;

  PredictedObjects predicted_objects;
  predicted_objects.header.frame_id = "map";
  predicted_objects.objects.push_back(
    create_box_object(x, y, car_size_x, car_size_y, car_size_z, ObjectClassification::CAR));
  return std::make_shared<const PredictedObjects>(predicted_objects);
}

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// Build a base_link obstacle grid (grid_map_msgs/GridMap) with one occupied block [x0,x1] x [y0,y1]
// at the production ROI/resolution (60 m x 40 m, 0.2 m cells, centered at (20, 0)); the occupied
// cells carry the given point_count, z band (min_height/max_height) and tallest in-band return
// (low_max_height). Empty cells stay NaN. The stamp drives the staleness watchdog.
grid_map_msgs::msg::GridMap make_obstacle_grid_msg(
  double x0, double x1, double y0, double y1, float min_height, float max_height, float low_max,
  float count, const rclcpp::Time & stamp, const std::string & frame = "base_link")
{
  grid_map::GridMap g({"max_height", "min_height", "point_count", "low_max_height"});
  g.setFrameId(frame);
  g.setGeometry(grid_map::Length(60.0, 40.0), 0.2, grid_map::Position(20.0, 0.0));
  g["point_count"].setConstant(kNaN);
  g["max_height"].setConstant(kNaN);
  g["min_height"].setConstant(kNaN);
  g["low_max_height"].setConstant(kNaN);
  for (double x = x0; x <= x1 + 1e-9; x += 0.2) {
    for (double y = y0; y <= y1 + 1e-9; y += 0.2) {
      grid_map::Index idx;
      if (!g.getIndex(grid_map::Position(x, y), idx)) continue;
      g.at("point_count", idx) = count;
      g.at("max_height", idx) = max_height;
      g.at("min_height", idx) = min_height;
      g.at("low_max_height", idx) = low_max;
    }
  }
  auto msg = *grid_map::GridMapRosConverter::toMessage(g);
  msg.header.stamp = stamp;
  return msg;
}

// An all-NaN heartbeat grid (no occupied cells) with a fresh stamp.
grid_map_msgs::msg::GridMap make_empty_grid_msg(
  const rclcpp::Time & stamp, const std::string & frame = "base_link")
{
  grid_map::GridMap g({"max_height", "min_height", "point_count", "low_max_height"});
  g.setFrameId(frame);
  g.setGeometry(grid_map::Length(60.0, 40.0), 0.2, grid_map::Position(20.0, 0.0));
  for (const char * layer : {"max_height", "min_height", "point_count", "low_max_height"}) {
    g[layer].setConstant(kNaN);
  }
  auto msg = *grid_map::GridMapRosConverter::toMessage(g);
  msg.header.stamp = stamp;
  return msg;
}

InputData create_input_data(
  Odometry::ConstSharedPtr current_odometry,
  AccelWithCovarianceStamped::ConstSharedPtr current_acceleration,
  PredictedObjects::ConstSharedPtr predicted_objects = nullptr,
  grid_map_msgs::msg::GridMap::ConstSharedPtr obstacle_grid = nullptr)
{
  InputData input;
  input.current_odometry = std::move(current_odometry);
  input.current_acceleration = std::move(current_acceleration);
  input.predicted_objects = std::move(predicted_objects);
  input.obstacle_grid = std::move(obstacle_grid);
  return input;
}

}  // namespace

class ObstacleStopIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    auto node_options = rclcpp::NodeOptions{};
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    autoware::test_utils::updateNodeOptions(
      node_options, {autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml"});

    node_ = std::make_shared<rclcpp::Node>("test_obstacle_stop_node", node_options);
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();

    set_up_default_params();

    // Create the context and the plugin once. Tests build per-frame InputData inline,
    // and inject any required TF directly into context_->tf_buffer.
    context_ = std::make_shared<TrajectoryModifierContext>(node_.get());
    plugin_ = std::make_unique<ObstacleStop>();
    plugin_->initialize("test_obstacle_stop", node_.get(), time_keeper_, context_, params_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    plugin_.reset();
    context_.reset();
    node_.reset();
  }

  void set_up_default_params()
  {
    params_.use_obstacle_stop = true;
    params_.use_stop_point_fixer = false;
    params_.trajectory_time_step = 0.1;

    params_.stopping_constraints.nominal_deceleration = 1.0;
    params_.stopping_constraints.maximum_deceleration = 4.0;
    params_.stopping_constraints.jerk_limit = 3.0;
    params_.stopping_constraints.arrived_distance_threshold = 0.5;

    auto & p = params_.obstacle_stop;
    p.use_objects = true;
    p.use_pointcloud = true;
    p.enable_stop_for_objects = true;
    p.enable_stop_for_pointcloud = true;
    p.stop_margin = 6.0;
    p.lateral_margin = 0.5;

    p.obstacle_tracking.on_time_buffer = 0.01;
    p.obstacle_tracking.off_time_buffer = 1.0;
    p.obstacle_tracking.object_distance_th = 1.0;
    p.obstacle_tracking.object_yaw_th = 0.1745;
    p.obstacle_tracking.pcd_distance_th = 0.5;
    p.obstacle_tracking.grace_period = 0.5;

    p.objects.object_types = {"car"};
    p.objects.max_velocity_th = 1.0;

    p.pointcloud.height_buffer = 0.5;
    p.pointcloud.min_point_count_cell = 1;
    p.pointcloud.obstacle_grid_timeout_sec = 0.5;
    p.pointcloud.clustering.min_height = 0.5;
    p.pointcloud.clustering.min_size = 10;

    p.rss_params.enable = true;
    p.rss_params.object_decel.car = 1.5;
    p.rss_params.reaction_time = 0.2;
    p.rss_params.safety_margin = 2.0;
    p.rss_params.ego_decel = 4.0;
    p.rss_params.lookahead_horizon = 1.5;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::unique_ptr<ObstacleStop> plugin_;
  trajectory_modifier_params::Params params_;
  std::shared_ptr<TrajectoryModifierContext> context_;
};

TEST_F(ObstacleStopIntegrationTest, TrajectoryNotModifiedWhenDisabled)
{
  // Arrange
  params_.use_obstacle_stop = false;
  plugin_->update_params(params_);
  TrajectoryPoints trajectory;

  // Act
  const bool modified = plugin_->modify_trajectory(trajectory, InputData{});

  // Assert
  EXPECT_FALSE(modified);
}

TEST_F(ObstacleStopIntegrationTest, TrajectoryNotModifiedForEmptyTrajectory)
{
  // Arrange
  TrajectoryPoints empty_trajectory;

  // Act
  const bool modified = plugin_->modify_trajectory(empty_trajectory, InputData{});

  // Assert
  EXPECT_FALSE(modified);
}

TEST_F(ObstacleStopIntegrationTest, TrajectoryNotModifiedWhenNoObstaclesDetected)
{
  // Arrange: no predicted objects and no obstacle pointcloud.
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto input = create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0));

  // Act
  const bool modified = plugin_->modify_trajectory(trajectory, input);

  // Assert
  EXPECT_FALSE(modified);
}

TEST_F(ObstacleStopIntegrationTest, TrajectoryNotModifiedWhenObjectIsBesidePath)
{
  // Arrange: object 10 m off the trajectory polygon (lateral margin 0.5 + half-width ~0.95).
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto car_beside_path = make_blocking_car(20.0, 10.0);
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), car_beside_path);

  // Act
  plugin_->modify_trajectory(trajectory, input);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const bool modified = plugin_->modify_trajectory(trajectory, input);

  // Assert
  EXPECT_FALSE(modified);
}

TEST_F(ObstacleStopIntegrationTest, TrajectoryModifiedWhenObjectBlocksPath)
{
  // Arrange
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto car_blocking_path = make_blocking_car(20.0, 0.0);
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), car_blocking_path);

  // Act: obstacle tracker requires `on_time_buffer` of continuous observation
  //      before becoming active
  plugin_->modify_trajectory(trajectory, input);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const bool modified = plugin_->modify_trajectory(trajectory, input);

  // Assert
  EXPECT_TRUE(modified);
}

TEST_F(ObstacleStopIntegrationTest, StopPointInsertedBeforeObject)
{
  // Arrange
  constexpr double object_x = 25.0;
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto car_blocking_path = make_blocking_car(object_x, 0.0);
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), car_blocking_path);

  // Act: obstacle tracker requires `on_time_buffer` of continuous observation
  //      before becoming active
  plugin_->modify_trajectory(trajectory, input);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const bool modified = plugin_->modify_trajectory(trajectory, input);

  // Assert
  ASSERT_TRUE(modified);
  EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
  EXPECT_LT(trajectory.back().pose.position.x, object_x);

  const auto obj_length = car_blocking_path->objects.at(0).shape.dimensions.x;
  const auto ego_front_offset = context_->vehicle_info.max_longitudinal_offset_m;
  const auto expected_stop_margin =
    params_.obstacle_stop.stop_margin + ego_front_offset + obj_length / 2.0;
  EXPECT_NEAR(object_x - trajectory.back().pose.position.x, expected_stop_margin, 0.1);
}

TEST_F(ObstacleStopIntegrationTest, StopPointInsertedBeforeObject_ReachMaxDecel)
{
  // Arrange
  constexpr double object_x = 20.0;
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto car_blocking_path = make_blocking_car(object_x, 0.0);
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), car_blocking_path);

  // Act: obstacle tracker requires `on_time_buffer` of continuous observation
  //      before becoming active
  plugin_->modify_trajectory(trajectory, input);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const bool modified = plugin_->modify_trajectory(trajectory, input);

  // Assert
  ASSERT_TRUE(modified);
  EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
  EXPECT_LT(trajectory.back().pose.position.x, object_x);

  const auto expected_stop_margin = 6.96;  // Computed based on max_decel limit and jerk limit
  EXPECT_NEAR(object_x - trajectory.back().pose.position.x, expected_stop_margin, 0.1);
}

// Runs two frames (the point tracker needs `on_time_buffer` of continuous observation before a
// point becomes active), returning whether the trajectory was modified on the second frame.
bool run_two_frames(ObstacleStop & plugin, TrajectoryPoints & trajectory, const InputData & input)
{
  plugin.modify_trajectory(trajectory, input);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  return plugin.modify_trajectory(trajectory, input);
}

TEST_F(ObstacleStopIntegrationTest, StopInsertedForBlockingObstacleGrid)
{
  // Arrange: 3 contiguous occupied cells (x in [14.8, 15.2]) with point_count 5 each -> summed
  // point_count 15 >= clustering.min_size (10); low_max_height 0.7 >= floor 0.5; min_height 0.2
  // <= z-band top (vehicle_height 2.5 + height_buffer 0.5 = 3.0). Ego at origin => base_link ==
  // map.
  constexpr double block_x = 15.0;
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto grid = std::make_shared<const grid_map_msgs::msg::GridMap>(
    make_obstacle_grid_msg(14.8, 15.2, 0.0, 0.0, 0.2f, 0.7f, 0.7f, 5.0f, node_->now()));
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid);

  // Act
  const bool modified = run_two_frames(*plugin_, trajectory, input);

  // Assert
  ASSERT_TRUE(modified);
  EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
  EXPECT_LT(trajectory.back().pose.position.x, block_x);
}

// Grid counterpart of the deceleration-clamp coverage the pointcloud branch used to carry: from
// 8 m/s the nominal stop margin is unreachable ahead of a 15 m obstacle, so the deceleration and
// jerk limits shorten the achieved margin instead of the trajectory being left unmodified.
TEST_F(ObstacleStopIntegrationTest, StopInsertedForBlockingObstacleGrid_ReachMaxDecel)
{
  // Arrange: same occupied block as above; the nearest emitted cell center is at x = 14.9 (0.2 m
  // cells over a 60 m ROI centered at x = 20 put centers on the odd multiples of 0.1).
  constexpr double nearest_cell_x = 14.9;
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto grid = std::make_shared<const grid_map_msgs::msg::GridMap>(
    make_obstacle_grid_msg(14.8, 15.2, 0.0, 0.0, 0.2f, 0.7f, 0.7f, 5.0f, node_->now()));
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid);

  // Act
  const bool modified = run_two_frames(*plugin_, trajectory, input);

  // Assert
  ASSERT_TRUE(modified);
  EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);

  // The stop is deceleration-clamped, so its position follows from the fixture alone. From
  // v0 = 8 m/s the jerk-limited profile takes t1 = maximum_deceleration / jerk_limit = 4.0 / 3.0 s
  // to reach full braking, covering v0*t1 - (1/6)*jerk_limit*t1^3 = 10.67 - 1.19 = 9.48 m and
  // shedding (1/2)*jerk_limit*t1^2 = 2.67 m/s. The remaining 5.33 m/s is bled at a constant
  // 4.0 m/s^2 over 5.33^2 / (2*4.0) = 3.56 m. Total stopping distance 13.04 m, so the stop lands
  // 14.9 - 13.04 = 1.86 m short of the nearest occupied cell.
  constexpr double expected_stop_margin = 1.86;
  const auto achieved_margin = nearest_cell_x - trajectory.back().pose.position.x;
  EXPECT_NEAR(achieved_margin, expected_stop_margin, 0.1);

  // ... which is far short of the margin the plugin would have used had it not been clamped,
  // confirming this test exercises the deceleration limit rather than the nominal path.
  const auto ego_front_offset = context_->vehicle_info.max_longitudinal_offset_m;
  const auto nominal_margin = params_.obstacle_stop.stop_margin + ego_front_offset;
  EXPECT_LT(achieved_margin, nominal_margin);
}

TEST_F(ObstacleStopIntegrationTest, NoStopForObstacleGridBesidePath)
{
  // Arrange: qualifying block 10 m off the trajectory centerline -> outside the corridor polygon.
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto grid = std::make_shared<const grid_map_msgs::msg::GridMap>(
    make_obstacle_grid_msg(14.8, 15.2, 10.0, 10.0, 0.2f, 0.7f, 0.7f, 5.0f, node_->now()));
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid);

  // Act & Assert
  EXPECT_FALSE(run_two_frames(*plugin_, trajectory, input));
}

TEST_F(ObstacleStopIntegrationTest, NoStopWhenComponentPointSumBelowMinSize)
{
  // Arrange: a single occupied cell with point_count 9 -> component sum 9 < min_size 10.
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto grid = std::make_shared<const grid_map_msgs::msg::GridMap>(
    make_obstacle_grid_msg(15.0, 15.0, 0.0, 0.0, 0.2f, 0.7f, 0.7f, 9.0f, node_->now()));
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid);

  // Act & Assert
  EXPECT_FALSE(run_two_frames(*plugin_, trajectory, input));
}

TEST_F(ObstacleStopIntegrationTest, StopWhenSingleCellPointSumMeetsMinSize)
{
  // Arrange: a single occupied cell with point_count 10 -> component sum 10 == min_size 10.
  constexpr double block_x = 15.0;
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto grid = std::make_shared<const grid_map_msgs::msg::GridMap>(
    make_obstacle_grid_msg(block_x, block_x, 0.0, 0.0, 0.2f, 0.7f, 0.7f, 10.0f, node_->now()));
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid);

  // Act & Assert
  ASSERT_TRUE(run_two_frames(*plugin_, trajectory, input));
  EXPECT_LT(trajectory.back().pose.position.x, block_x);
}

TEST_F(ObstacleStopIntegrationTest, DisjointSparseCellsDoNotPoolAcrossGap)
{
  // Arrange: two occupied cells 2 m apart (not 8-connected), point_count 5 each -> two components
  // of sum 5, both below min_size 10; the point-sum gate must not pool them.
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  grid_map::GridMap g({"max_height", "min_height", "point_count", "low_max_height"});
  g.setFrameId("base_link");
  g.setGeometry(grid_map::Length(60.0, 40.0), 0.2, grid_map::Position(20.0, 0.0));
  for (const char * layer : {"max_height", "min_height", "point_count", "low_max_height"}) {
    g[layer].setConstant(std::numeric_limits<float>::quiet_NaN());
  }
  for (const double cx : {15.0, 17.0}) {
    grid_map::Index idx;
    ASSERT_TRUE(g.getIndex(grid_map::Position(cx, 0.0), idx));
    g.at("point_count", idx) = 5.0f;
    g.at("max_height", idx) = 0.7f;
    g.at("min_height", idx) = 0.2f;
    g.at("low_max_height", idx) = 0.7f;
  }
  auto grid_msg = *grid_map::GridMapRosConverter::toMessage(g);
  grid_msg.header.stamp = node_->now();
  const auto grid_ptr = std::make_shared<const grid_map_msgs::msg::GridMap>(grid_msg);
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid_ptr);

  // Act & Assert
  EXPECT_FALSE(run_two_frames(*plugin_, trajectory, input));
}

TEST_F(ObstacleStopIntegrationTest, ConnectedSparseCellsPoolToMeetMinSize)
{
  // Arrange: two 8-adjacent cells (centers x 15.1 and 15.3; the 60 m ROI has an even cell count so
  // cell centers sit at X.1 / X.3, boundaries at X.0 / X.2), point_count 5 each -> one component
  // sum 10 == min_size.
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto grid = std::make_shared<const grid_map_msgs::msg::GridMap>(
    make_obstacle_grid_msg(15.1, 15.3, 0.1, 0.1, 0.2f, 0.7f, 0.7f, 5.0f, node_->now()));
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid);

  // Act & Assert
  ASSERT_TRUE(run_two_frames(*plugin_, trajectory, input));
  EXPECT_LT(trajectory.back().pose.position.x, 15.5);
}

TEST_F(ObstacleStopIntegrationTest, NoStopWhenLowMaxHeightBelowFloor)
{
  // Arrange: dense block (sum 15) but low_max_height 0.45 < floor 0.5 -> per-cell gate rejects all.
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto grid = std::make_shared<const grid_map_msgs::msg::GridMap>(
    make_obstacle_grid_msg(14.8, 15.2, 0.0, 0.0, 0.2f, 0.7f, 0.45f, 5.0f, node_->now()));
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid);

  // Act & Assert
  EXPECT_FALSE(run_two_frames(*plugin_, trajectory, input));
}

TEST_F(ObstacleStopIntegrationTest, StopWhenLowMaxHeightEqualsFloor)
{
  // Arrange: exact lower-gate boundary -- low_max_height 0.5 == clustering.min_height floor 0.5.
  // The per-cell gate uses `>=`, so the boundary value must qualify. Single cell, point_count 10
  // == min_size, min_height 0.2 <= z-band top 3.0.
  constexpr double block_x = 15.0;
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto grid = std::make_shared<const grid_map_msgs::msg::GridMap>(
    make_obstacle_grid_msg(block_x, block_x, 0.0, 0.0, 0.2f, 0.7f, 0.5f, 10.0f, node_->now()));
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid);

  // Act & Assert
  ASSERT_TRUE(run_two_frames(*plugin_, trajectory, input));
  EXPECT_LT(trajectory.back().pose.position.x, block_x);
}

TEST_F(ObstacleStopIntegrationTest, StopWhenMinHeightEqualsZBandTop)
{
  // Arrange: exact upper-gate boundary -- min_height 3.0 == z-band top (vehicle_height 2.5 +
  // height_buffer 0.5 = 3.0). The per-cell gate uses `<=`, so the boundary value must qualify.
  // low_max_height 3.0 >= floor 0.5, point_count 10 == min_size.
  constexpr double block_x = 15.0;
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto grid = std::make_shared<const grid_map_msgs::msg::GridMap>(
    make_obstacle_grid_msg(block_x, block_x, 0.0, 0.0, 3.0f, 3.5f, 3.0f, 10.0f, node_->now()));
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid);

  // Act & Assert
  ASSERT_TRUE(run_two_frames(*plugin_, trajectory, input));
  EXPECT_LT(trajectory.back().pose.position.x, block_x);
}

TEST_F(ObstacleStopIntegrationTest, NoStopForOverheadOnlyCells)
{
  // Arrange: min_height 3.5 > z-band top 3.0 and low_max_height NaN, high count -> overhead-only.
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto grid = std::make_shared<const grid_map_msgs::msg::GridMap>(
    make_obstacle_grid_msg(14.8, 15.2, 0.0, 0.0, 3.5f, 4.0f, kNaN, 50.0f, node_->now()));
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid);

  // Act & Assert
  EXPECT_FALSE(run_two_frames(*plugin_, trajectory, input));
}

TEST_F(ObstacleStopIntegrationTest, NoStopForOverheadWithGroundResidue)
{
  // Arrange: tall overhead (max_height 2.8) sharing cells with ground residue (min_height 0.05),
  // but the tallest in-band return low_max_height 0.05 is below the floor 0.5 -> rejected. Proves
  // low_max_height, not max_height, is the discriminator.
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto grid = std::make_shared<const grid_map_msgs::msg::GridMap>(
    make_obstacle_grid_msg(14.8, 15.2, 0.0, 0.0, 0.05f, 2.8f, 0.05f, 50.0f, node_->now()));
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid);

  // Act & Assert
  EXPECT_FALSE(run_two_frames(*plugin_, trajectory, input));
}

TEST_F(ObstacleStopIntegrationTest, StopForWallWithLowMaxHeightAboveFloor)
{
  // Arrange: same footprint as the residue case, but low_max_height 2.0 >= floor -> a real wall.
  constexpr double block_x = 15.0;
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto grid = std::make_shared<const grid_map_msgs::msg::GridMap>(
    make_obstacle_grid_msg(14.8, 15.2, 0.0, 0.0, 0.05f, 2.8f, 2.0f, 50.0f, node_->now()));
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid);

  // Act & Assert
  ASSERT_TRUE(run_two_frames(*plugin_, trajectory, input));
  EXPECT_LT(trajectory.back().pose.position.x, block_x);
}

TEST_F(ObstacleStopIntegrationTest, NoStopForEmptyHeartbeatGrid)
{
  // Arrange: all-NaN heartbeat grid with a fresh stamp -> no obstacle, no crash.
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto grid =
    std::make_shared<const grid_map_msgs::msg::GridMap>(make_empty_grid_msg(node_->now()));
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid);

  // Act & Assert
  EXPECT_FALSE(run_two_frames(*plugin_, trajectory, input));
}

TEST_F(ObstacleStopIntegrationTest, NoStopForWrongFrameGrid)
{
  // Arrange: a fully occupied block but the grid frame is 'map', not 'base_link' -> contract
  // violation, treated as unavailable.
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto grid = std::make_shared<const grid_map_msgs::msg::GridMap>(
    make_obstacle_grid_msg(14.8, 15.2, 0.0, 0.0, 0.2f, 0.7f, 0.7f, 50.0f, node_->now(), "map"));
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid);

  // Act & Assert
  EXPECT_FALSE(run_two_frames(*plugin_, trajectory, input));
}

TEST_F(ObstacleStopIntegrationTest, NoStopForStaleGrid)
{
  // Arrange: a fully occupied block but the stamp is 1.0 s old > obstacle_grid_timeout_sec 0.5.
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto stale_stamp = node_->now() - rclcpp::Duration::from_seconds(1.0);
  const auto grid = std::make_shared<const grid_map_msgs::msg::GridMap>(
    make_obstacle_grid_msg(14.8, 15.2, 0.0, 0.0, 0.2f, 0.7f, 0.7f, 50.0f, stale_stamp));
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid);

  // Act & Assert
  EXPECT_FALSE(run_two_frames(*plugin_, trajectory, input));
}

TEST_F(ObstacleStopIntegrationTest, MissingRequiredLayerIsRejectedWithoutThrow)
{
  // Arrange: a grid without the low_max_height layer -> intake validation rejects it.
  grid_map::GridMap g({"max_height", "min_height", "point_count"});  // no low_max_height
  g.setFrameId("base_link");
  g.setGeometry(grid_map::Length(60.0, 40.0), 0.2, grid_map::Position(20.0, 0.0));
  for (const char * layer : {"max_height", "min_height", "point_count"}) {
    g[layer].setConstant(std::numeric_limits<float>::quiet_NaN());
  }
  grid_map::Index idx;
  ASSERT_TRUE(g.getIndex(grid_map::Position(15.0, 0.0), idx));
  g.at("point_count", idx) = 50.0f;
  g.at("max_height", idx) = 0.7f;
  g.at("min_height", idx) = 0.2f;
  auto grid_msg = *grid_map::GridMapRosConverter::toMessage(g);
  grid_msg.header.stamp = node_->now();
  const auto grid_ptr = std::make_shared<const grid_map_msgs::msg::GridMap>(grid_msg);
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid_ptr);

  // Act & Assert
  bool modified = true;
  EXPECT_NO_THROW({ modified = run_two_frames(*plugin_, trajectory, input); });
  EXPECT_FALSE(modified);
}

TEST_F(ObstacleStopIntegrationTest, NullObstacleGridLeavesObjectBranchWorking)
{
  // Arrange: no obstacle grid, but a blocking object -> the object branch still inserts a stop,
  // proving a null grid does not break the plugin.
  constexpr double object_x = 20.0;
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto car_blocking_path = make_blocking_car(object_x, 0.0);
  const auto input = create_input_data(
    make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), car_blocking_path, nullptr);

  // Act & Assert
  ASSERT_TRUE(run_two_frames(*plugin_, trajectory, input));
  EXPECT_LT(trajectory.back().pose.position.x, object_x);
}

TEST_F(ObstacleStopIntegrationTest, NoStopWhenPointcloudStopDisabledByDefault)
{
  // Arrange: the default-off contract -- enable_stop_for_pointcloud=false with a fully blocking
  // grid and no objects must not apply a stop.
  params_.obstacle_stop.enable_stop_for_pointcloud = false;
  params_.obstacle_stop.use_objects = false;
  plugin_->update_params(params_);
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto grid = std::make_shared<const grid_map_msgs::msg::GridMap>(
    make_obstacle_grid_msg(14.8, 15.2, 0.0, 0.0, 0.2f, 0.7f, 0.7f, 50.0f, node_->now()));
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, grid);

  // Act & Assert
  EXPECT_FALSE(run_two_frames(*plugin_, trajectory, input));
}

TEST_F(ObstacleStopIntegrationTest, ShippedGridGateDefaultsArePinned)
{
  // Independent oracle: the compiled-in parameter defaults (from the generated struct) must match
  // the shipped values, so a drift in the parameter struct fails loudly.
  const trajectory_modifier_params::Params defaults;
  EXPECT_EQ(defaults.obstacle_stop.pointcloud.min_point_count_cell, 1);
  EXPECT_EQ(defaults.obstacle_stop.pointcloud.clustering.min_size, 10);
  EXPECT_DOUBLE_EQ(defaults.obstacle_stop.pointcloud.obstacle_grid_timeout_sec, 0.5);
  EXPECT_DOUBLE_EQ(defaults.obstacle_stop.pointcloud.clustering.min_height, 0.5);
}
