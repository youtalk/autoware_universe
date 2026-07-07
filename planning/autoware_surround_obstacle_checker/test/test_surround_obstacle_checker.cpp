// Copyright 2024 TIER IV, Inc.
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
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::surround_obstacle_checker
{

// Sample-vehicle (autoware_test_utils) geometry, written out independently so the oracles below
// never borrow the system-under-test's own footprint formula.
//   width = wheel_tread(1.64) + left_overhang(0.128) + right_overhang(0.128) = 1.896 m
//   pointcloud side margin (config default) = 0.5 m
//   ego half-width = (1.896 + 2*0.5)/2 = 1.448 m
constexpr double kVehicleWidth = 1.64 + 0.128 + 0.128;
constexpr double kSideMargin = 0.5;
constexpr double kResolution = 0.2;

class SurroundObstacleCheckerNodeTest : public ::testing::Test
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  static std::shared_ptr<SurroundObstacleCheckerNode> makeNode(const bool enable_pointcloud)
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
    if (enable_pointcloud) {
      node_options.append_parameter_override("pointcloud.enable_check", true);
    }
    return std::make_shared<SurroundObstacleCheckerNode>(node_options);
  }

  // Outcome of one intake + proximity-check pass. grid_available is false exactly when the grid is
  // unavailable (stale / wrong-frame / missing-layer / unconvertible / not-yet-received); such
  // silence must be held as "unknown", never read as "clear".
  struct IntakeResult
  {
    std::optional<StopObstacle> nearest_obstacle;
    bool grid_available;
  };

  // Run the grid intake with a hand-built grid injected directly into the private member (friend
  // access), no live pub/sub, then hand the result to the proximity checker exactly as onTimer
  // does. odometry is the sample initial pose so the map transform is well defined.
  static IntakeResult runPointCloudCheck(
    const std::shared_ptr<SurroundObstacleCheckerNode> & node,
    const grid_map_msgs::msg::GridMap & grid_msg)
  {
    auto odometry =
      std::make_shared<nav_msgs::msg::Odometry>(autoware::test_utils::makeInitialPose());
    node->odometry_ptr_ = odometry;
    node->obstacle_grid_ptr_ = std::make_shared<grid_map_msgs::msg::GridMap>(grid_msg);
    const auto obstacle_grid_pointcloud = node->toObstacleGridPointCloud();
    node->proximity_checker_->update_parameters(node->toProximityCheckerParameters());
    // 1e-3 is the PASS-state contact threshold used by onTimer.
    const auto result = node->proximity_checker_->check(
      node->toProximityCheckerInputs(obstacle_grid_pointcloud), 1e-3);
    return {result.nearest_obstacle, obstacle_grid_pointcloud.has_value()};
  }

  auto isStopRequired(
    const std::shared_ptr<SurroundObstacleCheckerNode> & node, const bool is_obstacle_found,
    const bool is_vehicle_stopped, const State & state,
    const std::optional<rclcpp::Time> & last_obstacle_found_time, const double time_threshold) const
    -> std::pair<bool, std::optional<rclcpp::Time>>
  {
    return node->isStopRequired(
      is_obstacle_found, is_vehicle_stopped, state, last_obstacle_found_time, time_threshold);
  }
};

// Build a base_link grid with the required layers, all cells empty (NaN). The caller marks cells.
static grid_map::GridMap makeEmptyBaseLinkGrid(
  const std::vector<std::string> & layers = {"point_count", "max_height"})
{
  grid_map::GridMap grid(layers);
  grid.setFrameId("base_link");
  // Odd cell count (3.8 / 0.2 = 19) so a cell center lands exactly on the base_link origin.
  grid.setGeometry(grid_map::Length(3.8, 3.8), kResolution, grid_map::Position(0.0, 0.0));
  for (const auto & layer : layers) {
    grid[layer].setConstant(std::numeric_limits<float>::quiet_NaN());
  }
  return grid;
}

static grid_map_msgs::msg::GridMap toFreshMsg(
  const grid_map::GridMap & grid, const std::shared_ptr<SurroundObstacleCheckerNode> & node)
{
  auto msg = grid_map::GridMapRosConverter::toMessage(grid);
  msg->header.stamp = node->now();
  return *msg;
}

// ---- Intake contract (degenerate pins), exercised through getNearestObstacleByPointCloud ----

TEST_F(SurroundObstacleCheckerNodeTest, IntakeDisabledCheckNeverBlocks)
{
  // enable_check=false: no obstacle AND available (must not hold the stop state).
  auto node = makeNode(/*enable_pointcloud=*/false);
  auto grid = makeEmptyBaseLinkGrid();
  grid_map::Index idx;
  grid.getIndex(grid_map::Position(0.0, 0.0), idx);
  grid.at("point_count", idx) = 5.0f;
  grid.at("max_height", idx) = 0.5f;
  const auto result = runPointCloudCheck(node, toFreshMsg(grid, node));
  EXPECT_FALSE(result.nearest_obstacle.has_value());
  EXPECT_TRUE(result.grid_available);
}

TEST_F(SurroundObstacleCheckerNodeTest, IntakeEmptyHeartbeatIsClearNotObstacle)
{
  // Fresh all-NaN "alive" grid: available, no obstacle (never read the heartbeat as an obstacle).
  auto node = makeNode(true);
  const auto result = runPointCloudCheck(node, toFreshMsg(makeEmptyBaseLinkGrid(), node));
  EXPECT_FALSE(result.nearest_obstacle.has_value());
  EXPECT_TRUE(result.grid_available);
}

TEST_F(SurroundObstacleCheckerNodeTest, IntakeStaleGridIsUnavailable)
{
  // An occupied but stale grid must be unavailable (fail-safe hold), never an obstacle nor clear.
  auto node = makeNode(true);
  auto grid = makeEmptyBaseLinkGrid();
  grid_map::Index idx;
  grid.getIndex(grid_map::Position(0.0, 0.0), idx);
  grid.at("point_count", idx) = 5.0f;
  grid.at("max_height", idx) = 0.5f;
  auto msg = grid_map::GridMapRosConverter::toMessage(grid);
  // default obstacle_grid_timeout_sec is 0.5 s; 1.0 s old is unambiguously stale.
  msg->header.stamp = node->now() - rclcpp::Duration::from_seconds(1.0);
  const auto result = runPointCloudCheck(node, *msg);
  EXPECT_FALSE(result.nearest_obstacle.has_value());
  EXPECT_FALSE(result.grid_available);
}

TEST_F(SurroundObstacleCheckerNodeTest, IntakeWrongFrameIsUnavailable)
{
  auto node = makeNode(true);
  auto grid = makeEmptyBaseLinkGrid();
  grid.setFrameId("map");
  grid_map::Index idx;
  grid.getIndex(grid_map::Position(0.0, 0.0), idx);
  grid.at("point_count", idx) = 5.0f;
  grid.at("max_height", idx) = 0.5f;
  const auto result = runPointCloudCheck(node, toFreshMsg(grid, node));
  EXPECT_FALSE(result.nearest_obstacle.has_value());
  EXPECT_FALSE(result.grid_available);
}

TEST_F(SurroundObstacleCheckerNodeTest, IntakeMissingLayerIsUnavailable)
{
  // Only point_count present; max_height missing -> contract violation -> unavailable.
  auto node = makeNode(true);
  auto grid = makeEmptyBaseLinkGrid({"point_count"});
  grid_map::Index idx;
  grid.getIndex(grid_map::Position(0.0, 0.0), idx);
  grid.at("point_count", idx) = 5.0f;
  const auto result = runPointCloudCheck(node, toFreshMsg(grid, node));
  EXPECT_FALSE(result.nearest_obstacle.has_value());
  EXPECT_FALSE(result.grid_available);
}

TEST_F(SurroundObstacleCheckerNodeTest, IntakeGateBoundaryPointCount)
{
  // point_count gate at N=1: point_count == 0 does not qualify; point_count == 1 qualifies.
  auto node = makeNode(true);
  {
    auto grid = makeEmptyBaseLinkGrid();
    grid_map::Index idx;
    grid.getIndex(grid_map::Position(0.0, 0.0), idx);
    grid.at("point_count", idx) = 0.0f;
    grid.at("max_height", idx) = 0.5f;
    const auto result = runPointCloudCheck(node, toFreshMsg(grid, node));
    EXPECT_FALSE(result.nearest_obstacle.has_value());
    EXPECT_TRUE(result.grid_available);
  }
  {
    auto grid = makeEmptyBaseLinkGrid();
    grid_map::Index idx;
    grid.getIndex(grid_map::Position(0.0, 0.0), idx);
    grid.at("point_count", idx) = 1.0f;
    grid.at("max_height", idx) = 0.5f;
    const auto result = runPointCloudCheck(node, toFreshMsg(grid, node));
    ASSERT_TRUE(result.nearest_obstacle.has_value());
    EXPECT_TRUE(result.nearest_obstacle.value().is_point_cloud);
  }
}

TEST_F(SurroundObstacleCheckerNodeTest, IntakeNoHeightGateSubGroundCellStillQualifies)
{
  // No height gate (Gate floor = lowest()): the legacy per-point loop never inspected point.z, and
  // the producing extractor deliberately retains returns down to z_min = -1.0, so a cell whose
  // obstacle mass sits entirely BELOW the ground plane (max_height < 0) must still qualify. A 0.0
  // floor here would silently drop exactly those sub-ground obstacles the legacy path stopped for.
  auto node = makeNode(true);
  {
    // A cell fully in the sub-ground band [-1.0, 0.0) still qualifies -> obstacle reported.
    auto grid = makeEmptyBaseLinkGrid();
    grid_map::Index idx;
    grid.getIndex(grid_map::Position(0.0, 0.0), idx);
    grid.at("point_count", idx) = 3.0f;
    grid.at("max_height", idx) = -0.5f;
    const auto result = runPointCloudCheck(node, toFreshMsg(grid, node));
    ASSERT_TRUE(result.nearest_obstacle.has_value());
    EXPECT_TRUE(result.nearest_obstacle.value().is_point_cloud);
  }
  {
    // A NaN max_height with point_count >= 1 is a producer contract violation: it stays
    // non-qualifying (safe, no crash) rather than being read as an obstacle.
    auto grid = makeEmptyBaseLinkGrid();
    grid_map::Index idx;
    grid.getIndex(grid_map::Position(0.0, 0.0), idx);
    grid.at("point_count", idx) = 3.0f;
    grid.at("max_height", idx) = std::numeric_limits<float>::quiet_NaN();
    const auto result = runPointCloudCheck(node, toFreshMsg(grid, node));
    EXPECT_FALSE(result.nearest_obstacle.has_value());
    EXPECT_TRUE(result.grid_available);
  }
}

TEST_F(SurroundObstacleCheckerNodeTest, IntakeCellOverlappingEgoIsZeroDistance)
{
  // A cell whose footprint overlaps the ego polygon reports distance 0 (edge-inclusive), the
  // condition the state machine reads as a STOP trigger.
  auto node = makeNode(true);
  auto grid = makeEmptyBaseLinkGrid();
  grid_map::Index idx;
  grid.getIndex(grid_map::Position(0.0, 0.0), idx);
  grid.at("point_count", idx) = 5.0f;
  grid.at("max_height", idx) = 0.5f;
  const auto result = runPointCloudCheck(node, toFreshMsg(grid, node));
  ASSERT_TRUE(result.nearest_obstacle.has_value());
  EXPECT_NEAR(result.nearest_obstacle.value().nearest_distance, 0.0, 1e-9);
  // The qualifying cell is centered on the base_link origin and is reported by its corners, so the
  // map-frame nearest point is one of those corners: within half a cell diagonal of the ego
  // position. Rotation preserves that offset, so the bound holds in map frame too. z stays on the
  // ego plane because the grid is 2D evidence and a cell has no single height.
  const auto pose = autoware::test_utils::makeInitialPose().pose.pose;
  const auto & nearest_point = result.nearest_obstacle.value().nearest_point;
  const double half_cell_diagonal = 0.5 * std::sqrt(2.0) * kResolution;
  const double dx = nearest_point.x - pose.position.x;
  const double dy = nearest_point.y - pose.position.y;
  // 1e-6 slack: the cell corners travel through the float32 point cloud before coming back.
  EXPECT_LE(std::hypot(dx, dy), half_cell_diagonal + 1e-6);
  EXPECT_NEAR(nearest_point.z, pose.position.z, 1e-6);
}

TEST_F(SurroundObstacleCheckerNodeTest, IntakeLateralDistanceOracle)
{
  // A single cell out to the side: edge-aware distance is purely lateral and hand-computable as
  //   (cell_near_edge_y) - ego_half_width  =  (center_y - res/2) - 1.448
  // independently of the utils helper.
  auto node = makeNode(true);
  auto grid = makeEmptyBaseLinkGrid();
  grid_map::Index idx;
  grid.getIndex(grid_map::Position(0.0, 1.8), idx);
  grid.at("point_count", idx) = 5.0f;
  grid.at("max_height", idx) = 0.5f;
  grid_map::Position center;
  grid.getPosition(idx, center);
  const double half_width = 0.5 * (kVehicleWidth + 2.0 * kSideMargin);  // 1.448
  const double expected_distance = (center.y() - 0.5 * kResolution) - half_width;
  ASSERT_GT(expected_distance, 0.0);
  const auto result = runPointCloudCheck(node, toFreshMsg(grid, node));
  ASSERT_TRUE(result.nearest_obstacle.has_value());
  EXPECT_NEAR(result.nearest_obstacle.value().nearest_distance, expected_distance, 1e-6);
}

TEST_F(SurroundObstacleCheckerNodeTest, IntakeCellOnEgoEdgeIsZeroDistance)
{
  // Boundary-tangent pin: a cell whose center sits exactly on the ego polygon's lateral edge
  // (y = half_width, from independent constants) has a footprint box that straddles that edge, so
  // the edge-aware distance is EXACTLY 0.0 -- the condition node.cpp reads as a STOP trigger
  // (nearest_distance < 1e-3). A cell reported as a tiny positive epsilon here instead of 0.0 would
  // silently fail to trigger the stop; this pins that it does not. A single-cell grid centered on
  // the edge places the cell center there without depending on the fixed grid's cell lattice.
  auto node = makeNode(true);
  const double half_width = 0.5 * (kVehicleWidth + 2.0 * kSideMargin);  // 1.448, independent oracle
  grid_map::GridMap grid({"point_count", "max_height"});
  grid.setFrameId("base_link");
  grid.setGeometry(
    grid_map::Length(kResolution, kResolution), kResolution, grid_map::Position(0.0, half_width));
  grid["point_count"].setConstant(std::numeric_limits<float>::quiet_NaN());
  grid["max_height"].setConstant(std::numeric_limits<float>::quiet_NaN());
  grid_map::Index idx;
  grid.getIndex(grid_map::Position(0.0, half_width), idx);
  grid.at("point_count", idx) = 5.0f;
  grid.at("max_height", idx) = 0.5f;
  grid_map::Position center;
  grid.getPosition(idx, center);
  ASSERT_NEAR(center.y(), half_width, 1e-9);  // cell center is on the ego lateral edge
  const auto result = runPointCloudCheck(node, toFreshMsg(grid, node));
  ASSERT_TRUE(result.nearest_obstacle.has_value());
  EXPECT_NEAR(result.nearest_obstacle.value().nearest_distance, 0.0, 1e-9);
}

// ---- ROI-superset arithmetic guard: shipped default margins fit inside the extractor ROI ----

TEST_F(SurroundObstacleCheckerNodeTest, DefaultMarginsFitInsideExtractorRoi)
{
  // Extractor default ROI in base_link: x in [-10, 50], y in [-20, 20].
  // Sample-vehicle extents with the pointcloud default margins (front/side/back = 0.5):
  //   base_to_front = max_longitudinal_offset(3.79) + 0.5 = 4.29
  //   base_to_rear  = rear_overhang(1.1) + 0.5 = 1.6
  //   half_width    = (1.896 + 2*0.5)/2 = 1.448
  constexpr double base_to_front = (2.79 + 1.0) + 0.5;
  constexpr double base_to_rear = 1.1 + 0.5;
  const double half_width = 0.5 * (kVehicleWidth + 2.0 * kSideMargin);
  EXPECT_LT(base_to_front, 50.0);
  EXPECT_GT(-base_to_rear, -10.0);
  EXPECT_LT(half_width, 20.0);
}

// ---- Original hysteresis characterization, preserved ----

TEST_F(SurroundObstacleCheckerNodeTest, isStopRequired)
{
  auto node = makeNode(false);
  const auto LAST_STOP_TIME = rclcpp::Clock{RCL_ROS_TIME}.now();

  using namespace std::literals::chrono_literals;
  rclcpp::sleep_for(500ms);

  {
    constexpr double THRESHOLD = 1.0;
    const auto [is_stop, stop_time] =
      isStopRequired(node, false, false, State::STOP, LAST_STOP_TIME, THRESHOLD);
    EXPECT_FALSE(is_stop);
    EXPECT_EQ(stop_time, std::nullopt);
  }

  {
    constexpr double THRESHOLD = 1.0;
    const auto [is_stop, stop_time] =
      isStopRequired(node, false, true, State::PASS, LAST_STOP_TIME, THRESHOLD);
    EXPECT_FALSE(is_stop);
    EXPECT_EQ(stop_time, std::nullopt);
  }

  {
    constexpr double THRESHOLD = 1.0;
    const auto [is_stop, stop_time] =
      isStopRequired(node, true, true, State::STOP, LAST_STOP_TIME, THRESHOLD);

    ASSERT_TRUE(stop_time.has_value());

    const auto time_diff = rclcpp::Clock{RCL_ROS_TIME}.now() - stop_time.value();

    EXPECT_TRUE(is_stop);
    EXPECT_NEAR(time_diff.seconds(), 0.0, 1e-3);
  }

  {
    constexpr double THRESHOLD = 1.0;
    const auto [is_stop, stop_time] =
      isStopRequired(node, false, true, State::STOP, LAST_STOP_TIME, THRESHOLD);

    ASSERT_TRUE(stop_time.has_value());

    const auto time_diff = rclcpp::Clock{RCL_ROS_TIME}.now() - stop_time.value();

    EXPECT_TRUE(is_stop);
    EXPECT_NEAR(time_diff.seconds(), 0.5, 1e-3);
  }

  {
    constexpr double THRESHOLD = 0.25;
    const auto [is_stop, stop_time] =
      isStopRequired(node, false, true, State::STOP, LAST_STOP_TIME, THRESHOLD);
    EXPECT_FALSE(is_stop);
    EXPECT_EQ(stop_time, std::nullopt);
  }

  {
    constexpr double THRESHOLD = 1.0;
    const auto [is_stop, stop_time] =
      isStopRequired(node, false, true, State::STOP, std::nullopt, THRESHOLD);
    EXPECT_FALSE(is_stop);
    EXPECT_EQ(stop_time, std::nullopt);
  }
}
}  // namespace autoware::surround_obstacle_checker
