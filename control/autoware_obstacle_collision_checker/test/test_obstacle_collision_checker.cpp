// Copyright 2022-2024 Tier IV, Inc. All rights reserved.
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

#include "../src/obstacle_collision_checker.cpp"  // NOLINT
#include "gtest/gtest.h"

#include <autoware_utils/geometry/geometry.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/time.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest-death-test.h>
#include <pcl/point_cloud.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{
pcl::PointXYZ pcl_point(const float x, const float y)
{
  pcl::PointXYZ p;
  p.x = x;
  p.y = y;
  p.z = 0.0;
  return p;
}

pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud(const std::vector<std::pair<float, float>> & points)
{
  pcl::PointCloud<pcl::PointXYZ> pcl;
  for (const auto & p : points) {
    pcl.push_back(pcl_point(p.first, p.second));
  }
  return pcl;
}

bool point_in_pcl_pointcloud(const pcl::PointXYZ & pt, const pcl::PointCloud<pcl::PointXYZ> & pcd)
{
  for (const auto & p : pcd) {
    if (p.x == pt.x && p.y == pt.y && p.z == pt.z) {
      return true;
    }
  }
  return false;
}

// Build a base_link obstacle grid (grid_map_msgs/GridMap) with the point_count and max_height
// layers set to all-NaN (the heartbeat), of the given geometry and frame. Cells are filled by the
// caller via the returned grid_map::GridMap before conversion with grid_to_msg.
grid_map::GridMap make_empty_grid(
  const double length, const double resolution, const std::string & frame = "base_link")
{
  grid_map::GridMap grid({"point_count", "max_height"});
  grid.setFrameId(frame);
  grid.setGeometry(grid_map::Length(length, length), resolution, grid_map::Position(0.0, 0.0));
  grid["point_count"].setConstant(std::numeric_limits<float>::quiet_NaN());
  grid["max_height"].setConstant(std::numeric_limits<float>::quiet_NaN());
  return grid;
}

grid_map_msgs::msg::GridMap grid_to_msg(const grid_map::GridMap & grid)
{
  return *grid_map::GridMapRosConverter::toMessage(grid);
}

// Occupy the cell containing position (x, y) with the given count and height.
void occupy_cell(
  grid_map::GridMap & grid, const double x, const double y, const float count, const float height)
{
  grid_map::Index idx;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(x, y), idx));
  grid.at("point_count", idx) = count;
  grid.at("max_height", idx) = height;
}
}  // namespace

TEST(test_obstacle_collision_checker, filterPointCloudByTrajectory)
{
  pcl::PointCloud<pcl::PointXYZ> pcl;
  autoware_planning_msgs::msg::Trajectory trajectory;
  pcl::PointXYZ pcl_point;
  autoware_planning_msgs::msg::TrajectoryPoint traj_point;
  pcl_point.y = 0.0;
  traj_point.pose.position.y = 0.99;
  for (float x = 0.0; x < 10.0; x += 1.0) {
    pcl_point.x = x;
    traj_point.pose.position.x = x;
    trajectory.points.push_back(traj_point);
    pcl.push_back(pcl_point);
  }
  // radius < 1: all points are filtered
  for (auto radius = 0.0; radius <= 0.99; radius += 0.1) {
    const auto filtered_pcl = filter_point_cloud_by_trajectory(pcl, trajectory, radius);
    EXPECT_EQ(filtered_pcl.size(), 0ul);
  }
  // radius >= 1.0: all points are kept
  for (auto radius = 1.0; radius < 10.0; radius += 0.1) {
    const auto filtered_pcl = filter_point_cloud_by_trajectory(pcl, trajectory, radius);
    ASSERT_EQ(pcl.size(), filtered_pcl.size());
    for (size_t i = 0; i < pcl.size(); ++i) {
      EXPECT_EQ(pcl[i].x, filtered_pcl[i].x);
      EXPECT_EQ(pcl[i].y, filtered_pcl[i].y);
    }
  }
}

TEST(test_obstacle_collision_checker, getTransformedPointCloud)
{
  sensor_msgs::msg::PointCloud2 pcd_msg;
  const auto pcl_pcd = pcl_pointcloud({
    {0.0, 0.0},
    {1.0, 1.0},
    {-2.0, 3.0},
  });
  pcl::toROSMsg(pcl_pcd, pcd_msg);

  {  // empty transform, expect same points
    geometry_msgs::msg::Transform transform;
    const auto transformed_pcd = get_transformed_point_cloud(pcd_msg, transform);
    EXPECT_EQ(pcl_pcd.size(), transformed_pcd.size());
    for (const auto & p : transformed_pcd.points) {
      EXPECT_TRUE(point_in_pcl_pointcloud(p, pcl_pcd));
    }
  }

  {  // translation
    geometry_msgs::msg::Transform transform;
    transform.translation.x = 2.0;
    transform.translation.y = 1.5;
    const auto transformed_pcd = get_transformed_point_cloud(pcd_msg, transform);
    EXPECT_EQ(pcl_pcd.size(), transformed_pcd.size());
    for (const auto & p : transformed_pcd.points) {
      auto transformed_p = p;
      transformed_p.x -= static_cast<float>(transform.translation.x);
      transformed_p.y -= static_cast<float>(transform.translation.y);
      EXPECT_TRUE(point_in_pcl_pointcloud(transformed_p, pcl_pcd));
    }
  }
  {  // rotation
    geometry_msgs::msg::Transform transform;
    transform.rotation = autoware_utils::create_quaternion_from_rpy(0.0, 0.0, M_PI);
    const auto transformed_pcd = get_transformed_point_cloud(pcd_msg, transform);
    EXPECT_EQ(pcl_pcd.size(), transformed_pcd.size());
    EXPECT_TRUE(point_in_pcl_pointcloud(pcl_point(0.0, 0.0), transformed_pcd));
    EXPECT_TRUE(point_in_pcl_pointcloud(pcl_point(-1.0, -1.0), transformed_pcd));
    EXPECT_TRUE(point_in_pcl_pointcloud(pcl_point(2.0, -3.0), transformed_pcd));
  }
  {  // translation + rotation
    geometry_msgs::msg::Transform transform;
    transform.translation.x = 0.5;
    transform.translation.y = -0.5;
    transform.rotation = autoware_utils::create_quaternion_from_rpy(0.0, 0.0, M_PI);
    const auto transformed_pcd = get_transformed_point_cloud(pcd_msg, transform);
    EXPECT_EQ(pcl_pcd.size(), transformed_pcd.size());
    EXPECT_TRUE(point_in_pcl_pointcloud(pcl_point(0.5, -0.5), transformed_pcd));
    EXPECT_TRUE(point_in_pcl_pointcloud(pcl_point(-0.5, -1.5), transformed_pcd));
    EXPECT_TRUE(point_in_pcl_pointcloud(pcl_point(2.5, -3.5), transformed_pcd));
  }
}

TEST(test_obstacle_collision_checker, calcBrakingDistance)
{
  EXPECT_TRUE(std::isnan(calc_braking_distance(0.0, 0.0, 0.0)));
  // if we cannot decelerate (max_decel = 0.0), then the result is infinity
  EXPECT_DOUBLE_EQ(calc_braking_distance(1.0, 0.0, 1.0), std::numeric_limits<double>::infinity());
  EXPECT_DOUBLE_EQ(
    calc_braking_distance(1.0, 1.0, 1.0),
    // 1s * 1m/s = 1m for the delay, then 1->0m/s at 1m/s² = 0.5m
    1.0 + 0.5);
}

TEST(test_obstacle_collision_checker, check_for_collisions)
{
  autoware::obstacle_collision_checker::Input input;
  // call with empty input causes a segfault
  // const auto output = check_for_collisions(input);
  input.param.delay_time = 1.0;
  input.param.footprint_margin = 0.0;
  input.param.max_deceleration = 1.0;
  input.param.resample_interval = 1.0;
  input.param.search_radius = 10.0;
  geometry_msgs::msg::PoseStamped ego_pose;  // default (0,0) ego pose
  geometry_msgs::msg::TransformStamped tf;   // (0,0) transform
  tf.transform.rotation.w = 1.0;
  input.current_pose = std::make_shared<const geometry_msgs::msg::PoseStamped>(ego_pose);
  input.obstacle_transform = std::make_shared<const geometry_msgs::msg::TransformStamped>(tf);
  // 2mx2m footprint
  input.vehicle_info.front_overhang_m = 1.0;
  input.vehicle_info.wheel_base_m = 0.0;
  input.vehicle_info.rear_overhang_m = 1.0;
  input.vehicle_info.left_overhang_m = 1.0;
  input.vehicle_info.right_overhang_m = 1.0;
  autoware_planning_msgs::msg::Trajectory trajectory;
  autoware_planning_msgs::msg::TrajectoryPoint point;
  point.pose.position.y = 0.0;
  for (auto x = 0.0; x < 6.0; x += 0.1) {
    point.pose.position.x = x;
    trajectory.points.push_back(point);
  }
  input.predicted_trajectory =
    std::make_shared<const autoware_planning_msgs::msg::Trajectory>(trajectory);
  input.reference_trajectory = {};  // TODO(someone): the reference trajectory is not used
  {
    // obstacle point on the trajectory
    sensor_msgs::msg::PointCloud2 pcd_msg;
    const auto pcl_pcd = pcl_pointcloud({
      {5.0, 0.0},
    });
    pcl::toROSMsg(pcl_pcd, pcd_msg);
    input.obstacle_pointcloud = std::make_shared<const sensor_msgs::msg::PointCloud2>(pcd_msg);
    geometry_msgs::msg::Twist twist;  // no velocity -> no collision
    twist.linear.x = 0.0;
    input.current_twist = std::make_shared<const geometry_msgs::msg::Twist>(twist);
    const auto output = check_for_collisions(input);
    EXPECT_FALSE(output.will_collide);
    // zero velocity: only the 1st point of the trajectory is kept
    EXPECT_EQ(output.resampled_trajectory.points.size(), 1UL);
  }
  {
    // moderate velocity: short braking distance so the trajectory is cut before the collision
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 1.0;
    input.current_twist = std::make_shared<const geometry_msgs::msg::Twist>(twist);
    const auto output = check_for_collisions(input);
    EXPECT_FALSE(output.will_collide);
    // 1s * 1m/s = 1m for the delay, then 1->0m/s at 1m/s² = 0.5m -> 1.5m braking distance
    EXPECT_DOUBLE_EQ(
      autoware_utils::calc_distance2d(
        output.resampled_trajectory.points.front(), output.resampled_trajectory.points.back()),
      1.5);
  }
  {
    // high velocity -> collision
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 10.0;
    input.current_twist = std::make_shared<const geometry_msgs::msg::Twist>(twist);
    const auto output = check_for_collisions(input);
    EXPECT_TRUE(output.will_collide);
    // high velocity: the full trajectory is resampled (original interval = 0.1, resample interval
    // = 1.0)
    EXPECT_EQ(
      output.resampled_trajectory.points.size(),
      input.predicted_trajectory->points.size() / 10 + 1);
  }
  {
    // obstacle point on the side of the trajectory but inside the ego footprint -> collision
    sensor_msgs::msg::PointCloud2 pcd_msg;
    const auto pcl_pcd = pcl_pointcloud({
      {5.0, 0.5},
    });
    pcl::toROSMsg(pcl_pcd, pcd_msg);
    input.obstacle_pointcloud = std::make_shared<const sensor_msgs::msg::PointCloud2>(pcd_msg);
    const auto output = check_for_collisions(input);
    EXPECT_TRUE(output.will_collide);
  }
  {
    // obstacle point on the side of the trajectory and outside the ego footprint -> no collision
    sensor_msgs::msg::PointCloud2 pcd_msg;
    const auto pcl_pcd = pcl_pointcloud({
      {5.0, 1.5},
    });
    pcl::toROSMsg(pcl_pcd, pcd_msg);
    input.obstacle_pointcloud = std::make_shared<const sensor_msgs::msg::PointCloud2>(pcd_msg);
    const auto output = check_for_collisions(input);
    EXPECT_FALSE(output.will_collide);
  }
}

namespace
{
using autoware::obstacle_collision_checker::extract_grid_obstacle_pointcloud;
using autoware::obstacle_collision_checker::has_collision;
using autoware::obstacle_collision_checker::is_grid_stale;

pcl::PointCloud<pcl::PointXYZ> extract_or_fail(const grid_map_msgs::msg::GridMap & msg)
{
  const auto out = extract_grid_obstacle_pointcloud(msg);
  EXPECT_TRUE(out.has_value());
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (out) {
    pcl::fromROSMsg(*out, cloud);
  }
  return cloud;
}
}  // namespace

TEST(test_obstacle_collision_checker, extractGridWrongFrame)
{
  // A grid framed in "map" cannot be re-framed cheaply: reject the whole grid as a wiring error.
  auto grid = make_empty_grid(2.0, 0.5, "map");
  occupy_cell(grid, 0.25, 0.25, 5.0f, 1.0f);
  EXPECT_FALSE(extract_grid_obstacle_pointcloud(grid_to_msg(grid)).has_value());
}

TEST(test_obstacle_collision_checker, extractGridMissingLayer)
{
  // Only point_count present (no max_height) -> reject without throwing (an uncaught
  // std::out_of_range would escape the timer callback and kill the component container).
  grid_map::GridMap grid({"point_count"});
  grid.setFrameId("base_link");
  grid.setGeometry(grid_map::Length(2.0, 2.0), 0.5, grid_map::Position(0.0, 0.0));
  grid["point_count"].setConstant(5.0f);
  const auto msg = grid_to_msg(grid);
  std::optional<sensor_msgs::msg::PointCloud2> out;
  EXPECT_NO_THROW({ out = extract_grid_obstacle_pointcloud(msg); });
  EXPECT_FALSE(out.has_value());
}

TEST(test_obstacle_collision_checker, extractGridHeartbeatIsEmptyNotViolation)
{
  // All-NaN grid with a valid frame and layers is the "alive, nothing detected" heartbeat: it must
  // validate (not nullopt) and yield exactly zero points, distinct from a contract violation.
  const auto grid = make_empty_grid(2.0, 0.5);
  const auto cloud = extract_or_fail(grid_to_msg(grid));
  EXPECT_EQ(cloud.size(), 0u);
}

TEST(test_obstacle_collision_checker, extractGridSingleCellFourCorners)
{
  // One occupied cell centered at (0.25, 0.25) with resolution 0.5 -> exactly the 4 corner points
  // (0,0), (0,0.5), (0.5,0.5), (0.5,0) at z = 0. Oracle: center +/- half (0.25), computed here
  // independently of the SUT's corner formula.
  auto grid = make_empty_grid(1.0, 0.5);
  occupy_cell(grid, 0.25, 0.25, 5.0f, 1.0f);
  const auto cloud = extract_or_fail(grid_to_msg(grid));
  ASSERT_EQ(cloud.size(), 4u);
  EXPECT_TRUE(point_in_pcl_pointcloud(pcl_point(0.0f, 0.0f), cloud));
  EXPECT_TRUE(point_in_pcl_pointcloud(pcl_point(0.0f, 0.5f), cloud));
  EXPECT_TRUE(point_in_pcl_pointcloud(pcl_point(0.5f, 0.5f), cloud));
  EXPECT_TRUE(point_in_pcl_pointcloud(pcl_point(0.5f, 0.0f), cloud));
  for (const auto & p : cloud) {
    EXPECT_FLOAT_EQ(p.z, 0.0f);
    EXPECT_NEAR(std::abs(p.x - 0.25), 0.25, 1e-6);
    EXPECT_NEAR(std::abs(p.y - 0.25), 0.25, 1e-6);
  }
}

TEST(test_obstacle_collision_checker, extractGridDensityGateBoundary)
{
  // Gate is {min_point_count_cell = 1, min_height = 0.0}.
  {  // count == 0 (below the threshold) -> no points emitted
    auto grid = make_empty_grid(1.0, 0.5);
    occupy_cell(grid, 0.25, 0.25, 0.0f, 1.0f);
    EXPECT_EQ(extract_or_fail(grid_to_msg(grid)).size(), 0u);
  }
  {  // count == 1 (meets the threshold exactly) -> qualifies, 4 corners
    auto grid = make_empty_grid(1.0, 0.5);
    occupy_cell(grid, 0.25, 0.25, 1.0f, 1.0f);
    EXPECT_EQ(extract_or_fail(grid_to_msg(grid)).size(), 4u);
  }
  {  // max_height == 0.0 exactly on the floor -> qualifies (cell_qualifies uses hi >= min_height)
    auto grid = make_empty_grid(1.0, 0.5);
    occupy_cell(grid, 0.25, 0.25, 1.0f, 0.0f);
    EXPECT_EQ(extract_or_fail(grid_to_msg(grid)).size(), 4u);
  }
  {  // max_height below the 0.0 floor -> rejected. NOTE: this drops not only sub-ground noise but a
     // real short obstacle whose entire top sits below the base_link horizontal plane (e.g. a low
     // object on a downslope); see the height-gate caveat in the source comment / README.
    auto grid = make_empty_grid(1.0, 0.5);
    occupy_cell(grid, 0.25, 0.25, 5.0f, -0.1f);
    EXPECT_EQ(extract_or_fail(grid_to_msg(grid)).size(), 0u);
  }
}

TEST(test_obstacle_collision_checker, isGridStaleBoundary)
{
  const rclcpp::Time now(100, 0, RCL_ROS_TIME);
  // age exactly == timeout (0.5 s) -> NOT stale (strict '>')
  EXPECT_FALSE(is_grid_stale(rclcpp::Time(99, 500000000, RCL_ROS_TIME), now, 0.5));
  // age == 0.6 s > 0.5 s -> stale
  EXPECT_TRUE(is_grid_stale(rclcpp::Time(99, 400000000, RCL_ROS_TIME), now, 0.5));
  // future stamp (negative age, clock skew) -> NOT stale, no crash
  EXPECT_FALSE(is_grid_stale(rclcpp::Time(101, 0, RCL_ROS_TIME), now, 0.5));
}

TEST(test_obstacle_collision_checker, cornerOnFootprintEdgeIsNotCollision)
{
  // Grid-quantized corners can land exactly on a footprint edge; boost::geometry::within uses
  // strict-interior (open-set) semantics, so an on-edge point is NOT a collision. Footprint is the
  // square [0,2]x[0,2]; (1,0) sits on the bottom edge, (1,1) is interior.
  autoware_utils::LinearRing2d footprint;
  footprint.push_back(autoware_utils::Point2d{0.0, 0.0});
  footprint.push_back(autoware_utils::Point2d{2.0, 0.0});
  footprint.push_back(autoware_utils::Point2d{2.0, 2.0});
  footprint.push_back(autoware_utils::Point2d{0.0, 2.0});
  footprint.push_back(autoware_utils::Point2d{0.0, 0.0});
  boost::geometry::correct(footprint);

  const auto edge_cloud = pcl_pointcloud({{1.0f, 0.0f}});
  EXPECT_FALSE(has_collision(edge_cloud, footprint));

  const auto interior_cloud = pcl_pointcloud({{1.0f, 1.0f}});
  EXPECT_TRUE(has_collision(interior_cloud, footprint));
}

TEST(test_obstacle_collision_checker, emptyGridEndToEndNoCollision)
{
  // A valid, unoccupied grid feeding the full check_for_collisions pipeline -> no collision, yet
  // the resampled trajectory / footprints are still produced (the zero-point generalization of the
  // existing single-point check_for_collisions case).
  autoware::obstacle_collision_checker::Input input;
  input.param.delay_time = 1.0;
  input.param.footprint_margin = 0.0;
  input.param.max_deceleration = 1.0;
  input.param.resample_interval = 1.0;
  input.param.search_radius = 10.0;
  geometry_msgs::msg::PoseStamped ego_pose;
  geometry_msgs::msg::TransformStamped tf;
  tf.transform.rotation.w = 1.0;
  input.current_pose = std::make_shared<const geometry_msgs::msg::PoseStamped>(ego_pose);
  input.obstacle_transform = std::make_shared<const geometry_msgs::msg::TransformStamped>(tf);
  input.vehicle_info.front_overhang_m = 1.0;
  input.vehicle_info.wheel_base_m = 0.0;
  input.vehicle_info.rear_overhang_m = 1.0;
  input.vehicle_info.left_overhang_m = 1.0;
  input.vehicle_info.right_overhang_m = 1.0;
  autoware_planning_msgs::msg::Trajectory trajectory;
  autoware_planning_msgs::msg::TrajectoryPoint point;
  point.pose.position.y = 0.0;
  for (auto x = 0.0; x < 6.0; x += 0.1) {
    point.pose.position.x = x;
    trajectory.points.push_back(point);
  }
  input.predicted_trajectory =
    std::make_shared<const autoware_planning_msgs::msg::Trajectory>(trajectory);
  input.reference_trajectory = {};
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 10.0;
  input.current_twist = std::make_shared<const geometry_msgs::msg::Twist>(twist);

  const auto grid = make_empty_grid(60.0, 0.5);  // valid contract, zero occupied cells
  const auto grid_cloud = extract_grid_obstacle_pointcloud(grid_to_msg(grid));
  ASSERT_TRUE(grid_cloud.has_value());
  input.obstacle_pointcloud = std::make_shared<const sensor_msgs::msg::PointCloud2>(*grid_cloud);

  const auto output = check_for_collisions(input);
  EXPECT_FALSE(output.will_collide);
  EXPECT_GE(output.resampled_trajectory.points.size(), 1u);
  EXPECT_FALSE(output.vehicle_footprints.empty());
}

TEST(test_obstacle_collision_checker, extractGridWorstCaseTiming)
{
  // Informational worst-case: a fully occupied production ROI (60 x 40 m @ 0.2 m). Not a pass/fail
  // timing gate; the assertion pins the corner count (4 per occupied cell) as an independent
  // oracle. This bounds the number of points later fed to the O(points x trajectory) corridor
  // filter (up to 4x the ROI cell count).
  grid_map::GridMap grid({"point_count", "max_height"});
  grid.setFrameId("base_link");
  grid.setGeometry(grid_map::Length(60.0, 40.0), 0.2, grid_map::Position(0.0, 0.0));
  grid["point_count"].setConstant(5.0f);
  grid["max_height"].setConstant(1.0f);
  const auto msg = grid_to_msg(grid);
  const auto num_cells =
    static_cast<size_t>(grid.getSize()(0)) * static_cast<size_t>(grid.getSize()(1));

  const auto t0 = std::chrono::steady_clock::now();
  const auto out = extract_grid_obstacle_pointcloud(msg);
  const auto t1 = std::chrono::steady_clock::now();
  ASSERT_TRUE(out.has_value());
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*out, cloud);
  EXPECT_EQ(cloud.size(), 4u * num_cells);
  const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  RecordProperty("worst_case_extract_ms", static_cast<int>(ms));
  std::cout << "[ INFO ] worst-case extract over " << num_cells << " cells: " << ms << " ms\n";
}
