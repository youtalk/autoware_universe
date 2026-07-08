// Copyright 2024 TIER IV
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

#include "test.hpp"

#include "autoware/autonomous_emergency_braking/node.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/utils.hpp>

#include <autoware_perception_msgs/msg/detail/shape__struct.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>

#include <gtest/gtest.h>
#include <pcl/memory.h>

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace autoware::motion::control::autonomous_emergency_braking::test
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_utils::Polygon2d;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::Header;

Header get_header(const char * const frame_id, rclcpp::Time t)
{
  std_msgs::msg::Header header;
  header.stamp = t;
  header.frame_id = frame_id;
  return header;
};

Odometry make_odometry_message(const Header & header, const double angular_velocity_z)
{
  Odometry odom_msg;
  odom_msg.header = header;
  odom_msg.child_frame_id = "base_link";
  odom_msg.twist.twist.angular.z = angular_velocity_z;
  return odom_msg;
};

VelocityReport make_velocity_report_msg(
  const Header & header, const double lat_velocity, const double long_velocity,
  const double heading_rate)
{
  VelocityReport velocity_msg;
  velocity_msg.header = header;
  velocity_msg.lateral_velocity = lat_velocity;
  velocity_msg.longitudinal_velocity = long_velocity;
  velocity_msg.heading_rate = heading_rate;
  return velocity_msg;
}

// Build a base_link obstacle grid (grid_map_msgs/GridMap) with one occupied block
// [x0,x1] x [y0,y1] at the production ROI/resolution; cells carry the given count + z band +
// tallest in-band return (low_max_height).
grid_map_msgs::msg::GridMap make_obstacle_grid_msg(
  double x0, double x1, double y0, double y1, float zmin, float zmax, float low_max, float count,
  const std::string & frame = "base_link")
{
  grid_map::GridMap g({"max_height", "min_height", "point_count", "low_max_height"});
  g.setFrameId(frame);
  g.setGeometry(grid_map::Length(60.0, 40.0), 0.2, grid_map::Position(20.0, 0.0));
  g["point_count"].setConstant(std::numeric_limits<float>::quiet_NaN());
  g["max_height"].setConstant(std::numeric_limits<float>::quiet_NaN());
  g["min_height"].setConstant(std::numeric_limits<float>::quiet_NaN());
  g["low_max_height"].setConstant(std::numeric_limits<float>::quiet_NaN());
  for (double x = x0; x <= x1 + 1e-9; x += 0.2) {
    for (double y = y0; y <= y1 + 1e-9; y += 0.2) {
      grid_map::Index idx;
      if (!g.getIndex(grid_map::Position(x, y), idx)) continue;
      g.at("point_count", idx) = count;
      g.at("max_height", idx) = zmax;
      g.at("min_height", idx) = zmin;
      g.at("low_max_height", idx) = low_max;
    }
  }
  return *grid_map::GridMapRosConverter::toMessage(g);
}

std::shared_ptr<AEB> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};

  const auto aeb_dir =
    ament_index_cpp::get_package_share_directory("autoware_autonomous_emergency_braking");
  const auto vehicle_info_util_dir =
    ament_index_cpp::get_package_share_directory("autoware_vehicle_info_utils");

  node_options.arguments(
    {"--ros-args", "--params-file", aeb_dir + "/config/autonomous_emergency_braking.param.yaml",
     "--ros-args", "--params-file", vehicle_info_util_dir + "/config/vehicle_info.param.yaml"});
  return std::make_shared<AEB>(node_options);
};

std::shared_ptr<PubSubNode> generatePubSubNode()
{
  auto node_options = rclcpp::NodeOptions{};
  node_options.arguments({"--ros-args"});
  return std::make_shared<PubSubNode>(node_options);
};

PubSubNode::PubSubNode(const rclcpp::NodeOptions & node_options)
: Node("test_aeb_pubsub", node_options)
{
  rclcpp::QoS qos{1};
  qos.transient_local();

  // Publish on the AEB node's fully-qualified input topics (node name "AEB") so the node's polling
  // subscribers actually receive these messages; a relative "~/input/..." name would resolve into
  // this publisher node's namespace and never reach the AEB node.
  pub_kinematic_state_ = create_publisher<Odometry>("/AEB/input/kinematic_state", qos);
  pub_obstacle_grid_ =
    create_publisher<grid_map_msgs::msg::GridMap>("/AEB/input/obstacle_grid", qos);
  pub_velocity_ = create_publisher<VelocityReport>("/AEB/input/velocity", qos);
  pub_predicted_traj_ = create_publisher<Trajectory>("/AEB/input/predicted_trajectory", qos);
  pub_predicted_objects_ = create_publisher<PredictedObjects>("/AEB/input/objects", qos);
  pub_autoware_state_ = create_publisher<AutowareState>("/autoware/state", qos);
}

TEST_F(TestAEB, checkCollision)
{
  constexpr double longitudinal_velocity = 3.0;
  ObjectData object_collision;
  object_collision.distance_to_object = 0.5;
  object_collision.velocity = 0.1;
  object_collision.position.x = 1.0;
  object_collision.position.y = 1.0;
  ASSERT_TRUE(aeb_node_->hasCollision(longitudinal_velocity, object_collision));

  ObjectData object_no_collision;
  object_no_collision.distance_to_object = 10.0;
  object_no_collision.velocity = 0.1;
  ASSERT_FALSE(aeb_node_->hasCollision(longitudinal_velocity, object_no_collision));
}

TEST_F(TestAEB, getObjectOnPathData)
{
  constexpr double longitudinal_velocity = 3.0;
  constexpr double yaw_rate = 0.0;
  const auto imu_path = aeb_node_->generateEgoPath(longitudinal_velocity, yaw_rate);
  ASSERT_FALSE(imu_path.empty());

  const double path_width = 2.0;
  const auto path_length = autoware::motion_utils::calcArcLength(imu_path);
  ASSERT_TRUE(
    path_length < aeb_node_->max_generated_imu_path_length_ +
                    aeb_node_->imu_prediction_time_interval_ * longitudinal_velocity +
                    std::numeric_limits<double>::epsilon());

  const auto stamp = rclcpp::Time();

  const auto longitudinal_offset_opt = utils::getLongitudinalOffset(imu_path, 1.0, -1.0);
  ASSERT_TRUE(longitudinal_offset_opt.has_value());
  const auto longitudinal_offset = longitudinal_offset_opt.value();
  ASSERT_DOUBLE_EQ(longitudinal_offset, 1.0);

  // Object in path if longitudinal_offset is considered
  Point obj_position;
  double path_expansion;

  {
    obj_position.x = path_length + std::numeric_limits<double>::epsilon();
    obj_position.y = 1.0;
    path_expansion = 0.0;
    auto obj_data_opt = utils::getObjectOnPathData(
      imu_path, obj_position, stamp, path_length, path_width, path_expansion, longitudinal_offset,
      0.0);
    ASSERT_TRUE(obj_data_opt.has_value());
    ASSERT_TRUE(obj_data_opt.value().is_target);
  }

  // Object outside of path
  {
    obj_position.x = path_length + std::numeric_limits<double>::epsilon();
    obj_position.y = 3.0;
    path_expansion = 0.0;
    auto obj_data_opt = utils::getObjectOnPathData(
      imu_path, obj_position, stamp, path_length, path_width, path_expansion, longitudinal_offset,
      0.0);
    ASSERT_FALSE(obj_data_opt.has_value());
  }

  // Object outside of path
  {
    obj_position.x = -1.0;
    obj_position.y = 0.0;
    path_expansion = 0.0;
    auto obj_data_opt = utils::getObjectOnPathData(
      imu_path, obj_position, stamp, path_length, path_width, path_expansion, longitudinal_offset,
      0.0);
    ASSERT_FALSE(obj_data_opt.has_value());
  }

  // Object is covered by path expansion
  {
    obj_position.x = path_length + std::numeric_limits<double>::epsilon();
    obj_position.y = 3.0;
    path_expansion = 2.0;
    auto obj_data_opt = utils::getObjectOnPathData(
      imu_path, obj_position, stamp, path_length, path_width, 2.0, longitudinal_offset, 0.0);
    ASSERT_TRUE(obj_data_opt.has_value());
    ASSERT_FALSE(obj_data_opt.value().is_target);
  }
}

TEST_F(TestAEB, checkImuPathGeneration)
{
  constexpr double longitudinal_velocity = 3.0;
  constexpr double yaw_rate = 0.05;
  const auto imu_path = aeb_node_->generateEgoPath(longitudinal_velocity, yaw_rate);
  ASSERT_FALSE(imu_path.empty());

  const double dt = aeb_node_->imu_prediction_time_interval_;
  const double horizon = aeb_node_->imu_prediction_time_horizon_;
  ASSERT_TRUE(imu_path.size() >= static_cast<size_t>(horizon / dt));

  const auto footprint = aeb_node_->generatePathFootprint(imu_path, 0.0);
  ASSERT_FALSE(footprint.empty());
  ASSERT_TRUE(footprint.size() == imu_path.size() - 1);

  const auto stamp = rclcpp::Time();
  // dense occupied block on the ego path -> survives the per-cell gate + component point-sum, and
  // the emitted cell corner points are found on the imu path.
  const auto grid_msg = make_obstacle_grid_msg(0.5, 1.5, -0.5, 0.5, 0.5f, 0.5f, 0.5f, 5.0f);
  PointCloud::Ptr points_belonging_to_cluster_hulls = pcl::make_shared<PointCloud>();
  aeb_node_->getCellsFromObstacleGrid(grid_msg, points_belonging_to_cluster_hulls);
  ASSERT_FALSE(points_belonging_to_cluster_hulls->empty());
  std::vector<ObjectData> objects;
  aeb_node_->getClosestObjectsOnPath(imu_path, stamp, points_belonging_to_cluster_hulls, objects);
  ASSERT_FALSE(objects.empty());
}

TEST_F(TestAEB, checkIncompleteImuPathGeneration)
{
  const double dt = aeb_node_->imu_prediction_time_interval_;
  const double horizon = aeb_node_->imu_prediction_time_horizon_;
  const double min_generated_path_length = aeb_node_->min_generated_imu_path_length_;
  const double slow_velocity = min_generated_path_length / (2.0 * horizon);
  constexpr double yaw_rate = 0.05;
  const auto imu_path = aeb_node_->generateEgoPath(slow_velocity, yaw_rate);

  ASSERT_FALSE(imu_path.empty());
  ASSERT_TRUE(imu_path.size() >= static_cast<size_t>(horizon / dt));
  ASSERT_TRUE(autoware::motion_utils::calcArcLength(imu_path) >= min_generated_path_length);

  const auto footprint = aeb_node_->generatePathFootprint(imu_path, 0.0);
  ASSERT_FALSE(footprint.empty());
  ASSERT_TRUE(footprint.size() == imu_path.size() - 1);
}

TEST_F(TestAEB, checkImuPathGenerationIsCut)
{
  const double dt = aeb_node_->imu_prediction_time_interval_;
  const double horizon = aeb_node_->imu_prediction_time_horizon_;
  const double max_generated_path_length = aeb_node_->max_generated_imu_path_length_;
  const double fast_velocity = 2.0 * max_generated_path_length / (horizon);
  constexpr double yaw_rate = 0.05;
  const auto imu_path = aeb_node_->generateEgoPath(fast_velocity, yaw_rate);

  ASSERT_FALSE(imu_path.empty());
  constexpr double epsilon{1e-3};
  ASSERT_TRUE(
    autoware::motion_utils::calcArcLength(imu_path) <=
    max_generated_path_length + dt * fast_velocity + epsilon);

  const auto footprint = aeb_node_->generatePathFootprint(imu_path, 0.0);
  ASSERT_FALSE(footprint.empty());
  ASSERT_TRUE(footprint.size() == imu_path.size() - 1);
}

TEST_F(TestAEB, checkEmptyPathAtZeroSpeed)
{
  const double velocity = 0.0;
  constexpr double yaw_rate = 0.0;
  const auto imu_path = aeb_node_->generateEgoPath(velocity, yaw_rate);
  ASSERT_EQ(imu_path.size(), 0);
}

TEST_F(TestAEB, checkParamUpdate)
{
  std::vector<rclcpp::Parameter> parameters{rclcpp::Parameter("param")};
  const auto result = aeb_node_->onParameter(parameters);
  ASSERT_TRUE(result.successful);
}

TEST_F(TestAEB, checkEmptyFetchData)
{
  ASSERT_FALSE(aeb_node_->fetchLatestData());
}

TEST_F(TestAEB, checkConvertObjectToPolygon)
{
  using autoware_perception_msgs::msg::Shape;
  PredictedObject obj_cylinder;
  obj_cylinder.shape.type = Shape::CYLINDER;
  obj_cylinder.shape.dimensions.x = 1.0;
  Pose obj_cylinder_pose;
  obj_cylinder_pose.position.x = 1.0;
  obj_cylinder_pose.position.y = 1.0;
  obj_cylinder.kinematics.initial_pose_with_covariance.pose = obj_cylinder_pose;
  const auto cylinder_polygon = utils::convertObjToPolygon(obj_cylinder);
  ASSERT_FALSE(cylinder_polygon.outer().empty());

  PredictedObject obj_box;
  obj_box.shape.type = Shape::BOUNDING_BOX;
  obj_box.shape.dimensions.x = 1.0;
  obj_box.shape.dimensions.y = 2.0;
  Pose obj_box_pose;
  obj_box_pose.position.x = 1.0;
  obj_box_pose.position.y = 1.0;
  obj_box.kinematics.initial_pose_with_covariance.pose = obj_box_pose;
  const auto box_polygon = utils::convertObjToPolygon(obj_box);
  ASSERT_FALSE(box_polygon.outer().empty());

  geometry_msgs::msg::TransformStamped tf_stamped;
  geometry_msgs::msg::Transform transform;

  constexpr double yaw{0.0};
  transform.rotation = autoware_utils::create_quaternion_from_yaw(yaw);
  geometry_msgs::msg::Vector3 translation;
  translation.x = 1.0;
  translation.y = 0.0;
  translation.z = 0.0;
  transform.translation = translation;
  tf_stamped.set__transform(transform);
  const auto t_obj_box = utils::transformObjectFrame(obj_box, tf_stamped);
  const auto t_pose = t_obj_box.kinematics.initial_pose_with_covariance.pose;
  Pose expected_pose;
  expected_pose.position.x = obj_box_pose.position.x + translation.x;
  expected_pose.position.y = obj_box_pose.position.y + translation.y;
  expected_pose.position.z = obj_box_pose.position.z + translation.z;

  ASSERT_DOUBLE_EQ(expected_pose.position.x, t_pose.position.x);
  ASSERT_DOUBLE_EQ(expected_pose.position.y, t_pose.position.y);
  ASSERT_DOUBLE_EQ(expected_pose.position.z, t_pose.position.z);
}

TEST_F(TestAEB, CollisionDataKeeper)
{
  using namespace std::literals::chrono_literals;
  constexpr double collision_keeping_sec{1.0}, previous_obstacle_keep_time{1.0};
  CollisionDataKeeper collision_data_keeper_(aeb_node_->get_clock());
  collision_data_keeper_.setTimeout(collision_keeping_sec, previous_obstacle_keep_time);
  ASSERT_TRUE(collision_data_keeper_.checkCollisionExpired());
  ASSERT_TRUE(collision_data_keeper_.checkPreviousObjectDataExpired());

  ObjectData obj;
  obj.stamp = aeb_node_->now();
  obj.velocity = 0.0;
  obj.position.x = 0.0;
  rclcpp::sleep_for(100ms);

  ObjectData obj2;
  obj2.stamp = aeb_node_->now();
  obj2.velocity = 0.0;
  obj2.position.x = 0.1;
  rclcpp::sleep_for(100ms);

  constexpr double ego_longitudinal_velocity = 3.0;
  constexpr double yaw_rate = 0.0;
  const auto imu_path = aeb_node_->generateEgoPath(ego_longitudinal_velocity, yaw_rate);

  const auto speed_null =
    collision_data_keeper_.calcObjectSpeedFromHistory(obj, imu_path, ego_longitudinal_velocity);
  ASSERT_FALSE(speed_null.has_value());

  const auto median_velocity =
    collision_data_keeper_.calcObjectSpeedFromHistory(obj2, imu_path, ego_longitudinal_velocity);
  ASSERT_TRUE(median_velocity.has_value());

  // object speed is 1.0 m/s greater than ego's = 0.1 [m] / 0.1 [s] + longitudinal_velocity
  ASSERT_TRUE(std::abs(median_velocity.value() - 4.0) < 1e-2);
  rclcpp::sleep_for(1100ms);
  ASSERT_TRUE(collision_data_keeper_.checkCollisionExpired());
}

TEST_F(TestAEB, getCellsFromObstacleGridGating)
{
  const float floor = static_cast<float>(aeb_node_->cluster_minimum_height_);  // 0.1 m
  const double z_band_top =
    aeb_node_->vehicle_info_.vehicle_height_m + aeb_node_->detection_range_max_height_margin_;
  auto cells_of = [&](const grid_map_msgs::msg::GridMap & g) {
    PointCloud::Ptr cells = pcl::make_shared<PointCloud>();
    aeb_node_->getCellsFromObstacleGrid(g, cells);
    return cells;
  };
  // Package defaults the point-sum gate is calibrated against; pinned so a default change fails
  // loudly instead of silently. All block fixtures use cell-CENTER coordinates (odd multiples of
  // 0.1 for ROI offset 20 / res 0.2) so each point lands in a distinct, contiguous cell.
  ASSERT_EQ(aeb_node_->minimum_cluster_size_, 10);
  ASSERT_EQ(aeb_node_->min_point_count_cell_, 1);

  // (a) dense block spanning a real z range -> survives; emitted z must be the tallest IN-BAND
  //     return (low_max_height): independent oracle 0.9, distinct from zmin 0.2 and zmax 1.1.
  {
    const auto cells =
      cells_of(make_obstacle_grid_msg(5.1, 5.9, -0.3, 0.3, 0.2f, 1.1f, 0.9f, 5.0f));
    ASSERT_FALSE(cells->empty());
    for (const auto & p : *cells) {
      EXPECT_GE(p.z, floor);
      EXPECT_FLOAT_EQ(p.z, 0.9f);  // emitted z == low_max_height of the cell
    }
  }
  // (b) a block whose tallest in-band return is just below cluster_minimum_height -> rejected by
  //     the height floor. Derived from the param so it stays a genuine just-below-floor reject.
  {
    const auto cells = cells_of(
      make_obstacle_grid_msg(5.1, 5.9, -0.3, 0.3, 0.0f, floor - 0.05f, floor - 0.05f, 5.0f));
    EXPECT_TRUE(cells->empty());
  }
  // (c) overhead-only obstacle (gantry with no returns below the producer's overhead_split):
  //     low_max_height is NaN and min_height is above the z-band top -> rejected regardless of
  //     point count. Independent oracle from the node's own vehicle_info/margin.
  {
    const float zmin = static_cast<float>(z_band_top) + 0.5f;
    const auto cells = cells_of(make_obstacle_grid_msg(
      5.1, 5.9, -0.3, 0.3, zmin, zmin + 0.3f, std::numeric_limits<float>::quiet_NaN(), 50.0f));
    EXPECT_TRUE(cells->empty());
  }
  // (d) overhead structure sharing its cells with ground returns: min_height is low (0.05,
  //     under the band top) and max_height is overhead (2.8), but the tallest IN-BAND return is
  //     only the ground residue (0.05 < floor) -> rejected. A max_height-based floor would have
  //     qualified these cells and emergency-braked under a passable gantry.
  {
    const auto cells =
      cells_of(make_obstacle_grid_msg(5.1, 5.9, -0.3, 0.3, 0.05f, 2.8f, 0.05f, 50.0f));
    EXPECT_TRUE(cells->empty());
  }
  // (e) wall/truck: same (min_height, max_height) signature as (d) but with real in-band mass
  //     (low_max_height 2.0 >= floor) -> detected. low_max_height is the only discriminator
  //     between (d) and (e); (min_height, max_height) alone cannot tell them apart.
  {
    ASSERT_LE(2.0, z_band_top + 1e-9);
    const auto cells =
      cells_of(make_obstacle_grid_msg(5.1, 5.9, -0.3, 0.3, 0.05f, 2.8f, 2.0f, 50.0f));
    EXPECT_FALSE(cells->empty());
  }
  // (f) small-footprint obstacle (pedestrian-class): a 2x2-cell block whose summed point_count
  //     (4 x 10 = 40) reaches minimum_cluster_size -> detected. The former per-window CELL count
  //     structurally rejected any obstacle under ~10 occupied cells (~0.4 m^2 footprint).
  {
    const auto cells =
      cells_of(make_obstacle_grid_msg(5.1, 5.3, -0.1, 0.1, 0.5f, 1.7f, 1.7f, 10.0f));
    EXPECT_FALSE(cells->empty());
  }
  // (g) single dense cell (a pole trunk: one cell, 30 returns) -> sum 30 >= 10 -> detected, and
  //     exactly the 4 corner points of the cell [5.0,5.2]x[0.0,0.2] are emitted (edge-aware).
  {
    const auto cells =
      cells_of(make_obstacle_grid_msg(5.1, 5.1, 0.1, 0.1, 0.2f, 1.2f, 1.2f, 30.0f));
    ASSERT_EQ(cells->size(), 4u);
    for (const auto & p : *cells) {
      EXPECT_NEAR(std::abs(p.x - 5.1), 0.1, 1e-6);
      EXPECT_NEAR(std::abs(p.y - 0.1), 0.1, 1e-6);
      EXPECT_FLOAT_EQ(p.z, 1.2f);
    }
  }
  // (h) single sparse cell (5 returns < 10) -> rejected: isolated sparse returns stay rejected.
  {
    const auto cells = cells_of(make_obstacle_grid_msg(5.1, 5.1, 0.1, 0.1, 0.5f, 0.5f, 0.5f, 5.0f));
    EXPECT_TRUE(cells->empty());
  }
  // (i) disjoint sparse cells must NOT pool: two 5-return cells 0.6 m apart (not 8-connected)
  //     each form a component of sum 5 < 10 -> rejected; making one of them 8-adjacent to a
  //     second 5-return cell connects them (5 + 5 = 10) -> that pair is detected.
  {
    grid_map::GridMap g({"max_height", "min_height", "point_count", "low_max_height"});
    g.setFrameId("base_link");
    g.setGeometry(grid_map::Length(60.0, 40.0), 0.2, grid_map::Position(20.0, 0.0));
    for (const char * name : {"max_height", "min_height", "point_count", "low_max_height"}) {
      g[name].setConstant(std::numeric_limits<float>::quiet_NaN());
    }
    auto occupy = [&g](const double x, const double y) {
      grid_map::Index idx;
      ASSERT_TRUE(g.getIndex(grid_map::Position(x, y), idx));
      g.at("point_count", idx) = 5.0f;
      g.at("max_height", idx) = 0.5f;
      g.at("min_height", idx) = 0.5f;
      g.at("low_max_height", idx) = 0.5f;
    };
    occupy(5.1, 0.1);
    occupy(5.7, 0.1);  // 0.6 m away -> a full empty cell between -> disjoint
    const auto cells = cells_of(*grid_map::GridMapRosConverter::toMessage(g));
    EXPECT_TRUE(cells->empty());
    occupy(5.3, 0.1);  // 8-adjacent to the 5.1 cell -> component sum 10
    const auto cells2 = cells_of(*grid_map::GridMapRosConverter::toMessage(g));
    EXPECT_FALSE(cells2->empty());
  }
  // (j) wrong frame -> the whole grid is rejected (a grid cannot be re-framed): no cells, no
  //     silent misinterpretation of map-frame coordinates as base_link.
  {
    const auto cells =
      cells_of(make_obstacle_grid_msg(5.1, 5.9, -0.3, 0.3, 0.2f, 0.9f, 0.9f, 30.0f, "map"));
    EXPECT_TRUE(cells->empty());
  }
  // (k) missing required layer -> the whole grid is rejected without throwing (an uncaught
  //     std::out_of_range would escape the diag-updater timer and kill the component container).
  {
    grid_map::GridMap g({"max_height", "min_height", "point_count"});  // no low_max_height
    g.setFrameId("base_link");
    g.setGeometry(grid_map::Length(60.0, 40.0), 0.2, grid_map::Position(20.0, 0.0));
    for (const char * name : {"max_height", "min_height", "point_count"}) {
      g[name].setConstant(0.5f);
    }
    PointCloud::Ptr cells = pcl::make_shared<PointCloud>();
    EXPECT_NO_THROW(
      aeb_node_->getCellsFromObstacleGrid(*grid_map::GridMapRosConverter::toMessage(g), cells));
    EXPECT_TRUE(cells->empty());
  }
}

// Repeatedly publish then pump both executors, giving the AEB node's polling subscribers time to
// discover the publishers and receive the (transient-local) samples before fetchLatestData reads.
// AEB derives from autoware::agnocast_wrapper::Node, which is not an rclcpp::Node, so it is pumped
// through its node base interface.
void deliver(
  const rclcpp::Node::SharedPtr & pub_node, const std::shared_ptr<AEB> & sub_node,
  const std::function<void()> & publish_fn, const int iterations = 40)
{
  for (int i = 0; i < iterations; ++i) {
    publish_fn();
    rclcpp::spin_some(pub_node);
    rclcpp::spin_some(sub_node->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

// Pin generateEgoPath(v, w) against an INDEPENDENT closed-form bicycle-model recurrence written
// directly in this test (never calling the SUT and never reading its output back). The stop
// condition is constrained to the time horizon alone so the pose count is deterministic.
TEST_F(TestAEB, imuPathBicycleModelOracle)
{
  aeb_node_->limit_imu_path_length_ = false;
  aeb_node_->limit_imu_path_lat_dev_ = false;
  aeb_node_->min_generated_imu_path_length_ = 0.0;
  aeb_node_->imu_prediction_time_interval_ = 0.1;
  aeb_node_->imu_prediction_time_horizon_ = 0.5;
  constexpr double dt = 0.1;

  // Independent recurrence: position advances with the PREVIOUS yaw, then yaw integrates. The loop
  // breaks once t exceeds the horizon (0.5 s), matching the node's break-before-push ordering.
  const auto expected_poses = [&](const double v, const double w) {
    std::vector<std::array<double, 3>> poses;  // {x, y, yaw}
    poses.push_back({0.0, 0.0, 0.0});
    double x = 0.0, y = 0.0, yaw = 0.0, t = 0.0;
    while (true) {
      const double nx = x + v * std::cos(yaw) * dt;
      const double ny = y + v * std::sin(yaw) * dt;
      const double nyaw = yaw + w * dt;
      t += dt;
      if (t > 0.5) break;
      x = nx;
      y = ny;
      yaw = nyaw;
      poses.push_back({x, y, yaw});
    }
    return poses;
  };

  // Turning case.
  {
    constexpr double v = 1.0;
    constexpr double w = 0.2;
    const auto path = aeb_node_->generateEgoPath(v, w);
    const auto oracle = expected_poses(v, w);
    ASSERT_EQ(path.size(), 6u);
    ASSERT_EQ(path.size(), oracle.size());
    // Anchor the first integrated pose with pure numeric literals.
    EXPECT_NEAR(path.at(1).position.x, 0.1, 1e-9);
    EXPECT_NEAR(path.at(1).position.y, 0.0, 1e-9);
    EXPECT_NEAR(tf2::getYaw(path.at(1).orientation), 0.02, 1e-9);
    for (size_t i = 0; i < path.size(); ++i) {
      EXPECT_NEAR(path.at(i).position.x, oracle.at(i).at(0), 1e-9);
      EXPECT_NEAR(path.at(i).position.y, oracle.at(i).at(1), 1e-9);
      EXPECT_NEAR(tf2::getYaw(path.at(i).orientation), oracle.at(i).at(2), 1e-9);
    }
  }

  // Degenerate straight-line case (w = 0): pure-literal oracle, y and yaw stay zero.
  {
    const auto path = aeb_node_->generateEgoPath(1.0, 0.0);
    ASSERT_EQ(path.size(), 6u);
    const std::array<double, 6> expected_x{0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
    for (size_t i = 0; i < path.size(); ++i) {
      EXPECT_NEAR(path.at(i).position.x, expected_x.at(i), 1e-9);
      EXPECT_NEAR(path.at(i).position.y, 0.0, 1e-12);
      EXPECT_NEAR(tf2::getYaw(path.at(i).orientation), 0.0, 1e-12);
    }
  }
}

// The yaw rate must be copied verbatim from the published kinematic_state twist into
// angular_velocity_ptr_->z (a direct field copy, so exact equality is the correct oracle).
TEST_F(TestAEB, kinematicStateYawRatePlumbing)
{
  aeb_node_->check_autoware_state_ = false;
  aeb_node_->use_pointcloud_data_ = false;
  aeb_node_->use_predicted_object_data_ = true;  // satisfies the object-detection-method gate
  aeb_node_->use_predicted_trajectory_ = false;  // isolate the IMU (kinematic_state) path
  aeb_node_->use_imu_path_ = true;

  constexpr double yaw_rate = -0.1234;
  const auto publish = [&]() {
    const auto header = get_header("base_link", pub_sub_node_->now());
    pub_sub_node_->pub_velocity_->publish(make_velocity_report_msg(header, 0.0, 3.0, 0.0));
    pub_sub_node_->pub_predicted_objects_->publish(PredictedObjects{});
    pub_sub_node_->pub_kinematic_state_->publish(make_odometry_message(header, yaw_rate));
  };
  deliver(pub_sub_node_, aeb_node_, publish);

  ASSERT_TRUE(aeb_node_->fetchLatestData());
  ASSERT_NE(aeb_node_->angular_velocity_ptr_, nullptr);
  ASSERT_DOUBLE_EQ(aeb_node_->angular_velocity_ptr_->z, yaw_rate);
}

// Intake guard: with the IMU path enabled and no other path, a missing kinematic_state makes
// fetchLatestData fail; supplying kinematic_state flips it to succeed (positive control).
TEST_F(TestAEB, missingKinematicStateTripsImuPathGuard)
{
  aeb_node_->check_autoware_state_ = false;
  aeb_node_->use_pointcloud_data_ = false;
  aeb_node_->use_predicted_object_data_ = true;
  aeb_node_->use_predicted_trajectory_ = false;
  aeb_node_->use_imu_path_ = true;

  const auto publish_without_kinematic_state = [&]() {
    const auto header = get_header("base_link", pub_sub_node_->now());
    pub_sub_node_->pub_velocity_->publish(make_velocity_report_msg(header, 0.0, 3.0, 0.0));
    pub_sub_node_->pub_predicted_objects_->publish(PredictedObjects{});
  };
  deliver(pub_sub_node_, aeb_node_, publish_without_kinematic_state);
  EXPECT_FALSE(aeb_node_->fetchLatestData());

  const auto publish_with_kinematic_state = [&]() {
    const auto header = get_header("base_link", pub_sub_node_->now());
    pub_sub_node_->pub_velocity_->publish(make_velocity_report_msg(header, 0.0, 3.0, 0.0));
    pub_sub_node_->pub_predicted_objects_->publish(PredictedObjects{});
    pub_sub_node_->pub_kinematic_state_->publish(make_odometry_message(header, 0.05));
  };
  deliver(pub_sub_node_, aeb_node_, publish_with_kinematic_state);
  EXPECT_TRUE(aeb_node_->fetchLatestData());
}

// With use_imu_path=false, the kinematic_state subscription must not be consulted at all: the path
// comes solely from the predicted trajectory and angular_velocity_ptr_ stays null.
TEST_F(TestAEB, useImuPathFalseIgnoresKinematicState)
{
  aeb_node_->check_autoware_state_ = false;
  aeb_node_->use_pointcloud_data_ = false;
  aeb_node_->use_predicted_object_data_ = true;
  aeb_node_->use_predicted_trajectory_ = true;
  aeb_node_->use_imu_path_ = false;

  const auto publish = [&]() {
    const auto header = get_header("base_link", pub_sub_node_->now());
    pub_sub_node_->pub_velocity_->publish(make_velocity_report_msg(header, 0.0, 3.0, 0.0));
    pub_sub_node_->pub_predicted_objects_->publish(PredictedObjects{});
    pub_sub_node_->pub_predicted_traj_->publish(Trajectory{});
    // kinematic_state deliberately withheld.
  };
  deliver(pub_sub_node_, aeb_node_, publish);

  ASSERT_TRUE(aeb_node_->fetchLatestData());
  EXPECT_EQ(aeb_node_->angular_velocity_ptr_, nullptr);
}

// The obstacle-grid intake in fetchLatestData accepts a FRESH grid (populates obstacle_grid_ptr_)
// and the staleness watchdog REJECTS a grid older than obstacle_grid_timeout_sec_ — the polling
// subscriber returns the last grid forever and the producer stays silent on its failure paths, so
// an aged grid must read as "unavailable", never as "clear". Independent oracle: the age is
// hand-set through the message header stamp and compared against the pinned package default.
TEST_F(TestAEB, obstacleGridStalenessWatchdog)
{
  ASSERT_DOUBLE_EQ(aeb_node_->obstacle_grid_timeout_sec_, 0.5);  // pin the calibrated default
  aeb_node_->check_autoware_state_ = false;
  aeb_node_->use_pointcloud_data_ = true;
  aeb_node_->use_predicted_object_data_ = false;  // isolate the obstacle-grid detection method
  aeb_node_->use_predicted_trajectory_ = false;
  aeb_node_->use_imu_path_ = true;

  // Fresh grid (stamp = publish time): age ~0 < 0.5 s -> accepted; fetchLatestData succeeds and
  // obstacle_grid_ptr_ is populated.
  const auto publish_fresh = [&]() {
    const auto header = get_header("base_link", pub_sub_node_->now());
    pub_sub_node_->pub_velocity_->publish(make_velocity_report_msg(header, 0.0, 3.0, 0.0));
    pub_sub_node_->pub_kinematic_state_->publish(make_odometry_message(header, 0.05));
    auto grid = make_obstacle_grid_msg(5.1, 5.9, -0.3, 0.3, 0.2f, 1.1f, 0.9f, 5.0f);
    grid.header.stamp = pub_sub_node_->now();
    pub_sub_node_->pub_obstacle_grid_->publish(grid);
  };
  deliver(pub_sub_node_, aeb_node_, publish_fresh);
  ASSERT_TRUE(aeb_node_->fetchLatestData());
  ASSERT_NE(aeb_node_->obstacle_grid_ptr_, nullptr);

  // Stale grid (stamp = 5 s in the past): age 5 s > 0.5 s -> watchdog trips; fetchLatestData fails
  // even though a grid message is present, so a frozen grid can never be mistaken for a clear road.
  const auto publish_stale = [&]() {
    const auto header = get_header("base_link", pub_sub_node_->now());
    pub_sub_node_->pub_velocity_->publish(make_velocity_report_msg(header, 0.0, 3.0, 0.0));
    pub_sub_node_->pub_kinematic_state_->publish(make_odometry_message(header, 0.05));
    auto grid = make_obstacle_grid_msg(5.1, 5.9, -0.3, 0.3, 0.2f, 1.1f, 0.9f, 5.0f);
    grid.header.stamp = rclcpp::Time(pub_sub_node_->now()) - rclcpp::Duration::from_seconds(5.0);
    pub_sub_node_->pub_obstacle_grid_->publish(grid);
  };
  deliver(pub_sub_node_, aeb_node_, publish_stale);
  EXPECT_FALSE(aeb_node_->fetchLatestData());
}

// Intake guard: with the pointcloud detection method enabled and no grid published at all,
// fetchLatestData fails on the missing-grid guard (distinct from the staleness path: a missing
// grid never reaches the age check).
TEST_F(TestAEB, missingObstacleGridTripsPointcloudGuard)
{
  aeb_node_->check_autoware_state_ = false;
  aeb_node_->use_pointcloud_data_ = true;
  aeb_node_->use_predicted_object_data_ = false;
  aeb_node_->use_predicted_trajectory_ = false;
  aeb_node_->use_imu_path_ = true;

  const auto publish_without_grid = [&]() {
    const auto header = get_header("base_link", pub_sub_node_->now());
    pub_sub_node_->pub_velocity_->publish(make_velocity_report_msg(header, 0.0, 3.0, 0.0));
    pub_sub_node_->pub_kinematic_state_->publish(make_odometry_message(header, 0.05));
    // obstacle grid deliberately withheld.
  };
  deliver(pub_sub_node_, aeb_node_, publish_without_grid);
  EXPECT_FALSE(aeb_node_->fetchLatestData());
}

// The kinematic_state yaw source (topic ~/input/kinematic_state) has no producer-side heartbeat
// guarantee, and the Latest polling subscriber returns the last Odometry forever, so a frozen or
// diverged localization would feed a stale yaw rate into the integrated ego-path prediction
// indefinitely. A kinematic_state older than kinematic_state_timeout_sec_ must make the imu-path
// yaw source read as UNAVAILABLE (angular_velocity_ptr_ reset to null so the imu path is skipped),
// never as a usable estimate. Independent oracle: the age is hand-set through the Odometry header
// stamp and compared against the pinned package default; a second (predicted-trajectory) path is
// supplied so fetchLatestData can still succeed and the assertion isolates the yaw source alone.
TEST_F(TestAEB, kinematicStateStalenessWatchdog)
{
  ASSERT_DOUBLE_EQ(aeb_node_->kinematic_state_timeout_sec_, 0.5);  // pin the calibrated default
  aeb_node_->check_autoware_state_ = false;
  aeb_node_->use_pointcloud_data_ = false;
  aeb_node_->use_predicted_object_data_ = true;  // satisfies the object-detection-method gate
  aeb_node_->use_predicted_trajectory_ = true;   // yaw-independent second path -> fetch can succeed
  aeb_node_->use_imu_path_ = true;

  constexpr double yaw_rate = 0.05;

  // Fresh kinematic_state (stamp = publish time): age ~0 < 0.5 s -> yaw source accepted and
  // angular_velocity_ptr_ is populated with the published yaw rate.
  const auto publish_fresh = [&]() {
    const auto header = get_header("base_link", pub_sub_node_->now());
    pub_sub_node_->pub_velocity_->publish(make_velocity_report_msg(header, 0.0, 3.0, 0.0));
    pub_sub_node_->pub_predicted_objects_->publish(PredictedObjects{});
    pub_sub_node_->pub_predicted_traj_->publish(Trajectory{});
    pub_sub_node_->pub_kinematic_state_->publish(make_odometry_message(header, yaw_rate));
  };
  deliver(pub_sub_node_, aeb_node_, publish_fresh);
  ASSERT_TRUE(aeb_node_->fetchLatestData());
  ASSERT_NE(aeb_node_->angular_velocity_ptr_, nullptr);
  ASSERT_DOUBLE_EQ(aeb_node_->angular_velocity_ptr_->z, yaw_rate);

  // Stale kinematic_state (stamp = 5 s in the past): age 5 s > 0.5 s -> watchdog trips; the yaw
  // source is treated as unavailable so angular_velocity_ptr_ is reset to null and the imu path is
  // skipped this cycle. fetchLatestData still SUCCEEDS because the predicted trajectory supplies a
  // second, yaw-independent path -> a stale twist is never integrated into the ego-path prediction.
  const auto publish_stale = [&]() {
    const auto header = get_header("base_link", pub_sub_node_->now());
    pub_sub_node_->pub_velocity_->publish(make_velocity_report_msg(header, 0.0, 3.0, 0.0));
    pub_sub_node_->pub_predicted_objects_->publish(PredictedObjects{});
    pub_sub_node_->pub_predicted_traj_->publish(Trajectory{});
    const auto stale_stamp =
      rclcpp::Time(pub_sub_node_->now()) - rclcpp::Duration::from_seconds(5.0);
    const auto stale_header = get_header("base_link", stale_stamp);
    pub_sub_node_->pub_kinematic_state_->publish(make_odometry_message(stale_header, yaw_rate));
  };
  deliver(pub_sub_node_, aeb_node_, publish_stale);
  EXPECT_TRUE(aeb_node_->fetchLatestData());
  EXPECT_EQ(aeb_node_->angular_velocity_ptr_, nullptr);  // stale yaw must NOT feed prediction
}

}  // namespace autoware::motion::control::autonomous_emergency_braking::test
