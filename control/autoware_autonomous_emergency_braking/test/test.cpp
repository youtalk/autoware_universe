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

#include <autoware_perception_msgs/msg/detail/shape__struct.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>

#include <gtest/gtest.h>
#include <pcl/memory.h>

#include <limits>
#include <memory>
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

Imu make_imu_message(
  const Header & header, const double ax, const double ay, const double yaw,
  const double angular_velocity_z)
{
  Imu imu_msg;
  imu_msg.header = header;
  imu_msg.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
  imu_msg.angular_velocity.z = angular_velocity_z;
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  return imu_msg;
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
// [x0,x1] x [y0,y1] at the production ROI/resolution; cells carry the given count + z band.
grid_map_msgs::msg::GridMap make_obstacle_grid_msg(
  double x0, double x1, double y0, double y1, float zmin, float zmax, float count)
{
  grid_map::GridMap g({"max_height", "min_height", "point_count"});
  g.setFrameId("base_link");
  g.setGeometry(grid_map::Length(60.0, 40.0), 0.2, grid_map::Position(20.0, 0.0));
  g["point_count"].setConstant(std::numeric_limits<float>::quiet_NaN());
  g["max_height"].setConstant(std::numeric_limits<float>::quiet_NaN());
  g["min_height"].setConstant(std::numeric_limits<float>::quiet_NaN());
  for (double x = x0; x <= x1 + 1e-9; x += 0.2) {
    for (double y = y0; y <= y1 + 1e-9; y += 0.2) {
      grid_map::Index idx;
      if (!g.getIndex(grid_map::Position(x, y), idx)) continue;
      g.at("point_count", idx) = count;
      g.at("max_height", idx) = zmax;
      g.at("min_height", idx) = zmin;
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

  pub_imu_ = create_publisher<Imu>("~/input/imu", qos);
  pub_obstacle_grid_ = create_publisher<grid_map_msgs::msg::GridMap>("~/input/obstacle_grid", qos);
  pub_velocity_ = create_publisher<VelocityReport>("~/input/velocity", qos);
  pub_predicted_traj_ = create_publisher<Trajectory>("~/input/predicted_trajectory", qos);
  pub_predicted_objects_ = create_publisher<PredictedObjects>("~/input/objects", qos);
  pub_autoware_state_ = create_publisher<AutowareState>("autoware/state", qos);
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
  // dense occupied block on the ego path -> survives the per-cell + window gate, and the surviving
  // cell centers are found on the imu path.
  const auto grid_msg = make_obstacle_grid_msg(0.5, 1.5, -0.5, 0.5, 0.5f, 0.5f, 5.0f);
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

  // (a) dense block spanning a real z range -> survives; emitted cell z must be max_height (not
  //     min_height): independent oracle zmax=0.9, distinct from zmin=0.2.
  {
    const auto cells = cells_of(make_obstacle_grid_msg(5.0, 6.0, -0.4, 0.4, 0.2f, 0.9f, 5.0f));
    ASSERT_FALSE(cells->empty());
    for (const auto & p : *cells) {
      EXPECT_GE(p.z, floor);
      EXPECT_FLOAT_EQ(p.z, 0.9f);  // emitted z == max_height of the cell
    }
  }
  // (b) a block whose max_height is just below cluster_minimum_height -> rejected by the height
  // gate.
  //     Derive the height from the param so it stays a genuine just-below-floor reject.
  {
    const auto cells =
      cells_of(make_obstacle_grid_msg(5.0, 6.0, -0.4, 0.4, 0.0f, floor - 0.05f, 5.0f));
    EXPECT_TRUE(cells->empty());
  }
  // (c) an overhead-only obstacle whose lowest return is above the z-band top (e.g. a gantry) ->
  //     rejected by the upper z-band gate (min_height <= vehicle_height + margin). Independent
  //     oracle from the node's own vehicle_info/margin, not the SUT formula.
  {
    const float zmin = static_cast<float>(z_band_top) + 0.5f;
    const auto cells =
      cells_of(make_obstacle_grid_msg(5.0, 6.0, -0.4, 0.4, zmin, zmin + 0.3f, 5.0f));
    EXPECT_TRUE(cells->empty());
  }
  // (d) a single sparse cell -> below the window min-occupied-cells threshold -> rejected.
  {
    const auto cells = cells_of(make_obstacle_grid_msg(5.0, 5.0, 0.0, 0.0, 0.5f, 0.5f, 5.0f));
    EXPECT_TRUE(cells->empty());
  }
  // (e) window min-occupied-cells boundary (assumes the package default minimum_cluster_size=10,
  //     window_size=2; pinned below so a default change fails loudly instead of silently). Uses
  //     cell-CENTER coordinates (odd multiples of 0.1 for offset 20 / res 0.2) so each point lands
  //     in a distinct, contiguous cell (boundary-aligned coords map ambiguously).
  ASSERT_EQ(aeb_node_->minimum_cluster_size_, 10);
  ASSERT_EQ(aeb_node_->window_size_, 2);
  {
    // a compact 3x3 block = 9 qualifying cells: the best-connected cell sees 9 < 10 -> all
    // rejected.
    const auto cells = cells_of(make_obstacle_grid_msg(5.1, 5.5, -0.1, 0.3, 0.5f, 0.5f, 5.0f));
    EXPECT_TRUE(cells->empty());
    // a 4x3 block = 12 qualifying cells: an interior cell sees 12 >= 10 -> survives.
    const auto cells2 = cells_of(make_obstacle_grid_msg(5.1, 5.7, -0.1, 0.3, 0.5f, 0.5f, 5.0f));
    EXPECT_FALSE(cells2->empty());
  }
}

}  // namespace autoware::motion::control::autonomous_emergency_braking::test
