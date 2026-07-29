// Copyright 2020 Tier IV, Inc.
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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "autoware_utils/ros/logger_level_configure.hpp"
#include "autoware_utils/ros/polling_subscriber.hpp"
#include "debug_marker.hpp"
#include "type_alias.hpp"
#include "types.hpp"

#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <autoware/obstacle_grid_utils/obstacle_grid_utils.hpp>
#include <autoware/obstacle_proximity_checker/obstacle_proximity_checker.hpp>
#include <autoware_surround_obstacle_checker/surround_obstacle_checker_node_parameters.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit_clear_command.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::surround_obstacle_checker
{

enum class State { PASS, STOP };

class SurroundObstacleCheckerNode : public rclcpp::Node
{
public:
  explicit SurroundObstacleCheckerNode(const rclcpp::NodeOptions & node_options);

private:
  std::array<double, 3> getCheckDistances(const std::string & str_label) const;

  bool getUseDynamicObject() const;

  void onTimer();

  obstacle_proximity_checker::Parameters toProximityCheckerParameters() const;

  // Converts the qualifying cells of the latest obstacle grid into a base_link point cloud.
  // std::nullopt means the grid is unavailable (not yet received / wrong frame / unconvertible /
  // missing a required layer / stale), whereas an empty cloud means the grid is valid and nothing
  // qualifies inside the ROI. The distinction drives exactly one decision, in onTimer: in
  // State::STOP an unavailable grid blocks the STOP -> PASS release, so a latched stop is held
  // across ticks until a fresh valid grid restores normal hysteresis-based clearing. In State::PASS
  // an unavailable grid simply yields no pointcloud-based protection for that cycle -- the same
  // exposure a dead ~/input/pointcloud produced before this migration, carried over unchanged.
  std::optional<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> toObstacleGridPointCloud() const;

  obstacle_proximity_checker::Inputs toProximityCheckerInputs(
    const std::optional<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> & obstacle_grid_pointcloud) const;

  auto isStopRequired(
    const bool is_obstacle_found, const bool is_vehicle_stopped, const State & state,
    const std::optional<rclcpp::Time> & last_obstacle_found_time, const double time_threshold) const
    -> std::pair<bool, std::optional<rclcpp::Time>>;

  // ros
  rclcpp::TimerBase::SharedPtr timer_;
  // Cached clock for throttled logging from const methods. Dereferencing this shared_ptr yields a
  // non-const rclcpp::Clock &, which the RCLCPP_*_THROTTLE macros require; the const get_clock()
  // overload would instead return a ConstSharedPtr whose Clock::now() is non-const on Humble.
  rclcpp::Clock::SharedPtr clock_;

  // publisher and subscriber
  autoware_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry> sub_odometry_{
    this, "~/input/odometry"};
  // Obstacle-grid intake (plain rclcpp; InterProcessPollingSubscriber's default QoS{1} is already
  // RELIABLE / KEEP_LAST(1), matching the producer). Replaces the raw no-ground point cloud.
  autoware_utils::InterProcessPollingSubscriber<grid_map_msgs::msg::GridMap> sub_obstacle_grid_{
    this, "~/input/obstacle_grid"};
  autoware_utils::InterProcessPollingSubscriber<PredictedObjects> sub_dynamic_objects_{
    this, "~/input/objects"};
  rclcpp::Publisher<VelocityLimitClearCommand>::SharedPtr pub_clear_velocity_limit_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_velocity_limit_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_processing_time_;

  // stop checker
  std::unique_ptr<VehicleStopChecker> vehicle_stop_checker_;

  // proximity checker
  std::unique_ptr<obstacle_proximity_checker::ProximityChecker> proximity_checker_;

  // debug
  std::shared_ptr<SurroundObstacleCheckerDebugNode> debug_ptr_;

  // parameter
  std::shared_ptr<surround_obstacle_checker_node::ParamListener> param_listener_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  // data
  nav_msgs::msg::Odometry::ConstSharedPtr odometry_ptr_;
  grid_map_msgs::msg::GridMap::ConstSharedPtr obstacle_grid_ptr_;
  PredictedObjects::ConstSharedPtr object_ptr_;

  // State Machine
  State state_ = State::PASS;
  std::optional<rclcpp::Time> last_obstacle_found_time_;

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;

  std::unordered_map<int, std::string> label_map_;

public:
  friend class SurroundObstacleCheckerNodeTest;
};
}  // namespace autoware::surround_obstacle_checker

#endif  // NODE_HPP_
