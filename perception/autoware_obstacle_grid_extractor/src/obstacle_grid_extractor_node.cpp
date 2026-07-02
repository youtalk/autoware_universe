// Copyright 2026 The Autoware Contributors
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
#include "obstacle_grid_extractor_node.hpp"

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>

#include <functional>
#include <memory>

namespace autoware::obstacle_grid_extractor
{
ObstacleGridExtractorNode::ObstacleGridExtractorNode(const rclcpp::NodeOptions & options)
: Node("obstacle_grid_extractor", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  ExtractorParams p;
  p.roi_length_x = declare_parameter<double>("roi.length_x");
  p.roi_length_y = declare_parameter<double>("roi.length_y");
  p.roi_offset_x = declare_parameter<double>("roi.offset_x");
  p.resolution = declare_parameter<double>("resolution");
  p.crop_z_min = static_cast<float>(declare_parameter<double>("crop.z_min"));
  p.crop_z_max = static_cast<float>(declare_parameter<double>("crop.z_max"));
  p.overhead_split = static_cast<float>(declare_parameter<double>("overhead_split"));
  extractor_ = std::make_unique<ObstacleGridExtractor>(p);

  using std::placeholders::_1;
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&ObstacleGridExtractorNode::onCloud, this, _1));
  // RELIABLE KEEP_LAST(1): small fixed-size grid on the last-resort path.
  pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
    "~/output/obstacle_grid", rclcpp::QoS(1).reliable());
}

void ObstacleGridExtractorNode::onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 in_base_link;
  try {
    const auto tf = tf_buffer_.lookupTransform(
      "base_link", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
    tf2::doTransform(*msg, in_base_link, tf);  // static extrinsic only
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF to base_link failed: %s", e.what());
    // Never publish a wrong-frame grid. Nothing (not even the all-NaN heartbeat) is published on
    // this path, so consumers MUST treat a stale grid stamp as "unavailable", never as "clear".
    return;
  }
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(in_base_link, cloud);
  std_msgs::msg::Header header = msg->header;
  header.frame_id = "base_link";
  pub_->publish(extractor_->extract(cloud, header));  // empty cloud -> heartbeat
}
}  // namespace autoware::obstacle_grid_extractor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::obstacle_grid_extractor::ObstacleGridExtractorNode)
