// Copyright 2023 Intelligent Robotics Lab
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


#include <memory>
#include <utility>
#include <algorithm>
#include <vector>
#include <optional>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "obstacle_detector/ObstacleDetectorNode.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace obstacle_detector
{

ObstacleDetectorNode::ObstacleDetectorNode(const rclcpp::NodeOptions & options)
: Node("obstacle_detector", options)
{
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(), std::bind(&ObstacleDetectorNode::scan_callback, this, _1));
  obstacle_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("obstacle_pose", 100);

  timer_ = create_wall_timer(50ms, std::bind(&ObstacleDetectorNode::control_cycle, this));
}

void
ObstacleDetectorNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void
ObstacleDetectorNode::control_cycle()
{
  // Skip cycle if no valid recent scan available
  if (last_scan_ == nullptr || (now() - last_scan_->header.stamp) > 1s) {
    return;
  }

  auto obstacle_coord = get_obstacle(*last_scan_);

  if (obstacle_pub_->get_subscription_count() > 0 && obstacle_coord.has_value()) {
    obstacle_pub_->publish(obstacle_coord.value());
  }

}

std::optional<geometry_msgs::msg::PointStamped>
ObstacleDetectorNode::get_obstacle(const sensor_msgs::msg::LaserScan & scan)
{
  geometry_msgs::msg::PointStamped ret;
  ret.header = scan.header;

  // Get the index of nearest obstacle
  int min_idx = std::min_element(scan.ranges.begin(), scan.ranges.end()) - scan.ranges.begin();

  // Get the distance to nearest obstacle
  float distance_min = scan.ranges[min_idx];
  float angle = scan.angle_min + scan.angle_increment * min_idx;

  if (std::isinf(distance_min) || std::isnan(distance_min)) {
    return {};
  }

  ret.point.x = cos(angle) * distance_min;
  ret.point.y = sin(angle) * distance_min;
  ret.point.z = 0.0;

  return ret;
}

}  // namespace obstacle_detector

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_detector::ObstacleDetectorNode)
