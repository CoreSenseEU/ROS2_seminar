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

#ifndef OBSTACLE_DETECTOR__OBSTACLEDETECTORNODE_HPP_
#define OBSTACLE_DETECTOR__OBSTACLEDETECTORNODE_HPP_

#include <memory>
#include <vector>
#include <optional>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

namespace obstacle_detector
{

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void control_cycle();

protected:
  std::optional<geometry_msgs::msg::PointStamped>
  get_obstacle(const sensor_msgs::msg::LaserScan & scan);

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr obstacle_pub_;
   rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
};

}  // namespace obstacle_detector

#endif  // OBSTACLE_DETECTOR__OBSTACLEDETECTORNODE_HPP_
