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

#include <limits>
#include <vector>
#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "obstacle_detector/ObstacleDetector.hpp"

#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"

using namespace std::chrono_literals;

sensor_msgs::msg::LaserScan get_scan_test_1(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, std::numeric_limits<float>::infinity());

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_2(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 0.0);

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_3(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[2] = 0.3;

  return ret;
}


TEST(obstacle_detector_tests, get_obstacle)
{
  auto node_obstacle = rclcpp::Node::make_shared("node_obstacle");

  rclcpp::Time ts = node_obstacle->now();

  sensor_msgs::msg::LaserScan scan1 = get_scan_test_1(ts);
  sensor_msgs::msg::LaserScan scan2 = get_scan_test_2(ts);
  sensor_msgs::msg::LaserScan scan3 = get_scan_test_3(ts);

  auto res1 = obstacle_detector::get_obstacle(
    scan1.ranges, scan1.angle_min, scan1.angle_increment);
  ASSERT_FALSE(res1.has_value());

  auto res2 = obstacle_detector::get_obstacle(
    scan2.ranges, scan2.angle_min, scan2.angle_increment);
  ASSERT_TRUE(res2.has_value());
  ASSERT_NEAR(res2.value().x(), 0.0f, 0.00001f);
  ASSERT_NEAR(res2.value().y(), 0.0f, 0.00001f);
  ASSERT_NEAR(res2.value().z(), 0.0f, 0.00001f);

  auto res3 = obstacle_detector::get_obstacle(
    scan3.ranges, scan3.angle_min, scan3.angle_increment);
  ASSERT_TRUE(res3.has_value());
  ASSERT_LT(res3.value().x(), -0.0f);
  ASSERT_LT(res3.value().y(), -0.0f);
  ASSERT_NEAR(res3.value().z(), 0.0f, 0.00001f);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
