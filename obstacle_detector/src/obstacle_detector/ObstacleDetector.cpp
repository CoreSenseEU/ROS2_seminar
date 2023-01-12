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


#include <vector>
#include <optional>
#include <utility>
#include <algorithm>

#include "tf2/LinearMath/Vector3.h"

namespace obstacle_detector
{

std::optional<tf2::Vector3>
get_obstacle(const std::vector<float> & distances, float angle_min, float angle_increment)
{
  tf2::Vector3 ret;

  // Get the index of nearest obstacle
  int min_idx = std::min_element(distances.begin(), distances.end()) - distances.begin();

  // Get the distance to nearest obstacle
  float distance_min = distances[min_idx];
  float angle = angle_min + angle_increment * min_idx;

  if (std::isinf(distance_min) || std::isnan(distance_min)) {
    return {};
  }

  ret.setX(cos(angle) * distance_min);
  ret.setY(sin(angle) * distance_min);
  ret.setZ(0.0);

  return ret;
}

}  // namespace obstacle_detector
