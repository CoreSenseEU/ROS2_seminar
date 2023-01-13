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

#ifndef OBSTACLE_DETECTOR__OBSTACLEDETECTOR_HPP_
#define OBSTACLE_DETECTOR__OBSTACLEDETECTOR_HPP_

#include <vector>
#include <optional>

#include "tf2/LinearMath/Vector3.h"

namespace obstacle_detector
{
    //! A function that detects the point of the nearest obstacle
    /*!
      \sa get_obstacle()
      \param distances the laser readings.
      \param angle_min the starting angle of the reading.
      \param angle_increment the angle increment for reading i to i+1.
      \return An optional that contains the 3D coordinate of the nearest point
      \sa QTstyle_Test(), ~QTstyle_Test(), testMeToo() and publicVar()
    */ 
    std::optional<tf2::Vector3>
    get_obstacle(const std::vector<float> & distances, float angle_min, float angle_increment);

}  // namespace obstacle_detector

#endif  // OBSTACLE_DETECTOR__OBSTACLEDETECTOR_HPP_
