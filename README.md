# ROS2_seminar
[![main](https://github.com/CoreSenseEU/ROS2_seminar/actions/workflows/main.yaml/badge.svg)](https://github.com/CoreSenseEU/ROS2_seminar/actions/workflows/main.yaml)
[![codecov](https://codecov.io/gh/CoreSenseEU/ROS2_seminar/branch/master/graph/badge.svg)](https://codecov.io/gh/CoreSenseEU/ROS2_seminar)

This repository contains an example ros2 package quality 1 according to the Coresense project guidelines.

## Install and building

Inside the src folder of your [workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) execute the following commands:
```
$ git clone https://github.com/CoreSenseEU/ROS2_seminar.git
$ cd ..
$ vcs import . < ROS2_seminar/third_parties.repos
$ colcon build --symlink-install
```

## Usage

on one terminal run:

```
$ ros2 launch tiago_gazebo tiago_gazebo.launch.py
```
now open a different terminal and execute the node:
```
$ ros2 run obstacle_detector obstacle_detector_main --ros-args -r input_scan:=/scan_raw
```

## Nodes

* **obstacle_detector**
  * Publishers:
    * `/obstacle_pose` `[geometry_msgs/msg/PointStamped]` 
  * Subscribers:
    * `/input_scan   ` `[sensor_msgs/msg/LaserScan]` 

## Acknowledgments

<img src="https://coresenseeu.github.io/_images/funding.png" width="400" height="100">