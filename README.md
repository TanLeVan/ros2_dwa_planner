## Overview
Very basic Dynamic Window Approach implementation. 
This is an reimplementation of DWA based on the repo https://github.com/amslabtech/dwa_planner/tree/master for ROS 2 Humble.
This repo was made as an attemp to fully understand the implementation of the Dynamic Window Approach. The repository is for learning purpose.

## Environment
- Ubuntu 22.04
- ROS Humble

## Dependencies
- geometry_msgs
- nav_msgs
- sensor_msgs
- visualization_msgs
- rosgraph_msgs
- builtin_interfaces
- rclcpp
- tf2
- tf2_ros
- tf2_geometry_msgs: For Quaternion related calculation
- Eigen3: For vector and matrix operation

## Installation
- `mkdir ~/ros_ws/src`
- `cd /ros_ws/src`
- `git clone https://github.com/TanLeVan/ros2_dwa_planner.git`
- `colcon build --packages-select ros2_dwa_planner`

## Feature 
- Publish appropriate velocity command for the robot to avoid obstacle while navigating to goal

### Input:
- "/scan" topic: 
    - Description: 2D scan message for obstacle detection
    - Type: sensor_msgs/msg/LaserScan
- "/odom" topic: 
    - Decsription: current velocity and position of the robot
    - Type: sensor_msgs/msg/Odom
- "/goal" topic: 
    - Description: 2D pose of goal
    - Type: geometry_msgs/msg/PoseStamped

### Output: 
- "/cmd_vel" topic: 
    - Description: Topic which the robot is expected to subscribe. The velocity of the robot is controlled via this topic
    - Type: geometry_msgs/msg/Twist

### Parameter
Adjust the behaviour of DWA through these parameter:
- linear_vel_sample_size_: how many discrete linear velocity sample is divided from the dinamic window. 
- yaw_rate_sample_size_: how many discrete angular velocity sample is divided from the dinamic window excluding 0(rad/s) (no turn motion). NOTE that 0 angular velocity case is always considered seperately.
- sim_time_: how far in the future the motion of robot is predicted for 1 pair of linear angular velocity
- sim_timestep_: time intervel in the process of predicting robot trajectory
- scan_range_: scan data point within this range will be acknowledged as obstacle. Point outside this range will not be considered as obstacle.
- skip_point_: while converting scan data point to obstacle point, how many points in scan data will be skipped consercutively. This is to reduce the number  of obstacle point to be processed.
- use_footprint_: If this is set to true, footprint of the robot should be provided via the "/footprint" topic with type *geometry_msgs/msg/PolygonStamped*.
- circle_footprint_radius_: Default footprint is circle. This is the radius of the default footprint.
- distance_to_goal_threshold_: How close to the goal to be considered as goal reached.

## Demonstration
Using turtlebot3_gazebo for demostration
- Start turtlebot3 simulation:
    `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`
- Start goal publisher:
    `ros2 run ros2_dwa_planner goal_publisher --ros-arg --params-file <path/to/config.yaml/file.installation>`
- Start dwa node:
    `ros2 run ros2_dwa_planner dwa_planner_node --ros-arg --params-file <path/to/config.yaml/file.installation>`

## To be improve 
- The process of calculating the distance between polygon footprint to obstacle take to long
- Create trajectory visualization
- Adding rotation at goal to rotate to the desired posed.
- Adding more custom criteria for selecting velocity beside goal cost, obstacle cost and speed cost
- Update README

## Reference
The original proposal of DWA algorithm collision avoidance is in:
- D. Fox, W. Burgard, and S.Thrun, "The dynamic window approach to collision avoidance", IEEE Robotics Automation Magazine, 1997.

https://github.com/amslabtech/dwa_planner/tree/master 