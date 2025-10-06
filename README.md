# Robot Navigation System

This repository implements a **navigation stack** for a differential-drive robot in **Gazebo (Ignition)** using **ROS 2 Humble**.  
It performs waypoint-based navigation with a **B-spline path smoother**, **time-parameterized trajectory generator**, and **Pure Pursuit controller**.

---

## Overview

### Data Flow

waypoints.txt
->
Path Smoother (B-spline)
->
Trajectory Generator (time-parameterized [x, y, yaw, v, w, t])
->
Pure Pursuit Controller (follows trajectory)
->
/cmd_vel → Rover MITI robot in Gazebo

---

## Package Breakdown

| Package | Description |
|----------|--------------|
| **`path_smoothing`** | Reads `waypoints.txt`, generates a smooth **B-spline path**, and publishes `/smooth_path` (`nav_msgs/Path`). |
| **`trajectory_generation`** | Converts the smooth path into a **time-parameterized trajectory** (`Float64MultiArray`) for tracking. |
| **`trajectory_tracking`** | Implements a **Pure Pursuit controller** that follows the trajectory using `/odometry/wheels` feedback. |
| **`simulation_launch`** | Launches Gazebo, the robot description, and all above nodes for end-to-end operation. |

---

##  Prerequisites

Ensure the following are installed:
- **Ubuntu 22.04**
- **ROS 2 Humble**
- **Gazebo Fortress (Ignition)**
- **Rover Robotics MITI Description**

### Build the Workspace

```bash
cd ~/robot10x_ws
colcon build --symlink-install
source install/setup.bash

### To Launch the System
ros2 launch simulation_launch navigation.launch.py use_sim_time:=true

```

This will:

Start Gazebo with an empty world

Spawn the Rover MITI robot

Launch:

path_smoothing

trajectory_generation

trajectory_tracking (Pure Pursuit)

ros_gz_bridge

robot_state_publisher

You’ll see console messages confirming path, trajectory, and tracking initialization.

## For Debugging

ros2 topic list

ros2 topic echo /smooth_path

ros2 topic echo /trajectory

## To test with different waypoints

In x y format, add your waypoints to the waypoints.txt file. The system requires there to more than 3 waypoints.

```bash
cd ~/robot10x_ws/path_smoothing
nano waypoints.txt
```
