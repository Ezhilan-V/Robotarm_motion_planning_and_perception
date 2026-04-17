# SO101 Robotics Assignment – Setup Guide

## Overview

This repository contains the required packages for the SO101 perception-driven motion planning assignment.

The workspace includes:

- SO101 robot description
- Bringup launch files
- MoveIt configuration
- Behavior Tree template

You are responsible for setting up the runtime environment and resolving any missing system dependencies.

---

## Prerequisites

- Ubuntu (22.04 or newer) 
- ROS2(Humble or newer)  installed
- Isaac Sim installed and configured
- MoveIt2 compatible with ROS2 

## Prefered

- Ubuntu (24.04) 
- ROS2 Jazzy  installed
- Isaac Sim installed and configured
- MoveIt2 compatible with ROS2 


Ensure your ROS 2 environment is sourced before building.

---

## Workspace Structure

Place all provided repositories inside a ROS 2 workspace:
```bash
so101_ws/
└── src
    ├── so101_description
    ├── so101_bringup
    ├── so101_moveit_config
    ├── so101_state_machine
    └── (additional packages)
```

## Build Instructions

```bash
cd so101_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Launch SO101 with MoveIt

```bash
ros2 launch so101_bringup bringup_moveit.launch.py use_fake_hardware:=true
```

Verify that:

- The robot loads successfully
- Controllers are active
- MoveIt initializes correctly

## Run Behavior Tree Node

In a separate terminal:

```bash
source so101_ws/install/setup.bash
ros2 run so101_state_machine bt_node
```


