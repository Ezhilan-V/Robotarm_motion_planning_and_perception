# SO101 Robotics Assignment – Setup & Launch Guide

## Overview

Full pick-and-place system for the SO101 robotic arm using:
- **ROS 2 Humble** + **MoveIt 2** for motion planning
- **Isaac Sim 5.0.0** (Docker headless + WebRTC streaming)
- **py_trees** behavior tree for task sequencing
- **OpenCV** RGB-D perception for red cup detection

---

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble (`source /opt/ros/humble/setup.bash`)
- MoveIt 2 for Humble (`sudo apt install ros-humble-moveit`)
- Docker + NVIDIA Container Toolkit (for Isaac Sim)
- Python packages: `py_trees`, `opencv-python`, `cv_bridge`

---

## Workspace Structure

```
so101_ws/
└── src/
    ├── so101_description      # URDF, meshes
    ├── so101_bringup          # Launch files, RViz configs
    ├── so101_moveit_config    # MoveIt2 planning config
    └── so101_state_machine    # BT executor + cup detector
```

---

## Build

```bash
cd so-arm/so101_ws

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

---

## Launch

### Step 1 – Start Isaac Sim (Docker headless)

```bash
cd ~/isaac-sim-dockerfiles
docker compose up -d

# Wait ~3 minutes, then watch for readiness
docker logs -f isaac-sim   # look for "App is loaded"

# Open WebRTC viewer in local browser
# URL: http://<VM_PUBLIC_IP>:49100
# Then press ▶ Play in the WebRTC UI
```

The container automatically:
- Loads `isaac-usd/isaac_sim_scene/scene.usda`
- Auto-plays the simulation
- Publishes joint states on `/isaac_joint_states`
- Publishes RGB camera on `/camera/color/image_raw` + `/camera/color/camera_info`
- Publishes depth camera on `/camera/depth/image_raw`

### Step 2 – Launch ROS 2 stack

```bash
cd so-arm/so101_ws
source install/setup.bash

# Option A: Full system (MoveIt + Perception + BT)
ros2 launch so101_bringup bringup_full.launch.py simulation:=true

# Option B: MoveIt only (no BT/perception)
ros2 launch so101_bringup bringup_moveit.launch.py simulation:=true
```

Option A launches:
1. `robot_state_publisher` + `ros2_control_node`
2. `arm_controller` + `gripper_controller` + `joint_state_broadcaster`
3. `move_group` (MoveIt2)
4. `cup_detector` (RGB-D perception)
5. `bt_node` (behavior tree executor, delayed 15 s to let move_group start)
6. `rviz2`

### Option C – Manual BT (after launching MoveIt separately)

```bash
# Terminal 1
ros2 launch so101_bringup bringup_moveit.launch.py simulation:=true

# Terminal 2 – perception
ros2 run so101_state_machine cup_detector \
  --ros-args -p rgb_topic:=/camera/color/image_raw \
             -p depth_topic:=/camera/depth/image_raw

# Terminal 3 – behavior tree
ros2 run so101_state_machine bt_node
```

---

## Camera Topic Mapping (Isaac Sim)

If Isaac Sim publishes on different topic names, pass them as launch arguments:

```bash
ros2 launch so101_bringup bringup_full.launch.py \
  rgb_topic:=/rgb \
  depth_topic:=/depth \
  camera_info_topic:=/camera_info
```

---

## Verify System Health

```bash
# Isaac Sim bridge
ros2 topic echo /isaac_joint_states --once

# Controllers active
ros2 control list_controllers

# Joint states flowing
ros2 topic echo /joint_states --once

# Camera publishing
ros2 topic hz /camera/color/image_raw

# Cup detection running
ros2 topic echo /detected_cup_pose --once

# Visualise detection in RViz: add Marker topic /detected_cup_marker
# Debug image
ros2 topic hz /cup_detector/debug_image
```

---

## Build a Single Package

```bash
colcon build --packages-select so101_state_machine
source install/setup.bash
```

## Run Tests

```bash
colcon test --packages-select so101_state_machine
colcon test-result --verbose
```

---

## Task Sequence (Behavior Tree)

| Step | Behavior | Implementation |
|------|----------|----------------|
| 1 | OpenGripper | MoveIt2 → gripper "open" state (1.7453 rad) |
| 2 | Grabbing | MoveIt2 → perceived cup pose or "Position_1" fallback |
| 3 | AttachCube | Publishes `True` → `/isaac_attach_cube` (USD FixedJoint) |
| 4 | MoveToBoxPosition | MoveIt2 → "Position_3" drop-off |
| 5 | DetachCube | Publishes `False` → `/isaac_attach_cube` (removes joint) |
| 6 | OpenGripper | MoveIt2 → gripper "open" state |

Each step has `Retry(2)` for robustness. `Grabbing` falls back to `Position_1`
if no cup pose is detected, then returns home if both fail. `MoveToBoxPosition`
tries via intermediate `Position_2` waypoint if the direct plan fails.

---

## Obstacle Avoidance (Bonus)

The `bt_node` automatically adds a table collision box to the MoveIt2 planning
scene at startup so trajectories avoid the table surface. No extra configuration
is required.
