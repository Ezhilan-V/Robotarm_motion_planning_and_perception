# SO101 Pick-and-Place — Startup Guide

Step-by-step instructions to start Isaac Sim via Docker, connect via WebRTC, and run the full ROS 2 pick-and-place pipeline.

---

## Prerequisites (one-time check)

```bash
# Confirm Docker is running
docker info | grep "Server Version"

# Confirm workspace is built
ls ~/project/Robotarm_motion_planning_and_perception/so-arm/so101_ws/install/setup.bash

# Confirm CycloneDDS is the active RMW (required — FastDDS drops Isaac Sim topics
# across the Docker network=host boundary). Must print rmw_cyclonedds_cpp.
echo $RMW_IMPLEMENTATION
```

If CycloneDDS isn't active, add to `~/.bashrc`:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE
```
Install once: `sudo apt install -y ros-humble-rmw-cyclonedds-cpp`.

Also open GCP firewall port **8765** (Foxglove) if not done already:
```bash
gcloud compute firewall-rules create allow-foxglove-8765 --direction=INGRESS --action=ALLOW --rules=tcp:8765 --source-ranges=0.0.0.0/0
```

---

## Step 1 — Start the Isaac Sim container (shell entry)

Isaac Sim 5.0.0 segfaults on the auto-load + auto-play one-liner (crash inside `libisaacsim.ros2.bridge.plugin.so` → `libomni.graph.image.core.plugin.so` during camera graph init). The reliable path is to enter the container first, then launch `runheadless.sh` inside it and open the scene manually via the WebRTC UI.

```bash
docker run --name isaac-sim --entrypoint bash -it --rm \
  --runtime=nvidia --gpus all --network=host \
  -e "ACCEPT_EULA=Y" \
  -e "PRIVACY_CONSENT=Y" \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/documents:/root/Documents:rw \
  -v /home/ezhilan_veluchami/project/Robotarm_motion_planning_and_perception/isaac-usd:/scene:ro \
  nvcr.io/nvidia/isaac-sim:5.0.0
```

This drops you into a `bash` prompt inside the container at `/isaac-sim`. `--rm` removes the container on exit — no manual `docker rm` needed.

> To stop Isaac Sim: `Ctrl-C` the `runheadless.sh` process, then `exit` the container shell.

---

## Step 2 — Launch the headless streaming server (inside the container)

```bash
PUBLIC_IP=$(curl -s ifconfig.me) && \
  ./runheadless.sh \
    --/app/livestream/publicEndpointAddress=$PUBLIC_IP \
    --/app/livestream/port=49100
```

Wait ~2–3 minutes for the UI to fully initialize. Watch the log for `App is loaded` / `Simulation Ready`.

> **Do NOT pass `--/app/content/file=…` or `--/app/player/autoPlay=true`.** Combining scene autoload with autoPlay triggers the ROS2 bridge + camera OmniGraph segfault in Isaac Sim 5.0.0 — open the scene manually in Step 4 instead.

---

## Step 3 — Connect via WebRTC (from your local PC browser)

1. Note the `PUBLIC_IP` printed at the start of Step 2 (or re-run `curl ifconfig.me` on the VM).
2. Open in browser: `http://<VM_PUBLIC_IP>:49100`
3. You should see the empty Isaac Sim viewport once the app finishes loading.

---

## Step 4 — Open the scene manually in the WebRTC UI

In the Isaac Sim toolbar: **File → Open…** and navigate to:

```
/scene/isaac_sim_scene/scene.usda
```

(The `/scene` mount point comes from the `-v …/isaac-usd:/scene:ro` bind in Step 1.)

Wait for the SO101 arm, table, red cup, green cup, and bin to appear.

---

## Step 5 — Press Play in Isaac Sim

Click the **▶ Play** button in the WebRTC UI toolbar.

> This activates all OmniGraph nodes — joint states and camera images start publishing to ROS 2.

---

## Step 6 — Verify Isaac Sim is publishing (VM terminal)

Open a new terminal on the VM:

```bash
source /opt/ros/humble/setup.bash

# Robot joint states + simulated clock
ros2 topic hz /isaac_joint_states        # ~60 Hz (may run lower if sim throttled)
ros2 topic hz /clock                     # same rate

# Eye-in-hand camera (on gripper_link)
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw

# External overhead camera (primary perception source)
ros2 topic hz /external_cam/color/image_raw
ros2 topic hz /external_cam/depth/image_raw

# Bootstrap script status (from inside Isaac Sim container logs)
docker logs isaac-sim 2>&1 | grep bootstrap_graphs
# expect: "Creating /Graph/ROS_Camera / ExternalCamera / ROS_Clock / GraspAttachGraph" + "created successfully"
```

> If any `/external_cam/*` or `/camera/*` topic has 0 publishers, check the `runheadless.sh` terminal for OmniGraph errors and confirm Play is active.

---

## Step 7 — Rebuild when source files change

Needed after any edit to: URDF (`so101_description`), SRDF / MoveIt config (`so101_moveit_config`), launch files (`so101_bringup`), perception / BT (`so101_state_machine`), or Isaac scene / bootstrap scripts (mounted into the container read-only, so no rebuild on the ROS side — but you must restart Isaac Sim + re-Play).

```bash
cd ~/project/Robotarm_motion_planning_and_perception/so-arm/so101_ws
colcon build --packages-select so101_description so101_moveit_config so101_bringup so101_state_machine --symlink-install
source install/setup.bash
```

> `robot_state_publisher` caches the URDF at startup — any URDF edit requires a full **bringup relaunch**, not just a new `ros2 run`.

---

## Step 8 — Launch the full ROS 2 stack

Open a new terminal:

```bash
cd ~/project/Robotarm_motion_planning_and_perception/so-arm/so101_ws
source install/setup.bash

ros2 launch so101_bringup bringup_full.launch.py simulation:=true
```

Defaults route `cup_detector` at the **external overhead camera** (`/external_cam/*`). Eye-in-hand topics `/camera/*` are still published but unused by cup_detector. (Override with `rgb_topic:=/camera/color/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/color/camera_info output_frame:=camera_color_optical_frame` to swap back to eye-in-hand for testing.)

This starts:

| # | Node | Role |
|---|---|---|
| 1 | `robot_state_publisher` | Publishes URDF TF tree (arm + eye-in-hand camera) |
| 2 | `ros2_control_node` | Bridges `/isaac_joint_states` to controllers |
| 3 | `joint_state_broadcaster` | Publishes `/joint_states` |
| 4 | `arm_controller` + `gripper_controller` | Accept trajectory commands |
| 5 | `move_group` | MoveIt 2 planning server |
| 6 | `external_cam_static_tf` | Publishes `base_link → external_cam_optical_frame` static TF |
| 7 | `external_camera_marker` | Publishes `/external_camera_marker` MarkerArray (camera body + lens) for 3D view |
| 8 | `cup_detector` | Perception — HSV on external cam, publishes `/detected_cup_pose` |
| 9 | `foxglove_bridge` | WebSocket on port 8765 for browser visualization |
| 10 | `rviz2` | RViz (requires X display — use `ssh -X` from your laptop) |
| 11 | `bt_node` | Starts after **15 s delay** — runs the pick-and-place sequence |

---

## Step 9 — Verify controllers are active (~10 s after launch)

```bash
source ~/project/Robotarm_motion_planning_and_perception/so-arm/so101_ws/install/setup.bash

ros2 control list_controllers
```

Expected output:

```
arm_controller[joint_trajectory_controller/JointTrajectoryController] active
gripper_controller[joint_trajectory_controller/JointTrajectoryController] active
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
```

> If any show `inactive` or `unconfigured`:
> ```bash
> pkill -9 -f "ros2|move_group|ros2_control"
> ros2 daemon stop && ros2 daemon start
> # Then repeat Step 7
> ```

---

## Step 10 — Verify perception is working

```bash
# Cup pose being detected and published (overhead camera)
ros2 topic echo /detected_cup_pose --once

# External camera TF (static, publishes at startup)
ros2 run tf2_ros tf2_echo base_link external_cam_optical_frame
# expect translation ≈ (0.160, -1.0, -1.010), rotation (qw=0.5, qx=0.5, qy=-0.5, qz=-0.5)

# Eye-in-hand camera TF (moves with gripper — useful for debugging URDF)
ros2 run tf2_ros tf2_echo gripper_link camera_link
# expect translation (0.04, 0, 0.025), rpy (0, 1.309, 0) — 15° forward tilt
```

**Visual checks in Foxglove (http://<VM_IP>:8765):**
- Image panel on `/external_cam/color/image_raw` — cup visible from above.
- Image panel on `/cup_detector/debug_image` — red cup has green contour + red centroid.
- 3D panel with `/external_camera_marker` + RobotModel — you'll see the eye-in-hand camera on the gripper AND a floating black cuboid for the overhead camera.

---

## Step 11 — Watch the behavior tree execute

After the 15 s delay, `bt_node` starts automatically. Expected log in the launch terminal:

```
[bt_node] BT node started – waiting for MoveIt2 ready …
[bt_node] Planning scene: table collision object added
[bt_node] OpenGripper1: SUCCESS
[bt_node] Grabbing: reached cup via perception     # overhead cam always sees cup
[bt_node] BT: Isaac attach=True on /isaac_attach_cube
[bt_node] MoveToBoxPosition: SUCCESS
[bt_node] BT: Isaac attach=False on /isaac_attach_cube
[bt_node] OpenGripper2: SUCCESS
```

Watch the robot move in the Isaac Sim WebRTC window simultaneously.

---

## Verification Checklist

| Check | Command |
|---|---|
| Cyclone RMW active | `echo $RMW_IMPLEMENTATION` → `rmw_cyclonedds_cpp` |
| Isaac joint states flowing | `ros2 topic hz /isaac_joint_states` |
| Sim clock | `ros2 topic hz /clock` |
| Eye-in-hand camera active | `ros2 topic hz /camera/color/image_raw` |
| External camera active | `ros2 topic hz /external_cam/color/image_raw` |
| External cam TF | `ros2 run tf2_ros tf2_echo base_link external_cam_optical_frame` |
| Eye-in-hand TF (15° tilt) | `ros2 run tf2_ros tf2_echo gripper_link camera_link` |
| External cam marker | `ros2 topic hz /external_camera_marker` → ~1 Hz |
| Cup detected | `ros2 topic echo /detected_cup_pose --once` |
| Controllers active | `ros2 control list_controllers` |
| Attach topic working | `ros2 topic echo /isaac_attach_cube` |

---

## Quick Restart (if anything goes wrong)

```bash
# Kill all ROS processes
pkill -9 -f "ros2|move_group|ros2_control|rviz2"
ros2 daemon stop && ros2 daemon start

# Restart Isaac Sim: Ctrl-C inside the container to stop runheadless.sh,
# then `exit` the container shell (the --rm flag auto-removes it).
# Then repeat Steps 1–5.

# Relaunch ROS 2 stack
cd ~/project/Robotarm_motion_planning_and_perception/so-arm/so101_ws
source install/setup.bash
ros2 launch so101_bringup bringup_full.launch.py simulation:=true
```

---

## Launch Options

| Scenario | Command |
|---|---|
| Full system (default) | `ros2 launch so101_bringup bringup_full.launch.py simulation:=true` |
| MoveIt only (no BT/perception) | `ros2 launch so101_bringup bringup_moveit.launch.py simulation:=true` |
| Skip BT, start manually later | `ros2 launch so101_bringup bringup_full.launch.py simulation:=true launch_bt:=false` |
| Skip perception | `ros2 launch so101_bringup bringup_full.launch.py simulation:=true launch_perception:=false` |
| Run BT manually | `ros2 run so101_state_machine bt_node` |
| Run perception manually | `ros2 run so101_state_machine cup_detector` |
