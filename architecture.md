# SO-101 Perception-Driven Pick-and-Place — Architecture

**Target:** detect the red cup, pick it, place it in the gray bin. Ignore the green cup.
**Stack:** ROS 2 Humble · Isaac Sim 5.0.0 (Docker + WebRTC) · MoveIt 2 · py_trees · OpenCV · CycloneDDS.

---

## 1. High-level data flow

```
┌─────────────────────┐        /isaac_joint_states          ┌─────────────────────┐
│                     │ ──────────────────────────────────► │                     │
│     Isaac Sim       │        /clock, /external_cam/*      │  ros2_control       │
│  (Docker + WebRTC)  │                                     │  TopicBasedSystem   │
│   - SO-101 robot    │                                     │                     │
│   - red + green cup │ ◄────── /isaac_joint_command ────── │  arm_controller     │
│   - gray bin        │ ◄────── /isaac_attach_cube ──────── │  gripper_controller │
└─────────────────────┘                                     └──────────┬──────────┘
          ▲                                                            │
          │                                                  /joint_states
          │                                                            ▼
          │                                                 ┌────────────────────┐
          │                                                 │     move_group     │
          │         /external_cam/color,depth,info          │   (MoveIt 2 OMPL)  │
          │                  │                              └─────────┬──────────┘
          │                  ▼                                        │
          │         ┌────────────────┐      /detected_cup_pose        │
          │         │  cup_detector  │ ─────(in base_link)──────────► │
          │         │  (HSV + RGB-D) │                                │
          │         └────────────────┘                                │
          │                                                           ▼
          │                                              ┌────────────────────────┐
          └────────────── /isaac_attach_cube ─────────── │   bt_node (py_trees)   │
                                                         │   Pick-and-Place BT    │
                                                         └────────────────────────┘
```

All nodes run with `use_sim_time:=true`. RMW = **CycloneDDS** (FastDDS drops discovery across the Docker `network=host` boundary in Isaac Sim 5.0.0).

---

## 2. Packages

| Package | Role |
|---|---|
| `so101_description` | URDF + meshes. Arm + eye-in-hand camera link (`camera_link` → optical frames) + external `external_cam_optical_frame` static TF child. |
| `so101_moveit_config` | Planning group `arm` (5-DOF, KDL **position-only IK**) + `gripper`. SRDF named states `open`, `close`, `start`. |
| `so101_bringup` | `bringup_full.launch.py` (OpaqueFunction) spawns `robot_state_publisher`, `ros2_control_node`, controller spawners, `move_group`, `cup_detector`, static TF for the overhead camera, Foxglove bridge, and a 15 s-delayed `bt_node`. |
| `so101_state_machine` | `cup_detector.py` (perception) + `bt_node.py` (py_trees BT executor). |
| `isaac-usd/` | Isaac scene (`scene.usda`) + `bootstrap_graphs.py` (builds ROS camera/clock/grasp OmniGraphs on first Play tick) + `attach_detach_fixed_joint.py` (simulated grasp via `UsdPhysics.FixedJoint`). |

---

## 3. Perception pipeline (`cup_detector.py`)

1. **Subscribe** — RGB (BEST_EFFORT QoS), depth (RELIABLE QoS), CameraInfo. Depth frames are cached (latest wins); no `ApproximateTimeSynchronizer` (Isaac Sim timestamp skews > 200 ms).
2. **HSV red mask** — two-range (H `[0,12]` ∪ `[160,180]`, S ≥ 80, V ≥ 40). 31 × 31 close kernel merges handle into body; 3 × 3 open removes noise.
3. **Centroid + depth** — largest contour centroid (u, v); 5 × 5 median depth sample at (u, v).
4. **Back-projection** — `image_geometry.PinholeCameraModel.projectPixelTo3dRay(u, v)` scaled by z.
5. **TF to robot frame** — `tf2_ros.Buffer.transform()` with `time=0` (latest TF, avoiding sim-time / wall-time stamp mismatch) → pose is in **`base_link`** before publishing.

---

## 4. Behavior Tree (`bt_node.py`)

**MoveItPy** drives `arm` and `gripper` groups with explicit `PlanRequestParameters(pipeline=ompl, planner=RRTConnect)`. Every motion leaf inherits from a shared `_AsyncMotion` base class that runs `_plan_and_execute` on a daemon thread, so `update()` never blocks the 10 Hz tick. Failures are logged with MoveIt's `error_code`.

```
WaitForCup          gate: RUNNING until /detected_cup_pose publishes
OpenGripper1        MoveIt → SRDF "open"
Grabbing            MoveIt → perceived cup pose (Cartesian, base_link)
CloseGripper        MoveIt → SRDF "close"
SettleBeforeAttach  1.0 s Wait
AttachCube          [PROVIDED] publishes /isaac_attach_cube=True
MoveToBoxPosition   MoveIt → Cartesian BOX_DROP_POSITION above gray bin
SettleBeforeRelease 1.5 s Wait
OpenGripper2        MoveIt → SRDF "open"
SettleBeforeDetach  1.0 s Wait
DetachCube          [PROVIDED] publishes /isaac_attach_cube=False
```

Every motion leaf is wrapped in `Retry(2)`. Root is a `OneShot(ON_COMPLETION)` sequence. Hard-coded joint configurations have been eliminated — both the grasp pose and the drop pose are Cartesian.

---

## 5. Isaac Simulation integration

- `topic_based_ros2_control/TopicBasedSystem` hardware plugin bridges `/isaac_joint_states` ↔ `/isaac_joint_command`; `simulation:=true` activates this path.
- Joint drives in `scene.usda`: stiffness = 17.8, damping = 0.6.
- OmniGraphs are built by `bootstrap_graphs.py` on first Play tick (embedded ScriptNode in `scene.usda`): `/Graph/ROS_Camera`, `/Graph/ExternalCamera`, `/Graph/ROS_Clock`, `/Graph/GraspAttachGraph`.
- Grasp simulated by `attach_detach_fixed_joint.py` — creates a `UsdPhysics.FixedJoint` between `moving_jaw_so101_v1_link` and `SM_Mug_A2_red` at the jaw's current world pose, so nothing teleports on attach.

---

## 6. ROS 2 topic inventory

Grouped by producer. **Subscribers** in parentheses.

### 6.1 Isaac Sim (OmniGraph producers)

| Topic | Type | QoS | Purpose |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | reliable | Simulation clock driving every `use_sim_time` node (move_group, ros2_control, bt_node, cup_detector, foxglove_bridge). |
| `/isaac_joint_states` | `sensor_msgs/JointState` | reliable | Raw joint states from the Isaac articulation (ros2_control `TopicBasedSystem`). |
| `/isaac_joint_command` | `sensor_msgs/JointState` | reliable | Joint commands into Isaac (published by ros2_control, consumed by the `/Graph/ROS_JointStates` OmniGraph). |
| `/external_cam/color/image_raw` | `sensor_msgs/Image` | best_effort | Overhead RGB (640 × 480) — cup_detector, foxglove_bridge. |
| `/external_cam/depth/image_raw` | `sensor_msgs/Image` 32FC1 | reliable | Overhead depth — cup_detector (subscribed reliable to avoid Cyclone dropping the 1.2 MB/frame payload). |
| `/external_cam/color/camera_info` | `sensor_msgs/CameraInfo` | reliable | Overhead intrinsics — cup_detector (populates `PinholeCameraModel`). |
| `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info` | same as above | same | Eye-in-hand camera on `gripper_link`. Published but unused by the default perception path (reachable by overriding `rgb_topic` / `depth_topic` / `camera_info_topic` launch args). |
| `/isaac_attach_cube` | `std_msgs/Bool` | reliable | Grasp command. `true` → Isaac creates a FixedJoint between the jaw and the cup; `false` → removes it. |

### 6.2 ros2_control

| Topic / Action | Type | Purpose |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Aggregated joint state published by `joint_state_broadcaster` (move_group, RViz, foxglove). |
| `/dynamic_joint_states` | `control_msgs/DynamicJointState` | Per-interface state (position/velocity/effort). |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` **action** | MoveIt streams planned arm trajectories here. |
| `/gripper_controller/follow_joint_trajectory` | same | Gripper trajectory streaming. |
| `/arm_controller/controller_state`, `/gripper_controller/controller_state` | `control_msgs/JointControllerState` | Controller feedback. |
| `/arm_controller/transition_event`, `/gripper_controller/transition_event` | `lifecycle_msgs/TransitionEvent` | Lifecycle transitions on controller activation. |

### 6.3 robot_state_publisher + TF

| Topic | Type | Purpose |
|---|---|---|
| `/tf` | `tf2_msgs/TFMessage` | Dynamic kinematic chain (base_link → shoulder → … → gripper_frame_link). |
| `/tf_static` | `tf2_msgs/TFMessage` | `base_link → external_cam_optical_frame` (published by `external_cam_static_tf` node). |
| `/robot_description` | `std_msgs/String` | URDF XML (consumed by move_group + controllers). |

### 6.4 MoveIt 2 (move_group + MoveItPy)

| Topic | Type | Purpose |
|---|---|---|
| `/monitored_planning_scene` | `moveit_msgs/PlanningScene` | Live scene including the table collision box added by `bt_node._setup_planning_scene`. |
| `/planning_scene`, `/planning_scene_world` | `moveit_msgs/PlanningScene*` | Scene-update inbound topics. |
| `/collision_object` | `moveit_msgs/CollisionObject` | Dynamic collision-object updates. |
| `/attached_collision_object` | `moveit_msgs/AttachedCollisionObject` | Object-attachment updates (not used — we weld via Isaac FixedJoint). |
| `/display_planned_path` | `moveit_msgs/DisplayTrajectory` | Trajectory preview for RViz. |
| `/motion_plan_request` | `moveit_msgs/MotionPlanRequest` | Plan-request diagnostics. |
| `/trajectory_execution_event` | `std_msgs/String` | Execution events. |

Services & actions (`/compute_ik`, `/plan_kinematic_path`, `/execute_trajectory`, etc.) are exercised indirectly via MoveItPy.

### 6.5 Perception (`cup_detector`)

| Topic | Type | Purpose |
|---|---|---|
| `/detected_cup_pose` | `geometry_msgs/PoseStamped` | **Primary output.** Red-cup 3-D pose in `base_link`. Subscribed by `bt_node`'s `WaitForCup` (gate) and `Grabbing` (Cartesian goal). |
| `/detected_cup_marker` | `visualization_msgs/Marker` | Red sphere at the detected cup in RViz. |
| `/cup_detector/debug_image` | `sensor_msgs/Image` | Overlay for Foxglove: red-tinted HSV mask + green contour + red centroid + status text. |

### 6.6 BT executor (`bt_node`)

| Topic | Type | Purpose |
|---|---|---|
| `/bt_node/grasp_target` | `geometry_msgs/PoseStamped` | Current MoveIt Cartesian target (arrow in Foxglove/RViz). |
| `/bt_node/status` | `std_msgs/String` | Per-tick BT status (`"tick=N root=RUNNING leaf=Grabbing(RUNNING)"`). |

### 6.7 External tooling

- `foxglove_bridge` — WebSocket on port 8765, relays every topic to a browser client.

---

## 7. Coverage of assignment objectives

| Requirement | Where |
|---|---|
| ROS 2 workspace, launch, MoveIt loads, controllers active | `so101_bringup`, verified by `ros2 control list_controllers` |
| Motion planning (plan, execute, gripper, failure handling) | `_plan_and_execute` + `OpenGripper`/`CloseGripper`/`MoveTo*` + MoveIt `error_code` logging |
| RGB-D camera, red cup detected, pose in base frame | `/external_cam/*` + `cup_detector.py` (TF-transforms to `base_link` before publish) |
| Ignores green cup | HSV range in `cup_detector` is red-only; largest red contour wins |
| Pick-and-place via Behavior Tree | `bt_node.py` `create_tree()` — 11-step sequence, all actions through BT |
| **Bonus:** Planning scene obstacle avoidance | `bt_node._setup_planning_scene` adds a 1.2 × 0.8 × 0.05 m table box in `base_link` |
| **Bonus:** Dynamic collision objects | Added at runtime via `PlanningSceneInterface.applyCollisionObject` |
| **Bonus:** BT recovery | `Retry(2)` decorators on every motion leaf |
| **Bonus:** Execution timeout | `MoveItPy.execute(blocking=True)` + Retry gives bounded attempts |
| Frame consistency | Perception publishes directly in `base_link`; no hardcoded frame assumptions downstream |
| Sim-time consistency | All nodes with `use_sim_time:=true`; `moveit_py` yaml pre-declares `qos_overrides./clock.subscription.*` |
| Non-blocking BT | `_AsyncMotion` base class — every motion leaf runs on a daemon thread; `update()` returns RUNNING until the thread finishes |

---

## 8. Repo layout

```
Robotarm_motion_planning_and_perception/
├── isaac-usd/
│   ├── isaac_sim_scene/scene.usda
│   ├── bootstrap_graphs.py
│   └── omni_graph_script_node_usda/attach_detach_fixed_joint.py
├── so-arm/so101_ws/src/
│   ├── so101_description/
│   ├── so101_moveit_config/
│   ├── so101_bringup/
│   └── so101_state_machine/
│       └── so101_state_machine/
│           ├── bt_node.py       (BT + MoveItPy)
│           └── cup_detector.py  (HSV + RGB-D perception)
├── README.md
├── STARTUP.md
├── architecture.md   ← this file
```
