## Isaac sim scene

This repo contains all the usd files that are required to load the scene.


The scene contains
1. SO101 robot
2. Table
3. Cups
4. Container Box

## Attach / Detach in Isaac (OmniGraph + ROS 2)

The Isaac USD scene includes an **OmniGraph-based attach/detach mechanism** for the grasped object.

### How it works (high level)
- An OmniGraph Script Node listens to a **ROS 2 Bool topic**.
- When the topic is **true**, the graph creates a **FixedJoint** between:
  - the gripper jaw link (SO101 end-effector jaw)
  - the target cup rigid body
- When the topic is **false**, the graph removes that FixedJoint.

This provides a simple “attach while grasped / detach when released” behavior in simulation.

### ROS 2 Interface
- **Topic:** `isaac_attach_cube`
- **Type:** `std_msgs/Bool`
- **Semantics:**
  - `data: true`  → attach cup to gripper
  - `data: false` → detach cup from gripper

### Scene Assumptions
The USD contains:
- a rigid body prim for the cup
- a rigid body prim for the gripper jaw link
- an OmniGraph that creates/removes the joint at runtime

### Usage
The provided Behavior Tree template publishes to `isaac_attach_cube` at the appropriate points in the sequence.

You do not need to modify the OmniGraph to use this mechanism.

### If attach/detach does not occur
Verify:
- Isaac simulation is running (Play pressed)
- the topic exists and messages are being published:
  ```bash
  ros2 topic echo /isaac_attach_cube
- Check the script for the prim bodies between which the joints are created