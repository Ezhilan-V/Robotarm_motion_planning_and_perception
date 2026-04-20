#!/usr/bin/env python3
"""
SO101 Pick-and-Place Behavior Tree.

Integrates MoveIt 2 motion planning (via MoveItPy) with a py_trees behaviour
tree for perception-driven manipulation. The tree gates on a fresh cup pose,
drives the arm to the perceived cup, closes the gripper, carries the cup to a
Cartesian drop pose above the gray bin, and releases.

BT sequence:
  WaitForCup
  OpenGripper1 -> Grabbing -> CloseGripper
  SettleBeforeAttach
  AttachCube            [PROVIDED]
  MoveToBoxPosition
  SettleBeforeRelease -> OpenGripper2 -> SettleBeforeDetach
  DetachCube            [PROVIDED]
"""

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import py_trees
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Quaternion

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform with tf2 Buffer

from moveit.planning import MoveItPy, PlanRequestParameters


# ── Module constants ─────────────────────────────────────────────────────────
PLANNING_FRAME = "base_link"

# Drop pose above the gray bin in base_link (derived from the Isaac scene:
# bin_b02 world (-0.083, -0.132, 1.140) minus robot base world (-0.135, 0, 1.010)
# plus ~12 cm clearance in Z).
BOX_DROP_POSITION = (0.052, -0.132, 0.25)

# Workspace safety — SO-101 reach is ~0.35 m; warn past this distance.
REACH_WARN_M = 0.40

# Approach offset applied to the perceived cup before planning.
APPROACH_Z_OFFSET = 0.05

# Planning-scene table (obstacle avoidance bonus).
TABLE_SIZE = (1.2, 0.8, 0.05)   # x, y, z in metres
TABLE_ORIGIN = (0.35, 0.0, -0.03)

# BT tick rate.
TICK_HZ = 10.0


# ─── Helpers ─────────────────────────────────────────────────────────────────

def _plan_and_execute(moveit_py, planning_component, logger) -> bool:
    """Plan and execute synchronously with RRTConnect; return True on success."""
    planning_component.set_start_state_to_current_state()
    params = PlanRequestParameters(moveit_py)
    params.planning_pipeline = "ompl"
    params.planner_id = "RRTConnect"

    result = planning_component.plan(parameters=params)
    group = planning_component.planning_group_name
    if not result:
        err = getattr(result, "error_code", None)
        code = getattr(err, "val", None) if err is not None else None
        logger.warn(
            f"Planning FAILED for group '{group}'"
            + (f"  (error_code={code})" if code is not None else "")
        )
        return False
    ok = bool(moveit_py.execute(group, result.trajectory, blocking=True))
    if not ok:
        logger.warn(f"Execution FAILED for group '{group}'")
    return ok


# ─── Async motion base class ─────────────────────────────────────────────────

class _AsyncMotion(py_trees.behaviour.Behaviour):
    """Base leaf for any motion that must not block the BT tick timer.

    Subclasses implement ``_run`` which returns True/False.  The base class
    spins it on a daemon thread, flips status to RUNNING until the thread
    finishes, then returns SUCCESS or FAILURE.
    """

    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self.node = node
        self._thread: threading.Thread | None = None
        self._done = False
        self._success = False

    def initialise(self):
        self._thread = None
        self._done = False
        self._success = False

    def _execute(self):
        try:
            self._success = self._run()
        except Exception as exc:
            self.node.get_logger().error(f"{self.name}: {exc}")
            self._success = False
        finally:
            self._done = True

    def _run(self) -> bool:
        raise NotImplementedError

    def update(self) -> py_trees.common.Status:
        if self._thread is None:
            self._thread = threading.Thread(target=self._execute, daemon=True)
            self._thread.start()
            return py_trees.common.Status.RUNNING
        if not self._done:
            return py_trees.common.Status.RUNNING
        if self._success:
            self.node.get_logger().info(f"{self.name}: SUCCESS")
            return py_trees.common.Status.SUCCESS
        self.node.get_logger().warn(f"{self.name}: FAILURE")
        return py_trees.common.Status.FAILURE


# ─── Gripper leaves ──────────────────────────────────────────────────────────

class _GripperMove(_AsyncMotion):
    """Move gripper to an SRDF named state ('open' / 'close')."""

    def __init__(self, name: str, node: Node, configuration_name: str):
        super().__init__(name, node)
        self._configuration_name = configuration_name

    def _run(self) -> bool:
        self.node.get_logger().info(
            f"{self.name}: START — goal=SRDF '{self._configuration_name}'"
        )
        with self.node.gripper_lock:
            self.node.gripper_group.set_goal_state(
                configuration_name=self._configuration_name
            )
            return _plan_and_execute(
                self.node.moveit, self.node.gripper_group, self.node.get_logger()
            )


class OpenGripper(_GripperMove):
    """Move gripper to the SRDF 'open' state (1.7453 rad)."""
    def __init__(self, name: str, node: Node):
        super().__init__(name, node, configuration_name="open")


class CloseGripper(_GripperMove):
    """Move gripper to the SRDF 'close' state (0 rad)."""
    def __init__(self, name: str, node: Node):
        super().__init__(name, node, configuration_name="close")


# ─── Wait ────────────────────────────────────────────────────────────────────

class Wait(py_trees.behaviour.Behaviour):
    """Non-blocking timed delay."""

    def __init__(self, name: str, node: Node, seconds: float):
        super().__init__(name)
        self.node = node
        self.seconds = seconds
        self._start = None

    def initialise(self):
        self._start = time.monotonic()

    def update(self) -> py_trees.common.Status:
        if (time.monotonic() - self._start) >= self.seconds:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


# ─── WaitForCup ──────────────────────────────────────────────────────────────

class WaitForCup(py_trees.behaviour.Behaviour):
    """Gate the sequence on perception being live.

    Returns RUNNING until cup_detector has published at least one PoseStamped
    on /detected_cup_pose, then SUCCESS.  No timestamp-based staleness check:
    cup_detector's sim-time stamps compared against this node's wall-time
    clock produced bogus "stale" results — continuous publication is the gate.
    """

    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self.node = node
        self._announced = False

    def initialise(self):
        self._announced = False

    def update(self) -> py_trees.common.Status:
        pose = self.node.cup_pose
        if pose is None:
            if not self._announced:
                self.node.get_logger().info("WaitForCup: waiting for /detected_cup_pose …")
                self._announced = True
            return py_trees.common.Status.RUNNING
        p = pose.pose.position
        self.node.get_logger().info(
            f"WaitForCup: cup pose received at ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) — starting sequence"
        )
        return py_trees.common.Status.SUCCESS


# ─── Grabbing ────────────────────────────────────────────────────────────────

class Grabbing(_AsyncMotion):
    """Move arm to the perceived cup pose (base_link) with a Z approach offset."""

    def _run(self) -> bool:
        target = self._prepare_target()
        if target is None:
            return False
        with self.node.arm_lock:
            self.node.grasp_target_pub.publish(target)
            self.node.arm_group.set_goal_state(
                pose_stamped_msg=target, pose_link="gripper_frame_link"
            )
            ok = _plan_and_execute(
                self.node.moveit, self.node.arm_group, self.node.get_logger()
            )
            if ok:
                self.node.get_logger().info("Grabbing: reached cup via perception")
            else:
                self.node.get_logger().warn("Grabbing: perception-planned grasp FAILED")
            return ok

    def _prepare_target(self) -> PoseStamped | None:
        cup = self.node.cup_pose
        if cup is None:
            self.node.get_logger().warn("Grabbing: no cup pose available")
            return None
        if cup.header.frame_id != PLANNING_FRAME:
            try:
                cup = self.node.tf_buffer.transform(
                    cup, PLANNING_FRAME, timeout=Duration(seconds=1.0)
                )
            except Exception as exc:
                self.node.get_logger().warn(
                    f"Grabbing: TF {cup.header.frame_id} -> {PLANNING_FRAME} failed: {exc}"
                )
                return None
        p = cup.pose.position
        self.node.get_logger().info(
            f"Grabbing: cup in {PLANNING_FRAME} = ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
        )
        reach = (p.x ** 2 + p.y ** 2 + p.z ** 2) ** 0.5
        if reach > REACH_WARN_M:
            self.node.get_logger().warn(
                f"Grabbing: cup at {reach:.2f}m > {REACH_WARN_M}m; likely out of reach"
            )
        cup.pose.position.z += APPROACH_Z_OFFSET
        cup.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return cup


# ─── MoveToBoxPosition ───────────────────────────────────────────────────────

class MoveToBoxPosition(_AsyncMotion):
    """Move arm to BOX_DROP_POSITION (Cartesian target above the gray bin)."""

    def _run(self) -> bool:
        self.node.get_logger().info(
            f"MoveToBoxPosition: START — Cartesian target base_link={BOX_DROP_POSITION}"
        )
        drop = PoseStamped()
        drop.header.frame_id = PLANNING_FRAME
        drop.header.stamp = self.node.get_clock().now().to_msg()
        drop.pose.position.x, drop.pose.position.y, drop.pose.position.z = BOX_DROP_POSITION
        drop.pose.orientation.w = 1.0
        with self.node.arm_lock:
            self.node.grasp_target_pub.publish(drop)
            self.node.arm_group.set_goal_state(
                pose_stamped_msg=drop, pose_link="gripper_frame_link"
            )
            return _plan_and_execute(
                self.node.moveit, self.node.arm_group, self.node.get_logger()
            )


# ─── PROVIDED: Attach / Detach Cube ──────────────────────────────────────────

class AttachDetachCube(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, topic_name, attach, delay_sec=1.0):
        super().__init__(name)
        self.node = node
        self.topic_name = topic_name
        self.attach = attach
        self.delay_sec = delay_sec

        self.pub = self.node.create_publisher(Bool, topic_name, 10)
        self._start_time = None
        self._done = False

    def initialise(self):
        self._start_time = time.monotonic()
        self._done = False
        self.node.get_logger().info(
            f"{self.name}: START — will publish attach={self.attach} on {self.topic_name} after {self.delay_sec}s"
        )

    def update(self):
        if not self._done and (time.monotonic() - self._start_time) >= self.delay_sec:
            msg = Bool()
            msg.data = self.attach
            self.pub.publish(msg)
            self.node.get_logger().info(
                f"{self.name}: published attach={self.attach} on {self.topic_name}"
            )
            self._done = True
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


# ─── Tree ────────────────────────────────────────────────────────────────────

def create_tree(node: Node):
    STEP_RETRIES = 2
    ATTACH_TOPIC = "/isaac_attach_cube"
    ATTACH_DELAY = 0.5

    def _retry(name: str, leaf: py_trees.behaviour.Behaviour) -> py_trees.decorators.Retry:
        return py_trees.decorators.Retry(name, leaf, STEP_RETRIES)

    seq = py_trees.composites.Sequence(name="TaskSequence", memory=True)
    seq.add_children([
        WaitForCup("WaitForCup", node),
        _retry("RetryOpen1",      OpenGripper("OpenGripper1", node)),
        _retry("RetryGrabbing",   Grabbing("Grabbing", node)),
        _retry("RetryCloseGrip",  CloseGripper("CloseGripper", node)),
        Wait("SettleBeforeAttach", node, seconds=1.0),
        # PROVIDED
        AttachDetachCube("AttachCube", node, ATTACH_TOPIC, attach=True, delay_sec=ATTACH_DELAY),
        _retry("RetryMoveToBox",  MoveToBoxPosition("MoveToBoxPosition", node)),
        Wait("SettleBeforeRelease", node, seconds=1.5),
        _retry("RetryOpen2",      OpenGripper("OpenGripper2", node)),
        Wait("SettleBeforeDetach", node, seconds=1.0),
        # PROVIDED
        AttachDetachCube("DetachCube", node, ATTACH_TOPIC, attach=False, delay_sec=ATTACH_DELAY),
    ])

    return py_trees.decorators.OneShot(
        name="RunOnce",
        child=seq,
        policy=py_trees.common.OneShotPolicy.ON_COMPLETION,
    )


# ─── Node ────────────────────────────────────────────────────────────────────

class BTNode(Node):
    def __init__(self):
        super().__init__("bt_node")

        # MoveIt 2 Python interface (arm + gripper planning components)
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm_group = self.moveit.get_planning_component("arm")
        self.gripper_group = self.moveit.get_planning_component("gripper")
        self.arm_lock = threading.Lock()
        self.gripper_lock = threading.Lock()

        # TF buffer — used by Grabbing if /detected_cup_pose ever arrives in a
        # non-base_link frame (cup_detector already publishes in base_link).
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Perceived cup pose (subscriber callback + thread-safe accessor)
        self._cup_pose: PoseStamped | None = None
        self._cup_pose_lock = threading.Lock()
        self.create_subscription(PoseStamped, "/detected_cup_pose", self._on_cup_pose, 10)

        # Debug publishers: current MoveIt target + live BT status
        self.grasp_target_pub = self.create_publisher(PoseStamped, "/bt_node/grasp_target", 10)
        self.status_pub = self.create_publisher(String, "/bt_node/status", 10)

        self._setup_planning_scene()

        self.tree = py_trees.trees.BehaviourTree(create_tree(self))
        self._tick_count = 0
        self._last_tip_name = None
        self._last_root_status = None
        self.timer = self.create_timer(1.0 / TICK_HZ, self._tick)
        self.get_logger().info(f"BT node started – tree built, tick timer running @ {TICK_HZ:.0f} Hz")

    @property
    def cup_pose(self) -> PoseStamped | None:
        with self._cup_pose_lock:
            return self._cup_pose

    def _on_cup_pose(self, msg: PoseStamped):
        with self._cup_pose_lock:
            self._cup_pose = msg

    def _setup_planning_scene(self):
        """Add a table collision box so OMPL plans avoid the workbench."""
        try:
            from moveit_msgs.msg import CollisionObject
            from shape_msgs.msg import SolidPrimitive
            from geometry_msgs.msg import Pose

            table = CollisionObject()
            table.header.frame_id = PLANNING_FRAME
            table.id = "table"

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = list(TABLE_SIZE)

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = TABLE_ORIGIN
            pose.orientation.w = 1.0

            table.primitives = [box]
            table.primitive_poses = [pose]
            table.operation = CollisionObject.ADD

            with self.moveit.get_planning_scene_monitor().read_write() as scene:
                scene.apply_collision_object(table)
                scene.current_state.update()

            self.get_logger().info("Planning scene: table collision object added")
        except Exception as exc:
            self.get_logger().warn(f"Could not add table to planning scene: {exc}")

    def _tick(self):
        self._tick_count += 1
        try:
            self.tree.tick()
        except Exception as exc:
            self.get_logger().error(f"tree.tick() raised: {exc}")
            return

        tip = self.tree.root.tip()
        tip_name = tip.name if tip is not None else "<idle>"
        tip_status = tip.status.name if tip is not None else "NONE"
        root_status = self.tree.root.status.name

        self.status_pub.publish(
            String(data=f"tick={self._tick_count} root={root_status} leaf={tip_name}({tip_status})")
        )

        # Log on state change or once every ~5 s.
        changed = tip_name != self._last_tip_name or root_status != self._last_root_status
        if self._tick_count == 1 or changed or self._tick_count % 50 == 0:
            self.get_logger().info(
                f"BT tick #{self._tick_count}  root={root_status}  active_leaf={tip_name} ({tip_status})"
            )
            self._last_tip_name = tip_name
            self._last_root_status = root_status


# ─── Entry point ─────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = BTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
