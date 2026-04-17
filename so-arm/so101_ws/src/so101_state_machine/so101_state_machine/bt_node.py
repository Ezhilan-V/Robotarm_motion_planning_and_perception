#!/usr/bin/env python3
"""
INTERVIEW TEMPLATE: ROS2 + py_trees Behaviour Tree (Implementation-Agnostic)

Task sequence:
1) open_gripper
2) grabbing
3) attach (simulation)        <-- PROVIDED (from our side)
4) move_to_box_position
5) detach (simulation)        <-- PROVIDED (from our side)
6) open_gripper

Instructions:
- Implement ONLY the TODO logic in OpenGripper / Grabbing / MoveToBoxPosition.
- You can use ANY ROS approach you want (services/actions/topics/MoveIt/etc.).
- Each leaf must return proper py_trees status: RUNNING / SUCCESS / FAILURE.
- Do not block inside update() (no sleep). Use state/futures/timers if needed.
"""

#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import py_trees
from std_msgs.msg import Bool


# -------------------------
# Candidate BT Leaves (blank)
# -------------------------
class OpenGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self.node = node
        # TODO: initialise any clients/publishers/actions you want here
        # self.some_client = ...

    def initialise(self):
        # TODO: reset internal state (futures/flags/timers) if needed
        pass

    def update(self) -> py_trees.common.Status:
        # TODO: implement open gripper
        # Return RUNNING until done, then SUCCESS/FAILURE
        return py_trees.common.Status.FAILURE


class Grabbing(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self.node = node
        # TODO: initialise any clients/publishers/actions you want here

    def initialise(self):
        # TODO: reset internal state
        pass

    def update(self) -> py_trees.common.Status:
        # TODO: implement grabbing logic
        return py_trees.common.Status.FAILURE


class MoveToBoxPosition(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self.node = node
        # TODO: initialise any clients/publishers/actions 
    def initialise(self):
        # TODO: reset internal state
        pass

    def update(self) -> py_trees.common.Status:
        # TODO: implement move to box position
        return py_trees.common.Status.FAILURE


# -------------------------
# PROVIDED: Attach / Detach Cube BT Leaf 
# -------------------------
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

    def update(self):
        if not self._done and (time.monotonic() - self._start_time) >= self.delay_sec:
            msg = Bool()
            msg.data = self.attach
            self.pub.publish(msg)
            self.node.get_logger().info(
                f"BT: Isaac attach={self.attach} on {self.topic_name}"
            )
            self._done = True
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


# -------------------------
# Tree
# -------------------------
def create_tree(node: Node):
    STEP_RETRIES = 2  # optional retry per-step (kept simple)
    ATTACH_TOPIC = "/robot/isaac_attach_cube"
    ATTACH_DELAY = 0.5

    seq = py_trees.composites.Sequence(name="TaskSequence", memory=True)

    seq.add_children([
        py_trees.decorators.Retry(
            "RetryOpen1",
            OpenGripper("OpenGripper1", node),
            STEP_RETRIES,
        ),
        py_trees.decorators.Retry(
            "RetryGrabbing",
            Grabbing("Grabbing", node),
            STEP_RETRIES,
        ),

        # PROVIDED
        AttachDetachCube("AttachCube", node, ATTACH_TOPIC, attach=True, delay_sec=ATTACH_DELAY),

        py_trees.decorators.Retry(
            "RetryMoveToBox",
            MoveToBoxPosition("MoveToBoxPosition", node),
            STEP_RETRIES,
        ),

        # PROVIDED
        AttachDetachCube("DetachCube", node, ATTACH_TOPIC, attach=False, delay_sec=ATTACH_DELAY),

        py_trees.decorators.Retry(
            "RetryOpen2",
            OpenGripper("OpenGripper2", node),
            STEP_RETRIES,
        ),
    ])

    # Run once (optional, keeps it from repeating forever)
    root = py_trees.decorators.OneShot(
        name="RunOnce",
        child=seq,
        policy=py_trees.common.OneShotPolicy.ON_COMPLETION
    )

    return root


# -------------------------
# Node
# -------------------------
class BTNode(Node):
    def __init__(self):
        super().__init__("bt_interview_template_node")

        self.tree = py_trees.trees.BehaviourTree(create_tree(self))
        self.timer = self.create_timer(0.1, self._tick)

        self.get_logger().info("Interview BT template node started.")

    def _tick(self):
        self.tree.tick()


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
