"""
bringup_full.launch.py
Full SO101 system: MoveIt2 stack + RGB-D perception + behavior tree.

Usage:
  ros2 launch so101_bringup bringup_full.launch.py simulation:=true

Optional args:
  rgb_topic          (default /camera/color/image_raw)
  depth_topic        (default /camera/depth/image_raw)
  camera_info_topic  (default /camera/color/camera_info)
  launch_bt          (default true)   set false to start BT manually
  launch_perception  (default true)   set false if no camera connected
"""

import os
import yaml
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _as_bool(s: str) -> bool:
    return str(s).lower() in ["true", "1", "yes", "y", "on"]


def _extract_moveit_params(moveit_config):
    if hasattr(moveit_config, "to_dict"):
        return moveit_config.to_dict()
    params = {}
    for attr in [
        "robot_description",
        "robot_description_semantic",
        "robot_description_kinematics",
        "planning_pipelines",
        "trajectory_execution",
        "planning_scene_monitor_parameters",
        "joint_limits",
    ]:
        v = getattr(moveit_config, attr, None)
        if v is None:
            continue
        if isinstance(v, dict):
            params.update(v)
        else:
            try:
                params.update(dict(v))
            except Exception:
                pass
    return params


def _make_moveit_py_params_file(moveit_params, use_sim_time):
    """Write a params YAML under the 'moveit_py' node namespace so MoveItPy finds them."""
    params_with_sim = dict(moveit_params)
    params_with_sim["use_sim_time"] = use_sim_time

    # MoveItCpp reads planning_pipelines.pipeline_names, not planning_pipelines directly
    if "planning_pipelines" in params_with_sim and isinstance(
        params_with_sim["planning_pipelines"], list
    ):
        params_with_sim["planning_pipelines"] = {
            "pipeline_names": params_with_sim["planning_pipelines"]
        }

    # Pre-declare /clock subscription QoS with valid values so rclcpp's auto-override
    # under use_sim_time=true doesn't throw InvalidParameterValueException with an
    # empty reason (observed on Humble + moveit_py when the declaration races the set).
    params_with_sim["qos_overrides./clock.subscription.reliability"] = "reliable"
    params_with_sim["qos_overrides./clock.subscription.durability"] = "volatile"
    params_with_sim["qos_overrides./clock.subscription.history"] = "keep_last"
    params_with_sim["qos_overrides./clock.subscription.depth"] = 10

    yaml_content = {"moveit_py": {"ros__parameters": params_with_sim}}

    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", prefix="moveit_py_params_", delete=False
    )
    yaml.dump(yaml_content, tmp, default_flow_style=False, allow_unicode=True)
    tmp.flush()
    tmp.close()
    return tmp.name


def _launch_setup(context, *args, **kwargs):
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))

    moveit_config = MoveItConfigsBuilder(
        "so101_new_calib", package_name="so101_moveit_config"
    ).to_moveit_configs()
    moveit_params = _extract_moveit_params(moveit_config)

    # Write params under moveit_py namespace so MoveItPy's internal node picks them up
    moveit_py_params_file = _make_moveit_py_params_file(moveit_params, use_sim_time)

    # ── core MoveIt2 bringup ────────────────────────────────────────────
    moveit_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("so101_bringup"),
                "launch",
                "bringup_moveit.launch.py",
            ])
        ]),
        launch_arguments={
            "simulation": LaunchConfiguration("simulation"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # ── RGB-D perception node ───────────────────────────────────────────
    cup_detector = Node(
        package="so101_state_machine",
        executable="cup_detector",
        name="cup_detector",
        output="screen",
        parameters=[{
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "output_frame": LaunchConfiguration("output_frame"),
            "use_sim_time": use_sim_time,
        }],
        condition=IfCondition(LaunchConfiguration("launch_perception")),
    )

    # ── behavior tree executor (delayed 15s to let move_group fully start) ──
    # Parameters written under 'moveit_py' namespace so MoveItPy's internal node finds them
    bt_node = Node(
        package="so101_state_machine",
        executable="bt_node",
        name="bt_node",
        output="screen",
        parameters=[moveit_py_params_file],
        condition=IfCondition(LaunchConfiguration("launch_bt")),
    )

    delayed_bt = TimerAction(period=15.0, actions=[bt_node])

    # ── Fixed external overhead camera: static TF from base_link to optical frame ──
    # The camera prim is created by isaac-usd/bootstrap_graphs.py at world translate
    # (0.236, -0.31, 2.0). base_link is at world (-0.135, 0, 1.01). So the camera in
    # base_link coords is at (0.236 - (-0.135), -0.31 - 0, 2.0 - 1.01) = (0.371, -0.31, 0.99).
    # 180° rotation around X makes optical_frame +Z point DOWN (image forward = world -Z),
    # +X right = world +X, +Y down = world -Y — matching how Isaac renders the image.
    # If you tune the camera position in bootstrap_graphs.py, update these values too.
    external_cam_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="external_cam_static_tf",
        arguments=[
            # Camera at world (0.1, -0.2, 3.0), robot base_link at world
            # (-0.135, 0, 1.010) → relative (0.235, -0.2, 1.99).
            # USD rotateXYZ (0, 0, 0) — camera looks straight down along world -Z.
            # ROS optical: Z_forward = -Z_world, Y_down = -Y_world, X_right = +X_world.
            # Quaternion encoding 180° about X: (qw=0, qx=1, qy=0, qz=0).
            "--x", "0.235", "--y", "-0.2", "--z", "1.99",
            "--qx", "1.0", "--qy", "0.0", "--qz", "0.0", "--qw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "external_cam_optical_frame",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── Foxglove web bridge (browser visualization over WebSocket) ──────
    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("foxglove_port"),
            "address": "0.0.0.0",
            "use_sim_time": use_sim_time,
        }],
        condition=IfCondition(LaunchConfiguration("launch_foxglove")),
    )

    return [moveit_bringup, external_cam_static_tf, cup_detector, delayed_bt, foxglove]


def generate_launch_description():
    args = [
        DeclareLaunchArgument("simulation", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("launch_bt", default_value="true"),
        DeclareLaunchArgument("launch_perception", default_value="true"),
        # Perception defaults: use the fixed external overhead camera so the cup is
        # visible even when the arm is at the "start" pose. Swap back to
        # /camera/color/* to use the eye-in-hand camera instead.
        DeclareLaunchArgument("rgb_topic", default_value="/external_cam/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/external_cam/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/external_cam/color/camera_info"),
        DeclareLaunchArgument("output_frame", default_value="external_cam_optical_frame"),
        DeclareLaunchArgument("launch_foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
