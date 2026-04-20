import os

from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from moveit_configs_utils.substitutions import Xacro

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def _as_bool(s: str) -> bool:
    return str(s).lower() in ["true", "1", "yes", "y", "on"]


def _moveit_params(moveit_config):
    if hasattr(moveit_config, "to_dict"):
        return moveit_config.to_dict()

    params = {}
    for attr in [
        "robot_description",
        "robot_description_semantic",
        "robot_description_kinematics",
        "planning_pipelines",
        "trajectory_execution",
    ]:
        if hasattr(moveit_config, attr):
            v = getattr(moveit_config, attr)
            if isinstance(v, dict):
                params.update(v)
            else:
                try:
                    params.update(dict(v))
                except Exception:
                    pass

    if hasattr(moveit_config, "planning_scene_monitor_parameters"):
        v = getattr(moveit_config, "planning_scene_monitor_parameters")
        if isinstance(v, dict):
            params.update(v)
        else:
            try:
                params.update(dict(v))
            except Exception:
                pass

    return params


def generate_launch_description():
    declared_arguments = [DeclareLaunchArgument("moveit_config_pkg", default_value="so101_moveit_config"),
                          DeclareLaunchArgument("robot_name", default_value="so101_new_calib"),
                          DeclareLaunchArgument("rviz_config", default_value="config/moveit.rviz"),
                          DeclareLaunchArgument("ros2_controllers_file", default_value="config/ros2_controllers.yaml"),
                          DeclareLaunchArgument("use_sim_time", default_value="true"), DeclareLaunchArgument(
            "controller_names",
            default_value="joint_state_broadcaster arm_controller gripper_controller",
            description="Space-separated ros2_control controller names to spawn.",
        ), DeclareLaunchArgument(
            "simulation",
            default_value="true",
            description="Use mock ros2_control hardware instead of real hardware",
        )]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_launch_setup)])


def _launch_setup(context, *args, **kwargs):
    moveit_config_pkg = LaunchConfiguration("moveit_config_pkg").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)

    ros2_controllers_rel = LaunchConfiguration("ros2_controllers_file").perform(context)

    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))

    controller_names_str = LaunchConfiguration("controller_names").perform(context).strip()
    controller_names = controller_names_str.split() if controller_names_str else []

    moveit_share = get_package_share_directory(moveit_config_pkg)
    bringup_pkg_path = get_package_share_directory('so101_bringup')
    rviz_config_path = os.path.join(bringup_pkg_path, 'rviz', 'so101.rviz')
    ros2_controllers_path = os.path.join(moveit_share, ros2_controllers_rel)

    moveit_config = MoveItConfigsBuilder(robot_name, package_name=moveit_config_pkg).to_moveit_configs()
    moveit_common_params = _moveit_params(moveit_config)
    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("so101_moveit_config"),
                "config",
                "so101_new_calib.urdf.xacro",
            ]),
            " simulation:=", LaunchConfiguration("simulation"),
            " use_sim_time:=", LaunchConfiguration("use_sim_time"),
        ]),
        value_type=str,
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,  # ← FROM XACRO
            ros2_controllers_path,  # controllers yaml
            {"use_sim_time": use_sim_time},
        ]
    )

    all_controller_names = ["joint_state_broadcaster"] + [
        c for c in controller_names if c != "joint_state_broadcaster"
    ]

    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[c,
                       "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "60"],
            output="screen"
        )
        for c in all_controller_names
    ]

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_common_params, {"use_sim_time": use_sim_time}],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[moveit_common_params, {"use_sim_time": use_sim_time}],
    )

    # Delay spawners by 5s so controller_manager is fully up before spawn calls
    delayed_spawners = TimerAction(period=5.0, actions=spawners)

    return [
        robot_state_publisher,
        ros2_control_node,
        delayed_spawners,
        move_group,
        rviz2
    ]

