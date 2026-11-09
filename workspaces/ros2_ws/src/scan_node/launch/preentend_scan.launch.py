import copy
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _check_previous_simulation():
    """Avoid competing /clock publishers from another running Gazebo launch."""
    for command_line in Path("/proc").glob("[0-9]*/cmdline"):
        if int(command_line.parent.name) == os.getpid():
            continue
        try:
            arguments = command_line.read_bytes().decode(errors="replace").split("\0")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        for index, argument in enumerate(arguments):
            if argument.rsplit("/", 1)[-1] == "ros2" and arguments[index + 1:index + 3] == ["launch", "preentend_scan"]:
                raise RuntimeError("Сценарий preentend_scan уже запущен")
            if argument.rsplit("/", 1)[-1] == "gz" and arguments[index + 1:index + 2] == ["sim"]:
                raise RuntimeError("Gazebo уже запущен; завершите предыдущую симуляцию перед сканированием")


def _scan_moveit_config():
    """Add the scan-only dual-arm group without changing the base SRDF package."""
    config = (
        MoveItConfigsBuilder("preentend", package_name="preentend_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    # Keep scanning motion within the rail limit used by the previously
    # reliable setup, even if the shared MoveIt package is configured faster.
    # This override lives only in this launch and does not alter the base
    # preentend_moveit_config package.
    planning_limits = config.joint_limits["robot_description_planning"]["joint_limits"]
    for joint in ("left_linear_axis_joint", "right_linear_axis_joint"):
        planning_limits[joint]["max_velocity"] = min(
            float(planning_limits[joint]["max_velocity"]), 0.4
        )
    semantic_key = "robot_description_semantic"
    semantic = config.robot_description_semantic[semantic_key]
    if isinstance(semantic, str) and "name=\"dual_arm_camera\"" not in semantic:
        group = (
            "  <group name=\"dual_arm_camera\">"
            "<group name=\"left_arm_camera\"/>"
            "<group name=\"right_arm_camera\"/></group>\n"
        )
        config.robot_description_semantic[semantic_key] = semantic.replace(
            "</robot>", group + "</robot>"
        )
    ompl = config.planning_pipelines["ompl"]
    if "dual_arm_camera" not in ompl:
        ompl["dual_arm_camera"] = copy.deepcopy(ompl["left_arm_camera"])
    return config


def generate_launch_description():
    _check_previous_simulation()
    scan_config = LaunchConfiguration("scan_config")
    scan_output = LaunchConfiguration("scan_output")
    moveit_config = _scan_moveit_config()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("preentend_gazebo"), "launch", "_gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "world": PathJoinSubstitution(
                [FindPackageShare("preentend_scan"), "worlds", "scan_world.sdf"]
            )
        }.items(),
    )
    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("preentend_controller"), "launch", "_ros2_control.launch.py"]
            )
        ),
        launch_arguments={"hardware_plugin": "gz_ros2_control/GazeboSimSystem"}.items(),
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "allow_trajectory_execution": True,
                "publish_robot_description_semantic": True,
            },
        ],
    )
    collision = Node(
        package="preentend_scan",
        executable="add_car_collision.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("preentend_scan"), "rviz", "scan.rviz"]
            ),
        ],
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )
    console = Node(
        package="preentend_scan",
        executable="scan_console.py",
        name="scan_console",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "config_file": scan_config,
                "output_root": scan_output,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scan_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("preentend_scan"), "config", "scan.yaml"]
                ),
            ),
            DeclareLaunchArgument("scan_output", default_value="/ros2_ws/scans"),
            gazebo,
            ros2_control,
            move_group,
            collision,
            rviz,
            console,
        ]
    )
