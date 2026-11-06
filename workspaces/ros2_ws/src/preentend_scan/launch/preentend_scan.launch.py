import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _gazebo_and_scan_processes():
    for command_line in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            arguments = command_line.read_bytes().decode(errors="replace").split("\0")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        for index, argument in enumerate(arguments):
            if argument.rsplit("/", 1)[-1] == "ros2" and arguments[index + 1:index + 3] == ["launch", "preentend_scan"]:
                yield int(command_line.parent.name), "scan"
            if argument.rsplit("/", 1)[-1] == "gz" and arguments[index + 1:index + 2] == ["sim"]:
                yield int(command_line.parent.name), "gazebo"


def _check_previous_simulation():
    """A second Gazebo clock makes RViz repeatedly reset and can crash it."""
    for pid, kind in _gazebo_and_scan_processes():
        if pid == os.getpid():
            continue
        name = "Сканирование" if kind == "scan" else "Gazebo"
        raise RuntimeError(
            f"{name} уже запущен. Завершите предыдущий ros2 launch перед повторным запуском."
        )


def generate_launch_description():
    _check_previous_simulation()
    scan_config = LaunchConfiguration("scan_config")
    scan_output = LaunchConfiguration("scan_output")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("preentend_moveit_config"), "launch", "preentend_moveit_gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "rviz_config": PathJoinSubstitution(
                [FindPackageShare("preentend_scan"), "rviz", "scan.rviz"]
            )
        }.items(),
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
            simulation,
            console,
        ]
    )
