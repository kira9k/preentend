from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scan_config = LaunchConfiguration("scan_config")
    scan_output = LaunchConfiguration("scan_output")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("preentend_moveit_config"), "launch", "preentend_moveit_gazebo.launch.py"]
            )
        )
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
