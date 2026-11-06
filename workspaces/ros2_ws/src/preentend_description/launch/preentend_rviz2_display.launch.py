from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare("preentend_description")
    launch_dir = [description_share, "launch"]

    joint_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([*launch_dir, "_robot_joint_state.launch.py"])
        ),
    )

    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([*launch_dir, "_rviz2.launch.py"])
        )
    )

    return LaunchDescription(
        [
            joint_state_publisher,
            rviz2,
        ]
    )
