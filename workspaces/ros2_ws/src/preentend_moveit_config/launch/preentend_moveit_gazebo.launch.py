from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("preentend_gazebo"), "launch", "_gazebo.launch.py"]
            )
        ),
    )

    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("preentend_controller"), "launch", "_ros2_control.launch.py"]
            )
        ),
        launch_arguments={"hardware_plugin": "gz_ros2_control/GazeboSimSystem"}.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("preentend_moveit_config"), "launch", "_moveit_common.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": "true", "use_rviz": "true"}.items(),
    )

    return LaunchDescription([gazebo, ros2_control, moveit])
