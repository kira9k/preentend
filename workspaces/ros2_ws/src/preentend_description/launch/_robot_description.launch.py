from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    robot_xacro = PathJoinSubstitution(
        [FindPackageShare("preentend_description"), "robot", "preentend.urdf.xacro"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    robot_xacro,
                    " use_gazebo:=",
                    use_gazebo,
                    " use_ros2_control:=",
                    use_ros2_control,
                    " hardware_plugin:=",
                    hardware_plugin,
                ]
            ),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_gazebo", default_value="false"),
            DeclareLaunchArgument("use_ros2_control", default_value="false"),
            DeclareLaunchArgument(
                "hardware_plugin",
                default_value="mock_components/GenericSystem",
                choices=["mock_components/GenericSystem", "gz_ros2_control/GazeboSimSystem"],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
