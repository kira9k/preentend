from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare("preentend_description")
    controller_share = FindPackageShare("preentend_controller")
    controllers_file = PathJoinSubstitution(
        [controller_share, "config", "ros2_controllers.yaml"]
    )

    def controller_spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "120",
                "--param-file",
                controllers_file,
            ],
            output="screen",
        )

    joint_states = controller_spawner("joint_state_broadcaster")
    left_controller = controller_spawner("left_arm_controller")
    right_controller = controller_spawner("right_arm_controller")

    def launch_setup(context):
        hardware_plugin = LaunchConfiguration("hardware_plugin").perform(context)
        use_gazebo = (
            "true" if hardware_plugin == "gz_ros2_control/GazeboSimSystem" else "false"
        )
        robot_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [description_share, "launch", "_robot_description.launch.py"]
                )
            ),
            launch_arguments={
                "use_sim_time": use_gazebo,
                "use_gazebo": use_gazebo,
                "use_ros2_control": "true",
                "hardware_plugin": hardware_plugin,
            }.items(),
        )

        actions = [
            RegisterEventHandler(
                OnProcessExit(target_action=joint_states, on_exit=[left_controller])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=left_controller, on_exit=[right_controller])
            ),
            robot_description,
        ]
        if hardware_plugin == "mock_components/GenericSystem":
            actions.append(
                Node(
                    package="controller_manager",
                    executable="ros2_control_node",
                    parameters=[controllers_file, {"use_sim_time": False}],
                    output="screen",
                )
            )
        actions.append(joint_states)
        return actions

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_plugin",
                default_value="mock_components/GenericSystem",
                choices=["mock_components/GenericSystem", "gz_ros2_control/GazeboSimSystem"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
