from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": ["-r -v 2 ", world]}.items(),
    )

    # Spawns the model from the robot_description topic, so a
    # robot_state_publisher must be running alongside this launch file.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "preentend",
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Topics come from preentend_description/urdf/camera/realsense.gazebo.xacro
    camera_prefixes = ("left_", "right_")
    image = "sensor_msgs/msg/Image[gz.msgs.Image"
    camera_info = "sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            arg
            for prefix in camera_prefixes
            for arg in (
                f"/{prefix}camera/depth/image_rect_raw@{image}",
                f"/{prefix}camera/depth/camera_info@{camera_info}",
                f"/{prefix}camera/color/image_raw@{image}",
                f"/{prefix}camera/color/camera_info@{camera_info}",
                f"/{prefix}camera/infra1/image_rect_raw@{image}",
                f"/{prefix}camera/infra1/camera_info@{camera_info}",
                f"/{prefix}camera/infra2/image_rect_raw@{image}",
                f"/{prefix}camera/infra2/camera_info@{camera_info}",
            )
        ],
        # only move data for topics that have ROS subscribers
        parameters=[{"use_sim_time": True, "lazy": True}],
        output="screen",
    )

    # gz depth_camera computes its own /points in the body frame (x forward) but stamps
    # them with the optical frame, so the cloud is rebuilt from the depth image instead
    point_clouds = [
        Node(
            package="depth_image_proc",
            executable="point_cloud_xyz_node",
            name=f"{prefix}camera_point_cloud",
            remappings=[
                ("image_rect", f"/{prefix}camera/depth/image_rect_raw"),
                ("camera_info", f"/{prefix}camera/depth/camera_info"),
                ("points", f"/{prefix}camera/depth/points"),
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )
        for prefix in camera_prefixes
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("preentend_gazebo"), "worlds", "empty.sdf"]
                ),
            ),
            gazebo,
            spawn_robot,
            clock_bridge,
            camera_bridge,
            *point_clouds,
        ]
    )
