from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("ldlidar_node"),
                "/launch/ldlidar_with_mgr.launch.py",
            ]
        )
    )

    robot_driver_node = Node(
        package="robot_driver",
        executable="robot_driver_node",
        name="robot_driver",
        output="screen",
    )

    ldlidar_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_ldlidar_base",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "base_link",
            "ldlidar_base",
        ],
    )

    return LaunchDescription(
        [
            robot_driver_node,
            ldlidar_base_tf,
            ldlidar_launch,
        ]
    )
