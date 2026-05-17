from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_odom',
            executable='serial_to_odom',
            name='serial_to_odom',
            output='screen',
        ),
        Node(
            package='robot_control',
            executable='robot_controller_node',
            name='robot_serial_controller',
            output='screen',
        ),
    ])
