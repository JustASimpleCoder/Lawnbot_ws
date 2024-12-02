from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hoverboard_driver",
            executable="hoverboard_driver",
            output='screen',
            parameters=[{'param1': 'value1'}],
        ),
        Node(
            package="manual_control",
            executable="manual_control",
            output="screen"
        )
    ])
