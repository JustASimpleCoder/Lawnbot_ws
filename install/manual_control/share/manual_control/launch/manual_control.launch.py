from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hoverboard_driver",
            executable="hoverboard_driver",
            output="screen"
        ),
        Node(
            package="manual_control",
            executable="manual_control",
            output="screen"
        )
    ])
