"""Start the service server and one service client.

The client sends a single request and exits; launch it again (or call the
service from the CLI) for another round.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("a", default_value="2"),
        DeclareLaunchArgument("b", default_value="3"),
        Node(
            package="ros2_use_example",
            executable="service_server",
            output="screen",
        ),
        Node(
            package="ros2_use_example",
            executable="service_client",
            output="screen",
            arguments=[LaunchConfiguration("a"), LaunchConfiguration("b")],
        ),
    ])
