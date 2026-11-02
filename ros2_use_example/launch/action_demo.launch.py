"""Start the action server and one action client.

Set cancel_after to a positive value to cancel the goal mid-execution.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("order", default_value="10"),
        DeclareLaunchArgument("cancel_after", default_value="0.0"),
        Node(
            package="ros2_use_example",
            executable="action_server",
            output="screen",
        ),
        Node(
            package="ros2_use_example",
            executable="action_client",
            output="screen",
            arguments=[
                LaunchConfiguration("order"),
                LaunchConfiguration("cancel_after"),
            ],
        ),
    ])
