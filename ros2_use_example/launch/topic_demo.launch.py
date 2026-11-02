"""Start the topic publisher and subscriber together."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros2_use_example",
            executable="topic_subscriber",
            output="screen",
        ),
        Node(
            package="ros2_use_example",
            executable="topic_publisher",
            output="screen",
        ),
    ])
