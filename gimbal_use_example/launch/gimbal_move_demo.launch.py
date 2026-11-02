"""Start the gimbal move action server and one client.

Set cancel_after to a positive value to cancel the motion mid-way.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("delta_yaw_deg", default_value="30.0"),
        DeclareLaunchArgument("speed_dps", default_value="0.0"),
        DeclareLaunchArgument("cancel_after", default_value="0.0"),
        Node(
            package="gimbal_use_example",
            executable="gimbal_move_server",
            output="screen",
        ),
        Node(
            package="gimbal_use_example",
            executable="gimbal_move_client",
            output="screen",
            arguments=[
                LaunchConfiguration("delta_yaw_deg"),
                LaunchConfiguration("speed_dps"),
                LaunchConfiguration("cancel_after"),
            ],
        ),
    ])
