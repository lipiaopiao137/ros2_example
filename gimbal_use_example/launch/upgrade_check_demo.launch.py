"""Start the upgrade-check server and one client.

The client asks once and exits; relaunch it (or use `ros2 service call`) for
another check. Server health state can be changed while it runs, e.g.

    ros2 param set /upgrade_check_server battery_percent 20.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("current_version", default_value="1.0.0"),
        DeclareLaunchArgument("target_version", default_value="1.1.0"),
        Node(
            package="gimbal_use_example",
            executable="upgrade_check_server",
            output="screen",
        ),
        Node(
            package="gimbal_use_example",
            executable="upgrade_check_client",
            output="screen",
            arguments=[
                LaunchConfiguration("current_version"),
                LaunchConfiguration("target_version"),
            ],
        ),
    ])
