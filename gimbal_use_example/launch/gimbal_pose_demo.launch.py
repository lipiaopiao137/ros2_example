"""Start the gimbal pose publisher and subscriber (two processes)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rate_hz", default_value="20.0"),
        DeclareLaunchArgument("yaw_amplitude_deg", default_value="30.0"),
        Node(
            package="gimbal_use_example",
            executable="gimbal_pose_subscriber",
            output="screen",
        ),
        Node(
            package="gimbal_use_example",
            executable="gimbal_pose_publisher",
            output="screen",
            parameters=[{
                "rate_hz": LaunchConfiguration("rate_hz"),
                "yaw_amplitude_deg": LaunchConfiguration("yaw_amplitude_deg"),
            }],
        ),
    ])
