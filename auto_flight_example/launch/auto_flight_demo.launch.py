"""Start the complete auto-flight demo.

Brings up the vehicle simulator, the gimbal manager and the auto-flight node,
then sends one square mission from the CLI client.

Try an outdated gimbal firmware to see preflight refuse the mission:

    ros2 launch auto_flight_example auto_flight_demo.launch.py firmware_version:=1.0.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("firmware_version", default_value="1.2.0"),
        DeclareLaunchArgument("required_version", default_value="1.1.0"),
        DeclareLaunchArgument("altitude", default_value="3.0"),
        DeclareLaunchArgument("radius", default_value="5.0"),
        DeclareLaunchArgument("cancel_after", default_value="0.0"),
        Node(
            package="auto_flight_example",
            executable="drone_sim",
            output="screen",
        ),
        Node(
            package="auto_flight_example",
            executable="gimbal_manager",
            output="screen",
            parameters=[{
                "firmware_version": LaunchConfiguration("firmware_version"),
                "required_version": LaunchConfiguration("required_version"),
            }],
        ),
        Node(
            package="auto_flight_example",
            executable="auto_flight_node",
            output="screen",
        ),
        # Give the other three processes a head start so DDS discovery is done
        # before the client's wait_for_action_server window opens.
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="auto_flight_example",
                    executable="auto_flight_client",
                    output="screen",
                    arguments=[
                        LaunchConfiguration("altitude"),
                        LaunchConfiguration("radius"),
                        LaunchConfiguration("cancel_after"),
                    ],
                ),
            ],
        ),
    ])
