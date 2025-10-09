from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="makeblock", executable="megapi", name="megapi_node"),
            Node(package="makeblock", executable="mbot", name="mbot_node"),
        ]
    )
