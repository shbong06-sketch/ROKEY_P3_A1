"""ros2 launch smart_farm_navigation navigation_node.launch.py

Starts the /cmd_vel-based Navigation Node that executes /navigation/command requests.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(package="smart_farm_navigation", executable="navigation_node", name="navigation_node",
             output="screen", emulate_tty=True),
    ])
