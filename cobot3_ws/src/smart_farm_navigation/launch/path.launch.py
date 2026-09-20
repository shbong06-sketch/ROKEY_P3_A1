"""ros2 launch smart_farm_navigation path.launch.py auto_start:=true

Starts path_runner with config/path_runner.yaml (waypoints in the start frame).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = os.path.join(get_package_share_directory("smart_farm_navigation"), "config", "path_runner.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=params),
        Node(
            package="smart_farm_navigation",
            executable="path_runner",
            name="path_runner",
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration("params_file"), {"auto_start": LaunchConfiguration("auto_start")}],
        ),
    ])
