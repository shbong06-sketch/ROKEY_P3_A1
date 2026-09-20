"""ros2 launch smart_farm_navigation escape.launch.py auto_start:=true

Starts escape_controller with config/escape_controller.yaml.  auto_start
defaults to false (hold zero Twist only); pass auto_start:=true to run the
arc-reverse -> forward sequence on smart_farm_nav2_01.usd.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = os.path.join(
        get_package_share_directory("smart_farm_navigation"), "config", "escape_controller.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("auto_start", default_value="false",
                              description="true: run the escape sequence; false: hold zero Twist"),
        DeclareLaunchArgument("params_file", default_value=params),
        Node(
            package="smart_farm_navigation",
            executable="escape_controller",
            name="escape_controller",
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration("params_file"),
                        {"auto_start": LaunchConfiguration("auto_start")}],
        ),
    ])
