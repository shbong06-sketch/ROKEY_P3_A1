"""ros2 launch smart_farm_navigation carter2_dock.launch.py auto_start:=true

carter2 corridor entry and slow stop inside the M0609 docking window, using path_runner with
namespaced topics (/carter2/cmd_vel, /carter2/chassis/odom) from config/carter2_dock.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = os.path.join(get_package_share_directory("smart_farm_navigation"), "config", "carter2_dock.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=params),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/carter2/cmd_vel"),
        DeclareLaunchArgument("odom_topic", default_value="/carter2/chassis/odom"),
        Node(
            package="smart_farm_navigation", executable="path_runner", name="path_runner",
            namespace="carter2", output="screen", emulate_tty=True,
            parameters=[LaunchConfiguration("params_file"),
                        {"auto_start": LaunchConfiguration("auto_start"),
                         "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                         "odom_topic": LaunchConfiguration("odom_topic")}],
        ),
    ])
