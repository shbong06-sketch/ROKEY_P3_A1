"""ros2 launch smart_farm_navigation navigation_node.launch.py [timeout_s:=600.0]

Task Manager 의 /navigation/command (TaskCommand) 를 받아 Nav2 로 주행하는 노드를 띄운다.
Nav2 와 feeder_dock 은 nav2.launch.py 로 먼저 띄워 두어야 한다.
use_sim_time 은 반드시 true 여야 한다. false 면 목표 자세의 시각이 Isaac 의 시뮬레이션
시각과 어긋나 Nav2 가 목표를 즉시 거부한다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory("smart_farm_navigation")


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("timeout_s", default_value="600.0"),
        DeclareLaunchArgument("dock_timeout_s", default_value="240.0"),
        Node(
            package="smart_farm_navigation", executable="navigation_node", name="navigation_node",
            output="screen", emulate_tty=True,
            parameters=[{
                "use_sim_time": True,
                "destinations_file": os.path.join(SHARE, "config", "destinations_nav2.yaml"),
                "stations_file": os.path.join(SHARE, "config", "stations.yaml"),
                "timeout_s": LaunchConfiguration("timeout_s"),
                "dock_timeout_s": LaunchConfiguration("dock_timeout_s"),
            }],
        ),
    ])
