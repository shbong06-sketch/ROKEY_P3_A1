"""ros2 launch smart_farm_navigation navigation_node.launch.py [mode:=nav2|cmd_vel] [timeout_s:=600.0]

Starts the Navigation Node that executes /navigation/command (smart_farm_interfaces/TaskCommand).
  mode:=nav2     destinations_nav2.yaml -> go_to_station (Nav2 must already be running)   [default]
  mode:=cmd_vel  destinations.yaml      -> path_runner launches (1차 시연 방식)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    name = "destinations_nav2.yaml" if mode == "nav2" else "destinations.yaml"
    path = os.path.join(get_package_share_directory("smart_farm_navigation"), "config", name)
    timeout = float(LaunchConfiguration("timeout_s").perform(context))
    return [Node(package="smart_farm_navigation", executable="navigation_node", name="navigation_node",
                 output="screen", emulate_tty=True,
                 parameters=[{"destinations_file": path, "timeout_s": timeout}])]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="nav2"),
        DeclareLaunchArgument("timeout_s", default_value="600.0"),   # 벽시계 기준. Isaac 실시간 배율 0.4 에서 왕복 3~4 분
        OpaqueFunction(function=_setup),
    ])
