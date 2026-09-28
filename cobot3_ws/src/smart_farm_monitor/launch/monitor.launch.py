"""관제 기록 노드와 관제 웹을 함께 실행한다.

팀의 task_manager.launch.py 를 건드리지 않기 위해 별도 launch 로 둔다.
시연 절차에서 터미널 하나를 더 쓰는 대신, 팀 파일에 손대지 않는다.

  ros2 launch smart_farm_monitor monitor.launch.py            # 기록 + 웹
  ros2 launch smart_farm_monitor monitor.launch.py web:=false  # 기록만
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """설정 파일을 적용한 recorder 와 web launch 구성을 만든다."""

    package_share = get_package_share_directory("smart_farm_monitor")
    parameter_file = os.path.join(package_share, "config", "monitor.yaml")

    web = LaunchConfiguration("web")
    port = LaunchConfiguration("port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "web", default_value="true",
                description="관제 웹을 함께 띄울지 여부"),
            DeclareLaunchArgument(
                "port", default_value="8080",
                description="관제 웹 포트"),
            Node(
                package="smart_farm_monitor",
                executable="recorder",
                name="monitor_recorder",
                output="screen",
                parameters=[parameter_file],
            ),
            Node(
                package="smart_farm_monitor",
                executable="web",
                name="monitor_web",
                output="screen",
                condition=IfCondition(web),
                arguments=["--port", port],
            ),
        ]
    )
