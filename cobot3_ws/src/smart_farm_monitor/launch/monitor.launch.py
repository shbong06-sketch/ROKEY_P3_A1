"""관제 기록 노드를 실행한다.

팀의 task_manager.launch.py 를 건드리지 않기 위해 별도 launch 로 둔다.
시연 절차에서 터미널 하나를 더 쓰는 대신, 팀 파일에 손대지 않는다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """설정 파일을 적용한 recorder launch 구성을 만든다."""

    package_share = get_package_share_directory("smart_farm_monitor")
    parameter_file = os.path.join(package_share, "config", "monitor.yaml")

    return LaunchDescription(
        [
            Node(
                package="smart_farm_monitor",
                executable="recorder",
                name="monitor_recorder",
                output="screen",
                parameters=[parameter_file],
            )
        ]
    )
