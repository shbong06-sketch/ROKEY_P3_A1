"""스마트팜 Task Manager 노드를 실행한다."""

import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """설정 파일을 적용한 Task Manager launch 구성을 생성한다."""

    package_share = get_package_share_directory(
        "smart_farm_manager"
    )

    parameter_file = os.path.join(
        package_share,
        "config",
        "demo_harvest.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="smart_farm_manager",
                executable="task_manager",
                name="task_manager",
                output="screen",
                parameters=[parameter_file],
            )
        ]
    )