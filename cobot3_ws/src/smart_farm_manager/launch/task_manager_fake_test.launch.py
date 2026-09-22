"""Task Manager와 세 executor mock을 함께 실행한다."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Isaac Sim 없이 전체 사이클을 검증하는 launch 구성을 생성한다."""

    package_share = get_package_share_directory("smart_farm_manager")
    parameter_file = os.path.join(
        package_share,
        "config",
        "demo_harvest.yaml",
    )

    nodes = [
        Node(
            package="smart_farm_manager",
            executable="task_manager",
            name="task_manager",
            output="screen",
            parameters=[parameter_file],
        )
    ]

    for executor in ("sim_task", "navigation", "inspection"):
        nodes.append(
            Node(
                package="smart_farm_manager",
                executable="mock_executor",
                name=f"mock_{executor}_executor",
                output="screen",
                parameters=[{"executor": executor}],
            )
        )

    return LaunchDescription(nodes)
