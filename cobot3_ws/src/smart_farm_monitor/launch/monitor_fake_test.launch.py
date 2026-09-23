"""Task Manager · executor mock · 기록 노드를 함께 실행한다.

Isaac Sim 없이 8단계 사이클 전체를 돌리고, 그 기록이 JSONL 로 남는지 확인한다.

    ros2 launch smart_farm_monitor monitor_fake_test.launch.py
    ros2 service call /start_cycle smart_farm_interfaces/srv/StartCycle \
        "{scenario_id: DEMO_HARVEST_01}"

대시보드는 http://localhost:8080 에서 봅니다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """기록 노드를 포함한 통합 시험 launch 구성을 생성한다."""

    parameter_file = os.path.join(
        get_package_share_directory("smart_farm_manager"),
        "config",
        "demo_harvest.yaml",
    )
    output_dir = LaunchConfiguration("output_dir")

    nodes = [
        DeclareLaunchArgument(
            "output_dir",
            default_value="records",
            description="JSONL 기록을 남길 디렉터리",
        ),
        Node(
            package="smart_farm_manager",
            executable="task_manager",
            name="task_manager",
            output="screen",
            parameters=[parameter_file],
        ),
        Node(
            package="smart_farm_monitor",
            executable="dashboard_server",
            name="dashboard_server",
            output="screen",
            # recorder 와 같은 곳을 보게 해서, 지난 사이클을 이력에 띄웁니다.
            parameters=[{"records_dir": output_dir}],
        ),
        Node(
            package="smart_farm_monitor",
            executable="cycle_recorder",
            name="cycle_recorder",
            output="screen",
            parameters=[{"output_dir": output_dir}],
        ),
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
