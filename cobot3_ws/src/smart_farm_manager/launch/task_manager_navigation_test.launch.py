"""
task_manager
mock_sim_task
navigation_node
mock_inspection

다음 네 노드를 넣는다.
"""

import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    manager_share = get_package_share_directory(
        "smart_farm_manager"
    )

    parameter_file = os.path.join(
        manager_share,
        "config",
        "demo_harvest.yaml",
    )

    return LaunchDescription([
        Node(
            package="smart_farm_manager",
            executable="task_manager",
            name="task_manager",
            output="screen",
            parameters=[parameter_file],
        ),
        Node(
            package="smart_farm_manager",
            executable="mock_executor",
            name="mock_sim_task_executor",
            output="screen",
            parameters=[{"executor": "sim_task"}],
        ),
        Node(
            package="smart_farm_navigation",
            executable="navigation_node",
            name="navigation_node",
            output="screen",
        ),
        Node(
            package="smart_farm_manager",
            executable="mock_executor",
            name="mock_inspection_executor",
            output="screen",
            parameters=[{"executor": "inspection"}],
        ),
    ])