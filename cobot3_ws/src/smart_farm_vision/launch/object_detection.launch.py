"""Launch the standalone object detection node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create the object detection launch description."""
    config_file = LaunchConfiguration('config_file')
    default_config = PathJoinSubstitution(
        [
            FindPackageShare('smart_farm_vision'),
            'config',
            'object_detection.yaml',
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                default_value=default_config,
                description='Object detection parameter file',
            ),
            Node(
                package='smart_farm_vision',
                executable='object_detection',
                name='object_detection',
                output='screen',
                parameters=[config_file],
            ),
        ]
    )
