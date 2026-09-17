import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    try:
        pkg_nav_dir = get_package_share_directory('mir100_navigation')
    except Exception:
        pkg_nav_dir = get_package_share_directory('smart_farm_navigation')

    home_dir = os.path.expanduser('~')

    # USD Path detection
    usd_candidates = [
        os.path.join(home_dir, 'ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/Collected_260916_AMR_test/260916_AMR_test.usd'),
        os.path.join(home_dir, 'cobot3_ws/src/smart_farm_navigation/Collected_260916_AMR_test/260916_AMR_test.usd'),
        '/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/Collected_260916_AMR_test/260916_AMR_test.usd',
    ]
    default_usd = next((p for p in usd_candidates if os.path.exists(p)), usd_candidates[0])

    declare_usd_cmd = DeclareLaunchArgument(
        'usd_path',
        default_value=default_usd,
        description='Full path to the Isaac Sim USD file'
    )

    isaac_sh = os.path.join(home_dir, 'isaacsim/isaac-sim.sh')

    # 1. Execute Isaac Sim Process with the given USD and autoplay
    isaac_process = ExecuteProcess(
        cmd=[
            isaac_sh,
            LaunchConfiguration('usd_path'),
            '--play-sim-on-start'
        ],
        output='screen'
    )

    # 2. Include Navigation 2 & RViz2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav_dir, 'launch', 'mir_navigation.launch.py')
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_usd_cmd)
    ld.add_action(isaac_process)
    ld.add_action(nav2_launch)

    return ld
