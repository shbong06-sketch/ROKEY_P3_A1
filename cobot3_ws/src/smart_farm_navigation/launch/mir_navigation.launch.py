import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    try:
        pkg_nav_dir = get_package_share_directory('mir100_navigation')
    except Exception:
        pkg_nav_dir = get_package_share_directory('smart_farm_navigation')

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Flexible default paths resolving to either ~/ROKEY_P3_A1/... or ~/cobot3_ws/...
    home_dir = os.path.expanduser('~')
    possible_map_paths = [
        os.path.join(home_dir, 'ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/maps/260916_AMR_test_yaml.yaml'),
        os.path.join(home_dir, 'cobot3_ws/src/smart_farm_navigation/maps/260916_AMR_test_yaml.yaml'),
        '/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/maps/260916_AMR_test_yaml.yaml',
    ]
    default_map = next((p for p in possible_map_paths if os.path.exists(p)), possible_map_paths[0])

    possible_urdf_paths = [
        os.path.join(home_dir, 'ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/Collected_260916_AMR_test/mir100.urdf'),
        os.path.join(home_dir, 'cobot3_ws/src/smart_farm_navigation/Collected_260916_AMR_test/mir100.urdf'),
        '/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/Collected_260916_AMR_test/mir100.urdf',
        os.path.join(home_dir, 'ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/260916_AMR_test/mir100.urdf'),
        os.path.join(home_dir, 'cobot3_ws/src/smart_farm_navigation/260916_AMR_test/mir100.urdf'),
        '/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/260916_AMR_test/mir100.urdf',
    ]
    default_urdf = next((p for p in possible_urdf_paths if os.path.exists(p)), possible_urdf_paths[0])

    default_params = os.path.join(pkg_nav_dir, 'params', 'mir100_navigation_params.yaml')
    default_rviz_config = os.path.join(pkg_nav_dir, 'rviz2', 'mir100_navigation.rviz')

    # Declare launch arguments
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock if true'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RViz config file'
    )

    declare_urdf_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=default_urdf,
        description='Full path to the MiR100 URDF file'
    )

    # Robot State Publisher
    robot_description = ParameterValue(
        Command(['cat ', LaunchConfiguration('urdf_file')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # RViz Launch
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'rviz_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'false',
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items()
    )

    # Nav2 Bringup Launch
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': 'False',
        }.items()
    )

    # Odom to TF Broadcaster (Isaac Sim /odom -> /tf odom->base_footprint 브로드캐스터)
    odom_to_tf_script = os.path.join(pkg_nav_dir, 'scripts', 'odom_to_tf.py')
    odom_to_tf_process = ExecuteProcess(
        cmd=['python3', odom_to_tf_script, '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen'
    )

    # Static Transform Publisher (map -> odom 연결 보장)
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_urdf_cmd)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(odom_to_tf_process)
    ld.add_action(static_tf_map_to_odom)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)

    return ld
