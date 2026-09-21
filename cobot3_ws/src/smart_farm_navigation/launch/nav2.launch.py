"""ros2 launch smart_farm_navigation nav2.launch.py [scan_mode:=auto|scan2d|cloud] [use_rviz:=true]

Nav2 (map_server + AMCL + planner/controller/behaviors + RViz2) for the carter in
Collected_smartfarm_v008.usd.  Runs on the PC that does NOT run Isaac Sim; the only
things it needs from Isaac over DDS are /clock, /tf, /chassis/odom and one lidar topic.

/scan source (scan_mode):
  scan2d  /front_2d_lidar/scan -> scan_sanitizer -> /scan            (about 30 kB/s, fine over Wi-Fi)
  cloud   /front_3d_lidar/lidar_points -> pointcloud_to_laserscan -> /scan   (several MB/s)
  auto    listen 6 s for /front_2d_lidar/scan; use scan2d when it arrives, cloud otherwise

AMCL initial pose: launch args initial_x/initial_y/initial_yaw_deg > config/stations.yaml initial_pose.
"""

import math
import os
import tempfile
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_MAP = "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/maps/Collected_smartfarm_v005.yaml"
SCAN2D_TOPIC = "/front_2d_lidar/scan"
CLOUD_TOPIC = "/front_3d_lidar/lidar_points"


def _probe_scan2d(timeout_s: float = 6.0) -> bool:
    """True when a LaserScan actually arrives on SCAN2D_TOPIC (publisher existing is not enough)."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = rclpy.create_node("nav2_launch_scan_probe", context=ctx)
    got = []
    node.create_subscription(LaserScan, SCAN2D_TOPIC, lambda m: got.append(1), qos_profile_sensor_data)
    ex = rclpy.executors.SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    end = time.monotonic() + timeout_s
    while not got and time.monotonic() < end:
        ex.spin_once(timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown(context=ctx)
    return bool(got)


def _setup(context):
    share = get_package_share_directory("smart_farm_navigation")
    params_in = LaunchConfiguration("params_file").perform(context)
    map_yaml = LaunchConfiguration("map").perform(context)
    mode = LaunchConfiguration("scan_mode").perform(context)

    ix, iy, iyaw = (LaunchConfiguration(k).perform(context) for k in ("initial_x", "initial_y", "initial_yaw_deg"))
    if ix and iy and iyaw:
        x, y, yaw_deg, source = float(ix), float(iy), float(iyaw), "launch arguments"
    else:
        st = yaml.safe_load(open(LaunchConfiguration("stations_file").perform(context)))["initial_pose"]
        x, y, yaw_deg, source = float(st["x"]), float(st["y"]), float(st["yaw_deg"]), "stations.yaml"

    if mode == "auto":
        mode = "scan2d" if _probe_scan2d() else "cloud"
        picked = f"auto -> {mode}"
    else:
        picked = mode
    if mode not in ("scan2d", "cloud"):
        raise RuntimeError(f"scan_mode must be auto, scan2d or cloud (got {mode})")

    params = yaml.safe_load(open(params_in))
    params["amcl"]["ros__parameters"]["initial_pose"] = {"x": x, "y": y, "z": 0.0, "yaw": math.radians(yaw_deg)}
    params["map_server"]["ros__parameters"]["yaml_filename"] = map_yaml
    tmp = tempfile.NamedTemporaryFile("w", prefix="nav2_params_", suffix=".yaml", delete=False)
    yaml.safe_dump(params, tmp)
    tmp.close()

    if mode == "scan2d":
        scan_node = Node(
            package="smart_farm_navigation", executable="scan_sanitizer", name="scan_sanitizer", output="screen",
            parameters=[{"use_sim_time": True, "input_topic": SCAN2D_TOPIC, "output_topic": "/scan",
                         "self_min_range_m": 0.60, "self_sector_deg": 85.0}],
        )
    else:
        scan_node = Node(
            package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan", output="screen",
            remappings=[("cloud_in", CLOUD_TOPIC), ("scan", "/scan")],
            parameters=[{
                "use_sim_time": True,
                "target_frame": "front_3d_lidar",
                "transform_tolerance": 0.3,
                "min_height": -0.35,          # lidar is 0.53 m above the floor -> ignore the floor itself
                "max_height": 1.5,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.4,             # lift posts and side plates are <= 0.31 m from the XT-32
                "range_max": 20.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }],
        )

    bringup = os.path.join(get_package_share_directory("nav2_bringup"), "launch")
    return [
        LogInfo(msg=f"[nav2.launch] scan_mode {picked}; AMCL initial pose ({x:.3f}, {y:.3f}, {yaw_deg:.1f}deg) "
                    f"from {source}; map {map_yaml}"),
        scan_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "bringup_launch.py")),
            launch_arguments={
                "map": map_yaml,
                "params_file": tmp.name,
                "use_sim_time": "True",
                "autostart": "True",
                "use_composition": LaunchConfiguration("use_composition"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "rviz_launch.py")),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            launch_arguments={"namespace": "", "use_namespace": "False", "use_sim_time": "True",
                              "rviz_config": LaunchConfiguration("rviz_config")}.items(),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("smart_farm_navigation")
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=DEFAULT_MAP),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "nav2_params.yaml")),
        DeclareLaunchArgument("stations_file", default_value=os.path.join(share, "config", "stations.yaml")),
        DeclareLaunchArgument("scan_mode", default_value="auto"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(
            get_package_share_directory("nav2_bringup"), "rviz", "nav2_default_view.rviz")),
        DeclareLaunchArgument("use_composition", default_value="False"),
        DeclareLaunchArgument("initial_x", default_value=""),
        DeclareLaunchArgument("initial_y", default_value=""),
        DeclareLaunchArgument("initial_yaw_deg", default_value=""),
        OpaqueFunction(function=_setup),
    ])
