"""ros2 launch smart_farm_navigation nav2.launch.py [scan_mode:=auto|scan2d|cloud] [use_rviz:=true] [record:=true] [record_cloud:=false]

record:=true (default) starts `ros2 bag record` alongside Nav2 into
results/bags/nav2_<YYYYmmdd_HHMM>/ with every topic needed to replay the run
(clock, tf, odom, /scan, cmd_vel chain, AMCL pose, costmaps, plan, BT log, /navigation/*).
record_cloud:=true adds the raw 3D point cloud (about 1.3 MB/s).

Nav2 (map_server + AMCL + planner/controller/behaviors + RViz2) for the carter in
Collected_smartfarm_v011.usd.  Runs on the PC that does NOT run Isaac Sim; the only
things it needs from Isaac over DDS are /clock, /tf, /chassis/odom and one lidar topic.

/scan source (scan_mode):
  scan2d  /front_2d_lidar/scan -> scan_sanitizer -> /scan            (about 30 kB/s, fine over Wi-Fi)
  cloud   /front_3d_lidar/lidar_points -> cloud_self_filter -> pointcloud_to_laserscan -> /scan
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_SHARE = get_package_share_directory("smart_farm_navigation")

BAG_DIR = os.path.join(
    os.path.expanduser("~"),
    ".ros",
    "smart_farm_navigation",
    "bags",
)

DEFAULT_MAP = os.path.join(
    PACKAGE_SHARE,
    "maps",
    "Collected_smartfarm_v011.yaml",
)

BAG_TOPICS = [
    "/clock", "/tf", "/tf_static", "/chassis/odom", "/scan",
    "/cmd_vel", "/cmd_vel_nav", "/cmd_vel_smoothed", "/collision_monitor_state",
    "/amcl_pose", "/particle_cloud", "/initialpose", "/map",
    "/plan", "/local_costmap/costmap", "/global_costmap/costmap", "/local_costmap/published_footprint",
    "/behavior_tree_log", "/diagnostics",
    "/navigation/command", "/navigation/result", "/navigation/status",
    "/goal_pose", "/stations_markers", "/feeder_dock/status", "/feeder_dock/result",
]
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
        self_filter = Node(
            package="smart_farm_navigation", executable="cloud_self_filter", name="cloud_self_filter", output="screen",
            parameters=[{"use_sim_time": True, "input_topic": CLOUD_TOPIC, "output_topic": CLOUD_TOPIC + "/filtered", "accumulate_s": 0.5, "self_box_x_m": 0.6, "self_box_y_m": 0.6, "self_box_z_m": 1.0}],
        )
        scan_node = Node(
            package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan", output="screen",
            remappings=[("cloud_in", CLOUD_TOPIC + "/filtered"), ("scan", "/scan")],
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
                "range_min": 0.3,             # rig self returns are removed by cloud_self_filter (base_link box)
                "range_max": 20.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }],
        )

    actions = []
    if LaunchConfiguration("record").perform(context).lower() in ("true", "1", "yes"):
        topics = BAG_TOPICS + ([CLOUD_TOPIC] if LaunchConfiguration("record_cloud").perform(context).lower() in ("true", "1", "yes") else [])
        bag = os.path.join(BAG_DIR, time.strftime("nav2_%Y%m%d_%H%M"))
        os.makedirs(BAG_DIR, exist_ok=True)
        actions.append(LogInfo(msg=f"[nav2.launch] rosbag -> {bag}  ({len(topics)} topics{', with 3D cloud' if CLOUD_TOPIC in topics else ''})"))
        actions.append(ExecuteProcess(
            cmd=["ros2", "bag", "record", "--use-sim-time", "-o", bag] + topics,
            output="log", name="rosbag_record"))

    bringup = os.path.join(get_package_share_directory("nav2_bringup"), "launch")
    return actions + [
        LogInfo(msg=f"[nav2.launch] scan_mode {picked}; AMCL initial pose ({x:.3f}, {y:.3f}, {yaw_deg:.1f}deg) "
                    f"from {source}; map {map_yaml}"),
        *( [self_filter, scan_node] if mode == "cloud" else [scan_node] ),
        Node(package="smart_farm_navigation", executable="feeder_dock", name="feeder_dock", output="screen",
             parameters=[{"use_sim_time": True, "auto_start": LaunchConfiguration("dock_auto").perform(context).lower() == "true"}]),
        Node(package="smart_farm_navigation", executable="station_markers", name="station_markers", output="screen",
             parameters=[{"use_sim_time": True, "stations_file": LaunchConfiguration("stations_file").perform(context)}]),
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
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(share, "rviz", "nav2_smartfarm.rviz")),
        DeclareLaunchArgument("use_composition", default_value="False"),
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument("dock_auto", default_value="true"),     # feeder_dock arms itself near FEEDER_APPROACH
        DeclareLaunchArgument("record_cloud", default_value="false"),
        DeclareLaunchArgument("initial_x", default_value=""),
        DeclareLaunchArgument("initial_y", default_value=""),
        DeclareLaunchArgument("initial_yaw_deg", default_value=""),
        OpaqueFunction(function=_setup),
    ])
