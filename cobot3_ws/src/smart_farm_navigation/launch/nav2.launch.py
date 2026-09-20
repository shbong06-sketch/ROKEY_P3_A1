"""ros2 launch smart_farm_navigation nav2.launch.py [use_rviz:=true] [initial_x:=.. initial_y:=.. initial_yaw_deg:=..]

Brings up Nav2 (map_server + AMCL + planner/controller/behaviors + RViz2) for one Nova
Carter in smartfarm_v1.usd, plus pointcloud_to_laserscan (/front_3d_lidar/lidar_points -> /scan).
AMCL initial pose precedence: launch args > results/robot_spawn.yaml (written by
scripts/launch_scene.py) > config/stations.yaml initial_pose.
"""

import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SPAWN_FILE = "/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/robot_spawn.yaml"


def _resolve_initial_pose(context, share):
    ix, iy, iyaw = (LaunchConfiguration(k).perform(context) for k in ("initial_x", "initial_y", "initial_yaw_deg"))
    if ix and iy and iyaw:
        return float(ix), float(iy), float(iyaw), "launch arguments"
    if os.path.exists(SPAWN_FILE):
        sp = yaml.safe_load(open(SPAWN_FILE)) or {}
        if all(k in sp for k in ("x", "y", "yaw_deg")):
            return float(sp["x"]), float(sp["y"]), float(sp["yaw_deg"]), SPAWN_FILE
    st = yaml.safe_load(open(os.path.join(share, "config", "stations.yaml")))["initial_pose"]
    return float(st["x"]), float(st["y"]), float(st["yaw_deg"]), "stations.yaml"


def _setup(context):
    share = get_package_share_directory("smart_farm_navigation")
    params_in = LaunchConfiguration("params_file").perform(context)
    map_yaml = LaunchConfiguration("map").perform(context)
    x, y, yaw_deg, source = _resolve_initial_pose(context, share)

    params = yaml.safe_load(open(params_in))
    params["amcl"]["ros__parameters"]["initial_pose"] = {"x": x, "y": y, "z": 0.0, "yaw": math.radians(yaw_deg)}
    params["map_server"]["ros__parameters"]["yaml_filename"] = map_yaml
    tmp = tempfile.NamedTemporaryFile("w", prefix="nav2_params_", suffix=".yaml", delete=False)
    yaml.safe_dump(params, tmp); tmp.close()

    bringup = os.path.join(get_package_share_directory("nav2_bringup"), "launch")
    return [
        LogInfo(msg=f"[nav2.launch] AMCL initial pose ({x:.2f}, {y:.2f}, {yaw_deg:.1f}deg) from {source}; map {map_yaml}"),
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
            launch_arguments={"namespace": "", "use_namespace": "False",
                              "rviz_config": LaunchConfiguration("rviz_config")}.items(),
        ),
        Node(
            package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan", output="screen",
            remappings=[("cloud_in", "/front_3d_lidar/lidar_points"), ("scan", "/scan")],
            parameters=[{
                "use_sim_time": True,
                "target_frame": "front_3d_lidar",
                "transform_tolerance": 0.3,
                "min_height": -0.1,
                "max_height": 1.5,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.3,
                "range_max": 20.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("smart_farm_navigation")
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=os.path.join(share, "maps", "smartfarm_v1.yaml")),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "nav2_params.yaml")),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(
            get_package_share_directory("nav2_bringup"), "rviz", "nav2_default_view.rviz")),
        DeclareLaunchArgument("use_composition", default_value="False"),
        DeclareLaunchArgument("initial_x", default_value=""),
        DeclareLaunchArgument("initial_y", default_value=""),
        DeclareLaunchArgument("initial_yaw_deg", default_value=""),
        OpaqueFunction(function=_setup),
    ])
