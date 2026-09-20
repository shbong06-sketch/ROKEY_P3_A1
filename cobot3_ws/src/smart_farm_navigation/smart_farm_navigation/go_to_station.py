"""Drive the carter to a named station with Nav2 Simple Commander.

    ros2 run smart_farm_navigation go_to_station --ros-args -p station:=INSPECT_ZONE

Reads config/stations.yaml (map-frame poses).  The AMCL initial pose comes
from results/robot_spawn.yaml (written by scripts/launch_scene.py) when it
exists, otherwise from stations.yaml.  Exit code 0 = SUCCEEDED, 2 = FAILED
or CANCELED, 3 = bad arguments.
"""

import math
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node

SPAWN_FILE = "/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/robot_spawn.yaml"


def make_pose(nav: BasicNavigator, x: float, y: float, yaw_deg: float) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    half = math.radians(yaw_deg) / 2.0
    p.pose.orientation.z = math.sin(half)
    p.pose.orientation.w = math.cos(half)
    return p


def load_initial_pose(stations: dict, log) -> dict:
    if os.path.exists(SPAWN_FILE):
        try:
            sp = yaml.safe_load(open(SPAWN_FILE)) or {}
            log.info(f"initial pose from {SPAWN_FILE}: {sp}")
            return {"x": float(sp["x"]), "y": float(sp["y"]), "yaw_deg": float(sp["yaw_deg"])}
        except Exception as e:  # noqa: BLE001
            log.warning(f"spawn file unreadable ({e}); using stations.yaml initial_pose")
    return stations["initial_pose"]


def main() -> None:
    rclpy.init()
    args = Node("go_to_station_args")
    args.declare_parameter("station", "INSPECT_ZONE")
    args.declare_parameter("stations_file", os.path.join(
        get_package_share_directory("smart_farm_navigation"), "config", "stations.yaml"))
    args.declare_parameter("set_initial_pose", True)
    args.declare_parameter("skip_initial_pose_if_localized", True)
    station = args.get_parameter("station").value
    stations_file = args.get_parameter("stations_file").value
    set_init = bool(args.get_parameter("set_initial_pose").value)
    log = args.get_logger()

    cfg = yaml.safe_load(open(stations_file))
    if station not in cfg["stations"]:
        log.error(f"unknown station '{station}'. known: {list(cfg['stations'])}")
        rclpy.shutdown(); sys.exit(3)
    target = cfg["stations"][station]

    nav = BasicNavigator()
    if set_init:
        ip = load_initial_pose(cfg, log)
        nav.setInitialPose(make_pose(nav, ip["x"], ip["y"], ip["yaw_deg"]))
        log.info(f"initial pose set to ({ip['x']:.2f}, {ip['y']:.2f}, {ip['yaw_deg']:.1f}deg)")
    nav.waitUntilNav2Active()
    log.info("Nav2 active")

    goal = make_pose(nav, target["x"], target["y"], target["yaw_deg"])
    log.info(f"goToPose {station}: ({target['x']:.2f}, {target['y']:.2f}, {target['yaw_deg']:.1f}deg)")
    nav.goToPose(goal)
    t0 = time.monotonic(); last = -1.0
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb and time.monotonic() - last >= 1.0:
            last = time.monotonic()
            log.info(f"  remaining {fb.distance_remaining:.2f} m, elapsed {time.monotonic() - t0:.0f}s, recoveries {fb.number_of_recoveries}")
        time.sleep(0.2)
    result = nav.getResult()
    code = 0 if result == TaskResult.SUCCEEDED else 2
    log.info(f"RESULT {result.name} for {station} after {time.monotonic() - t0:.0f}s")
    args.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
