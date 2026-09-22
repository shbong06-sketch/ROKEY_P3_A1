"""Drive the carter to a named station with Nav2 Simple Commander.

    ros2 run smart_farm_navigation go_to_station --ros-args -p station:=INSPECTION_DOCK

Reads config/stations.yaml (map-frame poses).  nav2.launch.py already gives AMCL its
initial pose, so this node only re-sends it with set_initial_pose:=true.
Exit code 0 = SUCCEEDED, 2 = FAILED or CANCELED, 3 = bad arguments.

Reverse-out zones (stations.yaml `reverse_out_zones`): while the carter stands inside a rack
corridor it must not turn in place (the rear of the rig sweeps 0.66 m and would hit the rack
and its pallets), so the node first runs Nav2's BackUp behavior straight out of the zone
(base_link -x = the carter's rear, caster side) and only then sends the NavigateToPose goal.
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
from rclpy.time import Time
import tf2_ros

def make_pose(nav: BasicNavigator, x: float, y: float, yaw_deg: float) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = "map"
    # stamp 0 = "latest available transform"; a wall-clock stamp would never match Isaac's sim-time TF
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    half = math.radians(yaw_deg) / 2.0
    p.pose.orientation.z = math.sin(half)
    p.pose.orientation.w = math.cos(half)
    return p


def current_pose(nav: BasicNavigator, timeout_s: float = 10.0):
    """(x, y, yaw_deg) of base_link in the map frame, or None."""
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, nav)
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        rclpy.spin_once(nav, timeout_sec=0.1)
        if buf.can_transform("map", "base_link", Time()):
            t = buf.lookup_transform("map", "base_link", Time()).transform
            yaw = 2.0 * math.atan2(t.rotation.z, t.rotation.w)
            return t.translation.x, t.translation.y, math.degrees(yaw)
    return None


def run_backup(nav: BasicNavigator, log, dist: float, speed: float, what: str, tries: int = 3) -> bool:
    """Nav2 BackUp with retries.  Right after bring-up the behavior server may still read sim time 0 and
    abort at once with 'Exceeded time allowance'; the local costmap may also not be filled yet."""
    allowance = int(dist / speed * 3 + 60)
    for attempt in range(1, tries + 1):
        t0 = time.monotonic()
        nav.backup(backup_dist=dist, backup_speed=speed, time_allowance=allowance)
        while not nav.isTaskComplete():
            time.sleep(0.2)
        if nav.getResult() == TaskResult.SUCCEEDED:
            log.info(f"{what} done")
            return True
        log.warning(f"{what} attempt {attempt}/{tries} failed after {time.monotonic() - t0:.1f} s"
                    + ("; retrying in 3 s" if attempt < tries else ""))
        time.sleep(3.0)
    return False


def wait_for_costmap(nav: BasicNavigator, log, timeout_s: float = 20.0) -> None:
    """Block until /local_costmap/costmap has been published once (behaviors check collisions against it)."""
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile
    got = []
    sub = nav.create_subscription(OccupancyGrid, "/local_costmap/costmap", lambda m: got.append(1),
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    end = time.monotonic() + timeout_s
    while not got and time.monotonic() < end:
        rclpy.spin_once(nav, timeout_sec=0.2)
    nav.destroy_subscription(sub)
    log.info("local costmap available" if got else "local costmap not seen within timeout; continuing")


def reverse_out_if_needed(nav: BasicNavigator, cfg: dict, log) -> bool:
    """Back straight out of a rack corridor before any turning.  Returns False when the back-up failed."""
    pose = current_pose(nav)
    if pose is None:
        log.warning("map -> base_link not available; skipping the reverse-out check")
        return True
    x, y, yaw = pose
    for z in cfg.get("reverse_out_zones", []):
        if not (z["x"][0] <= x <= z["x"][1] and z["y"][0] <= y <= z["y"][1]):
            continue
        rear = math.radians(yaw) + math.pi                         # direction of base_link -x (rear) in the map
        want = math.radians(z["exit_heading_deg"])
        off = abs(math.atan2(math.sin(rear - want), math.cos(rear - want)))
        if off > math.radians(z.get("max_heading_offset_deg", 20.0)):
            log.warning(f"in zone {z['name']} but the rear points {math.degrees(off):.0f}deg away from the exit; not backing up")
            return True
        ex, ey = math.cos(want), math.sin(want)
        dist = (z["exit_point"][0] - x) * ex + (z["exit_point"][1] - y) * ey
        if dist <= 0.05:
            return True
        speed = float(z.get("speed_mps", 0.25))
        log.info(f"in zone {z['name']} at ({x:.2f}, {y:.2f}, {yaw:.0f}deg): BackUp {dist:.2f} m at {speed:.2f} m/s before navigating")
        return run_backup(nav, log, dist, speed, "BackUp (reverse-out)")
    return True


def main() -> None:
    rclpy.init()
    args = Node("go_to_station_args")
    args.declare_parameter("station", "FEEDER_DOCK")
    args.declare_parameter("stations_file", os.path.join(
        get_package_share_directory("smart_farm_navigation"), "config", "stations.yaml"))
    args.declare_parameter("set_initial_pose", False)
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
        ip = cfg["initial_pose"]
        nav.setInitialPose(make_pose(nav, ip["x"], ip["y"], ip["yaw_deg"]))
        log.info(f"initial pose set to ({ip['x']:.2f}, {ip['y']:.2f}, {ip['yaw_deg']:.1f}deg)")
        nav.waitUntilNav2Active()
    else:
        # AMCL already has its pose from nav2.launch.py.  BasicNavigator.waitUntilNav2Active(localizer="amcl")
        # would keep publishing an all-zero /initialpose until /amcl_pose arrives and so move AMCL to (0, 0).
        nav._waitForNodeToActivate("amcl")
        nav.waitUntilNav2Active(localizer="robot_localization")
    log.info("Nav2 active")
    wait_for_costmap(nav, log)

    if not reverse_out_if_needed(nav, cfg, log):
        log.info(f"RESULT FAILED for {station} (reverse-out)")
        rclpy.shutdown(); sys.exit(2)

    t0 = time.monotonic()
    result = TaskResult.SUCCEEDED
    reverse_in = target.get("reverse_in")            # {from: <station>, distance_m: d}: drive to `from`, then back up d into the dock
    goals = list(target.get("via", [])) + ([reverse_in["from"]] if reverse_in else [station])
    for name in goals:                                 # `via` = stations to pass first (e.g. line up before a corridor)
        wp = cfg["stations"][name]
        log.info(f"goToPose {name}: ({wp['x']:.2f}, {wp['y']:.2f}, {wp['yaw_deg']:.1f}deg)")
        nav.goToPose(make_pose(nav, wp["x"], wp["y"], wp["yaw_deg"]))
        last = -1.0
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb and time.monotonic() - last >= 1.0:
                last = time.monotonic()
                log.info(f"  remaining {fb.distance_remaining:.2f} m, elapsed {time.monotonic() - t0:.0f}s, recoveries {fb.number_of_recoveries}")
            time.sleep(0.2)
        result = nav.getResult()
        if result != TaskResult.SUCCEEDED:
            break
    if result == TaskResult.SUCCEEDED and reverse_in:
        d, sp = float(reverse_in["distance_m"]), float(reverse_in.get("speed_mps", 0.2))
        log.info(f"reverse_in: BackUp {d:.2f} m at {sp:.2f} m/s into {station} (rear = arm side toward the dock)")
        result = TaskResult.SUCCEEDED if run_backup(nav, log, d, sp, "BackUp (reverse-in)") else TaskResult.FAILED
    code = 0 if result == TaskResult.SUCCEEDED else 2
    log.info(f"RESULT {result.name} for {station} after {time.monotonic() - t0:.0f}s")
    args.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
