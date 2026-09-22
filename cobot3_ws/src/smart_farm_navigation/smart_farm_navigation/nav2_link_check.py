"""Check, from the Nav2 PC, that everything Nav2 needs from Isaac Sim arrives over the network.

    ros2 run smart_farm_navigation nav2_link_check

Listens for 8 s and prints the rate of /clock, /tf (odom -> base_link), /chassis/odom and both
lidar topics, the lidar bandwidth, how many 3D points per scan fall inside the rig's own box
(lift / arm / carried pallet = "self returns", see cloud_self_filter.py) and which scan_mode
nav2.launch.py should use.
Exit code 0 = Nav2 can run, 2 = something mandatory is missing.
"""

import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_msgs.msg import TFMessage

WINDOW_S = 8.0


class LinkCheck(Node):
    def __init__(self) -> None:
        super().__init__("nav2_link_check")
        self.n = {"clock": 0, "tf": 0, "odom": 0, "scan2d": 0, "cloud": 0}
        self.bytes = {"scan2d": 0, "cloud": 0}
        self.frames = set()
        self.scan_frame = self.cloud_frame = ""
        self.cloud_pts = self.self_pts = 0
        self.self_zones = {"rear(x<-0.6)": 0, "mid(-0.6..-0.2)": 0, "front(x>-0.2)": 0}
        self.self_zmax = 0.0
        self.clock_first = self.clock_last = None
        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 100)
        self.create_subscription(Odometry, "/chassis/odom", lambda m: self._inc("odom"), qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/front_2d_lidar/scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/front_3d_lidar/lidar_points", self._cloud, qos_profile_sensor_data)

    def _inc(self, k): self.n[k] += 1

    def _on_clock(self, m):
        t = m.clock.sec + m.clock.nanosec * 1e-9
        if self.clock_first is None:
            self.clock_first = (t, time.monotonic())
        self.clock_last = (t, time.monotonic())
        self._inc("clock")

    def _on_tf(self, m):
        for t in m.transforms:
            if t.header.frame_id == "odom" and t.child_frame_id == "base_link":
                self._inc("tf")
            self.frames.add(t.child_frame_id)

    def _scan(self, m):
        self._inc("scan2d"); self.bytes["scan2d"] += 4 * (len(m.ranges) + len(m.intensities)) + 60; self.scan_frame = m.header.frame_id

    def _cloud(self, m):
        self._inc("cloud"); self.bytes["cloud"] += len(m.data) + 100; self.cloud_frame = m.header.frame_id
        try:
            pts = pc2.read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=True)
        except Exception:  # noqa: BLE001
            return
        if pts.size == 0:
            return
        x = pts[:, 0] - 0.232; y = pts[:, 1]; z = pts[:, 2] + 0.526      # lidar -> base_link
        inside = (x > -0.85) & (x < 0.6) & (np.abs(y) < 0.6) & (z > -0.2) & (z < 2.6)
        self.cloud_pts += len(pts); self.self_pts += int(inside.sum())
        if inside.any():
            xi, zi = x[inside], z[inside]
            self.self_zones["rear(x<-0.6)"] += int((xi < -0.6).sum())
            self.self_zones["mid(-0.6..-0.2)"] += int(((xi >= -0.6) & (xi < -0.2)).sum())
            self.self_zones["front(x>-0.2)"] += int((xi >= -0.2).sum())
            self.self_zmax = max(self.self_zmax, float(zi.max()))


def main() -> None:
    rclpy.init()
    node = LinkCheck()
    log = node.get_logger()
    log.info(f"listening for {WINDOW_S:.0f} s ...")
    end = time.monotonic() + WINDOW_S
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    hz = {k: v / WINDOW_S for k, v in node.n.items()}
    ok = True
    log.info(f"[1] /clock            {hz['clock']:6.1f} Hz" + ("" if hz["clock"] > 1 else "   <- MISSING: Isaac 을 scripts/launch_scene.py 로 실행했는지 확인"))
    if node.clock_first and node.clock_last and node.clock_last[1] > node.clock_first[1]:
        rtf = (node.clock_last[0] - node.clock_first[0]) / (node.clock_last[1] - node.clock_first[1])
        log.info(f"    sim time now {node.clock_last[0]:.1f} s, real-time factor {rtf:.2f}")
    log.info(f"[2] /tf odom->base_link {hz['tf']:5.1f} Hz" + ("" if hz["tf"] > 5 else "   <- MISSING"))
    log.info(f"[3] /chassis/odom     {hz['odom']:6.1f} Hz" + ("" if hz["odom"] > 5 else "   <- MISSING"))
    ok = hz["clock"] > 1 and hz["tf"] > 5 and hz["odom"] > 5
    log.info(f"[4] /front_2d_lidar/scan         {hz['scan2d']:5.1f} Hz, {node.bytes['scan2d'] / WINDOW_S / 1e3:8.1f} kB/s, frame '{node.scan_frame}'")
    pps = node.cloud_pts / node.n["cloud"] if node.n["cloud"] else 0
    log.info(f"[5] /front_3d_lidar/lidar_points {hz['cloud']:5.1f} Hz, {node.bytes['cloud'] / WINDOW_S / 1e3:8.1f} kB/s, frame '{node.cloud_frame}', {pps:.0f} points/msg"
             + ("   <- PARTIAL SLICES (fullScan off): cloud_self_filter merges them; AMCL/docking work but set fullScan in the Isaac script" if 0 < pps < 20000 else ""))
    if node.n["cloud"]:
        per = node.self_pts / node.n["cloud"]
        log.info(f"[6] self returns inside the rig box: {per:.0f} of {node.cloud_pts / node.n['cloud']:.0f} points/scan "
                 f"(rear {node.self_zones['rear(x<-0.6)'] / node.n['cloud']:.0f}, mid {node.self_zones['mid(-0.6..-0.2)'] / node.n['cloud']:.0f}, "
                 f"front {node.self_zones['front(x>-0.2)'] / node.n['cloud']:.0f}; highest z {node.self_zmax:.2f} m)"
                 + ("   <- cloud_self_filter removes these" if per > 0 else ""))
    for name, frame in (("scan", node.scan_frame), ("cloud", node.cloud_frame)):
        if frame and frame not in node.frames:
            log.info(f"    note: frame '{frame}' not seen on /tf in this window (it may be on /tf_static; Nav2 will report it if missing)")
    if hz["scan2d"] >= 3:
        mode = "scan2d"
    elif hz["cloud"] >= 3:
        mode = "cloud"
    else:
        mode = ""
        ok = False
    log.info(f"RESULT {'OK' if ok else 'FAIL'}" + (f" - nav2.launch.py scan_mode:={mode}" if mode else " - no lidar topic arrives at >= 3 Hz"))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    main()
