"""Check, from the Nav2 PC, that everything Nav2 needs from Isaac Sim arrives over the network.

    ros2 run smart_farm_navigation nav2_link_check

Listens for 8 s and prints the rate of /clock, /tf (odom -> base_link), /chassis/odom and both
lidar topics, the lidar bandwidth, and which scan_mode nav2.launch.py should use.
Exit code 0 = Nav2 can run, 2 = something mandatory is missing.
"""

import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage

WINDOW_S = 8.0


class LinkCheck(Node):
    def __init__(self) -> None:
        super().__init__("nav2_link_check")
        self.n = {"clock": 0, "tf": 0, "odom": 0, "scan2d": 0, "cloud": 0}
        self.bytes = {"scan2d": 0, "cloud": 0}
        self.frames = set()
        self.scan_frame = self.cloud_frame = ""
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
    log.info(f"[5] /front_3d_lidar/lidar_points {hz['cloud']:5.1f} Hz, {node.bytes['cloud'] / WINDOW_S / 1e3:8.1f} kB/s, frame '{node.cloud_frame}'")
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
