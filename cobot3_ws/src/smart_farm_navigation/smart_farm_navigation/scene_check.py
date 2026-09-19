"""Scene connectivity check for smart_farm_nav2_01.usd (run in terminal 2).

Prints, in order: /cmd_vel subscriber, /chassis/odom sample, TF parents of
base_link, odometry drift over a short window, then exits 0 (all good) or 1.
Replaces the manual `ros2 topic info/echo` sequence with one command:

    ros2 run smart_farm_navigation scene_check --ros-args -p drift_window_s:=15.0
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def yaw_deg(o) -> float:
    return math.degrees(math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z)))


class SceneCheck(Node):
    def __init__(self) -> None:
        super().__init__("scene_check")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/chassis/odom")
        self.declare_parameter("drift_window_s", 15.0)
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.drift_window = float(self.get_parameter("drift_window_s").value)
        self.odom = None
        self.odom_count = 0
        self.tf_parents = set()
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 50)

    def _on_odom(self, m: Odometry) -> None:
        self.odom = m
        self.odom_count += 1

    def _on_tf(self, m: TFMessage) -> None:
        for t in m.transforms:
            self.tf_parents.add((t.header.frame_id, t.child_frame_id))

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self) -> int:
        ok = True
        log = self.get_logger()

        subs = self.count_subscribers(self.cmd_vel_topic)
        pubs = self.count_publishers(self.cmd_vel_topic)
        log.info(f"[1] {self.cmd_vel_topic}: subscribers={subs} publishers={pubs}")
        if subs < 1:
            log.error("    no /cmd_vel subscriber: Isaac Sim scene not playing or bridge inactive")
            ok = False

        self.spin_for(3.0)
        if self.odom is None:
            log.error(f"[2] {self.odom_topic}: no message in 3 s")
            return 1
        p = self.odom.pose.pose
        log.info(
            f"[2] {self.odom_topic}: frame {self.odom.header.frame_id} -> {self.odom.child_frame_id}, "
            f"x={p.position.x:.3f} y={p.position.y:.3f} yaw={yaw_deg(p.orientation):.1f}deg, "
            f"stamp={self.odom.header.stamp.sec}.{self.odom.header.stamp.nanosec // 1000000:03d}"
        )

        base = sorted(c for (par, c) in self.tf_parents if par == "base_link")
        odom_children = sorted(c for (par, c) in self.tf_parents if par == "odom")
        log.info(f"[3] /tf: odom -> {odom_children}; base_link children={len(base)}")
        if "base_link" not in odom_children:
            log.error("    odom -> base_link transform missing")
            ok = False

        x0, y0, t0, n0 = p.position.x, p.position.y, time.monotonic(), self.odom_count
        log.info(f"[4] drift check: holding for {self.drift_window:.0f} s without commands")
        self.spin_for(self.drift_window)
        p1 = self.odom.pose.pose
        dt = time.monotonic() - t0
        d = math.hypot(p1.position.x - x0, p1.position.y - y0)
        rate = (self.odom_count - n0) / dt if dt > 0 else 0.0
        log.info(f"    moved {d:.4f} m in {dt:.1f} s wall; odom rate {rate:.1f} Hz")
        if d > 0.05:
            log.warning("    robot creeps without command; check scene physics before Nav2 work")

        log.info("RESULT: " + ("OK" if ok else "FAIL"))
        return 0 if ok else 1


def main() -> None:
    rclpy.init()
    node = SceneCheck()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
