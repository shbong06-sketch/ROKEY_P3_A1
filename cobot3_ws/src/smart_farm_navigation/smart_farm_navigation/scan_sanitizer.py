"""Republish the carter's 2D lidar scan as /scan with the rig's self-reflections removed.

The lift frame and the M0609 sit on top of the carter, so the lidars see four lift posts
at 0.2-0.55 m in the rear half.  Nav2's collision_monitor would treat those returns as an
obstacle inside the footprint and hold the robot still, and AMCL would score them against
the map.  Returns closer than `self_min_range_m` inside the rear sector are set to +inf
(= no return); everything else passes through untouched.

    ros2 run smart_farm_navigation scan_sanitizer
"""

import math

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanSanitizer(Node):
    def __init__(self) -> None:
        super().__init__("scan_sanitizer")
        self.declare_parameter("input_topic", "/front_2d_lidar/scan")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("self_min_range_m", 0.60)     # lift posts are at <= 0.53 m from the front RPLidar
        self.declare_parameter("self_sector_deg", 85.0)      # |angle| above this = rear half where the lift is
        self.declare_parameter("global_min_range_m", 0.05)
        self.min_r = float(self.get_parameter("self_min_range_m").value)
        self.sector = math.radians(float(self.get_parameter("self_sector_deg").value))
        self.gmin = float(self.get_parameter("global_min_range_m").value)
        self.pub = self.create_publisher(LaserScan, self.get_parameter("output_topic").value, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.get_parameter("input_topic").value, self._on_scan, qos_profile_sensor_data)
        self.count = 0
        self.removed = 0
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"{self.get_parameter('input_topic').value} -> {self.get_parameter('output_topic').value}; "
            f"drop < {self.min_r:.2f} m where |angle| > {math.degrees(self.sector):.0f} deg")

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = list(msg.ranges)
        a = msg.angle_min
        for i, r in enumerate(ranges):
            if r < self.gmin or (r < self.min_r and abs(math.atan2(math.sin(a), math.cos(a))) > self.sector):
                ranges[i] = float("inf")
                self.removed += 1
            a += msg.angle_increment
        msg.ranges = ranges
        self.pub.publish(msg)
        self.count += 1

    def _report(self) -> None:
        if self.count == 0:
            self.get_logger().warning("no LaserScan received in the last 5 s")
        else:
            self.get_logger().info(f"{self.count / 5.0:.1f} Hz, {self.removed / self.count:.0f} self-returns removed per scan")
        self.count = self.removed = 0


def main() -> None:
    rclpy.init()
    node = ScanSanitizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
