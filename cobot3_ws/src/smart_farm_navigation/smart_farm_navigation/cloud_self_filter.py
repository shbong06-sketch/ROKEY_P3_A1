"""Drop 3D-lidar points that fall inside the rig's own volume (lift, M0609, carried pallet).

    /front_3d_lidar/lidar_points -> cloud_self_filter -> /front_3d_lidar/filtered

The XT-32 sits at base_link (-0.232, 0, 0.526) with no rotation, so a point (x, y, z) in the
lidar frame is (x - 0.232, y, z + 0.526) in base_link.  Everything inside `box_*` (base_link
metres) is removed; the box covers the carter body, the lift, the arm in carry pose and a pallet
held above the rig, with margin for the arm drooping sideways (box x -0.85..0.6, y +-0.6).
Real obstacles that close to the rig are still in the static map.

    ros2 run smart_farm_navigation cloud_self_filter
"""

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

LIDAR_IN_BASE = (-0.232, 0.0, 0.526)


class CloudSelfFilter(Node):
    def __init__(self) -> None:
        super().__init__("cloud_self_filter")
        self.declare_parameter("input_topic", "/front_3d_lidar/lidar_points")
        self.declare_parameter("output_topic", "/front_3d_lidar/filtered")
        self.declare_parameter("box_x", [-0.85, 0.60])   # rear edge must stay clear of the Feeder face at the 1.0 m dock standoff
        self.declare_parameter("box_y", [-0.60, 0.60])
        self.declare_parameter("box_z", [-0.20, 2.60])
        # Isaac's ROS2RtxLidarHelper without fullScan publishes one ~60 deg slice per rendered frame (about 6,900
        # points at 20 Hz for the XT-32).  A LaserScan made from one slice covers one sector only, which breaks
        # AMCL and the Feeder face detection.  Slices arriving within `accumulate_s` are merged into one cloud.
        self.declare_parameter("accumulate_s", 0.35)
        self.declare_parameter("partial_max_points", 20000)     # a cloud with fewer points than this is treated as a slice
        self.bx = [float(v) for v in self.get_parameter("box_x").value]
        self.by = [float(v) for v in self.get_parameter("box_y").value]
        self.bz = [float(v) for v in self.get_parameter("box_z").value]
        self.pub = self.create_publisher(PointCloud2, self.get_parameter("output_topic").value, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.get_parameter("input_topic").value, self._on_cloud, qos_profile_sensor_data)
        self.acc_s = float(self.get_parameter("accumulate_s").value)
        self.partial_max = int(self.get_parameter("partial_max_points").value)
        self.slices = []            # (t, xyz) of recent partial clouds
        self.merged_mode = False
        self.n_msgs = self.n_in = self.n_removed = 0
        self.create_timer(5.0, self._report)
        self.get_logger().info(f"box x{self.bx} y{self.by} z{self.bz} (base_link) removed from "
                               f"{self.get_parameter('input_topic').value}")

    def _on_cloud(self, msg: PointCloud2) -> None:
        pts = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if pts.size == 0:
            self.pub.publish(msg); return
        x = pts[:, 0] + LIDAR_IN_BASE[0]; y = pts[:, 1] + LIDAR_IN_BASE[1]; z = pts[:, 2] + LIDAR_IN_BASE[2]
        inside = (x > self.bx[0]) & (x < self.bx[1]) & (y > self.by[0]) & (y < self.by[1]) & (z > self.bz[0]) & (z < self.bz[1])
        keep = pts[~inside]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if len(pts) < self.partial_max:                        # slice -> merge with the slices of the last accumulate_s
            if not self.merged_mode:
                self.merged_mode = True
                self.get_logger().warning(f"partial lidar slices detected ({len(pts)} points/msg): merging {self.acc_s:.2f} s of slices per output cloud")
            self.slices.append((t, keep))
            self.slices = [(ts, p) for ts, p in self.slices if t - ts <= self.acc_s]
            keep = np.concatenate([p for _, p in self.slices], axis=0) if self.slices else keep
        out = pc2.create_cloud_xyz32(msg.header, keep.astype(np.float32))
        self.pub.publish(out)
        self.n_msgs += 1; self.n_in += len(pts); self.n_removed += int(inside.sum())

    def _report(self) -> None:
        if self.n_msgs == 0:
            self.get_logger().warning("no PointCloud2 received in the last 5 s")
        else:
            self.get_logger().info(f"{self.n_msgs / 5.0:.1f} Hz, {self.n_in / self.n_msgs:.0f} points/scan"
                                   + (" (partial slices, merged)" if self.merged_mode else "")
                                   + f", {self.n_removed / self.n_msgs:.0f} self points removed/scan")
        self.n_msgs = self.n_in = self.n_removed = 0


def main() -> None:
    rclpy.init()
    node = CloudSelfFilter()
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
