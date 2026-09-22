"""Publish the stations in config/stations.yaml as a MarkerArray so RViz2 shows where to click.

    ros2 run smart_farm_navigation station_markers

Topic /stations_markers (map frame, TRANSIENT_LOCAL, republished every 2 s): one arrow (base_link +x
direction of the station pose) and one text label per station.  Used by rviz/nav2_smartfarm.rviz.
"""

import math
import os

import rclpy
import rclpy.executors
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

COLORS = {"FEEDER_DOCK": (0.1, 0.9, 0.2), "FEEDER_APPROACH": (0.2, 0.6, 1.0), "RACK_DOCK": (1.0, 0.7, 0.1)}


class StationMarkers(Node):
    def __init__(self) -> None:
        super().__init__("station_markers")
        self.declare_parameter("stations_file", os.path.join(
            get_package_share_directory("smart_farm_navigation"), "config", "stations.yaml"))
        cfg = yaml.safe_load(open(self.get_parameter("stations_file").value))
        self.stations = cfg["stations"]
        self.pub = self.create_publisher(MarkerArray, "/stations_markers",
                                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(2.0, self._publish)
        self._publish()
        self.get_logger().info(f"{len(self.stations)} stations -> /stations_markers: {list(self.stations)}")

    def _publish(self) -> None:
        arr = MarkerArray()
        for i, (name, st) in enumerate(self.stations.items()):
            r, g, b = COLORS.get(name, (0.8, 0.8, 0.8))
            yaw = math.radians(float(st["yaw_deg"]))
            arrow = Marker()
            arrow.header.frame_id = "map"; arrow.ns = "stations"; arrow.id = i; arrow.type = Marker.ARROW; arrow.action = Marker.ADD
            arrow.pose.position.x = float(st["x"]); arrow.pose.position.y = float(st["y"]); arrow.pose.position.z = 0.05
            arrow.pose.orientation.z = math.sin(yaw / 2); arrow.pose.orientation.w = math.cos(yaw / 2)
            arrow.scale.x = 0.6; arrow.scale.y = 0.08; arrow.scale.z = 0.08
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = r, g, b, 0.9
            text = Marker()
            text.header.frame_id = "map"; text.ns = "station_names"; text.id = i; text.type = Marker.TEXT_VIEW_FACING; text.action = Marker.ADD
            text.pose.position.x = float(st["x"]); text.pose.position.y = float(st["y"]) + 0.25; text.pose.position.z = 0.3
            text.scale.z = 0.22; text.text = f"{name} ({st['x']:.2f}, {st['y']:.2f}, {st['yaw_deg']:.0f}deg)"
            text.color.r, text.color.g, text.color.b, text.color.a = r, g, b, 1.0
            arr.markers += [arrow, text]
        self.pub.publish(arr)


def main() -> None:
    rclpy.init()
    node = StationMarkers()
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
