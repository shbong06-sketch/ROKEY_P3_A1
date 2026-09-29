#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTfBroadcaster(Node):

  def __init__(self):
    super().__init__('odom_to_tf_broadcaster')
    self.declare_parameter('use_sim_time', True)
    self.tf_broadcaster = TransformBroadcaster(self)
    self.sub = self.create_subscription(
        Odometry, '/odom', self.odom_callback, 10
    )
    self.get_logger().info('Odom to TF broadcaster started (/odom -> /tf)')

  def odom_callback(self, msg: Odometry):
    t = TransformStamped()
    # Isaac Sim의 시뮬레이션 타임스탬프가 0이면 현재 노드 시계 사용
    if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
      t.header.stamp = self.get_clock().now().to_msg()
    else:
      t.header.stamp = msg.header.stamp

    t.header.frame_id = 'odom'
    t.child_frame_id = (
        msg.child_frame_id if msg.child_frame_id else 'base_footprint'
    )

    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation = msg.pose.pose.orientation

    self.tf_broadcaster.sendTransform(t)


def main(args=None):
  rclpy.init(args=args)
  node = OdomToTfBroadcaster()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
