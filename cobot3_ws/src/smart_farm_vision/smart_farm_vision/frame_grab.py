r"""Save one frame from the inspection camera topic to a file.

Used to check detection against the live scene without running the whole
executor, and to re-measure slot ROIs from a real frame.

    ros_set
    python3 .../smart_farm_vision/frame_grab.py \
      --ros-args -p camera_topic:=/rgb -p out_path:=/tmp/frame.png

Isaac Sim only ticks its ROS camera graph while the timeline is playing.
With the timeline stopped nothing is published and this exits 1.
"""

from __future__ import annotations

import sys
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class FrameGrab(Node):
    """Subscribe to the camera topic and keep the first valid frame."""

    def __init__(self) -> None:
        """Read parameters and subscribe to the camera topic."""
        super().__init__('frame_grab')
        self.declare_parameter('camera_topic', '/rgb')
        self.declare_parameter('timeout_sec', 15.0)
        self.declare_parameter('out_path', '/tmp/frame.png')

        self._camera_topic = str(
            self.get_parameter('camera_topic').value
        )
        self._timeout_sec = float(
            self.get_parameter('timeout_sec').value
        )
        self._out_path = str(self.get_parameter('out_path').value)

        self._bridge = CvBridge()
        self._frame = None
        self._encoding = ''

        # 노드와 같은 QoS 로 받아야 매칭된다.
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Image,
            self._camera_topic,
            self._on_image,
            qos,
        )

    def _on_image(self, message: Image) -> None:
        if self._frame is not None:
            return
        try:
            self._frame = self._bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )
            self._encoding = message.encoding
        except Exception as exc:
            self.get_logger().error(
                f'변환 실패: {type(exc).__name__}: {exc}'
            )

    def run(self) -> int:
        """Wait for one frame, write it out, and return an exit code."""
        log = self.get_logger()
        log.info(
            f'{self._camera_topic} 대기 중 '
            f'(최대 {self._timeout_sec:.0f}초)'
        )
        end = time.monotonic() + self._timeout_sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._frame is not None:
                break

        if self._frame is None:
            publishers = self.count_publishers(self._camera_topic)
            log.error(
                f'프레임 없음. {self._camera_topic} 퍼블리셔 '
                f'{publishers}개.'
            )
            if publishers == 0:
                log.error(
                    'Isaac Sim 타임라인이 멈춰 있으면 ROS 카메라 '
                    '그래프가 틱하지 않습니다. Play 를 누르세요.'
                )
            else:
                log.error('퍼블리셔는 있는데 프레임이 안 옵니다. QoS 확인.')
            return 1

        height, width = self._frame.shape[:2]
        cv2.imwrite(self._out_path, self._frame)
        log.info(
            f'저장: {self._out_path} '
            f'({width}x{height}, encoding={self._encoding})'
        )
        return 0


def main(args: list[str] | None = None) -> None:
    """Grab one frame and exit with the result code."""
    rclpy.init(args=args)
    node = FrameGrab()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
