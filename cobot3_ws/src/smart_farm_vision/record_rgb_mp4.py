"""Record the live ROS /rgb image topic directly to an MP4 file."""

import argparse
import subprocess
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Recorder(Node):
    def __init__(self, output, fps):
        super().__init__('rgb_mp4_recorder')
        self.output = output
        self.fps = fps
        self.bridge = CvBridge()
        self.encoder = None
        self.size = None
        self.frames = 0
        self.create_subscription(Image, '/rgb', self.on_image, qos_profile_sensor_data)
        self.get_logger().info('Waiting for /rgb; press Ctrl+C to finish')

    def on_image(self, message):
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        if self.encoder is None:
            self.size = (width, height)
            self.encoder = subprocess.Popen([
                'ffmpeg', '-hide_banner', '-loglevel', 'error', '-n',
                '-f', 'rawvideo', '-pixel_format', 'bgr24',
                '-video_size', f'{width}x{height}', '-framerate', str(self.fps),
                '-i', 'pipe:0', '-an', '-c:v', 'libx264',
                '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                str(self.output),
            ], stdin=subprocess.PIPE)
            self.get_logger().info(f'Recording {width}x{height} to {self.output}')
        if (width, height) != self.size:
            self.get_logger().error('Camera resolution changed; stopping recording')
            raise RuntimeError('Camera resolution changed')
        self.encoder.stdin.write(frame.tobytes())
        self.frames += 1

    def finish(self):
        if self.encoder is None:
            self.get_logger().warning('No /rgb frames received; no MP4 was created')
            return
        self.encoder.stdin.close()
        if self.encoder.wait() != 0:
            raise RuntimeError('ffmpeg failed to write the MP4')
        self.get_logger().info(f'Saved {self.frames} frames to {self.output}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='output .mp4 path')
    parser.add_argument('--fps', type=float, default=10.0,
                        help='camera frame rate (default: 10)')
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error('--fps must be positive')
    if args.output.exists():
        parser.error(f'output already exists: {args.output}')
    args.output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    recorder = Recorder(args.output, args.fps)
    try:
        rclpy.spin(recorder)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            recorder.finish()
        finally:
            recorder.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
