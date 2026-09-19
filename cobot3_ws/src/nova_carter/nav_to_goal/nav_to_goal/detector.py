import math
import time
from datetime import datetime
from pathlib import Path

import cv2
import rclpy

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

def get_euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

def print_final_pose(pose_msg):
    if not pose_msg:
        return
    pos = pose_msg.pose.position
    ori = pose_msg.pose.orientation
    roll_rad, pitch_rad, yaw_rad = get_euler_from_quaternion(ori.x, ori.y, ori.z, ori.w)
    yaw_deg = math.degrees(yaw_rad)
    
    print("-" * 50)
    print(f"📍 최종 위치: X = {pos.x:.3f} m, Y = {pos.y:.3f} m")
    print(f"🧭 최종 바라보는 방향 (Heading): {yaw_deg:.1f}°")
    print("-" * 50)

def create_pose(navigator, x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    
    q = get_quaternion_from_euler(0, 0, math.radians(yaw_deg))
    
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

class CameraCaptureNode(Node):
    def __init__(
        self,
        image_topic='/front_stereo_camera/left/image_raw',
        save_directory='patrol_images'
    ):
        super().__init__('patrol_camera_capture')

        self.bridge = CvBridge()
        self.latest_image = None

        self.save_directory = Path(save_directory)
        self.save_directory.mkdir(parents=True, exist_ok=True)

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'카메라 토픽 구독 시작: {image_topic}'
        )

    def image_callback(self, msg):
        self.get_logger().info(
            f'이미지 수신: {msg.width}x{msg.height}',
            throttle_duration_sec=2.0
        )

        try:
            # OpenCV에서 저장하기 편하도록 BGR 형식으로 변환
            self.latest_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as error:
            self.get_logger().error(
                f'이미지 변환 실패: {error}'
            )

    def camera_capture(self, waypoint_index, timeout_sec=5.0):
        """
        호출된 시점 이후에 수신한 이미지 한 장을 저장한다.
        성공하면 저장 경로를 반환하고, 실패하면 None을 반환한다.
        """
        self.latest_image = None
        start_time = time.monotonic()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_image is not None:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                filename = (
                    f'waypoint_{waypoint_index:02d}_{timestamp}.jpg'
                )
                image_path = self.save_directory / filename

                success = cv2.imwrite(
                    str(image_path),
                    self.latest_image
                )

                if success:
                    self.get_logger().info(
                        f'사진 저장 완료: {image_path}'
                    )
                    return image_path

                self.get_logger().error(
                    f'사진 저장 실패: {image_path}'
                )
                return None

            if time.monotonic() - start_time >= timeout_sec:
                self.get_logger().error(
                    f'{timeout_sec:.1f}초 동안 카메라 이미지가 수신되지 않았습니다.'
                )
                return None

        return None


# ==========================================================
# 🚀 메인 주행 로직 (다중 경유지 주행)
# ==========================================================
def main():
    rclpy.init()
    nav = BasicNavigator()

    camera = CameraCaptureNode(
          image_topic='/front_stereo_camera/left/image_raw',
      )
    
    # 1. 출발점 설정
    init_pose = create_pose(nav, 0.002, -0.024, 0.0)
    nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active()
    
    # 2. 1번 순찰지점 생성
    detector_pts = []
    waypoints_1 = create_pose(nav, 2.01, 0.16, 0.0)
    waypoints_2 = create_pose(nav, -2.5, 5.0, 0.0)
    waypoints_3 = create_pose(nav, -6.00, -1.01, 0.0)
    
    detector_pts.append(waypoints_1)
    detector_pts.append(waypoints_2)
    detector_pts.append(waypoints_3)

    # 3. Task 실행 (goToPose -> goThroughPoses로 변경)
    print("🚀 순찰을 시작합니다...")

    try:
        for index, waypoint in enumerate(detector_pts, start=1):
            print(f'\n📍 {index}번 순찰 지점으로 이동합니다.')

            nav.goToPose(waypoint)
            last_pose = None

            while not nav.isTaskComplete():
                # 카메라 콜백 처리
                rclpy.spin_once(camera, timeout_sec=0.01)

                feedback = nav.getFeedback()
                if feedback:
                    last_pose = feedback.current_pose

                    print(
                        f'{index}번 지점까지 남은 거리: '
                        f'{feedback.distance_remaining:.2f} m'
                    )

                time.sleep(1.0)

            result = nav.getResult()

            if result == TaskResult.SUCCEEDED:
                print(f'✅ {index}번 순찰 지점 도착')
                print_final_pose(last_pose)

                # 정차 직후 카메라 노출이 안정되도록 필요시 잠깐 대기
                time.sleep(0.5)

                image_path = camera.camera_capture(
                    waypoint_index=index,
                    timeout_sec=5.0
                )

                if image_path is None:
                    print(f'⚠️ {index}번 지점 사진 촬영 실패')
                else:
                    print(f'📷 촬영 파일: {image_path}')

            elif result == TaskResult.CANCELED:
                print(f'⚠️ {index}번 지점 이동이 취소되었습니다.')
                break

            elif result == TaskResult.FAILED:
                print(f'❌ {index}번 지점 이동에 실패했습니다.')
                break

        else:
            print('\n🎉 모든 순찰 지점 방문을 완료했습니다.')

    finally:
        camera.destroy_node()
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()