"""Isaac Sim 대역. 지도 한 장으로 /clock /tf /chassis/odom 3D 점군을 만들어 낸다.

Isaac 을 못 쓰는 동안에도 내피에서 도킹 알고리즘을 끝까지 돌려 보기 위한 것이다.
점군은 지도 점유 격자를 광선으로 훑어 만들고, 리프트/팔의 자기 반사도 실측에서
관찰한 분포(x −0.58~−0.25, z ≤ 0.53)로 섞어 넣는다. 그래야 자기 반사 제거까지
같은 조건으로 시험된다.

    python3 fake_robot.py [--start x,y,yaw_deg] [--rtf 0.33] [--jam 6]

--jam 은 첫 속도 명령으로부터 그 초가 지나면 차체를 그 자리에 고정한다.
구조물에 완전히 끼인 상황을 만들어 멈춤 감지가 실제로 동작하는지 보기 위한 것이다.

차체 응답은 22·24차 bag 에서 읽은 대로 굼뜨게 만든다(기본값). 각속도는 명령×gain 을
목표로 시정수 tau 의 1차 지연으로 따라가고 초당 ang_accel 을 넘지 못한다. 제자리 회전은
이동 중보다 gain 이 낮고 tau 가 길다. 회전 중심은 base_link 뒤(pivot)라 제자리 회전에도
차체가 옆으로 밀린다. --ideal 을 주면 명령을 즉시 따르는 이상적 차체가 된다.
"""
import argparse
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_msgs.msg import TFMessage

DEFAULT_MAP = "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/maps/Collected_smartfarm_v011.yaml"
LIDAR_X = 0.026          # base_link 에서 라이다까지
LIDAR_MOUNT_X = 0.232    # front_3d_lidar 프레임 원점 보정
LIDAR_MOUNT_Z = 0.526
FOOTPRINT_X = (-0.607, 0.14)
FOOTPRINT_Y = (-0.25, 0.0, 0.25)


class FakeRobot(Node):
    def __init__(self, args):
        super().__init__("fake_robot")
        self.rtf = args.rtf
        self.jam = args.jam
        self.jam_from = None
        if args.ideal:
            self.ang_accel, self.tau_move, self.tau_inplace, self.gain_move, self.gain_inplace, self.pivot_x = 5.0, 0.05, 0.05, 1.0, 1.0, 0.0
        else:
            self.ang_accel, self.tau_move, self.tau_inplace = args.ang_accel, args.tau_move, args.tau_inplace
            self.gain_move, self.gain_inplace, self.pivot_x = args.gain_move, args.gain_inplace, args.pivot_x
        self.v_cmd = self.w_cmd = 0.0

        meta = yaml.safe_load(open(args.map))
        image = np.array(Image.open(args.map.replace(".yaml", ".png")).convert("L"))
        self.black = (image < 50)[::-1]   # 광선이 맞는 벽
        self.occ = (image < 200)[::-1]    # 차체가 못 지나가는 칸
        self.res = meta["resolution"]
        self.ox, self.oy = meta["origin"][:2]

        self.x0, self.y0, yaw0 = args.start
        self.yaw0 = math.radians(yaw0)
        self.x, self.y, self.yaw = self.x0, self.y0, self.yaw0
        self.v = self.w = 0.0
        self.last_cmd = 0.0
        self.t0 = self.prev = time.monotonic()
        self.laststamp = Time()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.clock = self.create_publisher(Clock, "/clock", 10)
        self.tf = self.create_publisher(TFMessage, "/tf", 50)
        self.odom = self.create_publisher(Odometry, "/chassis/odom", 10)
        self.cloud = self.create_publisher(
            PointCloud2, "/front_3d_lidar/lidar_points", qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_timer(1 / 60 / self.rtf, self.step)
        self.create_timer(0.1 / self.rtf, self.pub_scan)
        self.create_timer(2.0, self._log)

        mount = TransformStamped()
        mount.header.frame_id = "base_link"
        mount.child_frame_id = "front_3d_lidar"
        mount.transform.translation.x = -LIDAR_MOUNT_X
        mount.transform.translation.z = LIDAR_MOUNT_Z
        mount.transform.rotation.w = 1.0
        self.create_publisher(TFMessage, "/tf_static", latched).publish(
            TFMessage(transforms=[mount]))

    def _log(self):
        self.get_logger().info(
            f"world ({self.x:.2f},{self.y:.2f}) yaw {math.degrees(self.yaw):.0f} "
            f"v {self.v:.2f} w {self.w:.2f}")

    def _now(self):
        s = (time.monotonic() - self.t0) * self.rtf
        return Time(sec=int(s), nanosec=int((s % 1) * 1e9))

    def _on_cmd(self, m):
        self.v_cmd, self.w_cmd = m.linear.x, m.angular.z
        self.last_cmd = time.monotonic()

    def occupied(self, x, y):
        col = int((x - self.ox) / self.res)
        row = int((y - self.oy) / self.res)
        if row < 0 or col < 0 or row >= self.occ.shape[0] or col >= self.occ.shape[1]:
            return True
        return bool(self.occ[row, col])

    def collides(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        for fx in np.linspace(*FOOTPRINT_X, 8):
            for fy in FOOTPRINT_Y:
                if self.occupied(self.x + c * fx - s * fy, self.y + s * fx + c * fy):
                    return True
        return False

    def step(self):
        t = time.monotonic()
        dt = (t - self.prev) * self.rtf
        self.prev = t
        if t - self.last_cmd > 0.5 / self.rtf:
            self.v_cmd = self.w_cmd = 0.0

        # 차체 응답: 선속도는 빠르게(0.3 s), 각속도는 gain·tau·ang_accel 로 굼뜨게 따라간다
        self.v += (self.v_cmd - self.v) * min(1.0, dt / 0.3)
        moving = abs(self.v) > 0.03
        gain = self.gain_move if moving else self.gain_inplace
        tau = self.tau_move if moving else self.tau_inplace
        target = self.w_cmd * gain
        if abs(target) < abs(self.w) and target * self.w >= 0 or target * self.w < 0:
            lim = 1.4 * self.ang_accel * dt        # 멈추는 쪽은 마찰이 지배해 지연 없이 빨리 준다 (24차 0.14 rad/s^2)
            self.w += max(-lim, min(lim, target - self.w))
        else:
            lim = self.ang_accel * dt
            self.w += max(-lim, min(lim, (target - self.w) * min(1.0, dt / tau)))

        was = (self.x, self.y, self.yaw)
        px = self.x + self.pivot_x * math.cos(self.yaw)
        py = self.y + self.pivot_x * math.sin(self.yaw)
        self.yaw += self.w * dt
        self.x = px - self.pivot_x * math.cos(self.yaw) + self.v * math.cos(self.yaw) * dt
        self.y = py - self.pivot_x * math.sin(self.yaw) + self.v * math.sin(self.yaw) * dt
        if self.collides():
            self.x, self.y, self.yaw = was
        if self.jam > 0.0:
            if self.jam_from is None and (abs(self.v_cmd) > 1e-6 or abs(self.w_cmd) > 1e-6):
                self.jam_from = t
            if self.jam_from is not None and t - self.jam_from > self.jam:
                self.x, self.y, self.yaw = was   # 구조물에 완전히 끼인 상황

        stamp = self.laststamp = self._now()
        self.clock.publish(Clock(clock=stamp))

        dx, dy = self.x - self.x0, self.y - self.y0
        c, s = math.cos(-self.yaw0), math.sin(-self.yaw0)
        px, py, pyaw = c * dx - s * dy, s * dx + c * dy, self.yaw - self.yaw0

        tr = TransformStamped()
        tr.header.stamp = stamp
        tr.header.frame_id = "odom"
        tr.child_frame_id = "base_link"
        tr.transform.translation.x = px
        tr.transform.translation.y = py
        tr.transform.rotation.z = math.sin(pyaw / 2)
        tr.transform.rotation.w = math.cos(pyaw / 2)
        self.tf.publish(TFMessage(transforms=[tr]))

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = "odom"
        o.child_frame_id = "base_link"
        o.pose.pose.position.x = px
        o.pose.pose.position.y = py
        o.pose.pose.orientation.z = math.sin(pyaw / 2)
        o.pose.pose.orientation.w = math.cos(pyaw / 2)
        o.twist.twist.linear.x = self.v
        o.twist.twist.angular.z = self.w
        self.odom.publish(o)

    def pub_scan(self):
        self.step()
        n = 360
        ang = np.linspace(-math.pi, math.pi, n, endpoint=False)
        lx = self.x + LIDAR_X * math.cos(self.yaw)
        ly = self.y + LIDAR_X * math.sin(self.yaw)
        d = np.arange(0.05, 15.0, 0.04)
        out = np.full(n, np.inf, dtype=np.float32)
        ca, sa = np.cos(ang + self.yaw), np.sin(ang + self.yaw)
        col = ((lx + np.outer(ca, d) - self.ox) / self.res).astype(int)
        row = ((ly + np.outer(sa, d) - self.oy) / self.res).astype(int)
        ok = ((row >= 0) & (col >= 0)
              & (row < self.occ.shape[0]) & (col < self.occ.shape[1]))
        hit = np.zeros_like(ok)
        hit[ok] = self.black[row[ok], col[ok]]
        first, has = hit.argmax(axis=1), hit.any(axis=1)
        out[has] = d[first[has]]

        fin = np.isfinite(out)
        r, a = out[fin], ang[fin]
        px = r * np.cos(a) + (LIDAR_X + LIDAR_MOUNT_X)
        py = r * np.sin(a)
        pts = [(float(x), float(y), 0.0) for x, y in zip(px, py)]

        rng = np.random.default_rng()
        for _ in range(60):   # 리프트/팔 자기 반사 (실측 분포)
            bx = rng.uniform(-0.58, -0.25)
            by = rng.choice([-0.2, 0.2]) + rng.uniform(-0.02, 0.02)
            bz = rng.uniform(0.2, 0.53)
            pts.append((bx + LIDAR_MOUNT_X, by, bz - LIDAR_MOUNT_Z))

        h = TransformStamped().header
        h.stamp = self.laststamp
        h.frame_id = "front_3d_lidar"
        self.cloud.publish(pc2.create_cloud_xyz32(h, pts))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=DEFAULT_MAP)
    ap.add_argument("--start", default="-0.421,1.006,90",
                    type=lambda s: tuple(float(v) for v in s.split(",")))
    ap.add_argument("--rtf", type=float, default=1.0, help="실시간 배율 (고피 실측은 0.33)")
    ap.add_argument("--jam", type=float, default=0.0, help="이 초가 지나면 차체를 고정")
    ap.add_argument("--ideal", action="store_true", help="명령을 즉시 따르는 이상적 차체")
    ap.add_argument("--ang-accel", type=float, default=0.10, help="각속도 변화 한계 rad/s^2 (24차 bag)")
    ap.add_argument("--tau-move", type=float, default=2.0, help="이동 중 각속도 시정수 s")
    ap.add_argument("--tau-inplace", type=float, default=3.0, help="제자리 회전 각속도 시정수 s")
    ap.add_argument("--gain-move", type=float, default=0.7, help="이동 중 각속도 명령 대비 실제 비율")
    ap.add_argument("--gain-inplace", type=float, default=0.75, help="제자리 회전 명령 대비 실제 비율 (22차 0.12, 24차 0.75)")
    ap.add_argument("--pivot-x", type=float, default=-0.25, help="회전 중심의 base_link x (m)")
    args = ap.parse_args()

    rclpy.init()
    node = FakeRobot(args)
    try:
        rclpy.spin(node)
    except BaseException:
        pass


if __name__ == "__main__":
    main()
