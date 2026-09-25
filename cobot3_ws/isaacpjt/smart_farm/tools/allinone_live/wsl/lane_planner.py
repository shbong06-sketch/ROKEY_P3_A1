#!/usr/bin/env python3
"""바닥 노란 차선(흰 점선 중앙선) 경로를 주는 ComputePathToPose 액션 서버 'lane_planner' (lane_route_bt.xml 이 부른다).

차선 좌표는 씬 v014 바닥 선 메쉬에서 뽑은 값이다 (tools/allinone_live/floor_lines.py, 2026-09-25):
  남북 차선: 노란 테두리 x -0.925 / +0.067, 중앙 x -0.42, RACK_DOCK(y 1.01) ~ 모서리
  동서 차선: 노란 테두리 y -1.04 / -2.04, 중앙 y -1.54, 모서리 ~ FEEDER 도킹선 x -2.19
경로 (planner_id LaneRoute, 카터는 후진, 자세 = 진행 방향 + 180 deg) — A안:
  카터 위치 -> 남북 중앙선 x -0.42 남쪽 직진 -> 모서리 곡선(R1)을 45 deg 까지 -> BT 가 Nav2 기본 경로로 FEEDER_APPROACH
  -> feeder_dock 도킹. (도킹선 진입까지 차선으로 가는 full 경로는 카터 회전 반경 때문에 갇혀서 쓰지 않는다)
  도킹 구역에 들어가면 collision_monitor 의 사람 정지·감속 영역을 끄고 나오면 다시 켠다(안전 영역 전환).
제자리 회전은 쓰지 않는다: 리그가 제자리에서 돌지 못하고 (명령 직진 속도 0 인데도, 원인 미확인)
0.7 m 떨어진 점을 축으로 끌리며 돈다(2026-09-25 시험).
목표가 FEEDER_APPROACH 가 아니거나 카터가 남북 차선 밖이면 거절(abort) -> BT 가 Nav2 기본 경로로 간다.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionServer
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

LANE_X = -0.42            # 남북 차선 중앙
LANE_Y = -1.54            # 동서 차선 중앙
DOCK_X = -2.19            # FEEDER 도킹선 (stations.yaml FEEDER_APPROACH / FEEDER_DOCK x)
# 곡선 반경: 카터+리프트+팔+트레이 리그가 실제로 도는 반경은 주행 중 약 1.2 m (R 0.5 는 바깥으로 밀려나 갇힘, 2026-09-25 시험).
# 남북 중앙선 x -0.42 와 도킹선 x -2.19 사이가 1.77 m 라 R1 + R2 <= 1.77.
R1 = 0.85                 # 첫 모서리
R2 = 0.85                 # 도킹선 진입 -> 끝 (-2.19, -2.39)
DOCK_ZONE = (-1.50, -1.70)   # x 가 이보다 작고 y 가 이보다 작으면 도킹 구역: 사람 정지·감속 영역을 끈다(컨베이어를 사람으로 오인 방지)
FEEDER_APPROACH = (-2.19, -1.55)
GOAL_TOL = 0.35
STEP = 0.05


def _pose(x, y, yaw, stamp):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = stamp
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def _line(a, b):
    """(x, y, 진행방향) 점들, 끝점 제외."""
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(d / STEP))
    h = math.atan2(b[1] - a[1], b[0] - a[0])
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n, h) for i in range(n)]


def _arc(c, r, a0, a1):
    """중심 c, 반경 r, 각 a0 -> a1 호 위 (x, y, 진행방향) 점들, 끝점 제외."""
    n = max(2, int(abs(a1 - a0) * r / STEP))
    s = 1.0 if a1 > a0 else -1.0
    return [(c[0] + r * math.cos(a), c[1] + r * math.sin(a), a + s * math.pi / 2)
            for a in (a0 + (a1 - a0) * i / n for i in range(n))]


HANDOFF_DEG = 45.0        # 모서리 곡선을 이만큼만 돌고 Nav2 기본 경로(Hybrid-A*)에 넘긴다 (A안)


def lane_route(rx, ry, full=False):
    """카터 위치에서의 차선 경로 (x, y, 카터 자세) 목록.

    기본(A안): 남북 중앙선 남쪽 직진 -> 모서리 곡선 45 deg 까지 -> 끝 (-0.82, -1.29) 자세 약 45 deg.
      그다음은 BT 가 Nav2 기본 경로로 FEEDER_APPROACH 까지 간다. 카터가 두 번째 90 deg 곡선(도킹선 진입)을
      물리적으로 못 돌아서(회전 반경 약 1.2 m, 두 모서리 사이 1.77 m) 차선 끝까지는 Nav2 계획기에 맡긴다.
    full=True: 도킹선 진입까지 전부 (2026-09-25 시험에서 두 번째 곡선에서 갇힘. 참고용)
    """
    c1 = (LANE_X - R1, LANE_Y + R1)          # 모서리 곡선 중심
    pts = _line((rx, ry), (LANE_X, ry - 0.3))                     # 차선 중앙으로 들어서기
    pts += _line((LANE_X, ry - 0.3), (LANE_X, c1[1]))             # 남쪽 직진
    if not full:
        a1 = -math.radians(HANDOFF_DEG)
        pts += _arc(c1, R1, 0.0, a1)                              # 모서리 곡선 앞부분 (시계 방향)
        pts.append((c1[0] + R1 * math.cos(a1), c1[1] + R1 * math.sin(a1), a1 - math.pi / 2))
        return [(x, y, h + math.pi) for x, y, h in pts]
    c2 = (DOCK_X + R2, LANE_Y - R2)
    pts += _arc(c1, R1, 0.0, -math.pi / 2)                        # 모서리 (시계 방향)
    pts += _line((c1[0], LANE_Y), (c2[0], LANE_Y))                # 서쪽 직진
    pts += _arc(c2, R2, math.pi / 2, math.pi)                     # 도킹선 진입 (반시계)
    pts.append((DOCK_X, c2[1], -math.pi / 2))
    return [(x, y, h + math.pi) for x, y, h in pts]               # 후진: 카터 자세 = 진행 방향 + 180 deg


class LanePlanner(Node):
    def __init__(self):
        super().__init__("lane_planner")
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.path_pub = self.create_publisher(Path, "/lane_path", 1)
        ActionServer(self, ComputePathToPose, "lane_planner", self._plan)
        # 안전 영역 전환: 도킹 구역에서는 collision_monitor 의 HumanStop/HumanSlow 를 끈다 (사람 모드일 때만 있는 영역)
        from rcl_interfaces.srv import SetParameters

        self.param_cli = self.create_client(SetParameters, "/collision_monitor/set_parameters")
        self.in_dock_zone = None
        self.create_timer(0.2, self._field_switch)
        self.get_logger().info("lane_planner ready (LaneRoute)")

    def _field_switch(self):
        try:
            x, y = self._robot_xy(0.0)
        except Exception:  # noqa: BLE001
            return
        inside = x < DOCK_ZONE[0] and y < DOCK_ZONE[1]
        if inside == self.in_dock_zone or not self.param_cli.service_is_ready():
            return
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters

        req = SetParameters.Request()
        req.parameters = [Parameter(name=f"{n}.enabled", value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=not inside))
                          for n in ("HumanStop", "HumanSlow")]
        self.param_cli.call_async(req)
        self.in_dock_zone = inside
        self.get_logger().info(f"safety field {'DOCK (HumanStop/HumanSlow off)' if inside else 'LANE (HumanStop/HumanSlow on)'} at ({x:.2f}, {y:.2f})")

    def _robot_xy(self, wait_s=1.0):
        t = self.tf.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=wait_s))
        return t.transform.translation.x, t.transform.translation.y

    def _plan(self, handle):
        g = handle.request.goal.pose.position
        result = ComputePathToPose.Result()
        try:
            rx, ry = self._robot_xy()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"no robot pose: {e}")
            handle.abort()
            return result
        if handle.request.planner_id != "LaneRoute" or math.hypot(g.x - FEEDER_APPROACH[0], g.y - FEEDER_APPROACH[1]) > GOAL_TOL:
            self.get_logger().info(f"goal ({g.x:.2f}, {g.y:.2f}) is not FEEDER_APPROACH -> default Nav2 route")
            handle.abort()
            return result
        if abs(rx - LANE_X) > 0.35 or ry < LANE_Y + R1 + 0.3:
            self.get_logger().info(f"robot ({rx:.2f}, {ry:.2f}) not on the rack lane -> default Nav2 route")
            handle.abort()
            return result
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = stamp
        path.poses = [_pose(x, y, h, stamp) for x, y, h in lane_route(rx, ry)]
        self.path_pub.publish(path)
        result.path = path
        e = path.poses[-1].pose.position
        self.get_logger().info(f"LaneRoute: {len(path.poses)} poses from ({rx:.2f}, {ry:.2f}) to ({e.x:.2f}, {e.y:.2f})")
        handle.succeed()
        return result


def main():
    rclpy.init()
    rclpy.spin(LanePlanner())


if __name__ == "__main__":
    main()
