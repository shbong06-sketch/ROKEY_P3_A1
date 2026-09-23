"""Precise rear-in docking at the Feeder TurnTable using the lidar-measured face of the table.

    ros2 run smart_farm_navigation feeder_dock            (auto: arms itself near FEEDER_APPROACH)
    ros2 topic pub --once /feeder_dock/start std_msgs/msg/Empty '{}'   (manual trigger)

Why: Nav2/AMCL leaves the final heading 7-33 deg off (RPP cannot rotate in place while reversing and
AMCL yaw wobbles with the pallet on board).  This node ignores the map: it takes /scan (the self-filtered
3D lidar), finds the straight north face of Conveyor/TurnTable behind the robot, and closes the loop on
that line: (1) turn in place so the rear points at the goal point on the face's centre normal,
(2) reverse to it while holding the line, (3) turn in place until the rear is square to the face,
(4) creep to the exact standoff.  Result on /feeder_dock/result (String JSON) and a latched status.

Geometry (base_link, x forward = drive wheels, rear = -x = M0609 side):
  face line fitted to scan points 0.5..3.2 m behind the robot; the face is ~1.15 m long.
  goal: base_link `standoff_m` (default 0.85 m) in front of the face on the normal through the face centre,
        rear square to the face  ->  world (-2.19, -2.60, 90 deg) for the v011 scene.
"""

import json
import math
import time

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

LIDAR_X = -0.232          # XT-32 in base_link


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def detect_face(ranges, angle_min, angle_inc, search_x, search_y, len_lim, seed=0, debug=False):
    """Fit the TurnTable face line in the base_link frame from one LaserScan.
    Returns (dist, yaw_err, lat, cx, cy, n_pts, length) or None; with debug=True also a reason string."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r)) * angle_inc
    ok = np.isfinite(r) & (r > 0.05) & (r < 6.0)
    x = r[ok] * np.cos(a[ok]) + LIDAR_X; y = r[ok] * np.sin(a[ok])
    sel = (x > search_x[0]) & (x < search_x[1]) & (y > search_y[0]) & (y < search_y[1])
    x, y = x[sel], y[sel]
    if len(x) < 8:
        return (None, f"only {len(x)} points in search window") if debug else None
    rng = np.hypot(x, y)
    i0 = int(np.argmin(rng))
    near = np.hypot(x - x[i0], y - y[i0]) < 1.5
    px, py = x[near], y[near]
    if len(px) < 8:
        return (None, f"only {len(px)} points near nearest ({x[i0]:.2f},{y[i0]:.2f})") if debug else None
    pts = np.stack([px, py], axis=1)
    rs = np.random.default_rng(seed)
    best = None; rejected = {"short": 0, "normal": 0}
    for _ in range(120):
        i, j = rs.choice(len(pts), 2, replace=False)
        t = pts[j] - pts[i]
        nrm = np.hypot(*t)
        if nrm < 0.15:
            continue
        t /= nrm; nvec = np.array([-t[1], t[0]])
        inl = np.abs((pts - pts[i]) @ nvec) < 0.03
        cnt = int(inl.sum())
        if cnt < 8:
            rejected["short"] += 1; continue
        centre = pts[inl].mean(axis=0)
        if nvec @ centre > 0:
            nvec = -nvec
        if nvec[0] < math.cos(math.radians(60)):
            rejected["normal"] += 1; continue
        if best is None or cnt > best[0]:
            best = (cnt, inl)
    if best is None:
        return (None, f"no line: {len(pts)} pts, rejected {rejected}, nearest ({x[i0]:.2f},{y[i0]:.2f})") if debug else None
    cnt, inl = best
    px, py = pts[inl, 0], pts[inl, 1]
    cx, cy = px.mean(), py.mean()
    u = np.stack([px - cx, py - cy], axis=1)
    w, v = np.linalg.eigh(u.T @ u); tdir = v[:, 1]; ndir = np.array([-tdir[1], tdir[0]])
    proj = u @ tdir
    length = float(proj.max() - proj.min())
    centre = np.array([cx, cy]) + tdir * float((proj.max() + proj.min()) / 2.0)
    if ndir @ centre > 0:
        ndir = -ndir
    if not (len_lim[0] <= length <= len_lim[1]):
        return (None, f"line length {length:.2f} outside {len_lim} (centre {centre[0]:.2f},{centre[1]:.2f}, {cnt} pts)") if debug else None
    yaw_err = wrap(math.atan2(ndir[1], ndir[0]))
    dist = float(abs(ndir @ centre))
    face = (dist, yaw_err, float(centre[1]), float(centre[0]), float(centre[1]), int(cnt), length)
    return (face, "ok") if debug else face


class FeederDock(Node):
    def __init__(self) -> None:
        super().__init__("feeder_dock")
        d = self.declare_parameter
        d("scan_topic", "/scan"); d("cmd_vel_topic", "/cmd_vel")
        d("auto_start", False)          # 통합은 명령으로 시작한다. RViz2 수동 절차는 launch 인자 dock_auto:=true
        d("arm_x", -2.19); d("arm_y", -1.55); d("arm_radius_m", 0.6)     # FEEDER_APPROACH (map)
        d("standoff_m", 0.85)                     # base_link ~ TurnTable 앞면 거리. v011 면 y -3.60 -> base_link y -2.75.
        #   팀 robot_motion.BASE_TO_PALLET_X 가 팔 밑동~대상 0.89~1.05 m 를 요구한다. 팔 밑동은 base_link 에서 0.20 m 뒤이므로
        #   대상까지 거리 = 0.11 + standoff. 0.85 면 0.96 m 로 그 범위 한가운데다(0.75 는 0.86 m 로 범위 밖이었다).
        d("face_min_len_m", 0.6); d("face_max_len_m", 1.6)
        d("search_x", [-3.4, -0.5]); d("search_y", [-1.3, 1.3])
        d("reverse_speed_mps", 0.15); d("creep_speed_mps", 0.05)
        d("turn_speed_radps", 0.35); d("turn_min_radps", 0.08)
        d("yaw_tol_deg", 1.5); d("dist_tol_m", 0.03); d("lat_tol_m", 0.04)
        d("k_yaw", 1.5); d("k_lat", 1.2); d("timeout_s", 120.0)
        # 2026-09-23 실측: 후진 중 방향이 25도 틀어졌고, 도킹 지점에서 제자리 회전을 하다가
        # 팔이 든 팔레트가 컨베이어에 걸려 멈췄다. 도킹 근처에서는 제자리 회전을 하지 않는다.
        d("reverse_w_max", 0.20)          # 후진 중 조향 상한. 크면 짧은 거리에서 방향이 크게 흔들린다
        d("square_tol_deg", 3.0)          # 도착 시 허용 방향 오차
        d("backoff_extra_m", 0.60)        # 다시 맞출 때 면에서 얼마나 더 떨어져서 회전하는가
        d("max_retry", 2)
        d("stall_check_s", 3.0)           # 명령을 내는데 이만큼 움직임이 없으면 멈춘 것으로 본다
        d("stall_move_m", 0.02)
        d("stall_turn_rad", 0.02)         # 제자리 회전도 움직인 것으로 센다. 없으면 회전 구간이 멈춤으로 잘못 판정된다
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.auto = bool(p("auto_start")); self.arm = (float(p("arm_x")), float(p("arm_y"))); self.arm_r = float(p("arm_radius_m"))
        self.standoff = float(p("standoff_m")); self.len_lim = (float(p("face_min_len_m")), float(p("face_max_len_m")))
        self.sx = [float(v) for v in p("search_x")]; self.sy = [float(v) for v in p("search_y")]
        self.v_rev = float(p("reverse_speed_mps")); self.v_creep = float(p("creep_speed_mps"))
        self.w_turn = float(p("turn_speed_radps")); self.w_min = float(p("turn_min_radps"))
        self.yaw_tol = math.radians(float(p("yaw_tol_deg"))); self.dist_tol = float(p("dist_tol_m")); self.lat_tol = float(p("lat_tol_m"))
        self.k_yaw = float(p("k_yaw")); self.k_lat = float(p("k_lat")); self.timeout = float(p("timeout_s"))
        self.w_rev_max = float(p("reverse_w_max")); self.square_tol = math.radians(float(p("square_tol_deg")))
        self.backoff_extra = float(p("backoff_extra_m")); self.max_retry = int(p("max_retry"))
        self.stall_check = float(p("stall_check_s")); self.stall_move = float(p("stall_move_m"))
        self.stall_turn = float(p("stall_turn_rad"))

        self.cmd = self.create_publisher(Twist, p("cmd_vel_topic"), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "/feeder_dock/status", latched)
        self.result_pub = self.create_publisher(String, "/feeder_dock/result", latched)
        self.create_subscription(LaserScan, p("scan_topic"), self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, "/feeder_dock/start", lambda m: self._start(m.data or "manual"), 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
        self.create_subscription(Odometry, "/chassis/odom", self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)

        self.face = None           # (d, yaw_err, lat, cx, cy, n_pts, length)
        self.last_why = ""
        self.face_stamp = 0.0
        self.amcl_xy = None; self.last_ext_cmd = 0.0; self.near_since = None
        self.phase = "IDLE"; self.t_phase = 0.0; self.t_start = 0.0; self.done = False
        self.run_id = ""          # 명령을 보낸 쪽이 준 실행 식별자. 결과에 그대로 담아 과거 결과와 구분한다.
        self.retry = 0
        self.odom_pose = None; self.stall_ref = None; self.stall_since = 0.0
        self.auto_seq = 0
        self.create_timer(0.05, self._tick)
        self.create_timer(5.0, self._report)
        self._status("IDLE", "waiting" + (" (auto: arms within %.1f m of FEEDER_APPROACH)" % self.arm_r if self.auto else ""))

    # ---------- perception ----------
    def _on_scan(self, m: LaserScan) -> None:
        face, why = detect_face(m.ranges, m.angle_min, m.angle_increment, self.sx, self.sy, self.len_lim,
                                seed=int(m.header.stamp.nanosec) & 0xFFFF, debug=True)
        self.last_why = why
        self.face = face
        if face is not None:
            self.face_stamp = self._now()

    def _now(self) -> float:
        """Seconds on the node clock (= Isaac sim time with use_sim_time).  Wall time must not be used:
        at real-time factor 0.3 the scans arrive 0.9 s apart on the wall clock and every wall-clock
        freshness/timeout check fires (2026-09-22 19:06 run: face detected on every scan, docking failed anyway)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_amcl(self, m):  self.amcl_xy = (m.pose.pose.position.x, m.pose.pose.position.y)

    def _on_odom(self, m):
        q = m.pose.pose.orientation
        self.odom_pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                          math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def _stalled(self, now) -> bool:
        """명령을 내고 있는데 실제로 움직이지 않으면 True. 구조물에 걸린 채 밀지 않게 한다.

        제자리 회전은 위치가 거의 변하지 않으므로 방향 변화도 함께 본다. 위치만 보면
        회전 구간이 멈춤으로 잘못 판정되고, 그 타이머가 다음 구간으로 넘어간다(2026-09-23 실측).
        """
        if self.odom_pose is None:
            return False
        if self.stall_ref is None:
            self.stall_ref = self.odom_pose; self.stall_since = now
            return False
        moved = math.hypot(self.odom_pose[0] - self.stall_ref[0], self.odom_pose[1] - self.stall_ref[1])
        turned = abs(wrap(self.odom_pose[2] - self.stall_ref[2]))
        if moved > self.stall_move or turned > self.stall_turn:
            self.stall_ref = self.odom_pose; self.stall_since = now
            return False
        return now - self.stall_since > self.stall_check
    def _on_cmd(self, m):
        if self.phase in ("IDLE", "DONE", "FAILED") and (abs(m.linear.x) > 0.01 or abs(m.angular.z) > 0.01):
            self.last_ext_cmd = self._now()

    def _report(self):
        if self.phase != "IDLE":
            return
        if not self.auto and self.amcl_xy is not None and self.face is not None:
            if math.hypot(self.amcl_xy[0] - self.arm[0], self.amcl_xy[1] - self.arm[1]) < self.arm_r:
                self.get_logger().warning(
                    "FEEDER_APPROACH 부근에 서 있으나 자동 시작이 꺼져 있습니다. "
                    "navigation_node 명령으로 시작하거나 "
                    "'ros2 topic pub --once /feeder_dock/start std_msgs/msg/String \"{data: manual}\"' 로 직접 시작하십시오."
                )
        near = None
        if self.amcl_xy is not None:
            near = math.hypot(self.amcl_xy[0] - self.arm[0], self.amcl_xy[1] - self.arm[1])
        f = self.face
        self.get_logger().info(
            f"idle: amcl dist to FEEDER_APPROACH {near if near is None else round(near, 2)} m, "
            f"nav2 idle {self._now() - self.last_ext_cmd > 2.0}, face "
            + (f"d={f[0]:.2f} yaw={math.degrees(f[1]):+.1f} len={f[6]:.2f}" if f else f"NOT FOUND ({self.last_why})"))

    # ---------- state machine ----------
    def _status(self, phase, detail=""):
        self.phase = phase; self.t_phase = self._now()
        self.stall_ref = None          # 단계가 바뀌면 멈춤 판정을 처음부터 다시 센다
        self.status_pub.publish(String(data=json.dumps({"phase": phase, "detail": detail}, ensure_ascii=False)))
        self.get_logger().info(f"[{phase}] {detail}")

    def _start(self, run_id):
        if self.phase not in ("IDLE", "DONE", "FAILED"):
            self.get_logger().warning(f"start '{run_id}' 무시: 이미 {self.phase}")
            return
        self.done = False; self.t_start = self._now(); self.run_id = run_id
        self.retry = 0; self.stall_ref = None; self.stall_since = self._now()
        self._status("ALIGN_TO_GOAL", f"started (run_id={run_id})")

    def _finish(self, ok, reason):
        self._pub(0.0, 0.0)
        f = self.face
        res = {"run_id": self.run_id,
               "status": "SUCCEEDED" if ok else "FAILED", "reason": reason,
               "face_dist_m": round(f[0], 3) if f else None, "yaw_err_deg": round(math.degrees(f[1]), 2) if f else None,
               "lat_m": round(f[2], 3) if f else None}
        self.result_pub.publish(String(data=json.dumps(res)))
        self._status("DONE" if ok else "FAILED", json.dumps(res))
        self.done = True

    def _pub(self, v, w):
        t = Twist(); t.linear.x = float(v); t.angular.z = float(w); self.cmd.publish(t)
        if abs(v) < 1e-3 and abs(w) < 1e-3:
            self.stall_ref = None      # 정지 명령 중에는 멈춤 판정을 하지 않는다

    def _turn_cmd(self, err):
        w = max(min(self.k_yaw * err, self.w_turn), -self.w_turn)
        if abs(w) < self.w_min:
            w = math.copysign(self.w_min, w)
        return w

    def _tick(self):
        now = self._now()
        fresh = self.face is not None and now - self.face_stamp < 2.5      # sim seconds; the rear sector drops out of single scans for up to ~1 s
        if self.phase in ("IDLE", "DONE", "FAILED"):
            if self.auto and self.phase == "IDLE" and self.amcl_xy is not None:
                near = math.hypot(self.amcl_xy[0] - self.arm[0], self.amcl_xy[1] - self.arm[1]) < self.arm_r
                idle = now - self.last_ext_cmd > 2.0
                if near and idle and fresh:
                    if self.near_since is None:
                        self.near_since = now
                    elif now - self.near_since > 1.0:
                        self.auto_seq += 1
                        self._start(f"auto-{self.auto_seq}")
                else:
                    self.near_since = None
            return
        if now - self.t_start > self.timeout:
            self._finish(False, "TIMEOUT"); return
        if not fresh:
            self._pub(0.0, 0.0)
            if now - self.t_phase > 8.0:
                self._finish(False, "FACE_NOT_FOUND")
            return
        dist, yaw_err, lat, cx, cy, n, length = self.face
        # goal point G on the face-centre normal, `standoff` in front of the face, in base frame
        nx, ny = math.cos(yaw_err), math.sin(yaw_err)            # normal (face -> robot)
        gx, gy = cx + nx * self.standoff, cy + ny * self.standoff
        g_dist = math.hypot(gx, gy)
        bearing_rear = wrap(math.atan2(gy, gx) - math.pi)       # 0 when G is straight behind

        if self.phase == "ALIGN_TO_GOAL":
            # 면에서 충분히 떨어진 자리에서만 제자리 회전을 한다. 도킹 지점에서는 회전하지 않는다.
            if g_dist < 0.12:
                self._status("CHECK", f"already at goal point (|G|={g_dist:.2f})"); return
            if abs(bearing_rear) < self.yaw_tol:
                self._status("REVERSE", f"G behind at {g_dist:.2f} m; face d={dist:.2f} yaw={math.degrees(yaw_err):+.1f} lat={lat:+.2f}")
            else:
                if self._stalled(now):
                    self._finish(False, "STALLED_ALIGN"); return
                self._pub(0.0, self._turn_cmd(bearing_rear))
        elif self.phase == "REVERSE":
            if g_dist < 0.06 or dist <= self.standoff + 0.02:
                self._pub(0.0, 0.0); self._status("CHECK", f"reached G (|G|={g_dist:.2f}, d={dist:.2f}, yaw={math.degrees(yaw_err):+.1f})"); return
            if self._stalled(now):
                self._finish(False, "STALLED_REVERSE"); return
            v = self.v_rev if g_dist > 0.3 else max(self.v_creep, self.v_rev * g_dist / 0.3)
            # 멀리서는 뒤축을 목표점에 겨누고, 가까워질수록 면과 직각을 맞추는 쪽으로 넘어간다.
            # 목표점만 겨누면 거리가 짧아질수록 조향이 민감해져 방향이 크게 틀어진다(2026-09-23 실측 25도).
            blend = min(1.0, g_dist / 0.5)
            w = self.k_lat * bearing_rear * blend + self.k_yaw * yaw_err * (1.0 - blend)
            w = max(min(w, self.w_rev_max), -self.w_rev_max)
            self._pub(-v, w)
        elif self.phase == "CHECK":
            # 도킹 지점에서는 회전하지 않는다. 방향이 틀어졌으면 면에서 물러나 다시 맞춘다.
            if abs(yaw_err) <= self.square_tol:
                self._status("CREEP", f"square (yaw {math.degrees(yaw_err):+.1f}); d={dist:.3f} target {self.standoff:.2f}")
                return
            if self.retry >= self.max_retry:
                self._finish(False, f"YAW_OFF_{math.degrees(yaw_err):+.0f}DEG"); return
            self.retry += 1
            self._status("BACKOFF", f"yaw {math.degrees(yaw_err):+.1f} 도 틀어짐 -> 물러나 다시 맞춤 ({self.retry}/{self.max_retry})")
        elif self.phase == "BACKOFF":
            target = self.standoff + self.backoff_extra
            if dist >= target:
                self._pub(0.0, 0.0); self._status("ALIGN_TO_GOAL", f"물러남 완료 (d={dist:.2f})"); return
            if self._stalled(now):
                self._finish(False, "STALLED_BACKOFF"); return
            self._pub(self.v_rev, max(min(self.k_yaw * yaw_err, 0.15), -0.15))
        elif self.phase == "CREEP":
            e = dist - self.standoff
            if abs(e) < self.dist_tol:
                self._pub(0.0, 0.0)
                self._finish(True, "NONE"); return
            if self._stalled(now):
                self._finish(False, "STALLED_CREEP"); return
            v = math.copysign(self.v_creep, -e)               # e>0: 너무 멀다 -> 후진
            self._pub(v, max(min(self.k_yaw * yaw_err, 0.10), -0.10))
        if now - self.t_phase > 45.0:
            self._finish(False, f"PHASE_TIMEOUT_{self.phase}")


def main() -> None:
    rclpy.init()
    node = FeederDock()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:                  # 종료 신호로 컨텍스트가 닫히며 나는 오류는 무시한다
        if rclpy.ok():
            raise
    finally:
        try: node._pub(0.0, 0.0)
        except Exception: pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
