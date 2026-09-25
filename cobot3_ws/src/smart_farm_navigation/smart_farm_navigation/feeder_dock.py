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
  goal: base_link `standoff_m` (default 0.90 m) in front of the face on the normal through the face centre,
        rear square to the face  ->  world (-2.19, -2.60, 90 deg) for the v011 scene.
"""

import json
import math
from dataclasses import dataclass, fields

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
LAT_SIGN = 1.0            # 횡 오차 보정 방향. sim_test/dock_sim.py 로 확인한 부호


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


def clamp(v: float, lim: float) -> float:
    return max(min(v, lim), -lim)


def coast(wz: float, decel: float) -> float:
    """지금 명령을 끊어도 관성으로 더 도는 각도(부호 포함).
    카터(LiftRig+팔레트)는 각속도가 초당 `decel` 만큼만 줄어든다(2026-09-23 22·24차 bag)."""
    return math.copysign(wz * wz / (2.0 * max(decel, 1e-3)), wz)


def settled_err(e: float, wz: float, lag: float, decel: float) -> float:
    """측정 지연과 회전 관성을 반영한, '지금 명령을 끊으면 멈췄을 때 남는' 방향 오차.
    bearing_rear·yaw_err 는 둘 다 로봇이 반시계로 돌면 줄어든다(de/dt = -wz)."""
    return e - wz * lag - coast(wz, decel)


@dataclass
class DockParams:
    """도킹 파라미터. 노드는 이 필드를 그대로 ROS 파라미터로 선언한다 (단일 출처)."""
    standoff_m: float = 0.92          # base_link ~ 라이다가 검출한 TurnTable 앞면.
    #                                   0.92 의 출처: 팀이 feature/cabbage-place-fix 에서 올인원을 끝까지 돌려 확인한 값이다
    #                                   (scenes/Collected_smartfarm_v014/allinone_debug_and_changes_2026-09-25.md 의 P3).
    #                                   0.85 에서는 팔 베이스~놓을 자리가 0.72 m 라 DESCEND_5 가 관절 20.7 도(한계 20)로 실패했다.
    #                                   팀이 turntable_place_pose() 의 놓을 자리 정의를 함께 고쳤으므로 이 값은 그 수정과 한 쌍이다.
    face_min_len_m: float = 0.6
    face_max_len_m: float = 1.6
    reverse_speed_mps: float = 0.10   # 24차까지 0.15. 각속도 응답이 느린 차체가 조향할 시간을 주기 위해 늦춤
    creep_speed_mps: float = 0.05
    turn_speed_radps: float = 0.35    # 관성으로 넘어갈 각도는 settled_err 가 미리 빼므로 속도 자체는 유지
    turn_min_radps: float = 0.08
    dist_tol_m: float = 0.03
    lat_tol_m: float = 0.06           # 도착 시 허용 횡 오차. 넘으면 물러나 다시 (max_retry 안에서)
    k_yaw: float = 1.5
    k_lat: float = 1.2
    timeout_s: float = 120.0
    phase_timeout_s: float = 45.0
    reverse_w_max: float = 0.12       # 후진 중 조향 상한. 24차: 0.20 을 5 s 유지하다 정렬을 지나쳐 반대쪽 21도
    square_tol_deg: float = 3.0       # 도착 시 허용 방향 오차
    backoff_extra_m: float = 0.90     # 다시 맞출 때 면에서 얼마나 더 물러나는가. 재시도 후진 거리가 곧
    #                                   횡 오차를 갚을 거리다. 0.6 이면 한 번에 15% 만 줄어 0.07 m 가 남았다(모의·ROS 회귀)
    max_retry: int = 2
    stall_check_s: float = 3.0        # 명령을 내는데 이만큼 움직임이 없으면 멈춘 것으로 본다
    stall_move_m: float = 0.02
    stall_turn_rad: float = 0.01      # 제자리 회전도 움직인 것으로 센다 (23차: 회전 구간이 멈춤으로 오판). 22차처럼 아주 느려도 살린다
    align_giveup_deg: float = 15.0    # 제자리 회전이 안 먹혀도 이 안이면 후진하며 맞춘다 (22차: 명령의 12% 만 돎)
    # 22·24차 bag: 각속도가 초당 0.05~0.1 rad/s 씩만 따라오고 제자리 회전은 명령의 10~75% 만 나온다.
    # /scan 은 0.5 s 합친 점군이라 방향 측정이 그만큼 늦다. 이 셋을 제어에 넣는다.
    ang_decel_radps2: float = 0.08
    meas_lag_s: float = 0.5
    settle_w_radps: float = 0.03      # 이보다 느리게 돌 때만 멈춘 것으로 본다
    align_tol_deg: float = 4.0        # 제자리 회전은 이만큼만 맞추고 나머지는 후진하며 맞춘다
    blend_dist_m: float = 0.4         # 이 거리 안에서는 횡 오차를 포기하고 직각 맞추기만 한다
    lookahead_m: float = 0.6          # 횡 오차를 이 거리에 걸쳐 갚는다 (아래 _reverse_cmd)
    lat_heading_cap_deg: float = 10.0 # 횡 오차를 갚기 위해 법선에서 벗어나도 되는 최대 방향. 굼뜬 차체가 되돌릴 수 있는 만큼만
    quiet_s: float = 2.0              # 시작 전 Nav2 가 이만큼 조용해야 한다 (ADR 2.2)
    fresh_s: float = 2.5              # 면 측정 신선도

    @property
    def square_tol(self): return math.radians(self.square_tol_deg)
    @property
    def align_tol(self): return math.radians(self.align_tol_deg)
    @property
    def align_giveup(self): return math.radians(self.align_giveup_deg)
    @property
    def lat_heading_cap(self): return math.radians(self.lat_heading_cap_deg)


class DockLogic:
    """도킹 상태기계. ROS 없이 돌릴 수 있게 입력(면 측정·odom)과 출력(v, w)만 다룬다.
    노드와 오프라인 모의(sim_test/dock_sim.py)가 이 클래스를 그대로 쓴다.

    SETTLE -> ALIGN_TO_GOAL -> REVERSE -> CHECK -> (CREEP -> DONE | BACKOFF -> ALIGN_TO_GOAL) / FAILED
    """

    def __init__(self, p: DockParams):
        self.p = p
        self.phase = "IDLE"; self.t_phase = 0.0; self.t_start = 0.0
        self.run_id = ""; self.retry = 0; self.result = None
        self.stall_ref = None; self.stall_since = 0.0
        self.events = []              # (phase, detail) — 노드가 로그·상태 토픽으로 내보낸다
        self.last_face = None

    # ---- 상태 ----
    def _status(self, phase, detail, now):
        self.phase = phase; self.t_phase = now
        self.stall_ref = None         # 단계가 바뀌면 멈춤 판정을 처음부터 다시 센다
        self.events.append((phase, detail))

    def start(self, run_id, now) -> bool:
        if self.phase not in ("IDLE", "DONE", "FAILED"):
            return False
        self.result = None; self.t_start = now; self.run_id = run_id; self.retry = 0
        self.stall_ref = None; self.stall_since = now
        self._status("SETTLE", f"started (run_id={run_id}); Nav2 가 {self.p.quiet_s:.0f} s 조용해지고 차체가 멈추면 정렬", now)
        return True

    def active(self) -> bool:
        return self.phase not in ("IDLE", "DONE", "FAILED")

    def _finish(self, ok, reason, now):
        f = self.last_face
        self.result = {"run_id": self.run_id,
                       "status": "SUCCEEDED" if ok else "FAILED", "reason": reason,
                       "face_dist_m": round(f[0], 3) if f else None,
                       "yaw_err_deg": round(math.degrees(f[1]), 2) if f else None,
                       "lat_m": round(f[2], 3) if f else None}
        self._status("DONE" if ok else "FAILED", json.dumps(self.result), now)
        return 0.0, 0.0

    def _stalled(self, now, odom_pose, moving) -> bool:
        """명령을 내고 있는데 실제로 움직이지 않으면 True. 구조물에 걸린 채 밀지 않게 한다."""
        if odom_pose is None or not moving:
            self.stall_ref = None
            return False
        if self.stall_ref is None:
            self.stall_ref = odom_pose; self.stall_since = now
            return False
        moved = math.hypot(odom_pose[0] - self.stall_ref[0], odom_pose[1] - self.stall_ref[1])
        turned = abs(wrap(odom_pose[2] - self.stall_ref[2]))
        if moved > self.p.stall_move_m or turned > self.p.stall_turn_rad:
            self.stall_ref = odom_pose; self.stall_since = now
            return False
        return now - self.stall_since > self.p.stall_check_s

    # ---- 제어 법칙 ----
    def _align_cmd(self, bearing, wz) -> float:
        """제자리 회전 명령. 남을 오차에 비례하되 관성으로 넘어갈 만큼은 미리 뺀다.
        24차: 정렬 완료 선언 순간 차체가 아직 0.24 rad/s 로 돌고 있어 15도 더 돌아갔다."""
        p = self.p
        e = settled_err(bearing, wz, p.meas_lag_s, p.ang_decel_radps2)
        if abs(e) < p.align_tol:
            return 0.0
        w = clamp(p.k_yaw * e, p.turn_speed_radps)
        return math.copysign(max(abs(w), p.turn_min_radps), w)

    def _reverse_cmd(self, cx, cy, yaw_err, g_dist, wz) -> float:
        """후진 조향. 면 가운데를 지나는 법선(도킹 선)을 따라가되, 횡 오차를 갚느라 방향을
        크게 틀지 않는다.

        목표점 G 를 겨누는 방식은 안 된다: 남은 거리가 짧을 때 횡 오차가 있으면 방향을 크게 틀어
        G 에 닿고, 굼뜬 차체는 도착 전에 되돌리지 못한다(24차: 횡 0.16 m -> 도착 방향 21도).
        여기서는 '원하는 방향' 을 법선에서 횡 오차에 비례해 최대 lat_heading_cap 만큼만 기울이고,
        그 방향 오차에 비례해 조향한다. 마지막 blend_dist_m 안에서는 횡 오차를 버리고 직각만 맞춘다.
        오차는 관성·지연을 뺀 값을 쓰고 상한을 낮게 둔다."""
        p = self.p
        nx, ny = math.cos(yaw_err), math.sin(yaw_err)          # 법선 (면 -> 로봇), base_link 기준
        e_lat = cx * ny - cy * nx                               # 로봇이 도킹 선에서 옆으로 벗어난 거리 (부호 포함)
        lat_term = clamp(math.atan2(e_lat, p.lookahead_m), p.lat_heading_cap)
        blend = min(1.0, max(0.0, (g_dist - p.blend_dist_m) / p.blend_dist_m))
        e_y = settled_err(yaw_err, wz, p.meas_lag_s, p.ang_decel_radps2)
        return clamp(p.k_yaw * (e_y + LAT_SIGN * lat_term * blend), p.reverse_w_max)

    # ---- 한 틱 ----
    def update(self, now, face, face_age, wz, vx, odom_pose, ext_cmd_age):
        """(v, w) 를 돌려준다. face = (dist, yaw_err, lat, cx, cy, n, length) 또는 None."""
        p = self.p
        if not self.active():
            return 0.0, 0.0
        if face is not None:
            self.last_face = face
        if now - self.t_start > p.timeout_s:
            return self._finish(False, "TIMEOUT", now)
        fresh = face is not None and face_age < p.fresh_s
        if not fresh:
            if now - self.t_phase > 8.0:
                return self._finish(False, "FACE_NOT_FOUND", now)
            return 0.0, 0.0
        dist, yaw_err, lat, cx, cy, _n, _len = face
        # 면 가운데 법선 위 standoff 지점 G (base_link 기준)
        nx, ny = math.cos(yaw_err), math.sin(yaw_err)
        gx, gy = cx + nx * p.standoff_m, cy + ny * p.standoff_m
        g_dist = math.hypot(gx, gy)
        bearing = wrap(math.atan2(gy, gx) - math.pi)          # 0 이면 G 가 정확히 뒤
        # 제자리 회전은 G 가 아니라 더 먼 면 가운데 C 를 겨눈다. 같은 횡 오차라도 각도가 작아
        # 그대로 후진하면 G 를 지날 때 횡 오차가 standoff/dist 배로 줄고 방향 오차도 작다.
        bearing_c = wrap(math.atan2(cy, cx) - math.pi)

        if now - self.t_phase > p.phase_timeout_s:
            return self._finish(False, f"PHASE_TIMEOUT_{self.phase}", now)

        if self.phase == "SETTLE":
            # Nav2(velocity_smoother) 감속 명령과 겹치지 않게, 또 차체가 실제로 멈춘 뒤에 시작한다 (ADR 2.2).
            if ext_cmd_age > p.quiet_s and abs(wz) < p.settle_w_radps and abs(vx) < 0.02:
                self._status("ALIGN_TO_GOAL", f"face d={dist:.2f} yaw={math.degrees(yaw_err):+.1f} bearing={math.degrees(bearing):+.1f}", now)
            return 0.0, 0.0

        if self.phase == "ALIGN_TO_GOAL":
            # 면에서 충분히 떨어진 자리에서만 제자리 회전을 한다. 도킹 지점에서는 회전하지 않는다.
            if g_dist < 0.12:
                self._status("CHECK", f"already at goal point (|G|={g_dist:.2f})", now); return 0.0, 0.0
            w = self._align_cmd(bearing_c, wz)
            if w == 0.0 and abs(wz) < p.settle_w_radps:
                self._status("REVERSE", f"G behind at {g_dist:.2f} m; face d={dist:.2f} yaw={math.degrees(yaw_err):+.1f} lat={lat:+.2f}", now)
                return 0.0, 0.0
            if self._stalled(now, odom_pose, w != 0.0):
                # 제자리 회전이 안 먹힌다(22차: 캐스터·마찰). 이동 중 조향은 되므로 오차가 작으면 후진하며 맞춘다.
                if abs(bearing_c) < p.align_giveup:
                    self._status("REVERSE", f"제자리 회전 없음 -> 후진하며 맞춤 (bearing={math.degrees(bearing_c):+.1f})", now)
                    return 0.0, 0.0
                return self._finish(False, "STALLED_ALIGN", now)
            return 0.0, w

        if self.phase == "REVERSE":
            if g_dist < 0.06 or dist <= p.standoff_m + 0.02:
                self._status("CHECK", f"reached G (|G|={g_dist:.2f}, d={dist:.2f}, yaw={math.degrees(yaw_err):+.1f})", now)
                return 0.0, 0.0
            if self._stalled(now, odom_pose, True):
                return self._finish(False, "STALLED_REVERSE", now)
            v = p.reverse_speed_mps if g_dist > 0.3 else max(p.creep_speed_mps, p.reverse_speed_mps * g_dist / 0.3)
            return -v, self._reverse_cmd(cx, cy, yaw_err, g_dist, wz)

        if self.phase == "CHECK":
            # 도킹 지점에서는 회전하지 않는다. 방향이 틀어졌으면 면에서 물러나 다시 맞춘다.
            if abs(wz) > p.settle_w_radps or now - self.t_phase < p.meas_lag_s:
                return 0.0, 0.0                      # 아직 돌고 있거나 측정이 덜 따라왔다
            square = abs(yaw_err) <= p.square_tol
            centred = abs(lat) <= p.lat_tol_m
            if square and (centred or self.retry >= p.max_retry):
                self._status("CREEP", f"square (yaw {math.degrees(yaw_err):+.1f}, lat {lat:+.3f}); d={dist:.3f} target {p.standoff_m:.2f}", now)
                return 0.0, 0.0
            if self.retry >= p.max_retry:
                return self._finish(False, f"YAW_OFF_{math.degrees(yaw_err):+.0f}DEG", now)
            self.retry += 1
            why = f"yaw {math.degrees(yaw_err):+.1f} 도" if not square else f"횡 {lat:+.3f} m"
            self._status("BACKOFF", f"{why} 틀어짐 -> 물러나 다시 맞춤 ({self.retry}/{p.max_retry})", now)
            return 0.0, 0.0

        if self.phase == "BACKOFF":
            if dist >= p.standoff_m + p.backoff_extra_m:
                self._status("ALIGN_TO_GOAL", f"물러남 완료 (d={dist:.2f})", now); return 0.0, 0.0
            if self._stalled(now, odom_pose, True):
                return self._finish(False, "STALLED_BACKOFF", now)
            return p.reverse_speed_mps, clamp(p.k_yaw * settled_err(yaw_err, wz, p.meas_lag_s, p.ang_decel_radps2), 0.12)

        if self.phase == "CREEP":
            e = dist - p.standoff_m
            if abs(e) < p.dist_tol_m:
                return self._finish(True, "NONE", now)
            if self._stalled(now, odom_pose, True):
                return self._finish(False, "STALLED_CREEP", now)
            v = math.copysign(p.creep_speed_mps, -e)         # e>0: 너무 멀다 -> 후진
            return v, clamp(p.k_yaw * settled_err(yaw_err, wz, p.meas_lag_s, p.ang_decel_radps2), 0.08)
        return 0.0, 0.0


class FeederDock(Node):
    """ROS 배선: /scan -> 면 검출, odom/cmd_vel 감시, DockLogic 한 틱, /cmd_vel·상태·결과 발행."""

    def __init__(self) -> None:
        super().__init__("feeder_dock")
        d = self.declare_parameter
        d("scan_topic", "/scan"); d("cmd_vel_topic", "/cmd_vel")
        d("auto_start", False)          # 통합은 명령으로 시작한다. RViz2 수동 절차는 launch 인자 dock_auto:=true
        d("arm_x", -2.19); d("arm_y", -1.55); d("arm_radius_m", 0.6)     # FEEDER_APPROACH (map)
        d("search_x", [-3.4, -0.5]); d("search_y", [-1.3, 1.3])
        # DockParams 의 필드 하나하나를 같은 이름의 ROS 파라미터로 선언하고, 읽은 값을 다시 채워 넣는다.
        self.params = DockParams()
        for f in fields(DockParams):
            d(f.name, getattr(self.params, f.name))
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        for f in fields(DockParams):
            setattr(self.params, f.name, f.type(p(f.name)))
        self.logic = DockLogic(self.params)
        self.auto = bool(p("auto_start")); self.arm = (float(p("arm_x")), float(p("arm_y"))); self.arm_r = float(p("arm_radius_m"))
        self.sx = [float(v) for v in p("search_x")]; self.sy = [float(v) for v in p("search_y")]
        self.len_lim = (self.params.face_min_len_m, self.params.face_max_len_m)

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
        self.face_stamp = -1e9
        self.amcl_xy = None; self.last_ext_cmd = -1e9; self.near_since = None
        self.odom_pose = None; self.wz = 0.0; self.vx = 0.0
        self.auto_seq = 0
        self.create_timer(0.05, self._tick)
        self.create_timer(5.0, self._report)
        self._publish_status("IDLE", "waiting" + (" (auto: arms within %.1f m of FEEDER_APPROACH)" % self.arm_r if self.auto else ""))

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
        self.wz = m.twist.twist.angular.z; self.vx = m.twist.twist.linear.x
        self.odom_pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                          math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def _on_cmd(self, m):
        # 우리가 내는 단계(정렬~접근)를 빼면 /cmd_vel 은 전부 Nav2 것이다. SETTLE 중에는 아무것도 내지 않는다.
        if self.logic.phase in ("IDLE", "DONE", "FAILED", "SETTLE") and (abs(m.linear.x) > 0.01 or abs(m.angular.z) > 0.01):
            self.last_ext_cmd = self._now()

    def _report(self):
        if self.logic.phase != "IDLE":
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
            f"nav2 idle {self._now() - self.last_ext_cmd > self.params.quiet_s}, face "
            + (f"d={f[0]:.2f} yaw={math.degrees(f[1]):+.1f} len={f[6]:.2f}" if f else f"NOT FOUND ({self.last_why})"))

    # ---------- ROS 배선 ----------
    def _publish_status(self, phase, detail):
        self.status_pub.publish(String(data=json.dumps({"phase": phase, "detail": detail}, ensure_ascii=False)))
        self.get_logger().info(f"[{phase}] {detail}")

    def _start(self, run_id):
        if not self.logic.start(run_id, self._now()):
            self.get_logger().warning(f"start '{run_id}' 무시: 이미 {self.logic.phase}")
        self._flush_events()

    def _flush_events(self):
        for phase, detail in self.logic.events:
            if phase in ("DONE", "FAILED"):
                self.result_pub.publish(String(data=json.dumps(self.logic.result)))
            self._publish_status(phase, detail)
        self.logic.events.clear()

    def _pub(self, v, w):
        t = Twist(); t.linear.x = float(v); t.angular.z = float(w); self.cmd.publish(t)

    def _tick(self):
        now = self._now()
        if not self.logic.active():
            if self.auto and self.logic.phase == "IDLE" and self.amcl_xy is not None:
                near = math.hypot(self.amcl_xy[0] - self.arm[0], self.amcl_xy[1] - self.arm[1]) < self.arm_r
                idle = now - self.last_ext_cmd > self.params.quiet_s
                fresh = self.face is not None and now - self.face_stamp < self.params.fresh_s
                if near and idle and fresh:
                    if self.near_since is None:
                        self.near_since = now
                    elif now - self.near_since > 1.0:
                        self.auto_seq += 1
                        self._start(f"auto-{self.auto_seq}")
                else:
                    self.near_since = None
            return
        v, w = self.logic.update(now, self.face, now - self.face_stamp, self.wz, self.vx,
                                 self.odom_pose, now - self.last_ext_cmd)
        self._pub(v, w)
        self._flush_events()


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
