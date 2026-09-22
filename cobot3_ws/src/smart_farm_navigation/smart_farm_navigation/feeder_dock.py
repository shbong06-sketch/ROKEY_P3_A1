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
  goal: base_link `standoff_m` (default 1.00 m) in front of the face on the normal through the face centre,
        rear square to the face  ->  world (-2.19, -2.60, 90 deg) for the v011 scene.
"""

import json
import math
import time

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String

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
        d("auto_start", True)
        d("arm_x", -2.19); d("arm_y", -1.55); d("arm_radius_m", 0.6)     # FEEDER_APPROACH (map)
        d("standoff_m", 0.90)                     # base_link -> face distance at the dock (v011 face y -3.60 -> base_link y -2.70; rear edge 0.29 m from the face, arm base 0.48 m)
        d("face_min_len_m", 0.6); d("face_max_len_m", 1.6)
        d("search_x", [-3.4, -0.5]); d("search_y", [-1.3, 1.3])
        d("reverse_speed_mps", 0.15); d("creep_speed_mps", 0.05)
        d("turn_speed_radps", 0.35); d("turn_min_radps", 0.08)
        d("yaw_tol_deg", 1.5); d("dist_tol_m", 0.03); d("lat_tol_m", 0.04)
        d("k_yaw", 1.5); d("k_lat", 1.2); d("timeout_s", 120.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.auto = bool(p("auto_start")); self.arm = (float(p("arm_x")), float(p("arm_y"))); self.arm_r = float(p("arm_radius_m"))
        self.standoff = float(p("standoff_m")); self.len_lim = (float(p("face_min_len_m")), float(p("face_max_len_m")))
        self.sx = [float(v) for v in p("search_x")]; self.sy = [float(v) for v in p("search_y")]
        self.v_rev = float(p("reverse_speed_mps")); self.v_creep = float(p("creep_speed_mps"))
        self.w_turn = float(p("turn_speed_radps")); self.w_min = float(p("turn_min_radps"))
        self.yaw_tol = math.radians(float(p("yaw_tol_deg"))); self.dist_tol = float(p("dist_tol_m")); self.lat_tol = float(p("lat_tol_m"))
        self.k_yaw = float(p("k_yaw")); self.k_lat = float(p("k_lat")); self.timeout = float(p("timeout_s"))

        self.cmd = self.create_publisher(Twist, p("cmd_vel_topic"), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "/feeder_dock/status", latched)
        self.result_pub = self.create_publisher(String, "/feeder_dock/result", latched)
        self.create_subscription(LaserScan, p("scan_topic"), self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Empty, "/feeder_dock/start", lambda m: self._start("manual"), 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)

        self.face = None           # (d, yaw_err, lat, cx, cy, n_pts, length)
        self.last_why = ""
        self.face_stamp = 0.0
        self.amcl_xy = None; self.last_ext_cmd = 0.0; self.near_since = None
        self.phase = "IDLE"; self.t_phase = 0.0; self.t_start = 0.0; self.done = False
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
    def _on_cmd(self, m):
        if self.phase in ("IDLE", "DONE", "FAILED") and (abs(m.linear.x) > 0.01 or abs(m.angular.z) > 0.01):
            self.last_ext_cmd = self._now()

    def _report(self):
        if self.phase != "IDLE":
            return
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
        self.status_pub.publish(String(data=json.dumps({"phase": phase, "detail": detail}, ensure_ascii=False)))
        self.get_logger().info(f"[{phase}] {detail}")

    def _start(self, how):
        if self.phase not in ("IDLE", "DONE", "FAILED"):
            return
        self.done = False; self.t_start = self._now()
        self._status("ALIGN_TO_GOAL", f"started ({how})")

    def _finish(self, ok, reason):
        self._pub(0.0, 0.0)
        f = self.face
        res = {"status": "SUCCEEDED" if ok else "FAILED", "reason": reason,
               "face_dist_m": round(f[0], 3) if f else None, "yaw_err_deg": round(math.degrees(f[1]), 2) if f else None,
               "lat_m": round(f[2], 3) if f else None}
        self.result_pub.publish(String(data=json.dumps(res)))
        self._status("DONE" if ok else "FAILED", json.dumps(res))
        self.done = True

    def _pub(self, v, w):
        t = Twist(); t.linear.x = float(v); t.angular.z = float(w); self.cmd.publish(t)

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
                        self._start("auto: near FEEDER_APPROACH and Nav2 idle")
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
            if g_dist < 0.12:
                self._status("SQUARE", f"already at goal point (|G|={g_dist:.2f})"); return
            if abs(bearing_rear) < self.yaw_tol:
                self._status("REVERSE", f"G behind at {g_dist:.2f} m; face d={dist:.2f} yaw={math.degrees(yaw_err):+.1f} lat={lat:+.2f}")
            else:
                self._pub(0.0, self._turn_cmd(bearing_rear))
        elif self.phase == "REVERSE":
            if g_dist < 0.06 or dist <= self.standoff + 0.02:
                self._pub(0.0, 0.0); self._status("SQUARE", f"reached G (|G|={g_dist:.2f}, d={dist:.2f})"); return
            v = self.v_rev if g_dist > 0.3 else max(self.v_creep, self.v_rev * g_dist / 0.3)
            # reversing: steer so the rear keeps pointing at G (sign flips because we move backward)
            w = max(min(self.k_lat * bearing_rear, 0.5), -0.5)   # G left of the rear axis -> bearing < 0 -> turn CW so the rear swings left
            self._pub(-v, w)
        elif self.phase == "SQUARE":
            if abs(yaw_err) < self.yaw_tol:
                self._status("CREEP", f"square (yaw {math.degrees(yaw_err):+.1f}); d={dist:.3f} target {self.standoff:.2f}")
            else:
                self._pub(0.0, self._turn_cmd(yaw_err))
        elif self.phase == "CREEP":
            e = dist - self.standoff
            if abs(e) < self.dist_tol:
                self._pub(0.0, 0.0)
                self._finish(True, "NONE"); return
            v = math.copysign(self.v_creep, -e)               # e>0: too far -> reverse (negative x)
            self._pub(v, max(min(self.k_yaw * yaw_err, 0.15), -0.15))
        if now - self.t_phase > 45.0:
            self._finish(False, f"PHASE_TIMEOUT_{self.phase}")


def main() -> None:
    rclpy.init()
    node = FeederDock()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try: node._pub(0.0, 0.0)
        except Exception: pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
