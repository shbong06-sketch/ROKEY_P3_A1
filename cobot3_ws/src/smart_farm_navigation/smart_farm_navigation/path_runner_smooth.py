"""Smooth waypoint path runner: straight corridor exit, then one continuous curve to an
approach point on the final axis, then a short straight approach and stop.

Waypoints (start frame, X = forward at start, Y = left) come from path_runner.yaml-style
parameters.  All waypoints except the last are followed as straight legs (pure pursuit);
from the last straight waypoint a cubic Bezier curve leads to the approach point
`final - approach_length_m * final_heading`, and the final leg is a straight line-hold
approach so the robot arrives already aligned with final_heading_deg.

    ros2 launch smart_farm_navigation path_smooth.launch.py auto_start:=true
"""

import math
import sys
from enum import Enum, auto
from typing import List, Tuple

import rclpy
from rclpy.signals import SignalHandlerOptions

from smart_farm_navigation.escape_controller import wrap_angle
from smart_farm_navigation.path_runner import PathRunner


class SPhase(Enum):
    WAITING = auto()
    TRACK = auto()        # pure pursuit along the dense path (straight legs + curve)
    APPROACH = auto()     # straight line hold onto the final point
    FINAL_TURN = auto()
    COMPLETE = auto()
    ABORTED = auto()


def bezier(p0, p1, p2, p3, n: int) -> List[Tuple[float, float]]:
    pts = []
    for i in range(n + 1):
        t = i / n
        a, b, c, d = (1 - t) ** 3, 3 * (1 - t) ** 2 * t, 3 * (1 - t) * t ** 2, t ** 3
        pts.append((a * p0[0] + b * p1[0] + c * p2[0] + d * p3[0],
                    a * p0[1] + b * p1[1] + c * p2[1] + d * p3[1]))
    return pts


class SmoothPathRunner(PathRunner):
    def __init__(self) -> None:
        super().__init__()
        d = self.declare_parameter
        d("approach_length_m", 0.8)
        d("lookahead_m", 0.5)
        d("curve_speed_mps", 0.25)
        d("curve_tangent_scale", 0.5)
        d("path_step_m", 0.05)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.approach_len = float(p("approach_length_m"))
        self.lookahead = float(p("lookahead_m"))
        self.curve_speed = float(p("curve_speed_mps"))
        self.tangent_scale = float(p("curve_tangent_scale"))
        self.step = float(p("path_step_m"))
        self.sphase = SPhase.WAITING
        self.path: List[Tuple[float, float]] = []
        self.path_is_curve: List[bool] = []
        self.path_idx = 0
        self.approach_start: Tuple[float, float] = (0.0, 0.0)
        self.final_target: Tuple[float, float] = (0.0, 0.0)
        self.final_odom_heading = 0.0

    # ---------- path construction ----------
    def _build_path(self) -> None:
        start = self.xy
        h0 = self.start_heading
        c, s = math.cos(h0), math.sin(h0)
        to_odom = lambda X, Y: (start[0] + c * X - s * Y, start[1] + s * X + c * Y)  # noqa: E731
        pts_sf = list(zip(self.wp_x, self.wp_y))
        final_sf = pts_sf[-1]
        fh = self.final_heading                       # start-frame final heading
        appr_sf = (final_sf[0] - self.approach_len * math.cos(fh), final_sf[1] - self.approach_len * math.sin(fh))
        straight_sf = [(0.0, 0.0)] + pts_sf[:-1]
        dense: List[Tuple[float, float]] = []
        curve_flag: List[bool] = []
        for a, b in zip(straight_sf[:-1], straight_sf[1:]):
            n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / self.step))
            for i in range(n):
                t = i / n
                dense.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)); curve_flag.append(False)
        exit_pt = straight_sf[-1]
        if len(straight_sf) >= 2:
            prev = straight_sf[-2]
            t0 = math.atan2(exit_pt[1] - prev[1], exit_pt[0] - prev[0])
        else:
            t0 = 0.0
        k = self.tangent_scale * math.hypot(appr_sf[0] - exit_pt[0], appr_sf[1] - exit_pt[1])
        p1 = (exit_pt[0] + k * math.cos(t0), exit_pt[1] + k * math.sin(t0))
        p2 = (appr_sf[0] - k * math.cos(fh), appr_sf[1] - k * math.sin(fh))
        n = max(20, int(2.0 * math.hypot(appr_sf[0] - exit_pt[0], appr_sf[1] - exit_pt[1]) / self.step))
        for q in bezier(exit_pt, p1, p2, appr_sf, n):
            dense.append(q); curve_flag.append(True)
        self.path = [to_odom(X, Y) for X, Y in dense]
        self.path_is_curve = curve_flag
        self.approach_start = to_odom(*appr_sf)
        self.final_target = to_odom(*final_sf)
        self.final_odom_heading = wrap_angle(h0 + fh)
        self.path_idx = 0
        self.get_logger().info(
            f"path: {len(self.path)} points, curve from ({exit_pt[0]:.2f},{exit_pt[1]:.2f}) to approach "
            f"({appr_sf[0]:.2f},{appr_sf[1]:.2f}), final ({final_sf[0]:.2f},{final_sf[1]:.2f}) heading {math.degrees(fh):.0f}deg"
        )

    # ---------- pure pursuit ----------
    def _advance_index(self) -> None:
        best_i, best_d = self.path_idx, float("inf")
        for i in range(self.path_idx, min(len(self.path), self.path_idx + 40)):
            d = math.hypot(self.path[i][0] - self.xy[0], self.path[i][1] - self.xy[1])
            if d < best_d:
                best_i, best_d = i, d
        self.path_idx = best_i

    def _lookahead_point(self) -> Tuple[float, float]:
        for i in range(self.path_idx, len(self.path)):
            if math.hypot(self.path[i][0] - self.xy[0], self.path[i][1] - self.xy[1]) >= self.lookahead:
                return self.path[i]
        return self.approach_start

    def _path_distance(self) -> float:
        p = self.path[self.path_idx]
        return math.hypot(p[0] - self.xy[0], p[1] - self.xy[1])

    def _senter(self, phase: SPhase, now: float, nominal: float = 0.0) -> None:
        self.sphase = phase
        self.phase_started_at = now
        self.phase_timeout = max(self.timeout_min, self.timeout_factor * nominal)
        self._progress_value = 0.0
        self._progress_at = now
        self.get_logger().info(f"Phase {phase.name} at {self._pose_text()}")

    def _sabort(self, reason: str) -> None:
        if self.sphase in (SPhase.ABORTED, SPhase.COMPLETE):
            return
        self.sphase = SPhase.ABORTED
        self.exit_code = 2
        self._publish()
        self.get_logger().error(f"Path ABORTED at {self._pose_text()}: {reason}")

    def _tick(self) -> None:
        now = self._now()
        if self.sphase in (SPhase.COMPLETE, SPhase.ABORTED):
            self._publish()
            if self._finish_started_at is None:
                self._finish_started_at = now
            elif now - self._finish_started_at >= self.stop_hold:
                self.finished = True
            return
        if not self.auto_start:
            self._publish(); return
        if not self._parameters_valid() or len(self.wp_x) < 2:
            self._publish()
            if not self._invalid_reported:
                self.get_logger().error("need at least 2 waypoints (corridor exit + final) and valid speeds")
                self._invalid_reported = True
            return
        if not self._odom_is_fresh(now):
            if self.sphase is not SPhase.WAITING:
                self._sabort("no fresh Odometry")
            return

        if self.sphase is SPhase.WAITING:
            self.start_xy = self.xy
            self.start_heading = self._travel_heading()
            self._build_path()
            total = len(self.path) * self.step
            self._senter(SPhase.TRACK, now, total / self.curve_speed)

        elapsed = now - (self.phase_started_at or now)

        if self.sphase is SPhase.TRACK:
            self._advance_index()
            dist_to_appr = math.hypot(self.approach_start[0] - self.xy[0], self.approach_start[1] - self.xy[1])
            if self.path_idx >= len(self.path) - 1 or dist_to_appr <= self.wp_tol * 2:
                self.get_logger().info(f"  approach point reached at {self._pose_text()}")
                self.seg_from = self.xy
                self.seg_to = self.final_target
                self.seg_heading = self.final_odom_heading
                self.seg_length = (math.cos(self.seg_heading) * (self.seg_to[0] - self.seg_from[0])
                                   + math.sin(self.seg_heading) * (self.seg_to[1] - self.seg_from[1]))
                self._senter(SPhase.APPROACH, now, max(self.seg_length, 0.1) / self.drive_speed)
                return
            if self._path_distance() > self.max_cross_track:
                self._sabort(f"off path by {self._path_distance():.2f} m")
                return
            lx, ly = self._lookahead_point()
            alpha = wrap_angle(math.atan2(ly - self.xy[1], lx - self.xy[0]) - self._travel_heading())
            v = self.curve_speed if self.path_is_curve[self.path_idx] else self.drive_speed
            v = max(self.drive_min_speed, min(v, dist_to_appr * self.drive_speed / max(self.slow_zone, 1e-3)))
            w = 2.0 * v * math.sin(alpha) / self.lookahead
            w = max(-self.turn_speed, min(self.turn_speed, w))
            self._publish(self.drive_sign * v, w)
            if self._stalled(float(self.path_idx) * self.step, now):
                self._sabort("track stalled")
            elif elapsed > self.phase_timeout:
                self._sabort("track timeout")

        elif self.sphase is SPhase.APPROACH:
            remaining = self.seg_length - self._along_track()
            if remaining <= self.wp_tol:
                self._publish()
                self.turn_target = self.final_odom_heading
                self.turn_initial_err = abs(wrap_angle(self.turn_target - self._travel_heading()))
                self.get_logger().info(f"  final point reached at {self._pose_text()}; cross-track {self._cross_track():+.3f} m")
                self._senter(SPhase.FINAL_TURN, now, self.turn_initial_err / self.turn_speed)
                return
            v = max(self.drive_min_speed, min(self.drive_speed, remaining * self.drive_speed / max(self.slow_zone, 1e-3)))
            self._publish(self.drive_sign * v, self._follow_line())
            if self._stalled(self._along_track(), now):
                self._sabort("approach stalled")
            elif elapsed > self.phase_timeout:
                self._sabort("approach timeout")

        elif self.sphase is SPhase.FINAL_TURN:
            err = wrap_angle(self.turn_target - self._travel_heading())
            if abs(err) <= self.heading_tol:
                self._publish()
                self.sphase = SPhase.COMPLETE
                self.get_logger().info(f"Path COMPLETE at {self._pose_text()}; heading error {math.degrees(err):+.1f}deg")
                return
            self._publish(0.0, self._turn_command(err))
            if elapsed > self.phase_timeout:
                self._sabort("final turn timeout")


def main() -> None:
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SmoothPathRunner()
    code = 0
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        code = node.exit_code
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted; publishing zero Twist.")
        node.publish_stop_burst()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
