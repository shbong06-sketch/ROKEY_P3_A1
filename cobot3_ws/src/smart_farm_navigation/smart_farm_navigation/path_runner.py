"""Waypoint path runner for smartfarm_v1.usd (and any /cmd_vel + odom scene).

Waypoints are given in the START frame: X = visible forward direction at
start, Y = visible left.  For each segment the robot turns in place to the
segment heading, drives the segment while holding the segment line
(Stanley-style cross-track correction), settles, then continues.  After the
last waypoint it turns in place to final_heading_deg (relative to the start
heading) and stops.  Every phase ends on /chassis/odom progress, never on
elapsed time, and aborts with a zero Twist on stall or timeout.
"""

import math
import sys
from enum import Enum, auto
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from smart_farm_navigation.geometry import wrap_angle, yaw_from_odom


class Phase(Enum):
    WAITING = auto()
    TURN = auto()
    DRIVE = auto()
    SETTLE = auto()
    FINAL_TURN = auto()
    COMPLETE = auto()
    ABORTED = auto()


class PathRunner(Node):
    def __init__(self) -> None:
        super().__init__("path_runner")
        d = self.declare_parameter
        d("cmd_vel_topic", "/cmd_vel")
        d("odom_topic", "/chassis/odom")
        d("auto_start", False)
        d("drive_direction_sign", -1.0)
        d("waypoints_x", [0.0])
        d("waypoints_y", [0.0])
        d("final_heading_deg", 0.0)
        d("drive_speed_mps", 0.4)
        d("drive_min_speed_mps", 0.08)
        d("slow_zone_m", 0.5)
        d("turn_speed_radps", 0.5)
        d("turn_min_speed_radps", 0.1)
        d("turn_gain", 1.5)
        d("heading_tolerance_deg", 2.0)
        d("waypoint_tolerance_m", 0.05)
        d("heading_hold_gain", 1.0)
        d("heading_hold_max_radps", 0.3)
        d("cross_track_gain", 1.5)
        d("cross_track_max_angle_rad", 0.7)
        d("max_cross_track_m", 0.6)
        d("settle_duration_s", 0.5)
        d("odom_timeout_s", 1.0)
        d("phase_timeout_factor", 10.0)
        d("phase_timeout_min_s", 30.0)
        d("stall_timeout_s", 4.0)
        d("stall_min_progress", 0.01)
        d("stop_hold_duration_s", 1.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cmd_vel_topic = p("cmd_vel_topic")
        self.odom_topic = p("odom_topic")
        self.auto_start = bool(p("auto_start"))
        self.drive_sign = 1.0 if float(p("drive_direction_sign")) >= 0.0 else -1.0
        self.wp_x = [float(v) for v in p("waypoints_x")]
        self.wp_y = [float(v) for v in p("waypoints_y")]
        self.final_heading = math.radians(float(p("final_heading_deg")))
        self.drive_speed = float(p("drive_speed_mps"))
        self.drive_min_speed = float(p("drive_min_speed_mps"))
        self.slow_zone = float(p("slow_zone_m"))
        self.turn_speed = float(p("turn_speed_radps"))
        self.turn_min_speed = float(p("turn_min_speed_radps"))
        self.turn_gain = float(p("turn_gain"))
        self.heading_tol = math.radians(float(p("heading_tolerance_deg")))
        self.wp_tol = float(p("waypoint_tolerance_m"))
        self.hold_gain = float(p("heading_hold_gain"))
        self.hold_max = float(p("heading_hold_max_radps"))
        self.xt_gain = float(p("cross_track_gain"))
        self.xt_max_angle = float(p("cross_track_max_angle_rad"))
        self.max_cross_track = float(p("max_cross_track_m"))
        self.settle_duration = float(p("settle_duration_s"))
        self.odom_timeout = float(p("odom_timeout_s"))
        self.timeout_factor = float(p("phase_timeout_factor"))
        self.timeout_min = float(p("phase_timeout_min_s"))
        self.stall_timeout = float(p("stall_timeout_s"))
        self.stall_min_progress = float(p("stall_min_progress"))
        self.stop_hold = float(p("stop_hold_duration_s"))

        self.command_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)
        self.timer = self.create_timer(0.05, self._tick)

        self.phase = Phase.WAITING
        self.phase_started_at: Optional[float] = None
        self.phase_timeout = 0.0
        self.last_odom_at: Optional[float] = None
        self.xy: Tuple[float, float] = (0.0, 0.0)
        self.yaw = 0.0
        self.start_xy: Tuple[float, float] = (0.0, 0.0)
        self.start_heading = 0.0            # travel heading at start (odom frame)
        self.targets: List[Tuple[float, float]] = []   # waypoints in odom frame
        self.seg_index = -1
        self.seg_from: Tuple[float, float] = (0.0, 0.0)
        self.seg_to: Tuple[float, float] = (0.0, 0.0)
        self.seg_heading = 0.0
        self.seg_length = 0.0
        self.turn_target = 0.0
        self.turn_initial_err = 0.0
        self._progress_value = 0.0
        self._progress_at = 0.0
        self._invalid_reported = False
        self._finish_started_at: Optional[float] = None
        self.finished = False
        self.exit_code = 0

        self.get_logger().info(
            f"path_runner ready on {self.cmd_vel_topic}; {len(self.wp_x)} waypoint(s); "
            + ("auto_start=true" if self.auto_start else "auto_start=false: holding zero Twist")
        )

    # ---------- helpers ----------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _odom_callback(self, m: Odometry) -> None:
        self.yaw = yaw_from_odom(m)
        self.xy = (m.pose.pose.position.x, m.pose.pose.position.y)
        self.last_odom_at = self._now()

    def _odom_is_fresh(self, now: float) -> bool:
        return self.last_odom_at is not None and now - self.last_odom_at <= self.odom_timeout

    def _travel_heading(self) -> float:
        return self.yaw if self.drive_sign > 0 else wrap_angle(self.yaw + math.pi)

    def _publish(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.command_publisher.publish(msg)

    def _parameters_valid(self) -> bool:
        return (
            len(self.wp_x) >= 1 and len(self.wp_x) == len(self.wp_y)
            and self.drive_speed > 0.0 and self.turn_speed > 0.0
            and 0.0 < self.drive_min_speed <= self.drive_speed
            and 0.0 < self.turn_min_speed <= self.turn_speed
            and self.wp_tol > 0.0 and self.heading_tol > 0.0 and self.odom_timeout > 0.0
        )

    def _pose_text(self) -> str:
        return (f"odom x={self.xy[0]:.3f} y={self.xy[1]:.3f} "
                f"heading={math.degrees(self._travel_heading()):.1f}deg")

    def _cross_track(self) -> float:
        dx = self.xy[0] - self.seg_from[0]
        dy = self.xy[1] - self.seg_from[1]
        return math.cos(self.seg_heading) * dy - math.sin(self.seg_heading) * dx

    def _along_track(self) -> float:
        dx = self.xy[0] - self.seg_from[0]
        dy = self.xy[1] - self.seg_from[1]
        return math.cos(self.seg_heading) * dx + math.sin(self.seg_heading) * dy

    def _follow_line(self) -> float:
        e = self._cross_track()
        correction = max(-self.xt_max_angle, min(self.xt_max_angle, math.atan(self.xt_gain * e)))
        desired = wrap_angle(self.seg_heading - correction)
        err = wrap_angle(desired - self._travel_heading())
        return max(-self.hold_max, min(self.hold_max, self.hold_gain * err))

    def _turn_command(self, err: float) -> float:
        w = max(-self.turn_speed, min(self.turn_speed, self.turn_gain * err))
        if abs(w) < self.turn_min_speed:
            w = math.copysign(self.turn_min_speed, err)
        return w

    def _stalled(self, progress: float, now: float) -> bool:
        if progress - self._progress_value >= self.stall_min_progress:
            self._progress_value = progress
            self._progress_at = now
            return False
        return now - self._progress_at > self.stall_timeout

    def _enter(self, phase: Phase, now: float, nominal_time: float = 0.0) -> None:
        self.phase = phase
        self.phase_started_at = now
        self.phase_timeout = max(self.timeout_min, self.timeout_factor * nominal_time)
        self._progress_value = 0.0
        self._progress_at = now
        self.get_logger().info(f"Phase {phase.name} [seg {self.seg_index + 1}/{len(self.targets)}] at {self._pose_text()}")

    def _abort(self, reason: str) -> None:
        if self.phase in (Phase.ABORTED, Phase.COMPLETE):
            return
        self.phase = Phase.ABORTED
        self.exit_code = 2
        self._publish()
        self.get_logger().error(f"Path ABORTED at {self._pose_text()}: {reason}")

    # ---------- sequencing ----------
    def _start(self, now: float) -> None:
        self.start_xy = self.xy
        self.start_heading = self._travel_heading()
        c, s = math.cos(self.start_heading), math.sin(self.start_heading)
        self.targets = [
            (self.start_xy[0] + c * X - s * Y, self.start_xy[1] + s * X + c * Y)
            for X, Y in zip(self.wp_x, self.wp_y)
        ]
        self.get_logger().info(
            f"Start {self._pose_text()}; waypoints(start frame)="
            + ", ".join(f"({X:.2f},{Y:.2f})" for X, Y in zip(self.wp_x, self.wp_y))
            + f"; final heading {math.degrees(self.final_heading):.0f}deg"
        )
        self.seg_index = -1
        self._begin_next_segment(now)

    def _begin_next_segment(self, now: float) -> None:
        self.seg_index += 1
        if self.seg_index >= len(self.targets):
            self.turn_target = wrap_angle(self.start_heading + self.final_heading)
            self.turn_initial_err = abs(wrap_angle(self.turn_target - self._travel_heading()))
            self._enter(Phase.FINAL_TURN, now, self.turn_initial_err / self.turn_speed)
            return
        self.seg_from = self.xy if self.seg_index == 0 else self.targets[self.seg_index - 1]
        self.seg_from = self.xy  # always start the line from where we actually are
        self.seg_to = self.targets[self.seg_index]
        dx, dy = self.seg_to[0] - self.seg_from[0], self.seg_to[1] - self.seg_from[1]
        self.seg_length = math.hypot(dx, dy)
        self.seg_heading = math.atan2(dy, dx) if self.seg_length > 1e-6 else self._travel_heading()
        self.turn_target = self.seg_heading
        self.turn_initial_err = abs(wrap_angle(self.turn_target - self._travel_heading()))
        self.get_logger().info(
            f"  segment {self.seg_index + 1}: to odom ({self.seg_to[0]:.2f}, {self.seg_to[1]:.2f}) "
            f"length {self.seg_length:.2f} m heading {math.degrees(self.seg_heading):.1f}deg"
        )
        self._enter(Phase.TURN, now, self.turn_initial_err / self.turn_speed)

    def _tick(self) -> None:
        now = self._now()

        if self.phase in (Phase.COMPLETE, Phase.ABORTED):
            self._publish()
            if self._finish_started_at is None:
                self._finish_started_at = now
            elif now - self._finish_started_at >= self.stop_hold:
                self.finished = True
            return
        if not self.auto_start:
            self._publish()
            return
        if not self._parameters_valid():
            self._publish()
            if not self._invalid_reported:
                self.get_logger().error("waypoints/speeds invalid; no motion command will be sent")
                self._invalid_reported = True
            return
        if not self._odom_is_fresh(now):
            if self.phase is not Phase.WAITING:
                self._abort(f"no fresh Odometry on {self.odom_topic} within {self.odom_timeout:.2f}s")
            return

        if self.phase is Phase.WAITING:
            self._start(now)

        elapsed = now - (self.phase_started_at or now)

        if self.phase in (Phase.TURN, Phase.FINAL_TURN):
            err = wrap_angle(self.turn_target - self._travel_heading())
            if abs(err) <= self.heading_tol:
                self._publish()
                if self.phase is Phase.FINAL_TURN:
                    self.phase = Phase.COMPLETE
                    self.get_logger().info(f"Path COMPLETE at {self._pose_text()}; heading error {math.degrees(err):+.1f}deg")
                else:
                    self._enter(Phase.DRIVE, now, self.seg_length / self.drive_speed)
                return
            self._publish(0.0, self._turn_command(err))
            if self._stalled(self.turn_initial_err - abs(err), now):
                self._abort("turn stalled: heading not changing")
            elif elapsed > self.phase_timeout:
                self._abort("turn timeout")

        elif self.phase is Phase.DRIVE:
            along = self._along_track()
            remaining = self.seg_length - along
            xt = self._cross_track()
            if abs(xt) > self.max_cross_track:
                self._abort(f"cross-track {xt:+.2f} m exceeds max_cross_track_m")
                return
            if remaining <= self.wp_tol:
                self._publish()
                self.get_logger().info(f"  waypoint {self.seg_index + 1} reached at {self._pose_text()}; cross-track {xt:+.3f} m")
                self._enter(Phase.SETTLE, now)
                return
            v = max(self.drive_min_speed, min(self.drive_speed, remaining * self.drive_speed / max(self.slow_zone, 1e-3)))
            self._publish(self.drive_sign * v, self._follow_line())
            if self._stalled(along, now):
                self._abort("drive stalled: no progress along segment")
            elif elapsed > self.phase_timeout:
                self._abort("drive timeout")

        elif self.phase is Phase.SETTLE:
            self._publish()
            if elapsed >= self.settle_duration:
                self._begin_next_segment(now)

    def publish_stop_burst(self, count: int = 5) -> None:
        for _ in range(count):
            if not rclpy.ok(context=self.context):
                return
            self._publish()
            rclpy.spin_once(self, timeout_sec=0.05)

    def destroy_node(self) -> bool:
        if rclpy.ok(context=self.context):
            try:
                self._publish()
            except Exception:
                pass
        return super().destroy_node()


def main() -> None:
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PathRunner()
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
