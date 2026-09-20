"""Odometry-closed /cmd_vel escape sequence for smart_farm_nav2_01.usd.

Sequence: ARC_REVERSE (reverse while rotating turn_angle_rad, +CCW, so the
rear swings to the right and the robot ends parallel to the corridor)
-> stop -> FORWARD (forward_distance_m along the corridor centre line, which
is the line through the start pose in the post-turn travel direction; a
Stanley-style cross-track term steers back to the centre, then holds it)
-> stop -> exit.
drive_direction_sign maps "visible forward" onto the sign of linear.x.

Each phase ends when /chassis/odom shows the requested angle or distance,
not after a fixed time, so the result does not depend on the simulator's
real-time factor (measured about 0.27x on 고피).  A phase is aborted with a
zero Twist when odometry stops progressing for stall_timeout_s, when the
generous phase timeout expires, or when the reverse path exceeds
reverse_max_distance_m.
"""

import math
import sys
from enum import Enum, auto
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class Phase(Enum):
    WAITING = auto()
    ARC_REVERSE = auto()
    ARC_STOP = auto()
    FORWARD = auto()
    COMPLETE = auto()
    ABORTED = auto()


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_odom(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class EscapeController(Node):
    def __init__(self) -> None:
        super().__init__("escape_controller")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/chassis/odom")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("reverse_speed_mps", 0.0)
        self.declare_parameter("turn_speed_radps", 0.0)
        self.declare_parameter("turn_angle_rad", 0.0)
        self.declare_parameter("reverse_max_distance_m", 1.0)
        self.declare_parameter("drive_direction_sign", -1.0)
        self.declare_parameter("forward_distance_m", 0.0)
        self.declare_parameter("forward_speed_mps", 0.0)
        self.declare_parameter("heading_hold_gain", 1.0)
        self.declare_parameter("heading_hold_max_radps", 0.3)
        self.declare_parameter("cross_track_gain", 1.5)
        self.declare_parameter("cross_track_max_angle_rad", 0.7)
        self.declare_parameter("settle_duration_s", 0.5)
        self.declare_parameter("odom_timeout_s", 1.0)
        self.declare_parameter("phase_timeout_factor", 10.0)
        self.declare_parameter("phase_timeout_min_s", 10.0)
        self.declare_parameter("stall_timeout_s", 4.0)
        self.declare_parameter("stall_min_progress", 0.01)
        self.declare_parameter("stop_hold_duration_s", 1.0)

        p = self.get_parameter
        self.cmd_vel_topic = p("cmd_vel_topic").value
        self.odom_topic = p("odom_topic").value
        self.auto_start = bool(p("auto_start").value)
        self.reverse_speed = float(p("reverse_speed_mps").value)
        self.turn_speed = float(p("turn_speed_radps").value)
        self.turn_angle = float(p("turn_angle_rad").value)
        self.reverse_max_distance = float(p("reverse_max_distance_m").value)
        # +1: linear.x > 0 drives the visible front forward.  -1: this scene's
        # carter moves visibly forward on linear.x < 0 (observed 2026-09-19).
        self.drive_sign = 1.0 if float(p("drive_direction_sign").value) >= 0.0 else -1.0
        self.forward_distance = float(p("forward_distance_m").value)
        self.forward_speed = float(p("forward_speed_mps").value)
        self.hold_gain = float(p("heading_hold_gain").value)
        self.hold_max = float(p("heading_hold_max_radps").value)
        self.xt_gain = float(p("cross_track_gain").value)
        self.xt_max_angle = float(p("cross_track_max_angle_rad").value)
        self.settle_duration = float(p("settle_duration_s").value)
        self.odom_timeout = float(p("odom_timeout_s").value)
        self.timeout_factor = float(p("phase_timeout_factor").value)
        self.timeout_min = float(p("phase_timeout_min_s").value)
        self.stall_timeout = float(p("stall_timeout_s").value)
        self.stall_min_progress = float(p("stall_min_progress").value)
        self.stop_hold = float(p("stop_hold_duration_s").value)

        self.command_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)
        self.timer = self.create_timer(0.05, self._tick)

        self.phase = Phase.WAITING
        self.phase_started_at: Optional[float] = None
        self.phase_start_xy: Tuple[float, float] = (0.0, 0.0)
        self.phase_timeout: float = 0.0
        self.last_odom_at: Optional[float] = None
        self.xy: Tuple[float, float] = (0.0, 0.0)
        self.yaw: float = 0.0
        self.initial_yaw: float = 0.0
        self.target_yaw: float = 0.0
        self.line_origin: Tuple[float, float] = (0.0, 0.0)   # corridor centre = start pose
        self.line_heading: float = 0.0                         # travel direction along corridor
        self.turn_accum: float = 0.0
        self._accumulate_turn = False
        self._progress_value = 0.0
        self._progress_at: float = 0.0
        self._invalid_reported = False
        self._finish_started_at: Optional[float] = None
        self.finished = False
        self.exit_code = 0

        self.get_logger().info(
            f"Prepared for {self.cmd_vel_topic}; waiting for {self.odom_topic}. "
            + ("auto_start=true: sequence starts on first fresh odometry."
               if self.auto_start else "auto_start=false: holding zero Twist only.")
        )

    # ---------- helpers ----------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _odom_callback(self, message: Odometry) -> None:
        yaw = yaw_from_odom(message)
        if self._accumulate_turn:
            self.turn_accum += wrap_angle(yaw - self.yaw)
        self.yaw = yaw
        self.xy = (message.pose.pose.position.x, message.pose.pose.position.y)
        self.last_odom_at = self._now()

    def _odom_is_fresh(self, now: float) -> bool:
        return self.last_odom_at is not None and now - self.last_odom_at <= self.odom_timeout

    def _distance_from_phase_start(self) -> float:
        return math.hypot(self.xy[0] - self.phase_start_xy[0], self.xy[1] - self.phase_start_xy[1])

    def _hold_heading(self, target_yaw: float) -> float:
        error = wrap_angle(target_yaw - self.yaw)
        return max(-self.hold_max, min(self.hold_max, self.hold_gain * error))

    def _travel_heading(self) -> float:
        """Direction the robot actually moves when driving 'visibly forward'."""
        return self.yaw if self.drive_sign > 0 else wrap_angle(self.yaw + math.pi)

    def _cross_track_error(self) -> float:
        """Signed lateral offset from the corridor centre line (+ = left of travel)."""
        dx = self.xy[0] - self.line_origin[0]
        dy = self.xy[1] - self.line_origin[1]
        return math.cos(self.line_heading) * dy - math.sin(self.line_heading) * dx

    def _along_track(self) -> float:
        dx = self.xy[0] - self.phase_start_xy[0]
        dy = self.xy[1] - self.phase_start_xy[1]
        return math.cos(self.line_heading) * dx + math.sin(self.line_heading) * dy

    def _follow_line(self) -> float:
        """Angular command that returns to the centre line, then holds its heading."""
        e = self._cross_track_error()
        correction = max(-self.xt_max_angle, min(self.xt_max_angle, math.atan(self.xt_gain * e)))
        desired = wrap_angle(self.line_heading - correction)
        error = wrap_angle(desired - self._travel_heading())
        return max(-self.hold_max, min(self.hold_max, self.hold_gain * error))

    def _publish(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular_z
        self.command_publisher.publish(message)

    def _parameters_valid(self) -> bool:
        return (
            self.reverse_speed > 0.0 and self.reverse_max_distance > 0.0
            and self.turn_angle != 0.0 and self.turn_speed > 0.0
            and self.forward_distance > 0.0 and self.forward_speed > 0.0
            and self.settle_duration >= 0.0 and self.odom_timeout > 0.0
            and self.hold_max >= 0.0 and self.stop_hold >= 0.0
        )

    def _pose_text(self) -> str:
        return f"x={self.xy[0]:.3f} y={self.xy[1]:.3f} yaw={math.degrees(self.yaw):.1f}deg"

    def _enter(self, phase: Phase, now: float, nominal_time: float = 0.0) -> None:
        self.phase = phase
        self.phase_started_at = now
        self.phase_start_xy = self.xy
        self.phase_timeout = max(self.timeout_min, self.timeout_factor * nominal_time)
        self._accumulate_turn = phase is Phase.ARC_REVERSE
        if phase is Phase.ARC_REVERSE:
            self.turn_accum = 0.0
        self._progress_value = 0.0
        self._progress_at = now
        self.get_logger().info(f"Phase {phase.name} at {self._pose_text()} (timeout {self.phase_timeout:.1f}s)")

    def _stalled(self, progress: float, now: float) -> bool:
        """True when `progress` (m or rad) has not grown for stall_timeout_s."""
        if progress - self._progress_value >= self.stall_min_progress:
            self._progress_value = progress
            self._progress_at = now
            return False
        return now - self._progress_at > self.stall_timeout

    def _abort(self, reason: str) -> None:
        if self.phase in (Phase.ABORTED, Phase.COMPLETE):
            return
        self.phase = Phase.ABORTED
        self._accumulate_turn = False
        self.exit_code = 2
        self._publish()
        self.get_logger().error(f"Escape sequence ABORTED at {self._pose_text()}: {reason}")

    # ---------- main loop ----------
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
                self.get_logger().error(
                    "Motion parameters unset or invalid; no motion command will be sent."
                )
                self._invalid_reported = True
            return

        if not self._odom_is_fresh(now):
            if self.phase is Phase.WAITING:
                return  # still waiting for the first odometry
            self._abort(f"no fresh Odometry on {self.odom_topic} within {self.odom_timeout:.2f}s")
            return

        if self.phase is Phase.WAITING:
            self.initial_yaw = self.yaw
            self.target_yaw = wrap_angle(self.initial_yaw + self.turn_angle)
            self.line_origin = self.xy
            self.line_heading = wrap_angle(self._travel_heading() + self.turn_angle)
            self.get_logger().info(
                f"Start pose {self._pose_text()}; target heading after arc "
                f"{math.degrees(self.target_yaw):.1f}deg; corridor line through "
                f"({self.line_origin[0]:.2f}, {self.line_origin[1]:.2f}) "
                f"dir {math.degrees(self.line_heading):.1f}deg"
            )
            self._enter(Phase.ARC_REVERSE, now, abs(self.turn_angle) / self.turn_speed)

        elapsed = now - (self.phase_started_at or now)

        if self.phase is Phase.ARC_REVERSE:
            self._publish(-self.drive_sign * self.reverse_speed,
                          math.copysign(self.turn_speed, self.turn_angle))
            if abs(self.turn_accum) >= abs(self.turn_angle):
                self._enter(Phase.ARC_STOP, now)
            elif self._distance_from_phase_start() > self.reverse_max_distance:
                self._abort("ARC_REVERSE exceeded reverse_max_distance_m before reaching turn_angle_rad")
            elif self._stalled(abs(self.turn_accum), now):
                self._abort(f"ARC_REVERSE stalled: heading unchanged for {self.stall_timeout:.1f}s")
            elif elapsed > self.phase_timeout:
                self._abort("ARC_REVERSE timeout")

        elif self.phase is Phase.ARC_STOP:
            self._publish()
            if elapsed >= self.settle_duration:
                self._enter(Phase.FORWARD, now, self.forward_distance / self.forward_speed)
                self.get_logger().info(f"  cross-track at FORWARD start {self._cross_track_error():+.3f} m")

        elif self.phase is Phase.FORWARD:
            self._publish(self.drive_sign * self.forward_speed, self._follow_line())
            distance = self._along_track()
            if distance >= self.forward_distance:
                self._publish()
                self.phase = Phase.COMPLETE
                self.get_logger().info(
                    f"Escape sequence COMPLETE at {self._pose_text()}; "
                    f"cross-track {self._cross_track_error():+.3f} m"
                )
            elif self._stalled(distance, now):
                self._abort(f"FORWARD stalled: no progress for {self.stall_timeout:.1f}s")
            elif elapsed > self.phase_timeout:
                self._abort("FORWARD timeout")

    def publish_stop_burst(self, count: int = 5) -> None:
        """Repeat the zero Twist so the simulator's last-value subscriber holds zero."""
        for _ in range(count):
            if not rclpy.ok(context=self.context):
                return
            self._publish()
            rclpy.spin_once(self, timeout_sec=0.05)

    def destroy_node(self) -> bool:
        if rclpy.ok(context=self.context):
            try:
                self._publish()
            except Exception:  # middleware may already be closed during teardown
                pass
        return super().destroy_node()


def main() -> None:
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = EscapeController()
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
