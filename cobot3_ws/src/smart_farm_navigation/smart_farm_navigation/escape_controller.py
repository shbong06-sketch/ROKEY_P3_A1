"""Timed /cmd_vel escape sequence for smart_farm_nav2_01.usd.

The scene already subscribes to /cmd_vel and publishes /chassis/odom.  This
node deliberately does not publish a command until all motion parameters have
been explicitly supplied as positive values.  That prevents unmeasured scene
dimensions from becoming hidden motion defaults.
"""

from enum import Enum, auto
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.exceptions import RCLError
from rclpy.node import Node


class Phase(Enum):
    WAITING = auto()
    REVERSE = auto()
    REVERSE_STOP = auto()
    TURN_CCW = auto()
    TURN_STOP = auto()
    FORWARD = auto()
    COMPLETE = auto()
    ABORTED = auto()


class EscapeController(Node):
    """Publishes reverse, counter-clockwise turn, and forward commands once."""

    def __init__(self) -> None:
        super().__init__("escape_controller")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/chassis/odom")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("reverse_speed_mps", 0.0)
        self.declare_parameter("reverse_duration_s", 0.0)
        self.declare_parameter("ccw_angular_speed_radps", 0.0)
        self.declare_parameter("turn_duration_s", 0.0)
        self.declare_parameter("forward_speed_mps", 0.0)
        self.declare_parameter("forward_duration_s", 0.0)
        self.declare_parameter("settle_duration_s", 0.5)
        self.declare_parameter("odom_timeout_s", 1.0)

        self.cmd_vel_topic = self._string_parameter("cmd_vel_topic")
        self.odom_topic = self._string_parameter("odom_topic")
        self.auto_start = self._bool_parameter("auto_start")
        self.reverse_speed = self._float_parameter("reverse_speed_mps")
        self.reverse_duration = self._float_parameter("reverse_duration_s")
        self.turn_speed = self._float_parameter("ccw_angular_speed_radps")
        self.turn_duration = self._float_parameter("turn_duration_s")
        self.forward_speed = self._float_parameter("forward_speed_mps")
        self.forward_duration = self._float_parameter("forward_duration_s")
        self.settle_duration = self._float_parameter("settle_duration_s")
        self.odom_timeout = self._float_parameter("odom_timeout_s")

        self.command_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)
        self.timer = self.create_timer(0.05, self._tick)

        self.phase = Phase.WAITING
        self.phase_started_at: Optional[float] = None
        self.last_odom_at: Optional[float] = None
        self.last_odom: Optional[Odometry] = None
        self._invalid_parameters_reported = False
        self._completion_reported = False

        self.get_logger().info(
            f"Prepared for {self.cmd_vel_topic}; waiting for {self.odom_topic}. "
            "Set auto_start:=true and all six motion parameters to run."
        )

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _bool_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _float_parameter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _odom_callback(self, message: Odometry) -> None:
        self.last_odom = message
        self.last_odom_at = self._now_seconds()

    def _motion_parameters_are_valid(self) -> bool:
        values = (
            self.reverse_speed,
            self.reverse_duration,
            self.turn_speed,
            self.turn_duration,
            self.forward_speed,
            self.forward_duration,
        )
        return all(value > 0.0 for value in values) and self.settle_duration >= 0.0 and self.odom_timeout > 0.0

    def _odom_is_fresh(self, now: float) -> bool:
        return self.last_odom_at is not None and now - self.last_odom_at <= self.odom_timeout

    def _publish(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular_z
        self.command_publisher.publish(message)

    def _enter(self, phase: Phase, now: float) -> None:
        self.phase = phase
        self.phase_started_at = now
        self.get_logger().info(f"Phase: {phase.name}")

    def _abort(self, reason: str) -> None:
        if self.phase in (Phase.ABORTED, Phase.COMPLETE):
            return
        self.phase = Phase.ABORTED
        self._publish()
        self.get_logger().error(f"Escape sequence aborted: {reason}")

    def _tick(self) -> None:
        now = self._now_seconds()

        if self.phase in (Phase.COMPLETE, Phase.ABORTED):
            self._publish()
            if self.phase is Phase.COMPLETE and not self._completion_reported:
                self.get_logger().info("Escape sequence complete; zero Twist is being held.")
                self._completion_reported = True
            return

        if not self.auto_start:
            self._publish()
            return

        if not self._motion_parameters_are_valid():
            self._publish()
            if not self._invalid_parameters_reported:
                self.get_logger().error(
                    "Motion parameters are unset. Supply positive reverse/turn/forward "
                    "speeds and durations; no command will be sent."
                )
                self._invalid_parameters_reported = True
            return

        if not self._odom_is_fresh(now):
            self._abort(f"No fresh Odometry on {self.odom_topic} within {self.odom_timeout:.2f}s")
            return

        if self.phase is Phase.WAITING:
            self._enter(Phase.REVERSE, now)

        elapsed = now - (self.phase_started_at or now)
        if self.phase is Phase.REVERSE:
            self._publish(linear_x=-self.reverse_speed)
            if elapsed >= self.reverse_duration:
                self._enter(Phase.REVERSE_STOP, now)
        elif self.phase is Phase.REVERSE_STOP:
            self._publish()
            if elapsed >= self.settle_duration:
                self._enter(Phase.TURN_CCW, now)
        elif self.phase is Phase.TURN_CCW:
            self._publish(angular_z=self.turn_speed)
            if elapsed >= self.turn_duration:
                self._enter(Phase.TURN_STOP, now)
        elif self.phase is Phase.TURN_STOP:
            self._publish()
            if elapsed >= self.settle_duration:
                self._enter(Phase.FORWARD, now)
        elif self.phase is Phase.FORWARD:
            self._publish(linear_x=self.forward_speed)
            if elapsed >= self.forward_duration:
                self._publish()
                self._enter(Phase.COMPLETE, now)

    def destroy_node(self) -> bool:
        # SIGINT can invalidate the ROS context before this method runs.
        # The timer has already been publishing zero Twist while waiting or
        # after completion, so only make this final best-effort stop publish
        # when the context remains valid.
        if rclpy.ok(context=self.context):
            try:
                self._publish()
            except RCLError:
                pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = EscapeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted; publishing zero Twist.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
