"""
Task Manager 통합 테스트에 사용하는 executor mock 노드.
`sim_task`, `navigation`, `inspection` 역할 중 하나를 수행한다.

추후, 실제 executor 노드와 통합 시, 동일한 토픽 계약으로 실제 노드를 실행하면 된다.
"""

import json
from typing import Dict, Optional, Set

import rclpy
from rclpy.node import Node

from smart_farm_interfaces.msg import (
    ExecutorStatus,
    TaskCommand,
    TaskResult,
)

from std_msgs.msg import String


class MockExecutorNode(Node):
    """한 executor의 heartbeat와 작업 성공 결과를 모의 발행한다."""

    OPERATIONS: Dict[str, Set[str]] = {
        "sim_task": {
            "TRANSFER",
            "PICK_HARVEST",
            "PLACE_INSPECT",
            "CULL",
            "CONVEYOR_OUT",
        },
        "navigation": {"NAVIGATION"},
        "inspection": {"INSPECT"},
    }

    def __init__(self) -> None:
        """파라미터를 읽고 선택된 executor의 ROS 인터페이스를 생성한다."""

        super().__init__("mock_executor")

        # Mock 동작과 결과를 제어하는 실행 파라미터
        self.declare_parameter("executor", "sim_task")
        self.declare_parameter("heartbeat_period_sec", 0.5)
        self.declare_parameter("result_delay_sec", 0.2)
        self.declare_parameter(
            "defect_slots",
            ["SLOT_03", "SLOT_07"],
        )

        self.executor_name = str(self.get_parameter("executor").value)
        if self.executor_name not in self.OPERATIONS:
            raise ValueError(
                f"Unsupported mock executor: {self.executor_name}"
            )

        # 한 번에 하나의 명령만 실행하고 완료 결과는 재사용한다.
        self.pending_command: Optional[TaskCommand] = None
        self.pending_result_timer = None
        self.last_result_by_command: Dict[str, TaskResult] = {}

        # 선택된 executor 이름으로 command/result/status topic을 구성한다.
        topic_prefix = f"/{self.executor_name}"

        if self.executor_name == "sim_task":
            self.status_publisher = self.create_publisher(
                String,
                f"{topic_prefix}/status",
                10,
            )
            self.result_publisher = self.create_publisher(
                String,
                f"{topic_prefix}/result",
                10,
            )
            self.command_subscription = self.create_subscription(
                String,
                f"{topic_prefix}/command",
                self._sim_command_callback,
                10,
            )
        else:
            self.status_publisher = self.create_publisher(
                ExecutorStatus,
                f"{topic_prefix}/status",
                10,
            )
            self.result_publisher = self.create_publisher(
                TaskResult,
                f"{topic_prefix}/result",
                10,
            )
            self.command_subscription = self.create_subscription(
                TaskCommand,
                f"{topic_prefix}/command",
                self._command_callback,
                10,
            )

        # heartbeat는 status freshness 판정에 사용되므로 주기적으로 발행한다.
        heartbeat_period = float(
            self.get_parameter("heartbeat_period_sec").value
        )
        if heartbeat_period <= 0.0:
            raise ValueError("heartbeat_period_sec must be positive")

        self.heartbeat_timer = self.create_timer(
            heartbeat_period,
            self._publish_status,
        )
        self._publish_status()

        self.get_logger().info(
            f"Mock executor ready: executor={self.executor_name}"
        )

    def _publish_status(self) -> None:
        """현재 실행 상태를 READY 또는 EXECUTING heartbeat로 발행한다."""

        message = ExecutorStatus()
        message.executor = self.executor_name

        command = self.pending_command
        if command is None:
            message.state = "READY"
            message.detail = "mock ready"
        else:
            message.state = "EXECUTING"
            message.task_id = command.task_id
            message.command_id = command.command_id
            message.operation = command.operation
            message.phase = "MOCK_EXECUTION"
            message.detail = "mock command in progress"

        self._publish_status_message(message)

    def _sim_command_callback(self, message: String) -> None:
        """JSON String 명령을 기존 mock 명령 모델로 변환한다."""

        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("top-level JSON value must be an object")

            command = TaskCommand()
            for field in ("task_id", "command_id", "operation"):
                value = payload.get(field)
                if not isinstance(value, str) or not value:
                    raise ValueError(
                        f"field {field} must be a non-empty string"
                    )
                setattr(command, field, value)

            for field in (
                "recipe_id",
                "pallet_id",
                "source",
                "destination",
            ):
                value = payload.get(field, "")
                if not isinstance(value, str):
                    raise TypeError(f"field {field} must be a string")
                setattr(command, field, value)

            target_slots = payload.get("target_slots", [])
            if not isinstance(target_slots, list) or not all(
                isinstance(item, str) for item in target_slots
            ):
                raise TypeError(
                    "field target_slots must be an array of strings"
                )
            command.target_slots = target_slots
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                f"Invalid sim_task command JSON ignored: {error}"
            )
            return

        self._command_callback(command)

    def _command_callback(self, command: TaskCommand) -> None:
        """명령을 검증하고 지정된 지연 후 결과를 발행하도록 예약한다."""

        cached = self.last_result_by_command.get(command.command_id)
        if cached is not None:
            self._publish_result(cached)
            return

        if self.pending_command is not None:
            self._publish_failure(command, "BUSY")
            return

        if command.operation not in self.OPERATIONS[self.executor_name]:
            self._publish_failure(command, "INVALID_COMMAND")
            return

        self.pending_command = command
        self._publish_status()

        delay = float(self.get_parameter("result_delay_sec").value)
        if delay <= 0.0:
            self._complete_pending_command()
            return

        self.pending_result_timer = self.create_timer(
            delay,
            self._complete_pending_command,
        )

    def _complete_pending_command(self) -> None:
        """대기 중인 명령에 operation별 성공 결과를 채워 한 번 발행한다."""

        if self.pending_result_timer is not None:
            self.pending_result_timer.cancel()
            self.destroy_timer(self.pending_result_timer)
            self.pending_result_timer = None

        command = self.pending_command
        if command is None:
            return

        result = self._new_result(command)

        # 실제 executor의 성공 조건과 같은 필드를 operation별로 채운다.
        if command.operation == "TRANSFER":
            result.completed_units = [
                "PALLET_002:RACK_L3:RACK_L2",
                "PALLET_003:RACK_L4:RACK_L3",
            ]
        elif command.operation == "PICK_HARVEST":
            result.safe_to_navigate = True
        elif command.operation == "NAVIGATION":
            result.reached_station = command.destination
        elif command.operation == "INSPECT":
            result.defect_slots = list(
                self.get_parameter("defect_slots").value
            )
        elif command.operation == "CULL":
            result.completed_units = list(command.target_slots)

        self.last_result_by_command[command.command_id] = result
        self.pending_command = None
        self._publish_result(result)
        self._publish_status()

        self.get_logger().info(
            "Mock result published: "
            f"operation={command.operation}, "
            f"command_id={command.command_id}"
        )

    def _publish_failure(
        self,
        command: TaskCommand,
        reason: str,
    ) -> None:
        """수행할 수 없는 명령에 대한 실패 결과를 즉시 발행한다."""

        result = self._new_result(command)
        result.status = "FAILED"
        result.reason = reason
        self._publish_result(result)

    def _publish_status_message(self, status: ExecutorStatus) -> None:
        """Executor 상태를 역할에 맞는 ROS 메시지로 발행한다."""

        if self.executor_name != "sim_task":
            self.status_publisher.publish(status)
            return

        payload = {
            "executor": status.executor,
            "state": status.state,
            "task_id": status.task_id,
            "command_id": status.command_id,
            "operation": status.operation,
            "phase": status.phase,
            "detail": status.detail,
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.status_publisher.publish(message)

    def _publish_result(self, result: TaskResult) -> None:
        """작업 결과를 역할에 맞는 ROS 메시지로 발행한다."""

        if self.executor_name != "sim_task":
            self.result_publisher.publish(result)
            return

        payload = {
            "task_id": result.task_id,
            "command_id": result.command_id,
            "operation": result.operation,
            "status": result.status,
            "phase": result.phase,
            "reason": result.reason,
            "safe_to_navigate": result.safe_to_navigate,
            "reached_station": result.reached_station,
            "completed_units": list(result.completed_units),
            "defect_slots": list(result.defect_slots),
            "unknown_slots": list(result.unknown_slots),
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.result_publisher.publish(message)

    @staticmethod
    def _new_result(command: TaskCommand) -> TaskResult:
        """수신 명령과 식별자가 일치하는 기본 성공 결과를 생성한다."""

        result = TaskResult()
        result.task_id = command.task_id
        result.command_id = command.command_id
        result.operation = command.operation
        result.status = "SUCCEEDED"
        result.phase = "RESULT"
        result.reason = "NONE"
        return result


def main(args=None) -> None:
    """파라미터로 선택한 mock executor 노드를 실행한다."""

    rclpy.init(args=args)
    node = MockExecutorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
