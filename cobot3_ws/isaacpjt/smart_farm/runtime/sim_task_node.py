"""Sim Task Executor의 ROS 2 String/JSON 통신 노드."""

import json
from collections import OrderedDict
from dataclasses import dataclass
from typing import Optional, Tuple

from rclpy.node import Node
from std_msgs.msg import String


class CommandValidationError(ValueError):
    """Sim Task 명령의 JSON 형식이나 필드가 잘못된 경우."""


@dataclass(frozen=True)
class SimTaskCommand:
    """검증이 끝난 Sim Task 명령."""

    task_id: str
    command_id: str
    operation: str

    recipe_id: str = ""
    pallet_id: str = ""
    source: str = ""
    destination: str = ""
    target_slots: Tuple[str, ...] = ()


class SimTaskNode(Node):
    """Task Manager와 Sim Task Runtime 사이의 ROS 어댑터."""

    def __init__(
        self,
        supported_operations: set[str],
        heartbeat_period_sec: float = 0.5,
        result_cache_size: int = 64,
    ) -> None:
        super().__init__("sim_task_executor")

        self._supported_operations = set(supported_operations)
        self._result_cache_size = result_cache_size

        self._ready = False
        self._state = "STARTING"
        self._phase = "BOOT"
        self._detail = "initializing Isaac Sim"

        self._queued_command: Optional[SimTaskCommand] = None
        self._active_command: Optional[SimTaskCommand] = None

        self._completed_results: OrderedDict[str, dict] = OrderedDict()

        self._command_subscription = self.create_subscription(
            String,
            "/sim_task/command",
            self._command_callback,
            10,
        )
        self._result_publisher = self.create_publisher(
            String,
            "/sim_task/result",
            10,
        )
        self._status_publisher = self.create_publisher(
            String,
            "/sim_task/status",
            10,
        )

        self._heartbeat_timer = self.create_timer(
            heartbeat_period_sec,
            self.publish_status,
        )

        self.publish_status()

    @property
    def active_command(self) -> Optional[SimTaskCommand]:
        return self._active_command

    @property
    def has_active_command(self) -> bool:
        return self._active_command is not None

    def mark_ready(self, detail: str = "sim task executor ready") -> None:
        """장면과 제어기 초기화가 끝났음을 알린다."""

        if self._active_command is not None:
            raise RuntimeError("cannot become READY while a command is active")

        self._ready = True
        self._state = "READY"
        self._phase = "IDLE"
        self._detail = detail
        self.publish_status()

    def mark_error(self, detail: str) -> None:
        """새 명령을 받지 않는 오류 상태로 전환한다."""

        self._ready = False
        self._state = "ERROR"
        self._phase = "ERROR"
        self._detail = detail
        self.publish_status()

    def take_command(self) -> Optional[SimTaskCommand]:
        """Standalone 프레임 루프가 대기 명령을 가져간다."""

        if self._active_command is not None:
            return None

        command = self._queued_command
        if command is None:
            return None

        self._queued_command = None
        self._active_command = command

        self._state = "BUSY"
        self._phase = "ACCEPTED"
        self._detail = f"executing {command.operation}"
        self.publish_status()

        return command

    def set_phase(self, phase: str, detail: str = "") -> None:
        """현재 물리 동작의 내부 phase를 갱신한다."""

        if self._active_command is None:
            raise RuntimeError("no active sim task command")

        if phase == self._phase and detail == self._detail:
            return

        self._phase = phase
        self._detail = detail
        self.publish_status()

    def succeed(
        self,
        *,
        phase: str = "RESULT",
        safe_to_navigate: bool = False,
        reached_station: str = "",
        completed_units: Tuple[str, ...] = (),
    ) -> None:
        """현재 명령을 성공으로 종료한다."""

        self._finish_active_command(
            status="SUCCEEDED",
            phase=phase,
            reason="NONE",
            safe_to_navigate=safe_to_navigate,
            reached_station=reached_station,
            completed_units=completed_units,
        )

        self._state = "READY"
        self._phase = "IDLE"
        self._detail = "sim task executor ready"
        self.publish_status()

    def fail(
        self,
        *,
        reason: str,
        phase: str,
        completed_units: Tuple[str, ...] = (),
        reset_required: bool = True,
    ) -> None:
        """현재 명령을 실패로 종료한다."""

        self._finish_active_command(
            status="FAILED",
            phase=phase,
            reason=reason,
            completed_units=completed_units,
        )

        if reset_required:
            self.mark_error(f"{reason}; scene reset required")
        else:
            self._state = "READY"
            self._phase = "IDLE"
            self._detail = "sim task executor ready"
            self.publish_status()

    def publish_status(self) -> None:
        """현재 Executor 상태를 JSON heartbeat로 발행한다."""

        command = self._active_command

        payload = {
            "executor": "sim_task",
            "state": self._state,
            "task_id": command.task_id if command else "",
            "command_id": command.command_id if command else "",
            "operation": command.operation if command else "",
            "phase": self._phase,
            "detail": self._detail,
        }

        self._publish_json(self._status_publisher, payload)

    def _command_callback(self, message: String) -> None:
        """JSON 명령을 검증하고 Standalone 루프에서 처리하도록 저장한다."""

        try:
            command = self._parse_command(message.data)
        except (json.JSONDecodeError, CommandValidationError) as error:
            self.get_logger().warning(
                f"Invalid sim_task command JSON ignored: {error}"
            )
            return

        cached = self._completed_results.get(command.command_id)
        if cached is not None:
            if (
                cached["task_id"] != command.task_id
                or cached["operation"] != command.operation
            ):
                self._publish_immediate_failure(command, "INVALID_ID")
                return

            self._publish_json(self._result_publisher, cached)
            self.get_logger().info(
                f"Cached result republished: {command.command_id}"
            )
            return

        if self._is_same_command(command, self._active_command):
            self.get_logger().info(
                f"Active command duplicate ignored: {command.command_id}"
            )
            return

        if self._is_same_command(command, self._queued_command):
            self.get_logger().info(
                f"Queued command duplicate ignored: {command.command_id}"
            )
            return

        if command.operation not in self._supported_operations:
            self._publish_immediate_failure(command, "INVALID_COMMAND")
            return

        if not self._ready:
            self._publish_immediate_failure(command, "NOT_READY")
            return

        if (
            self._active_command is not None
            or self._queued_command is not None
        ):
            self._publish_immediate_failure(command, "BUSY")
            return

        self._queued_command = command

        self.get_logger().info(
            "Sim command queued: "
            f"operation={command.operation}, "
            f"command_id={command.command_id}"
        )

    def _finish_active_command(
        self,
        *,
        status: str,
        phase: str,
        reason: str,
        safe_to_navigate: bool = False,
        reached_station: str = "",
        completed_units: Tuple[str, ...] = (),
    ) -> None:
        command = self._active_command
        if command is None:
            raise RuntimeError("no active command to finish")

        payload = {
            "task_id": command.task_id,
            "command_id": command.command_id,
            "operation": command.operation,
            "status": status,
            "phase": phase,
            "reason": reason,
            "safe_to_navigate": safe_to_navigate,
            "reached_station": reached_station,
            "completed_units": list(completed_units),
            "defect_slots": [],
            "unknown_slots": [],
        }

        self._cache_result(payload)
        self._publish_json(self._result_publisher, payload)

        self.get_logger().info(
            "Sim result published: "
            f"operation={command.operation}, "
            f"status={status}, "
            f"command_id={command.command_id}"
        )

        self._active_command = None

    def _publish_immediate_failure(
        self,
        command: SimTaskCommand,
        reason: str,
    ) -> None:
        payload = {
            "task_id": command.task_id,
            "command_id": command.command_id,
            "operation": command.operation,
            "status": "FAILED",
            "phase": "COMMAND_VALIDATION",
            "reason": reason,
            "safe_to_navigate": False,
            "reached_station": "",
            "completed_units": [],
            "defect_slots": [],
            "unknown_slots": [],
        }

        self._cache_result(payload)
        self._publish_json(self._result_publisher, payload)

    def _cache_result(self, payload: dict) -> None:
        command_id = payload["command_id"]
        self._completed_results[command_id] = payload
        self._completed_results.move_to_end(command_id)

        while len(self._completed_results) > self._result_cache_size:
            self._completed_results.popitem(last=False)

    @classmethod
    def _parse_command(cls, raw_message: str) -> SimTaskCommand:
        payload = json.loads(raw_message)
        if not isinstance(payload, dict):
            raise CommandValidationError(
                "top-level JSON value must be an object"
            )

        return SimTaskCommand(
            task_id=cls._required_string(payload, "task_id"),
            command_id=cls._required_string(payload, "command_id"),
            operation=cls._required_string(payload, "operation"),
            recipe_id=cls._optional_string(payload, "recipe_id"),
            pallet_id=cls._optional_string(payload, "pallet_id"),
            source=cls._optional_string(payload, "source"),
            destination=cls._optional_string(payload, "destination"),
            target_slots=cls._string_tuple(payload, "target_slots"),
        )

    @staticmethod
    def _required_string(payload: dict, field: str) -> str:
        value = payload.get(field)
        if not isinstance(value, str) or not value:
            raise CommandValidationError(
                f"field {field} must be a non-empty string"
            )
        return value

    @staticmethod
    def _optional_string(payload: dict, field: str) -> str:
        value = payload.get(field, "")
        if not isinstance(value, str):
            raise CommandValidationError(
                f"field {field} must be a string"
            )
        return value

    @staticmethod
    def _string_tuple(payload: dict, field: str) -> Tuple[str, ...]:
        value = payload.get(field, [])
        if not isinstance(value, list) or not all(
            isinstance(item, str) for item in value
        ):
            raise CommandValidationError(
                f"field {field} must be an array of strings"
            )
        return tuple(value)

    @staticmethod
    def _is_same_command(
        left: SimTaskCommand,
        right: Optional[SimTaskCommand],
    ) -> bool:
        if right is None:
            return False

        return (
            left.task_id == right.task_id
            and left.command_id == right.command_id
            and left.operation == right.operation
        )

    @staticmethod
    def _publish_json(publisher, payload: dict) -> None:
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
        )
        publisher.publish(message)
