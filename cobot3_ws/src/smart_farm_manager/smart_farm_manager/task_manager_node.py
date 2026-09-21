"""ROS 2 인터페이스와 순수 Python 상태 머신을 연결하는 Task Manager 노드."""

import json
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from smart_farm_interfaces.msg import (
    CycleStatus,
    ExecutorStatus,
    TaskCommand,
    TaskResult,
)
from smart_farm_interfaces.srv import StartCycle

from std_msgs.msg import String

from .protocol import TaskCommandData, TaskResultData
from .scenario import CycleState
from .state_machine import CycleStateMachine


@dataclass
class ExecutorSnapshot:
    """한 executor에서 마지막으로 수신한 status와 수신 시각."""

    message: ExecutorStatus
    received_at: float


@dataclass(frozen=True)
class ExecutorReadiness:
    """한 executor의 heartbeat를 PREFLIGHT 관점에서 판정한 결과."""

    executor: str
    received: bool
    state: str
    age_sec: Optional[float]
    ready: bool
    stale: bool
    detail: str


class TaskManagerNode(Node):
    """executor를 조율하며 스마트팜 시연 사이클을 진행한다."""

    EXECUTORS = (
        "sim_task",
        "navigation",
        "inspection",
    )

    NOT_READY_REASONS = {
        "sim_task": "SIM_NOT_READY",
        "navigation": "NAV_NOT_READY",
        "inspection": "INSPECTION_NOT_READY",
    }

    def __init__(self) -> None:
        """파라미터, ROS 인터페이스, 상태 머신과 주기 타이머를 생성한다."""

        super().__init__("task_manager")

        # 실행 주기와 PREFLIGHT timeout을 외부 설정으로 노출한다.
        self.declare_parameter("tick_period_sec", 0.1)
        self.declare_parameter("preflight_timeout_sec", 10.0)
        self.declare_parameter("executor_status_timeout_sec", 3.0)

        # ROS 메시지와 분리된 상태 머신 및 heartbeat 수신 상태
        self.machine = CycleStateMachine()

        self.executor_statuses: Dict[
            str, ExecutorSnapshot
        ] = {}

        self.last_logged_status: Dict[str, tuple] = {}
        self.status_identity_warnings = set()
        self.preflight_started_at: Optional[float] = None
        self.command_deadline: Optional[float] = None

        # Task Manager가 각 executor로 보내는 공통 작업 명령
        self.command_publishers = {
            "sim_task": self.create_publisher(
                String,
                "/sim_task/command",
                10,
            ),
            "navigation": self.create_publisher(
                TaskCommand,
                "/navigation/command",
                10,
            ),
            "inspection": self.create_publisher(
                TaskCommand,
                "/inspection/command",
                10,
            ),
        }

        # 각 executor가 반환하는 terminal 작업 결과
        self.create_subscription(
            String,
            "/sim_task/result",
            self._sim_result_callback,
            10,
        )
        self.create_subscription(
            TaskResult,
            "/navigation/result",
            lambda msg: self._result_callback(msg, "navigation"),
            10,
        )
        self.create_subscription(
            TaskResult,
            "/inspection/result",
            lambda msg: self._result_callback(msg, "inspection"),
            10,
        )

        # executor 준비 상태 및 freshness를 확인하는 heartbeat
        self.create_subscription(
            String,
            "/sim_task/status",
            self._sim_status_callback,
            10,
        )
        self.create_subscription(
            ExecutorStatus,
            "/navigation/status",
            lambda msg: self._status_callback(msg, "navigation"),
            10,
        )
        self.create_subscription(
            ExecutorStatus,
            "/inspection/status",
            lambda msg: self._status_callback(msg, "inspection"),
            10,
        )

        # 외부에서 사용하는 사이클 상태 topic과 시작 service
        self.cycle_status_publisher = self.create_publisher(
            CycleStatus,
            "/cycle/status",
            10,
        )

        self.start_cycle_service = self.create_service(
            StartCycle,
            "/start_cycle",
            self._start_cycle_callback,
        )

        # timeout 계산은 시스템 시각 변경의 영향을 받지 않는 steady clock을 쓴다.
        tick_period = float(
            self.get_parameter("tick_period_sec").value
        )

        self.steady_clock = Clock(
            clock_type=ClockType.STEADY_TIME
        )

        self.timer = self.create_timer(
            tick_period,
            self._tick,
            clock=self.steady_clock,
        )

        self._publish_cycle_status(
            status="IDLE",
            reason="NONE",
        )

        self.get_logger().info(
            "Task Manager ready: scenario=DEMO_HARVEST_01"
        )

    def _start_cycle_callback(
        self,
        request: StartCycle.Request,
        response: StartCycle.Response,
    ) -> StartCycle.Response:
        """새 사이클 요청을 검증하고 PREFLIGHT 진입 여부를 응답한다."""

        if self.machine.state != CycleState.IDLE:
            response.accepted = False
            response.task_id = self.machine.task_id

            if self.machine.is_running:
                response.reason = "BUSY"
            else:
                response.reason = self.machine.terminal_status

            return response

        task_id = self._create_task_id()

        try:
            self.machine.start(
                task_id=task_id,
                scenario_id=request.scenario_id,
            )
        except ValueError as error:
            response.accepted = False
            response.task_id = ""
            response.reason = "INVALID_SCENARIO"

            self.get_logger().warning(str(error))
            return response

        self.preflight_started_at = time.monotonic()
        self.command_deadline = None

        response.accepted = True
        response.task_id = task_id
        response.reason = "NONE"

        self._publish_cycle_status(
            status="ACCEPTED",
            reason="NONE",
        )

        self.get_logger().info(
            f"Cycle accepted: {task_id}"
        )

        return response

    def _status_callback(
        self,
        message: ExecutorStatus,
        executor: str,
    ) -> None:
        """executor 식별자를 검증하고 최신 heartbeat를 저장한다."""

        if message.executor != executor:
            warning_key = (executor, message.executor)
            if warning_key not in self.status_identity_warnings:
                self.status_identity_warnings.add(warning_key)
                self.get_logger().warning(
                    "Executor name mismatch: "
                    f"topic={executor}, "
                    f"message={message.executor or '<empty>'}"
                )
            return

        self.executor_statuses[executor] = ExecutorSnapshot(
            message=message,
            received_at=time.monotonic(),
        )

        status_key = (message.state, message.detail)
        if self.last_logged_status.get(executor) != status_key:
            self.last_logged_status[executor] = status_key
            self.get_logger().info(
                "Executor status changed: "
                f"executor={executor}, state={message.state}, "
                f"detail={message.detail or '-'}"
            )

    def _result_callback(
        self,
        message: TaskResult,
        executor: str,
    ) -> None:
        """커스텀 ROS 결과를 내부 모델로 변환한다."""

        self._handle_result(
            self._task_result_from_ros(message),
            executor,
        )

    def _sim_result_callback(self, message: String) -> None:
        """Sim Task JSON 결과를 검증하고 내부 모델로 변환한다."""

        try:
            result_data = self._task_result_from_json(message.data)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                f"Invalid sim_task result JSON ignored: {error}"
            )
            return

        self._handle_result(result_data, "sim_task")

    def _sim_status_callback(self, message: String) -> None:
        """Sim Task JSON heartbeat를 검증하고 공통 상태 처리로 전달한다."""

        try:
            status = self._executor_status_from_json(message.data)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                f"Invalid sim_task status JSON ignored: {error}"
            )
            return

        self._status_callback(status, "sim_task")

    def _handle_result(
        self,
        result_data: TaskResultData,
        executor: str,
    ) -> None:
        """종료 결과가 현재 명령과 일치하는지 검증하고 상태를 전이한다."""

        if self.machine.active_command is None:
            self.get_logger().warning(
                f"Result ignored: no active command, executor={executor}"
            )
            return

        expected_executor = self._expected_executor()
        if expected_executor != executor:
            self.get_logger().warning(
                "Result ignored: unexpected executor, "
                f"expected={expected_executor}, actual={executor}"
            )
            return

        outcome = self.machine.handle_result(result_data)

        if not outcome.accepted:
            self.get_logger().warning(
                "Result ignored: "
                f"reason={outcome.reason}, "
                f"command_id={result_data.command_id}"
            )
            return

        self.command_deadline = None

        if self.machine.state == CycleState.COMPLETE:
            self._publish_cycle_status(
                status="SUCCEEDED",
                reason="NONE",
            )
            self.get_logger().info(
                f"Cycle completed: {self.machine.task_id}"
            )

        elif self.machine.state == CycleState.ERROR:
            self._publish_cycle_status(
                status=self.machine.terminal_status,
                reason=self.machine.failure_reason,
            )
            self.get_logger().error(
                "Cycle failed: "
                f"status={self.machine.terminal_status}, "
                f"reason={self.machine.failure_reason}"
            )

        else:
            self._publish_cycle_status(
                status="RUNNING",
                reason="NONE",
            )

    def _tick(self) -> None:
        """현재 상태에 따라 PREFLIGHT, 명령 발행 또는 timeout을 처리한다."""

        if self.machine.state == CycleState.PREFLIGHT:
            self._tick_preflight()
            return

        if self.machine.state in {
            CycleState.IDLE,
            CycleState.COMPLETE,
            CycleState.ERROR,
        }:
            return

        if self.machine.active_command is None:
            self._dispatch_current_step()
            return

        self._check_command_timeout()

    def _tick_preflight(self) -> None:
        """모든 executor의 최신 READY heartbeat를 기다리고 진단한다."""

        now = time.monotonic()

        readiness = self._executor_readiness(now)

        if all(item.ready for item in readiness):
            self.machine.complete_preflight(ready=True)
            self.preflight_started_at = None

            self._publish_cycle_status(
                status="RUNNING",
                reason="NONE",
            )

            self.get_logger().info(
                "PREFLIGHT complete: all executors READY"
            )
            return

        if self.preflight_started_at is None:
            self.preflight_started_at = now

        timeout = float(
            self.get_parameter(
                "preflight_timeout_sec"
            ).value
        )

        if now - self.preflight_started_at < timeout:
            return

        not_ready = [item for item in readiness if not item.ready]
        reason = self.NOT_READY_REASONS[not_ready[0].executor]

        self.machine.complete_preflight(
            ready=False,
            reason=reason,
        )
        self.preflight_started_at = None

        self._publish_cycle_status(
            status=self.machine.terminal_status,
            reason=self.machine.failure_reason,
        )

        names = ",".join(item.executor for item in not_ready)
        diagnostics = "; ".join(
            self._format_readiness(item) for item in readiness
        )
        self.get_logger().error(
            "PREFLIGHT failed: "
            f"reason={reason}, not_ready=[{names}], "
            f"executors=[{diagnostics}]"
        )

    def _executor_readiness(
        self,
        now: float,
    ) -> List[ExecutorReadiness]:
        """모든 executor의 수신 여부, age, stale 및 READY 상태를 판정한다."""

        max_age = float(
            self.get_parameter(
                "executor_status_timeout_sec"
            ).value
        )

        readiness: List[ExecutorReadiness] = []
        for executor in self.EXECUTORS:
            snapshot = self.executor_statuses.get(executor)

            if snapshot is None:
                readiness.append(
                    ExecutorReadiness(
                        executor=executor,
                        received=False,
                        state="<never>",
                        age_sec=None,
                        ready=False,
                        stale=False,
                        detail="",
                    )
                )
                continue

            age_sec = max(0.0, now - snapshot.received_at)
            stale = age_sec > max_age
            readiness.append(
                ExecutorReadiness(
                    executor=executor,
                    received=True,
                    state=snapshot.message.state or "<empty>",
                    age_sec=age_sec,
                    ready=(
                        not stale
                        and snapshot.message.state == "READY"
                    ),
                    stale=stale,
                    detail=snapshot.message.detail,
                )
            )

        return readiness

    @staticmethod
    def _format_readiness(item: ExecutorReadiness) -> str:
        """PREFLIGHT 판정 한 건을 로그에 사용할 문자열로 변환한다."""

        age = "never" if item.age_sec is None else f"{item.age_sec:.2f}s"
        detail = item.detail or "-"
        return (
            f"{item.executor}:received={item.received},"
            f"state={item.state},age={age},ready={item.ready},"
            f"stale={item.stale},detail={detail}"
        )

    def _dispatch_current_step(self) -> None:
        """현재 공정의 명령을 생성해 담당 executor로 발행한다."""

        step = self.machine.scenario.step_for(
            self.machine.state
        )
        command_data = self.machine.create_command()

        executor = step.executor.value
        publisher = self.command_publishers[executor]
        if executor == "sim_task":
            command_message = self._task_command_to_json(command_data)
        else:
            command_message = self._task_command_to_ros(command_data)
        publisher.publish(command_message)

        self.command_deadline = (
            time.monotonic() + step.timeout_sec
        )

        self._publish_cycle_status(
            status="RUNNING",
            reason="NONE",
        )

        self.get_logger().info(
            "Command published: "
            f"executor={executor}, "
            f"operation={command_data.operation}, "
            f"command_id={command_data.command_id}"
        )

    def _check_command_timeout(self) -> None:
        """활성 명령이 제한 시간을 넘기면 TIMEOUT 결과로 실패 처리한다."""

        if self.command_deadline is None:
            return

        if time.monotonic() < self.command_deadline:
            return

        command = self.machine.active_command
        if command is None:
            self.command_deadline = None
            return

        timeout_result = TaskResultData(
            task_id=command.task_id,
            command_id=command.command_id,
            operation=command.operation,
            status="TIMEOUT",
            phase="RESULT_TIMEOUT",
            reason="RESULT_TIMEOUT",
        )

        self.machine.handle_result(timeout_result)
        self.command_deadline = None

        self._publish_cycle_status(
            status=self.machine.terminal_status,
            reason=self.machine.failure_reason,
        )

        self.get_logger().error(
            "Command timeout: "
            f"operation={command.operation}, "
            f"command_id={command.command_id}"
        )

    def _expected_executor(self) -> Optional[str]:
        """현재 공정에 할당된 executor 이름을 반환한다."""

        try:
            step = self.machine.scenario.step_for(
                self.machine.state
            )
        except KeyError:
            return None

        return step.executor.value

    def _publish_cycle_status(
        self,
        status: str,
        reason: str,
    ) -> None:
        """현재 상태 머신 정보를 공통 CycleStatus 메시지로 발행한다."""

        message = CycleStatus()
        message.task_id = self.machine.task_id
        message.scenario_id = self.machine.scenario.scenario_id
        message.state = self.machine.state.value

        active = self.machine.active_command
        message.active_command_id = (
            active.command_id if active else ""
        )

        message.status = status
        message.reason = reason

        self.cycle_status_publisher.publish(message)

    @staticmethod
    def _task_command_to_ros(
        data: TaskCommandData,
    ) -> TaskCommand:
        """내부 명령 모델을 ROS TaskCommand 메시지로 변환한다."""

        message = TaskCommand()
        message.task_id = data.task_id
        message.command_id = data.command_id
        message.operation = data.operation
        message.recipe_id = data.recipe_id
        message.pallet_id = data.pallet_id
        message.source = data.source
        message.destination = data.destination
        message.target_slots = list(data.target_slots)
        return message

    @staticmethod
    def _task_command_to_json(data: TaskCommandData) -> String:
        """내부 명령 모델을 Sim Task용 JSON String으로 변환한다."""

        payload = {
            "task_id": data.task_id,
            "command_id": data.command_id,
            "operation": data.operation,
            "recipe_id": data.recipe_id,
            "pallet_id": data.pallet_id,
            "source": data.source,
            "destination": data.destination,
            "target_slots": list(data.target_slots),
        }
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
        )
        return message

    @staticmethod
    def _task_result_from_ros(
        message: TaskResult,
    ) -> TaskResultData:
        """ROS TaskResult 메시지를 상태 머신용 내부 모델로 변환한다."""

        return TaskResultData(
            task_id=message.task_id,
            command_id=message.command_id,
            operation=message.operation,
            status=message.status,
            phase=message.phase,
            reason=message.reason,
            safe_to_navigate=message.safe_to_navigate,
            reached_station=message.reached_station,
            completed_units=tuple(message.completed_units),
            defect_slots=tuple(message.defect_slots),
            unknown_slots=tuple(message.unknown_slots),
        )

    @classmethod
    def _task_result_from_json(cls, raw_message: str) -> TaskResultData:
        """Sim Task JSON 결과를 상태 머신용 내부 모델로 변환한다."""

        payload = cls._parse_json_object(raw_message)
        return TaskResultData(
            task_id=cls._json_string(payload, "task_id", required=True),
            command_id=cls._json_string(
                payload,
                "command_id",
                required=True,
            ),
            operation=cls._json_string(
                payload,
                "operation",
                required=True,
            ),
            status=cls._json_string(payload, "status", required=True),
            phase=cls._json_string(payload, "phase"),
            reason=cls._json_string(payload, "reason", default="NONE"),
            safe_to_navigate=cls._json_bool(
                payload,
                "safe_to_navigate",
            ),
            reached_station=cls._json_string(payload, "reached_station"),
            completed_units=cls._json_string_tuple(
                payload,
                "completed_units",
            ),
            defect_slots=cls._json_string_tuple(payload, "defect_slots"),
            unknown_slots=cls._json_string_tuple(payload, "unknown_slots"),
        )

    @classmethod
    def _executor_status_from_json(cls, raw_message: str) -> ExecutorStatus:
        """Sim Task JSON heartbeat를 공통 상태 메시지 모델로 변환한다."""

        payload = cls._parse_json_object(raw_message)
        message = ExecutorStatus()
        message.executor = cls._json_string(
            payload,
            "executor",
            required=True,
        )
        message.state = cls._json_string(
            payload,
            "state",
            required=True,
        )
        message.task_id = cls._json_string(payload, "task_id")
        message.command_id = cls._json_string(payload, "command_id")
        message.operation = cls._json_string(payload, "operation")
        message.phase = cls._json_string(payload, "phase")
        message.detail = cls._json_string(payload, "detail")
        return message

    @staticmethod
    def _parse_json_object(raw_message: str) -> dict:
        """문자열을 JSON object로 파싱한다."""

        payload = json.loads(raw_message)
        if not isinstance(payload, dict):
            raise ValueError("top-level JSON value must be an object")
        return payload

    @staticmethod
    def _json_string(
        payload: dict,
        field: str,
        *,
        required: bool = False,
        default: str = "",
    ) -> str:
        """JSON 문자열 필드를 타입과 필수 여부에 따라 검증한다."""

        if field not in payload:
            if required:
                raise ValueError(f"missing required field: {field}")
            return default

        value = payload[field]
        if not isinstance(value, str):
            raise TypeError(f"field {field} must be a string")
        if required and not value:
            raise ValueError(f"required field is empty: {field}")
        return value

    @staticmethod
    def _json_bool(payload: dict, field: str) -> bool:
        """JSON boolean 필드를 검증하며 누락 시 false를 반환한다."""

        value = payload.get(field, False)
        if not isinstance(value, bool):
            raise TypeError(f"field {field} must be a boolean")
        return value

    @staticmethod
    def _json_string_tuple(payload: dict, field: str) -> tuple:
        """JSON 문자열 배열을 검증해 tuple로 반환한다."""

        value = payload.get(field, [])
        if not isinstance(value, list) or not all(
            isinstance(item, str) for item in value
        ):
            raise TypeError(f"field {field} must be an array of strings")
        return tuple(value)

    @staticmethod
    def _create_task_id() -> str:
        """현재 로컬 시각으로 사이클 식별자를 생성한다."""

        return datetime.now().strftime(
            "TASK-%Y%m%d-%H%M%S"
        )


def main(args=None) -> None:
    """Task Manager ROS 노드를 초기화하고 종료될 때까지 실행한다."""

    rclpy.init(args=args)
    node = TaskManagerNode()

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