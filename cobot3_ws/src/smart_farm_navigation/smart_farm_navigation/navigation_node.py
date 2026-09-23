"""Task Manager의 Navigation 명령을 실제 /cmd_vel 주행으로 실행하는 노드.

인터페이스:
  /navigation/command
      smart_farm_interfaces/msg/TaskCommand

  /navigation/result
      smart_farm_interfaces/msg/TaskResult

  /navigation/status
      smart_farm_interfaces/msg/ExecutorStatus

TaskCommand를 받으면 destinations.yaml에 등록된 launch를 자식 프로세스로
실행한다. 자식 프로세스의 종료 코드를 TaskResult로 변환한다.

한 번에 하나의 명령만 실행하며, 실행 중 새로운 명령은 FAILED/BUSY로
응답한다. 같은 command_id를 다시 수신하면 작업을 재실행하지 않고
이전에 저장한 결과를 다시 발행한다.
"""

import os
import subprocess
import time

import rclpy
import rclpy.executors
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from smart_farm_interfaces.msg import (
    ExecutorStatus,
    TaskCommand,
    TaskResult,
)


class NavigationNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_node")
        share = get_package_share_directory("smart_farm_navigation")

        self.declare_parameter("destinations_file", os.path.join(share, "config", "destinations.yaml"))
        self.declare_parameter("timeout_s", 110.0)
        self.declare_parameter("executor", "navigation")

        destinations_file = str(
            self.get_parameter("destinations_file").value
        )

        with open(destinations_file, encoding="utf-8") as file:
            config = yaml.safe_load(file) or {}

        self.dest = config.get("destinations", {})

        if not self.dest:
            raise ValueError(
                f"No destinations configured: {destinations_file}"
            )
        
        self.timeout = float(self.get_parameter("timeout_s").value)
        self.executor_name = str(
            self.get_parameter("executor").value
        )

        self.share = share

        self.status_state = "READY"
        self.status_task_id = ""
        self.status_command_id = ""
        self.status_operation = "NAVIGATION"
        self.status_phase = "IDLE"
        self.status_detail = "waiting for /navigation/command"

        self.command_sub = self.create_subscription(
            TaskCommand,
            "/navigation/command",
            self._on_command,
            10
        )
        
        self.result_pub = self.create_publisher(
            TaskResult,
            "/navigation/result",
            10
        )

        self.status_pub = self.create_publisher(
            ExecutorStatus,
            "/navigation/status",
            10
        )

        self.active = None          # TaskCommand currently being executed
        self.proc = None
        self.started_at = 0.0
        self.done_ids = {}

        self.status_tiemr = self.create_timer(
            0.5,
            self._publish_status,
        )

        self.poll_timer = self.create_timer(
            0.2,
            self._poll,
        )
        
        self._status(
            state="READY",
            phase="IDLE",
            detail="waiting for /navigation/command",
        )
        self.get_logger().info(f"navigation_node ready; destinations: {list(self.dest)}")

    # ---------- publish helpers ----------
    def _status(
            self,
            state,
            task_id="",
            command_id="",
            phase="IDLE",
            detail=""
        ) -> None:
        self.status_state = state
        self.status_task_id = task_id
        self.status_command_id = command_id
        self.status_phase = phase
        self.status_detail = detail

        self._publish_status()

    def _publish_status(self) -> None:
        message = ExecutorStatus()
        message.executor = self.executor_name
        message.state = self.status_state
        message.task_id = self.status_task_id
        message.command_id = self.status_command_id
        message.operation = self.status_operation
        message.phase = self.status_phase
        message.detail = self.status_detail

        self.status_pub.publish(message)

    def _result(
        self,
        command: TaskCommand,
        status: str,
        reason: str,
        phase: str,
        reached_station: str = "",
    ) -> None:
        message = TaskResult()

        message.task_id = command.task_id
        message.command_id = command.command_id
        message.operation = command.operation or "NAVIGATION"

        message.status = status
        message.phase = phase
        message.reason = reason

        message.safe_to_navigate = False
        message.reached_station = reached_station

        message.completed_units = []
        message.defect_slots = []
        message.unknown_slots = []

        self.done_ids[command.command_id] = message
        self.result_pub.publish(message)

        self.get_logger().info(
            f"result {status}/{reason} "
            f"for {command.command_id} "
            f"(phase {phase})"
        )

    def _on_command(
        self,
        command: TaskCommand,
    ) -> None:
        if (
            not command.task_id
            or not command.command_id
            or not command.destination
        ):
            self._result(
                command,
                status="FAILED",
                reason="INVALID_COMMAND",
                phase="VALIDATE",
            )
            return

        if command.operation != "NAVIGATION":
            self._result(
                command,
                status="FAILED",
                reason="INVALID_COMMAND",
                phase="VALIDATE",
            )
            return

        cached_result = self.done_ids.get(
            command.command_id
        )

        if cached_result is not None:
            self.result_pub.publish(cached_result)
            self.get_logger().info(
                "Cached result republished: "
                f"{command.command_id}"
            )
            return

        if self.active is not None:
            self._result(
                command,
                status="FAILED",
                reason="BUSY",
                phase="VALIDATE",
            )
            return

        destination = self.dest.get(
            command.destination
        )

        if destination is None:
            self._result(
                command,
                status="FAILED",
                reason="INVALID_COMMAND",
                phase="VALIDATE",
            )
            return

        if "station" in destination:
            # Nav2 모드 (destinations_nav2.yaml): go_to_station 이 NavigateToPose 로 주행. exit code 0/2 규약은 동일.
            argv = [
                "ros2", "run", "smart_farm_navigation", "go_to_station",
                "--ros-args", "-p", f"station:={destination['station']}",
            ]
        else:
            params = os.path.join(
                self.share,
                "config",
                destination["params"],
            )

            argv = [
                "ros2",
                "launch",
                "smart_farm_navigation",
                destination["launch"],
                "auto_start:=true",
                f"params_file:={params}",
            ]

        self.get_logger().info(
            f"command {command.command_id}: "
            f"{command.destination} -> {' '.join(argv)}"
        )

        try:
            process = subprocess.Popen(argv)
        except OSError as error:
            self.get_logger().error(
                "Failed to launch navigation process: "
                f"{type(error).__name__}: {error}"
            )

            self._result(
                command,
                status="FAILED",
                reason="NAV_FAILED",
                phase="LAUNCH",
            )
            return

        self.proc = process
        self.active = command
        self.started_at = time.monotonic()

        self._status(
            state="BUSY",
            task_id=command.task_id,
            command_id=command.command_id,
            phase="DRIVING",
            detail=command.destination,
        )

    def _poll(self) -> None:
        if self.active is None or self.proc is None:
            return

        return_code = self.proc.poll()

        if return_code is None:
            elapsed = time.monotonic() - self.started_at

            if elapsed > self.timeout:
                self.proc.terminate()

                try:
                    self.proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                    self.proc.wait()

                self._finish(
                    status="TIMEOUT",
                    reason="RESULT_TIMEOUT",
                    phase="DRIVING",
                )

            return

        if return_code == 0:
            self._finish(
                status="SUCCEEDED",
                reason="NONE",
                phase="ARRIVED",
                reached_station=self.active.destination,
            )

        elif return_code == 2:
            self._finish(
                status="FAILED",
                reason="NAV_FAILED",
                phase="DRIVING",
            )

        else:
            self._finish(
                status="FAILED",
                reason="NAV_FAILED",
                phase="LAUNCH",
            )
    
    def _finish(
        self,
        status: str,
        reason: str,
        phase: str,
        reached_station: str = "",
    ) -> None:
        command = self.active

        if command is None:
            self.get_logger().warning(
                "Finish requested without active command"
            )
            return

        self.active = None
        self.proc = None

        self._result(
            command,
            status=status,
            reason=reason,
            phase=phase,
            reached_station=reached_station,
        )

        self._status(
            state="READY",
            phase="IDLE",
            detail=f"last {status}",
        )

    def shutdown(self) -> None:
        if (
            self.proc is not None
            and self.proc.poll() is None
        ):
            self.proc.terminate()

            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()


def main() -> None:
    rclpy.init()
    node = NavigationNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():                 # ros2 launch 의 Ctrl+C 는 컨텍스트를 먼저 닫으므로 두 번 shutdown 하지 않음
            rclpy.shutdown()


if __name__ == "__main__":
    main()
