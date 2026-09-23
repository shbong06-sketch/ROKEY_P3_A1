"""Task Manager의 Navigation 명령을 실제 /cmd_vel 주행으로 실행하는 노드.

인터페이스:
  /navigation/command
      smart_farm_interfaces/msg/TaskCommand

  /navigation/result
      smart_farm_interfaces/msg/TaskResult

  /navigation/status
      smart_farm_interfaces/msg/ExecutorStatus

TaskCommand를 받으면 destinations_nav2.yaml에 등록된 작업점으로 직접 주행한다.
목표는 NavigateToPose 액션으로 보내고, 완료 확인은 콜백과 타이머에서 한다.
approach 가 지정된 목적지는 접근 작업점까지 Nav2 로 간 뒤 feeder_dock 에
실행 식별자(command_id)를 주어 정밀 도킹을 시키고, 같은 식별자로 돌아온
결과만 인정한다. 자식 프로세스는 쓰지 않는다.

한 번에 하나의 명령만 실행하며, 실행 중 새로운 명령은 FAILED/BUSY로
응답한다. 같은 command_id를 다시 수신하면 작업을 재실행하지 않고
이전에 저장한 결과를 다시 발행한다.
"""

import json
import os
import time

import rclpy
import rclpy.executors
import yaml
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from smart_farm_interfaces.msg import (
    ExecutorStatus,
    TaskCommand,
    TaskResult,
)

from smart_farm_navigation import stations as stations_lib


class NavigationNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_node")
        share = get_package_share_directory("smart_farm_navigation")

        self.declare_parameter("destinations_file", os.path.join(share, "config", "destinations_nav2.yaml"))
        self.declare_parameter("timeout_s", 600.0)   # 벽시계 기준. 속도 0.3 m/s 와 Isaac 실시간 배율 0.3 이 겹치면 편도가 3 분을 넘는다
        self.declare_parameter("stations_file", stations_lib.default_path())
        self.declare_parameter("dock_timeout_s", 240.0)
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
        self.started_at = 0.0
        self.done_ids = {}

        self.stations = stations_lib.load(
            str(self.get_parameter("stations_file").value)
        )
        self.dock_timeout = float(self.get_parameter("dock_timeout_s").value)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.dock_start_pub = self.create_publisher(String, "/feeder_dock/start", 10)
        self.create_subscription(String, "/feeder_dock/result", self._on_dock_result, 10)

        # 진행 상태: None 이면 대기. goal_handle/result_future 는 NavigateToPose 용이다.
        self.stage = "IDLE"
        self.goal_handle = None
        self.result_future = None
        self.dock_result = None
        self.stage_started_at = 0.0

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

        if reason != "BUSY":          # BUSY 는 최종 결과가 아니므로 캐시하지 않는다
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
            if command.command_id == self.active.command_id:
                # 같은 명령의 재전송이다. BUSY 로 답하면 그 결과가 캐시되어 진짜 결과를 가린다.
                self.get_logger().info(
                    f"실행 중인 명령 재수신, 무시: {command.command_id}"
                )
                return

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

        station = destination.get("station", command.destination)
        if station not in self.stations.get("stations", {}):
            self.get_logger().error(f"작업점이 stations.yaml 에 없습니다: {station}")
            self._result(
                command,
                status="FAILED",
                reason="INVALID_COMMAND",
                phase="VALIDATE",
            )
            return

        self.active = command
        self.destination = destination
        self.started_at = time.monotonic()
        self.dock_result = None

        # approach 가 있으면 그 작업점까지 먼저 가고, 도착 후 정밀 도킹을 시킨다.
        first = destination.get("approach", station)
        if not self._send_goal(first):
            return

        self.stage = "NAV_APPROACH" if "approach" in destination else "NAV"
        self.stage_started_at = time.monotonic()

        self._status(
            state="BUSY",
            task_id=command.task_id,
            command_id=command.command_id,
            phase="DRIVING",
            detail=f"{command.destination} -> {first}",
        )

    # ---------- Nav2 ----------
    def _send_goal(self, station_name: str) -> bool:
        """NavigateToPose 목표를 보낸다. 콜백에서 결과를 받으므로 여기서 기다리지 않는다."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose 액션 서버가 없습니다. Nav2 가 떠 있는지 확인하십시오.")
            self._finish(status="FAILED", reason="NAV_FAILED", phase="LAUNCH")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = stations_lib.pose_of(self.stations, station_name)
        self.goal_handle = None
        self.result_future = None
        self.get_logger().info(f"goToPose {station_name}: {goal.pose.pose.position.x:.2f}, {goal.pose.pose.position.y:.2f}")
        self.nav_client.send_goal_async(goal).add_done_callback(self._on_goal_response)
        return True

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Nav2 가 목표를 거부했습니다.")
            self._finish(status="FAILED", reason="NAV_FAILED", phase="DRIVING")
            return
        self.goal_handle = handle
        self.result_future = handle.get_result_async()

    def _on_dock_result(self, message: String) -> None:
        """feeder_dock 결과. 실행 식별자가 현재 명령과 같은 것만 인정한다(래치된 과거 결과 차단)."""
        try:
            payload = json.loads(message.data)
        except ValueError:
            return
        if self.active is None or payload.get("run_id") != self.active.command_id:
            return
        self.dock_result = payload

    def _start_docking(self) -> None:
        command = self.active
        self.dock_result = None
        self.dock_start_pub.publish(String(data=command.command_id))
        self.stage = "DOCKING"
        self.stage_started_at = time.monotonic()
        self.get_logger().info(f"정밀 도킹 시작 요청: run_id={command.command_id}")
        self._status(
            state="BUSY",
            task_id=command.task_id,
            command_id=command.command_id,
            phase="DOCKING",
            detail=command.destination,
        )

    def _poll(self) -> None:
        if self.active is None:
            return

        if time.monotonic() - self.started_at > self.timeout:
            self._cancel_goal()
            self._finish(
                status="TIMEOUT",
                reason="RESULT_TIMEOUT",
                phase=self.stage,
            )
            return

        if self.stage in ("NAV", "NAV_APPROACH"):
            if self.result_future is None or not self.result_future.done():
                return

            status = self.result_future.result().status
            self.goal_handle = None
            self.result_future = None

            if status != GoalStatus.STATUS_SUCCEEDED:
                self._finish(status="FAILED", reason="NAV_FAILED", phase="DRIVING")
                return

            if self.stage == "NAV":
                self._finish(
                    status="SUCCEEDED",
                    reason="NONE",
                    phase="ARRIVED",
                    reached_station=self.active.destination,
                )
                return

            self._start_docking()
            return

        if self.stage == "DOCKING":
            if self.dock_result is None:
                if time.monotonic() - self.stage_started_at > self.dock_timeout:
                    self._finish(status="TIMEOUT", reason="RESULT_TIMEOUT", phase="DOCKING")
                return

            payload = self.dock_result
            detail = (
                f"face_dist={payload.get('face_dist_m')} "
                f"yaw_err={payload.get('yaw_err_deg')} lat={payload.get('lat_m')}"
            )
            self.get_logger().info(f"도킹 결과 {payload.get('status')}: {detail}")

            if payload.get("status") == "SUCCEEDED":
                self._finish(
                    status="SUCCEEDED",
                    reason="NONE",
                    phase="ARRIVED",
                    reached_station=self.active.destination,
                )
            else:
                self._finish(status="FAILED", reason="NAV_FAILED", phase="DOCKING")

    def _cancel_goal(self) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.result_future = None

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
        self.stage = "IDLE"
        self.goal_handle = None
        self.result_future = None
        self.dock_result = None

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
        """종료 시 남은 Nav2 목표를 취소해 카터가 계속 움직이지 않게 한다."""
        self._cancel_goal()


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
