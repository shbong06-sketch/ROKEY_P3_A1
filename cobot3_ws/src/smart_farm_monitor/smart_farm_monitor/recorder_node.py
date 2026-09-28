"""공정 토픽을 받아 관제 DB 에 기록하고, 웹이 넣은 명령을 서비스로 전달한다.

이 노드가 DB 에 쓰는 유일한 프로세스다. 웹(`web_app.py`)은 같은 파일을 읽기만 한다.

시각을 두 가지로 적는다.
  wall : `time.time()`. 행의 순서와 실제 경과를 보기 위한 **기록값**이다.
         타임아웃이나 신선도 판정에는 쓰지 않는다(ADR_basic §5-6).
  sim  : 노드 시계. `use_sim_time: true` 로 띄우면 `/clock` 시각이 들어오고,
         공정 소요시간 분석은 이 값으로만 한다.
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String

from smart_farm_interfaces.msg import (
    CycleStatus,
    ExecutorStatus,
    TaskCommand,
    TaskResult,
)
from smart_farm_interfaces.srv import StartCycle

from .store import MonitorStore


DEFAULT_DB_PATH = (
    "/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_monitor/data/farm.db"
)

# 사이클이 끝났다고 보는 상태
FINAL_STATES = ("COMPLETE", "ERROR")

# 검사 판정을 남기는 operation
INSPECT_OPERATIONS = ("INSPECT", "RECHECK")


def yaw_degrees(orientation) -> float:
    """쿼터니언에서 z 축 회전(yaw)만 도 단위로 뽑는다. map 좌표계 기준이다."""

    # 평면 주행이라 roll·pitch 는 0 에 가깝다. z 축 회전만 쓰면 충분하다.
    siny = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy = 1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2)
    return math.degrees(math.atan2(siny, cosy))


def latched_qos() -> QoSProfile:
    """늦게 붙어도 마지막 값을 받는 QoS. feeder_dock 이 이걸로 발행한다."""

    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class RecorderNode(Node):
    """공정 기록 노드."""

    def __init__(self, parameter_overrides=None) -> None:
        """노드를 만든다. parameter_overrides 는 시험에서 db_path 를 바꿀 때 쓴다."""

        super().__init__(
            "monitor_recorder",
            parameter_overrides=parameter_overrides or [],
        )

        self.declare_parameter("db_path", DEFAULT_DB_PATH)
        self.declare_parameter("web_poll_sec", 0.5)
        self.declare_parameter("status_keepalive_sec", 10.0)

        self.db_path = str(self.get_parameter("db_path").value)
        self.store = MonitorStore(self.db_path)

        # 사이클 추적 상태
        self.current_task_id = ""
        self.current_state = ""
        self.closed_task_ids = set()

        # executor 상태 중복 기록 방지: executor -> (직전 튜플, 마지막 기록 wall)
        self.last_status = {}

        # 도킹 결과에 붙일 마지막 map 좌표계 자세
        self.last_map_pose = None
        self.last_dock_run_id = ""

        self._make_subscriptions()

        self.start_cycle_client = self.create_client(StartCycle, "/start_cycle")
        self.create_timer(
            float(self.get_parameter("web_poll_sec").value),
            self._drain_web_commands,
        )

        self.get_logger().info(f"monitor_recorder ready: db={self.db_path}")

    # ------------------------------------------------------------------
    # 구독 배선
    # ------------------------------------------------------------------

    def _make_subscriptions(self) -> None:
        """기록할 토픽을 모두 구독한다."""

        self.create_subscription(
            CycleStatus, "/cycle/status", self._on_cycle_status, 10)

        self.create_subscription(
            String, "/sim_task/command", self._on_sim_command, 10)
        self.create_subscription(
            String, "/sim_task/result", self._on_sim_result, 10)
        self.create_subscription(
            String, "/sim_task/status", self._on_sim_status, 10)
        self.create_subscription(
            String, "/sim_task/inspection_data_status",
            self._on_inspection_data_status, 10)

        for executor in ("navigation", "inspection"):
            self.create_subscription(
                TaskCommand, f"/{executor}/command",
                lambda msg, name=executor: self._on_ros_command(msg, name), 10)
            self.create_subscription(
                TaskResult, f"/{executor}/result",
                lambda msg, name=executor: self._on_ros_result(msg, name), 10)
            self.create_subscription(
                ExecutorStatus, f"/{executor}/status",
                lambda msg, name=executor: self._on_ros_status(msg, name), 10)

        self.create_subscription(
            String, "/inspection/detections_2d", self._on_detections, 10)

        self.create_subscription(
            String, "/feeder_dock/result", self._on_dock_result, latched_qos())
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10)

    # ------------------------------------------------------------------
    # 시각
    # ------------------------------------------------------------------

    def _now(self):
        """(wall, sim) 두 시각을 돌려준다. 둘 다 기록용이다."""

        sim = self.get_clock().now().nanoseconds * 1e-9
        return time.time(), sim

    # ------------------------------------------------------------------
    # 사이클
    # ------------------------------------------------------------------

    def _on_cycle_status(self, msg: CycleStatus) -> None:
        """현재 공정 단계와 사이클 결과를 기록한다."""

        wall, sim = self._now()

        if msg.task_id and msg.task_id != self.current_task_id:
            self.current_task_id = msg.task_id
            if self.store.open_cycle(msg.task_id, msg.scenario_id, wall, sim):
                self.get_logger().info(f"cycle opened: {msg.task_id}")

        self.current_state = msg.state

        if msg.task_id:
            if msg.state in FINAL_STATES:
                if msg.task_id not in self.closed_task_ids:
                    self.closed_task_ids.add(msg.task_id)
                    self.store.close_cycle(msg.task_id, msg.state, msg.status,
                                           msg.reason, wall, sim)
                    self.get_logger().info(
                        f"cycle closed: {msg.task_id} "
                        f"state={msg.state} status={msg.status} reason={msg.reason}")
            else:
                self.store.update_cycle(msg.task_id, msg.status, msg.reason)

        self.store.set_live("cycle", {
            "task_id": msg.task_id,
            "scenario_id": msg.scenario_id,
            "state": msg.state,
            "active_command_id": msg.active_command_id,
            "status": msg.status,
            "reason": msg.reason,
        }, wall, sim)

    # ------------------------------------------------------------------
    # 명령·결과
    # ------------------------------------------------------------------

    def _on_sim_command(self, msg: String) -> None:
        """/sim_task/command JSON 을 기록한다."""

        payload = self._parse_json(msg.data, "/sim_task/command")
        if payload is None:
            return
        wall, sim = self._now()
        self.store.record_command(
            payload.get("task_id", ""), payload.get("command_id", ""),
            "sim_task", payload.get("operation", ""),
            payload.get("recipe_id", ""), payload.get("pallet_id", ""),
            payload.get("source", ""), payload.get("destination", ""),
            payload.get("target_slots") or [], self.current_state, wall, sim)

    def _on_sim_result(self, msg: String) -> None:
        """/sim_task/result JSON 을 기록한다."""

        payload = self._parse_json(msg.data, "/sim_task/result")
        if payload is None:
            return
        self._store_result(
            task_id=payload.get("task_id", ""),
            command_id=payload.get("command_id", ""),
            operation=payload.get("operation", ""),
            status=payload.get("status", ""),
            phase=payload.get("phase", ""),
            reason=payload.get("reason", "NONE"),
            reached_station=payload.get("reached_station", ""),
            pallet_id=payload.get("pallet_id", ""),
            defect_slots=payload.get("defect_slots") or [],
            unknown_slots=payload.get("unknown_slots") or [],
        )

    def _on_ros_command(self, msg: TaskCommand, executor: str) -> None:
        """navigation·inspection 의 TaskCommand 를 기록한다."""

        wall, sim = self._now()
        self.store.record_command(
            msg.task_id, msg.command_id, executor, msg.operation,
            msg.recipe_id, msg.pallet_id, msg.source, msg.destination,
            list(msg.target_slots), self.current_state, wall, sim)

    def _on_ros_result(self, msg: TaskResult, executor: str) -> None:
        """navigation·inspection 의 TaskResult 를 기록한다."""

        self._store_result(
            task_id=msg.task_id,
            command_id=msg.command_id,
            operation=msg.operation,
            status=msg.status,
            phase=msg.phase,
            reason=msg.reason,
            reached_station=msg.reached_station,
            pallet_id="",
            defect_slots=list(msg.defect_slots),
            unknown_slots=list(msg.unknown_slots),
        )

    def _store_result(self, task_id, command_id, operation, status, phase,
                      reason, reached_station, pallet_id,
                      defect_slots, unknown_slots) -> None:
        """결과 한 건을 step 에 넣고, 파생 기록(검사 판정·팔레트 이동)을 남긴다."""

        wall, sim = self._now()
        step = self.store.step_by_command(task_id, command_id)

        late = self.store.record_result(task_id, command_id, operation, status,
                                        phase, reason, reached_station, wall, sim)
        if late:
            self.get_logger().warning(
                "late result: "
                f"command_id={command_id} operation={operation} status={status}")

        if operation in INSPECT_OPERATIONS:
            self.store.record_inspection_verdict(
                task_id, command_id,
                pallet_id or (step["pallet_id"] if step is not None else ""),
                operation, defect_slots, unknown_slots, wall, sim)

        if late or status != "SUCCEEDED" or step is None:
            return

        # 팔레트가 어디로 갔는지 남긴다. destination 이 비면 단계 이름으로 대신한다.
        # (Task Manager 가 pallet_locations 를 발행하면 그 값으로 바꾼다.)
        moved_pallet = pallet_id or step["pallet_id"]
        if moved_pallet:
            location = step["destination"] or reached_station or step["cycle_state"]
            self.store.record_pallet_move(task_id, command_id, moved_pallet,
                                          location or "", step["source"] or "",
                                          wall, sim)

    # ------------------------------------------------------------------
    # executor 상태
    # ------------------------------------------------------------------

    def _on_sim_status(self, msg: String) -> None:
        """/sim_task/status JSON 을 기록한다."""

        payload = self._parse_json(msg.data, "/sim_task/status")
        if payload is None:
            return
        self._store_status(
            payload.get("executor", "sim_task"), payload.get("state", ""),
            payload.get("task_id", ""), payload.get("command_id", ""),
            payload.get("operation", ""), payload.get("phase", ""),
            payload.get("detail", ""))

    def _on_ros_status(self, msg: ExecutorStatus, executor: str) -> None:
        """navigation·inspection 의 ExecutorStatus 를 기록한다."""

        self._store_status(msg.executor or executor, msg.state, msg.task_id,
                           msg.command_id, msg.operation, msg.phase, msg.detail)

    def _on_inspection_data_status(self, msg: String) -> None:
        """검사 데이터가 Sim 에 저장됐는지(STORED/REJECTED)를 기록한다."""

        payload = self._parse_json(msg.data, "/sim_task/inspection_data_status")
        if payload is None:
            return
        self._store_status(
            "sim_task_data", payload.get("state", ""),
            payload.get("task_id", ""),
            payload.get("inspection_command_id", ""), "", "",
            json.dumps(payload, ensure_ascii=False))

    def _store_status(self, executor, state, task_id, command_id, operation,
                      phase, detail) -> None:
        """값이 바뀐 순간과 생존 표본만 남긴다. heartbeat 전부를 넣지 않는다."""

        wall, sim = self._now()
        identity = (state, task_id, command_id, operation, phase, detail)
        previous = self.last_status.get(executor)
        keepalive = float(self.get_parameter("status_keepalive_sec").value)

        if previous is not None:
            same_value = previous[0] == identity
            fresh = (wall - previous[1]) < keepalive
            if same_value and fresh:
                return

        self.last_status[executor] = (identity, wall)
        self.store.record_executor_status(executor, state, task_id, command_id,
                                         operation, phase, detail, wall, sim)

    # ------------------------------------------------------------------
    # 검사 검출·도킹·자세
    # ------------------------------------------------------------------

    def _on_detections(self, msg: String) -> None:
        """/inspection/detections_2d JSON 을 슬롯별 행으로 기록한다."""

        payload = self._parse_json(msg.data, "/inspection/detections_2d")
        if payload is None:
            return
        wall, _ = self._now()

        # RECHECK 로 다시 찍은 것인지 구분한다.
        step = self.store.step_by_command(payload.get("task_id", ""),
                                         payload.get("command_id", ""))
        pass_no = 2 if step is not None and step["operation"] == "RECHECK" else 1

        count = self.store.record_detections(payload, pass_no, wall)
        self.get_logger().info(
            f"detections recorded: {count} rows, pass={pass_no}, "
            f"command_id={payload.get('command_id', '')}")

    def _on_dock_result(self, msg: String) -> None:
        """/feeder_dock/result JSON 을 기록한다. latched 라 재발행을 걸러낸다."""

        payload = self._parse_json(msg.data, "/feeder_dock/result")
        if payload is None:
            return

        run_id = str(payload.get("run_id", ""))
        if run_id and run_id == self.last_dock_run_id:
            return
        self.last_dock_run_id = run_id

        wall, sim = self._now()
        step = self.store.last_open_step("navigation")
        task_id = step["task_id"] if step is not None else self.current_task_id
        command_id = step["command_id"] if step is not None else ""

        self.store.record_dock_attempt(task_id or "", command_id, payload,
                                       self.last_map_pose, wall, sim)
        self.get_logger().info(
            f"dock recorded: run_id={run_id} status={payload.get('status', '')} "
            f"lat_m={payload.get('lat_m')} retry={payload.get('retry')}")

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """map 좌표계 로봇 자세를 최신값으로만 유지한다(이력으로 쌓지 않는다)."""

        wall, sim = self._now()
        pose = {
            "x": round(msg.pose.pose.position.x, 3),
            "y": round(msg.pose.pose.position.y, 3),
            "yaw_deg": round(yaw_degrees(msg.pose.pose.orientation), 2),
            "frame_id": msg.header.frame_id,
        }
        self.last_map_pose = pose
        self.store.set_live("robot_pose", pose, wall, sim)

    # ------------------------------------------------------------------
    # 웹 -> ROS 우편함
    # ------------------------------------------------------------------

    def _drain_web_commands(self) -> None:
        """웹이 넣은 대기 명령 한 건을 집어 서비스로 보낸다."""

        row = self.store.take_pending_web_command()
        if row is None:
            return

        row_id = int(row["id"])
        if row["kind"] != "START_CYCLE":
            self.store.finish_web_command(
                row_id, "REJECTED", {"reason": "UNKNOWN_KIND"})
            return

        if not self.start_cycle_client.service_is_ready():
            self.store.finish_web_command(
                row_id, "REJECTED", {"reason": "SERVICE_NOT_AVAILABLE"})
            self.get_logger().warning("/start_cycle 이 아직 없다")
            return

        payload = self._parse_json(row["payload"] or "{}", "web_command") or {}
        request = StartCycle.Request()
        request.scenario_id = payload.get("scenario_id", "DEMO_HARVEST_01")

        future = self.start_cycle_client.call_async(request)
        future.add_done_callback(
            lambda done, rid=row_id: self._on_start_cycle_response(done, rid))
        self.get_logger().info(f"start_cycle 요청: {request.scenario_id}")

    def _on_start_cycle_response(self, future, row_id: int) -> None:
        """서비스 응답을 우편함 행에 적는다."""

        try:
            response = future.result()
        except Exception as error:  # 서비스 호출 실패
            self.store.finish_web_command(
                row_id, "REJECTED", {"reason": "CALL_FAILED",
                                     "detail": str(error)})
            self.get_logger().error(f"start_cycle 실패: {error}")
            return

        state = "ACCEPTED" if response.accepted else "REJECTED"
        self.store.finish_web_command(row_id, state, {
            "accepted": bool(response.accepted),
            "task_id": response.task_id,
            "reason": response.reason,
        })
        self.get_logger().info(
            f"start_cycle 응답: accepted={response.accepted} "
            f"task_id={response.task_id} reason={response.reason}")

    # ------------------------------------------------------------------

    def _parse_json(self, text: str, where: str):
        """JSON 한 개를 읽는다. 형식이 틀리면 경고만 남기고 None 을 준다."""

        try:
            payload = json.loads(text)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f"{where} JSON 파싱 실패: {error}")
            return None
        if not isinstance(payload, dict):
            self.get_logger().warning(f"{where} 최상위가 object 가 아니다")
            return None
        return payload

    def destroy_node(self) -> bool:
        """DB 연결을 닫고 노드를 정리한다."""

        self.store.close()
        return super().destroy_node()


def main(args=None) -> None:
    """노드를 실행한다."""

    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
