"""recorder 노드 통합 시험. 가짜 발행자로 토픽을 쏘고 DB 를 확인한다.

Isaac 도 Nav2 도 Task Manager 도 필요 없다. 같은 프로세스 안에서 노드 둘을 돌린다.
같은 도메인에 다른 노드가 끼지 않게 도메인 77 + LOCALHOST 로 띄워 실행한다.
"""

import json
import os
import sys

import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from smart_farm_interfaces.msg import (
    CycleStatus,
    ExecutorStatus,
    TaskCommand,
    TaskResult,
)

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from smart_farm_monitor.recorder_node import RecorderNode, latched_qos  # noqa: E402
from smart_farm_monitor.store import MonitorStore  # noqa: E402


TASK = "TASK-20260928-900"


class FakePublishers(Node):
    """Task Manager·executor·도킹 자리에서 토픽을 쏘는 시험용 노드."""

    def __init__(self) -> None:
        super().__init__("fake_publishers")
        self.cycle = self.create_publisher(CycleStatus, "/cycle/status", 10)
        self.sim_command = self.create_publisher(String, "/sim_task/command", 10)
        self.sim_result = self.create_publisher(String, "/sim_task/result", 10)
        self.sim_status = self.create_publisher(String, "/sim_task/status", 10)
        self.nav_command = self.create_publisher(
            TaskCommand, "/navigation/command", 10)
        self.nav_result = self.create_publisher(
            TaskResult, "/navigation/result", 10)
        self.nav_status = self.create_publisher(
            ExecutorStatus, "/navigation/status", 10)
        self.detections = self.create_publisher(
            String, "/inspection/detections_2d", 10)
        self.dock_result = self.create_publisher(
            String, "/feeder_dock/result", latched_qos())
        self.amcl = self.create_publisher(
            PoseWithCovarianceStamped, "/amcl_pose", 10)


@pytest.fixture()
def rig(tmp_path):
    """recorder 와 가짜 발행자를 띄우고, 끝나면 정리한다."""

    rclpy.init()
    db_path = str(tmp_path / "farm.db")
    recorder = RecorderNode(
        parameter_overrides=[Parameter("db_path", value=db_path)])
    publishers = FakePublishers()

    def pump(rounds: int = 40) -> None:
        """양쪽 노드를 번갈아 돌려 메시지가 도착하게 한다."""

        for _ in range(rounds):
            rclpy.spin_once(publishers, timeout_sec=0.01)
            rclpy.spin_once(recorder, timeout_sec=0.01)

    pump(20)  # 구독·발행 연결이 맺어질 시간을 준다
    try:
        yield recorder, publishers, pump, db_path
    finally:
        publishers.destroy_node()
        recorder.destroy_node()
        rclpy.shutdown()


def _cycle_msg(state: str, status: str = "RUNNING",
               command_id: str = "", reason: str = "NONE") -> CycleStatus:
    msg = CycleStatus()
    msg.task_id = TASK
    msg.scenario_id = "DEMO_HARVEST_01"
    msg.state = state
    msg.active_command_id = command_id
    msg.status = status
    msg.reason = reason
    return msg


def _read(db_path: str) -> MonitorStore:
    return MonitorStore(db_path, read_only=True)


def test_cycle_open_and_close(rig):
    _, publishers, pump, db_path = rig

    publishers.cycle.publish(_cycle_msg("TRANSFER"))
    pump()
    publishers.cycle.publish(_cycle_msg("COMPLETE", "SUCCEEDED"))
    pump()

    store = _read(db_path)
    row = store.latest_cycle()
    assert row["task_id"] == TASK
    assert row["scenario_id"] == "DEMO_HARVEST_01"
    assert row["final_state"] == "COMPLETE"
    assert row["terminal_status"] == "SUCCEEDED"
    assert row["ended_wall"] is not None

    live = store.live_values()
    assert json.loads(live["cycle"]["value"])["state"] == "COMPLETE"
    store.close()


def test_sim_command_and_result_pair_up(rig):
    _, publishers, pump, db_path = rig

    publishers.cycle.publish(_cycle_msg("TRANSFER"))
    pump()

    command = {
        "task_id": TASK, "command_id": TASK + "-CMD-001",
        "operation": "TRANSFER", "recipe_id": "RACK_REARRANGE_01",
        "pallet_id": "", "source": "", "destination": "", "target_slots": [],
    }
    publishers.sim_command.publish(String(data=json.dumps(command)))
    pump()

    result = dict(command)
    result.update({"status": "SUCCEEDED", "phase": "RESULT", "reason": "NONE",
                   "safe_to_navigate": False, "reached_station": "",
                   "completed_units": [], "defect_slots": [],
                   "unknown_slots": []})
    publishers.sim_result.publish(String(data=json.dumps(result)))
    pump()

    store = _read(db_path)
    steps = store.steps(TASK)
    assert len(steps) == 1
    step = steps[0]
    assert step["executor"] == "sim_task"
    assert step["operation"] == "TRANSFER"
    assert step["cycle_state"] == "TRANSFER"
    assert step["seq"] == 1
    assert step["status"] == "SUCCEEDED"
    assert step["late_result"] == 0
    assert step["duration_wall"] is not None
    store.close()


def test_navigation_result_records_station_and_pallet_move(rig):
    _, publishers, pump, db_path = rig

    publishers.cycle.publish(_cycle_msg("NAVIGATION"))
    pump()

    command = TaskCommand()
    command.task_id = TASK
    command.command_id = TASK + "-CMD-003"
    command.operation = "NAVIGATION"
    command.pallet_id = "PALLET_001"
    command.source = "RACK_L2"
    command.destination = "FEEDER_DOCK"
    publishers.nav_command.publish(command)
    pump()

    result = TaskResult()
    result.task_id = TASK
    result.command_id = command.command_id
    result.operation = "NAVIGATION"
    result.status = "SUCCEEDED"
    result.phase = "RESULT"
    result.reason = "NONE"
    result.reached_station = "FEEDER_DOCK"
    publishers.nav_result.publish(result)
    pump()

    store = _read(db_path)
    step = store.step_by_command(TASK, command.command_id)
    assert step["executor"] == "navigation"
    assert step["reached_station"] == "FEEDER_DOCK"

    move = store.pallet_moves(TASK)[0]
    assert move["pallet_id"] == "PALLET_001"
    assert move["location"] == "FEEDER_DOCK"
    assert move["source"] == "RACK_L2"
    store.close()


def test_detections_and_dock_result(rig):
    _, publishers, pump, db_path = rig

    publishers.cycle.publish(_cycle_msg("NAVIGATION"))
    pump()

    # 도킹 결과를 붙일 대상이 되도록 navigation 명령을 열어 둔다.
    command = TaskCommand()
    command.task_id = TASK
    command.command_id = TASK + "-CMD-004"
    command.operation = "NAVIGATION"
    publishers.nav_command.publish(command)
    pump()

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.pose.pose.position.x = 1.25
    pose.pose.pose.position.y = -3.10
    pose.pose.pose.orientation.z = 0.7071068
    pose.pose.pose.orientation.w = 0.7071068
    publishers.amcl.publish(pose)
    pump()

    publishers.dock_result.publish(String(data=json.dumps({
        "run_id": "RUN-1", "status": "SUCCEEDED", "reason": "OK",
        "face_dist_m": 0.921, "yaw_err_deg": -0.8, "lat_m": 0.037,
        "retry": 2,
    })))
    pump()

    publishers.detections.publish(String(data=json.dumps({
        "header": {"stamp": {"sec": 100, "nanosec": 0}, "frame_id": "camera_rgb"},
        "task_id": TASK, "command_id": TASK + "-CMD-005",
        "pallet_id": "PALLET_001", "image_width": 1280, "image_height": 720,
        "detections": [{"slot_id": "SLOT_03", "class_name": "lettuce_yellow",
                        "confidence": 0.81, "center_u": 300.0, "center_v": 200.0,
                        "bbox_x_min": 280.0, "bbox_y_min": 180.0,
                        "bbox_x_max": 320.0, "bbox_y_max": 220.0}],
    })))
    pump()

    store = _read(db_path)

    attempt = store.dock_attempts()[0]
    assert attempt["run_id"] == "RUN-1"
    assert attempt["retry"] == 2
    assert attempt["command_id"] == command.command_id
    assert attempt["map_x"] == 1.25
    # map 좌표계 yaw 90 도 (쿼터니언 z=w=0.7071)
    assert attempt["map_yaw_deg"] == pytest.approx(90.0, abs=0.1)

    rows = store.detections(TASK)
    assert len(rows) == 1
    assert rows[0]["slot_id"] == "SLOT_03"
    assert rows[0]["pass_no"] == 1
    store.close()


def test_status_is_not_recorded_twice(rig):
    _, publishers, pump, db_path = rig

    status = {"executor": "sim_task", "state": "READY", "task_id": "",
              "command_id": "", "operation": "", "phase": "IDLE",
              "detail": "ready"}
    for _ in range(3):
        publishers.sim_status.publish(String(data=json.dumps(status)))
        pump(10)

    nav_status = ExecutorStatus()
    nav_status.executor = "navigation"
    nav_status.state = "READY"
    nav_status.phase = "IDLE"
    nav_status.detail = "nav ready"
    publishers.nav_status.publish(nav_status)
    pump()

    store = _read(db_path)
    rows = store.conn.execute(
        "SELECT executor, COUNT(*) AS n FROM executor_status GROUP BY executor"
    ).fetchall()
    counts = {row["executor"]: row["n"] for row in rows}
    # 같은 값이 세 번 와도 한 줄만 남는다.
    assert counts["sim_task"] == 1
    assert counts["navigation"] == 1
    store.close()


def test_web_command_is_rejected_without_task_manager(rig):
    _, _, pump, db_path = rig

    writer = MonitorStore(db_path)
    row_id = writer.queue_web_command(
        "START_CYCLE", {"scenario_id": "DEMO_HARVEST_01"}, 1.0)
    writer.close()

    pump(60)

    store = _read(db_path)
    row = store.conn.execute(
        "SELECT * FROM web_command WHERE id = ?", (row_id,)).fetchone()
    assert row["state"] == "REJECTED"
    assert json.loads(row["response"])["reason"] == "SERVICE_NOT_AVAILABLE"
    store.close()
