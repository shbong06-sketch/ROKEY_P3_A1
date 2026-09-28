"""MonitorStore 회귀 시험. ROS·Isaac 없이 돌아간다."""

import json
import os
import sys

import pytest

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from smart_farm_monitor.store import (  # noqa: E402
    MonitorStore,
    command_sequence,
)


TASK = "TASK-20260928-001"


@pytest.fixture()
def store(tmp_path):
    """시험마다 새 DB 파일을 쓴다."""

    instance = MonitorStore(str(tmp_path / "farm.db"))
    yield instance
    instance.close()


def test_command_sequence():
    assert command_sequence("TASK-20260928-001-CMD-005") == 5
    assert command_sequence("TASK-20260928-001") is None


def test_open_cycle_is_idempotent(store):
    assert store.open_cycle(TASK, "DEMO_HARVEST_01", 100.0, 10.0) is True
    assert store.open_cycle(TASK, "DEMO_HARVEST_01", 200.0, 20.0) is False

    row = store.latest_cycle()
    assert row["scenario_id"] == "DEMO_HARVEST_01"
    assert row["started_wall"] == 100.0
    assert row["started_sim"] == 10.0
    assert row["terminal_status"] == "RUNNING"


def test_close_cycle_does_not_overwrite(store):
    store.open_cycle(TASK, "DEMO_HARVEST_01", 100.0, 10.0)
    store.close_cycle(TASK, "COMPLETE", "SUCCEEDED", "NONE", 400.0, 100.0)
    store.close_cycle(TASK, "ERROR", "FAILED", "LATE", 500.0, 200.0)

    row = store.latest_cycle()
    assert row["final_state"] == "COMPLETE"
    assert row["ended_sim"] == 100.0


def _issue(store, command_id, operation="TRANSFER", executor="sim_task",
           wall=100.0, sim=10.0, destination="", pallet_id=""):
    store.record_command(TASK, command_id, executor, operation, "RECIPE",
                         pallet_id, "", destination, [], "TRANSFER", wall, sim)


def test_result_fills_both_durations(store):
    store.open_cycle(TASK, "DEMO_HARVEST_01", 100.0, 10.0)
    _issue(store, "CMD-001", wall=100.0, sim=10.0)

    late = store.record_result(TASK, "CMD-001", "TRANSFER", "SUCCEEDED",
                               "RESULT", "NONE", "", 130.0, 19.0)

    assert late is False
    row = store.step_by_command(TASK, "CMD-001")
    # 벽시계로 30 초 걸렸지만 시뮬레이션 안에서는 9 초였다.
    assert row["duration_wall"] == 30.0
    assert row["duration_sim"] == 9.0
    assert row["status"] == "SUCCEEDED"
    assert row["late_result"] == 0


def test_second_result_is_marked_late(store):
    store.open_cycle(TASK, "DEMO_HARVEST_01", 100.0, 10.0)
    _issue(store, "CMD-002", wall=100.0, sim=10.0)
    store.record_result(TASK, "CMD-002", "CULL", "FAILED", "TIMEOUT",
                        "TIMEOUT", "", 200.0, 40.0)

    late = store.record_result(TASK, "CMD-002", "CULL", "SUCCEEDED",
                               "RESULT", "NONE", "", 260.0, 58.0)

    assert late is True
    rows = store.steps(TASK)
    assert len(rows) == 2
    first, second = rows
    assert first["status"] == "FAILED" and first["late_result"] == 0
    assert second["status"] == "SUCCEEDED" and second["late_result"] == 1
    # 늦은 결과도 발행 시각 기준으로 소요시간을 채운다.
    assert second["duration_sim"] == 48.0


def test_result_without_command_is_recorded(store):
    late = store.record_result(TASK, "CMD-099", "INSPECT", "SUCCEEDED",
                               "RESULT", "NONE", "", 300.0, 50.0)

    assert late is True
    rows = store.steps(TASK)
    assert len(rows) == 1
    assert rows[0]["duration_sim"] is None


def test_elapsed_rejects_rewound_clock(store):
    store.open_cycle(TASK, "DEMO_HARVEST_01", 100.0, 50.0)
    _issue(store, "CMD-003", wall=100.0, sim=50.0)

    # 장면을 Stop -> Play 하면 /clock 이 되감긴다.
    store.record_result(TASK, "CMD-003", "TRANSFER", "SUCCEEDED", "RESULT",
                        "NONE", "", 140.0, 5.0)

    row = store.step_by_command(TASK, "CMD-003")
    assert row["duration_wall"] == 40.0
    assert row["duration_sim"] is None


def test_last_open_step(store):
    _issue(store, "CMD-004", operation="NAVIGATION", executor="navigation")
    row = store.last_open_step("navigation")
    assert row["command_id"] == "CMD-004"

    store.record_result(TASK, "CMD-004", "NAVIGATION", "SUCCEEDED", "RESULT",
                        "NONE", "FEEDER_DOCK", 150.0, 20.0)
    assert store.last_open_step("navigation") is None


def test_record_detections(store):
    payload = {
        "header": {"stamp": {"sec": 1790074056, "nanosec": 500000000},
                   "frame_id": "camera_rgb"},
        "task_id": TASK,
        "command_id": "CMD-005",
        "pallet_id": "PALLET_001",
        "image_width": 1280,
        "image_height": 720,
        "detections": [
            {"slot_id": "SLOT_01", "class_name": "lettuce_dark_green",
             "confidence": 0.93, "center_u": 214.5, "center_v": 181.0,
             "bbox_x_min": 170.0, "bbox_y_min": 120.0,
             "bbox_x_max": 259.0, "bbox_y_max": 242.0},
            {"slot_id": "", "class_name": "lettuce_yellow", "confidence": 0.71,
             "center_u": 500.0, "center_v": 300.0, "bbox_x_min": 480.0,
             "bbox_y_min": 280.0, "bbox_x_max": 520.0, "bbox_y_max": 320.0},
        ],
    }

    assert store.record_detections(payload, pass_no=1, wall=310.0) == 2

    rows = store.detections(TASK)
    assert [row["slot_id"] for row in rows] == ["SLOT_01", ""]
    assert rows[0]["image_width"] == 1280
    assert rows[0]["stamp_sim"] == pytest.approx(1790074056.5)
    assert rows[1]["pass_no"] == 1


def test_record_detections_with_empty_list(store):
    assert store.record_detections({"detections": []}, pass_no=2, wall=1.0) == 0


def test_verdict_and_pallet_move(store):
    store.record_inspection_verdict(TASK, "CMD-006", "PALLET_001", "INSPECT",
                                    ("SLOT_02",), (), 320.0, 55.0)
    store.record_pallet_move(TASK, "CMD-006", "PALLET_001", "INSPECT_STATION",
                             "CARRY", 320.0, 55.0)

    verdict = store.verdicts(TASK)[0]
    assert json.loads(verdict["defect_slots"]) == ["SLOT_02"]
    assert json.loads(verdict["unknown_slots"]) == []

    move = store.pallet_moves(TASK)[0]
    assert move["location"] == "INSPECT_STATION"
    assert move["source"] == "CARRY"


def test_dock_attempt_and_kpi(store):
    store.open_cycle(TASK, "DEMO_HARVEST_01", 100.0, 10.0)
    store.close_cycle(TASK, "COMPLETE", "SUCCEEDED", "NONE", 700.0, 130.0)

    store.record_dock_attempt(
        TASK, "CMD-007",
        {"run_id": "R1", "status": "SUCCEEDED", "reason": "OK",
         "face_dist_m": 0.92, "yaw_err_deg": 1.2, "lat_m": -0.041, "retry": 1},
        {"x": 1.25, "y": -3.1, "yaw_deg": 90.4},
        710.0, 131.0,
    )

    attempt = store.dock_attempts()[0]
    assert attempt["retry"] == 1
    assert attempt["map_x"] == 1.25
    assert attempt["lat_m"] == -0.041

    kpi = store.kpi()
    assert kpi["cycle_total"] == 1
    assert kpi["cycle_succeeded"] == 1
    assert kpi["cycle_avg_sim_sec"] == pytest.approx(120.0)
    assert kpi["dock_total"] == 1
    assert kpi["dock_avg_retry"] == pytest.approx(1.0)


def test_step_durations_excludes_late_rows(store):
    _issue(store, "CMD-010", operation="CULL", wall=100.0, sim=10.0)
    store.record_result(TASK, "CMD-010", "CULL", "FAILED", "TIMEOUT",
                        "TIMEOUT", "", 200.0, 40.0)
    store.record_result(TASK, "CMD-010", "CULL", "SUCCEEDED", "RESULT",
                        "NONE", "", 400.0, 100.0)

    stats = store.step_durations()
    assert len(stats) == 1
    assert stats[0]["operation"] == "CULL"
    assert stats[0]["n"] == 1
    assert stats[0]["avg_sim"] == pytest.approx(30.0)


def test_web_command_mailbox(store):
    row_id = store.queue_web_command(
        "START_CYCLE", {"scenario_id": "DEMO_HARVEST_01"}, 500.0)

    taken = store.take_pending_web_command()
    assert taken["id"] == row_id
    assert json.loads(taken["payload"])["scenario_id"] == "DEMO_HARVEST_01"
    # 한 번 집어간 명령은 다시 나오지 않는다.
    assert store.take_pending_web_command() is None

    store.finish_web_command(row_id, "ACCEPTED",
                             {"accepted": True, "task_id": TASK})
    saved = store.conn.execute(
        "SELECT * FROM web_command WHERE id = ?", (row_id,)).fetchone()
    assert saved["state"] == "ACCEPTED"
    assert json.loads(saved["response"])["task_id"] == TASK


def test_live_values_overwrite(store):
    store.set_live("robot_pose", {"x": 1.0, "y": 2.0}, 600.0, 60.0)
    store.set_live("robot_pose", {"x": 3.0, "y": 4.0}, 601.0, 61.0)

    values = store.live_values()
    assert len(values) == 1
    assert json.loads(values["robot_pose"]["value"])["x"] == 3.0
    assert values["robot_pose"]["updated_sim"] == 61.0


def test_executor_states_keeps_latest_per_executor(store):
    store.record_executor_status("sim_task", "STARTING", "", "", "", "BOOT",
                                 "init", 100.0, 1.0)
    store.record_executor_status("sim_task", "READY", "", "", "", "IDLE",
                                 "ready", 101.0, 2.0)
    store.record_executor_status("navigation", "READY", "", "", "", "IDLE",
                                 "nav ready", 102.0, 3.0)

    states = {row["executor"]: row["state"] for row in store.executor_states()}
    assert states == {"sim_task": "READY", "navigation": "READY"}


def test_anomaly_label(store):
    store.add_anomaly_label(TASK, "CMD-010", "ANOMALY", "CULL 이 길었음", 900.0)
    row = store.conn.execute("SELECT * FROM anomaly_label").fetchone()
    assert row["label"] == "ANOMALY"
