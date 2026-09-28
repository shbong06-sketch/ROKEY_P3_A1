"""관제 웹 회귀 시험. ROS·Isaac 없이 돌아간다.

TestClient 로 앱을 직접 두드려 화면과 API 응답을 확인한다.
"""

import json
import os
import sys

import pytest
from fastapi.testclient import TestClient

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from smart_farm_monitor.store import MonitorStore  # noqa: E402
from smart_farm_monitor.web_app import (  # noqa: E402
    create_app,
    find_web_dir,
    load_map_meta,
    png_size,
)


TASK = "TASK-20260928-700"
MAP_YAML = ("/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/maps/"
            "Collected_smartfarm_v014.yaml")
STATIONS_YAML = ("/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/"
                 "config/stations.yaml")


def seed(db_path: str) -> None:
    """한 사이클 분량의 기록을 만들어 둔다."""

    store = MonitorStore(db_path)
    store.open_cycle(TASK, "DEMO_HARVEST_01", 1000.0, 100.0)

    store.record_command(TASK, TASK + "-CMD-001", "sim_task", "TRANSFER",
                         "RACK_REARRANGE_01", "", "", "", [], "TRANSFER",
                         1000.0, 100.0)
    store.record_result(TASK, TASK + "-CMD-001", "TRANSFER", "SUCCEEDED",
                        "RESULT", "NONE", "", 1100.0, 130.0)

    store.record_command(TASK, TASK + "-CMD-003", "navigation", "NAVIGATION",
                         "", "PALLET_001", "RACK_L2", "FEEDER_DOCK", [],
                         "NAVIGATION", 1100.0, 130.0)
    store.record_result(TASK, TASK + "-CMD-003", "NAVIGATION", "SUCCEEDED",
                        "RESULT", "NONE", "FEEDER_DOCK", 1200.0, 160.0)
    store.record_pallet_move(TASK, TASK + "-CMD-003", "PALLET_001",
                             "FEEDER_DOCK", "RACK_L2", 1200.0, 160.0)

    store.record_dock_attempt(
        TASK, TASK + "-CMD-003",
        {"run_id": "RUN-7", "status": "SUCCEEDED", "reason": "OK",
         "face_dist_m": 0.921, "yaw_err_deg": -0.8, "lat_m": 0.057, "retry": 1},
        {"x": -2.19, "y": -2.73, "yaw_deg": 90.0}, 1201.0, 161.0)

    store.record_detections({
        "header": {"stamp": {"sec": 200, "nanosec": 0}, "frame_id": "camera_rgb"},
        "task_id": TASK, "command_id": TASK + "-CMD-008",
        "pallet_id": "PALLET_001", "image_width": 1280, "image_height": 720,
        "detections": [
            {"slot_id": "SLOT_01", "class_name": "lettuce_dark_green",
             "confidence": 0.93, "center_u": 214.5, "center_v": 181.0,
             "bbox_x_min": 170.0, "bbox_y_min": 120.0, "bbox_x_max": 259.0,
             "bbox_y_max": 242.0},
            {"slot_id": "SLOT_02", "class_name": "lettuce_yellow",
             "confidence": 0.77, "center_u": 400.0, "center_v": 190.0,
             "bbox_x_min": 380.0, "bbox_y_min": 170.0, "bbox_x_max": 420.0,
             "bbox_y_max": 210.0},
        ],
    }, pass_no=1, wall=1300.0)
    store.record_inspection_verdict(TASK, TASK + "-CMD-008", "PALLET_001",
                                    "INSPECT", ("SLOT_02",), (), 1301.0, 200.0)

    store.record_executor_status("sim_task", "READY", "", "", "", "IDLE",
                                 "ready", 1000.0, 100.0)
    store.set_live("cycle", {"task_id": TASK, "scenario_id": "DEMO_HARVEST_01",
                             "state": "INSPECT", "active_command_id": "",
                             "status": "RUNNING", "reason": "NONE"},
                   1301.0, 200.0)
    store.set_live("robot_pose", {"x": -2.19, "y": -2.73, "yaw_deg": 90.0,
                                  "frame_id": "map"}, 1301.0, 200.0)
    store.close()


@pytest.fixture()
def client(tmp_path):
    """앱과 채워 둔 DB 를 함께 준다."""

    db_path = str(tmp_path / "farm.db")
    seed(db_path)
    app = create_app(db_path, MAP_YAML, STATIONS_YAML)
    with TestClient(app) as test_client:
        yield test_client, db_path


def test_png_size_reads_header():
    image = os.path.join(os.path.dirname(MAP_YAML), "Collected_smartfarm_v014.png")
    if not os.path.isfile(image):
        pytest.skip("지도 이미지가 이 기기에 없음")
    assert png_size(image) == (285, 460)


def test_png_size_on_missing_file():
    assert png_size("/no/such/file.png") == (None, None)


def test_web_dir_has_pages():
    web_dir = find_web_dir()
    for name in ("index.html", "history.html", "inspection.html", "dock.html",
                 "pallet.html", "style.css", "app.js"):
        assert os.path.isfile(os.path.join(web_dir, name)), name


def test_map_meta_matches_yaml():
    if not os.path.isfile(MAP_YAML):
        pytest.skip("지도 yaml 이 이 기기에 없음")
    meta = load_map_meta(MAP_YAML, STATIONS_YAML)
    assert meta["available"] is True
    assert meta["resolution"] == 0.05
    assert meta["origin_x"] == -4.525
    assert meta["origin_y"] == -10.025
    assert meta["width_px"] == 285
    assert meta["height_px"] == 460
    names = {station["name"] for station in meta["stations"]}
    assert {"RACK_DOCK", "FEEDER_APPROACH", "FEEDER_DOCK"} <= names


def test_map_meta_without_files(tmp_path):
    meta = load_map_meta(str(tmp_path / "none.yaml"), str(tmp_path / "none2.yaml"))
    assert meta == {"available": False}


def test_pages_load(client):
    test_client, _ = client
    for path in ("/", "/history", "/inspection", "/dock", "/pallet"):
        response = test_client.get(path)
        assert response.status_code == 200, path
        assert "스마트팜 관제" in response.text
    assert test_client.get("/static/style.css").status_code == 200
    assert test_client.get("/static/app.js").status_code == 200


def test_api_state(client):
    test_client, _ = client
    state = test_client.get("/api/state").json()

    assert state["cycle_live"]["state"] == "INSPECT"
    assert state["cycle"]["task_id"] == TASK
    assert state["robot_pose"]["x"] == -2.19
    assert len(state["step_order"]) == 12
    assert len(state["steps"]) == 2
    assert state["kpi"]["dock_total"] == 1
    assert state["dock_recent"][0]["run_id"] == "RUN-7"


def test_api_cycle_detail(client):
    test_client, _ = client
    data = test_client.get(f"/api/cycle/{TASK}").json()

    assert data["cycle"]["scenario_id"] == "DEMO_HARVEST_01"
    assert [step["operation"] for step in data["steps"]] == ["TRANSFER", "NAVIGATION"]
    assert data["steps"][0]["duration_sim"] == 30.0
    assert json.loads(data["verdicts"][0]["defect_slots"]) == ["SLOT_02"]
    assert data["pallet_moves"][0]["location"] == "FEEDER_DOCK"
    assert len(data["detections"]) == 2


def test_api_cycle_missing(client):
    test_client, _ = client
    assert test_client.get("/api/cycle/NO_SUCH_TASK").status_code == 404


def test_api_detections_defaults_to_latest(client):
    test_client, _ = client
    data = test_client.get("/api/detections").json()
    assert data["task_id"] == TASK
    assert {row["slot_id"] for row in data["detections"]} == {"SLOT_01", "SLOT_02"}
    assert "yellow" in data["defect_hints"]


def test_api_dock_and_durations(client):
    test_client, _ = client
    attempts = test_client.get("/api/dock").json()["attempts"]
    assert attempts[0]["lat_m"] == 0.057
    assert attempts[0]["map_yaw_deg"] == 90.0

    durations = test_client.get("/api/step-durations").json()["durations"]
    operations = {row["operation"]: row for row in durations}
    assert operations["TRANSFER"]["avg_sim"] == 30.0
    assert operations["NAVIGATION"]["avg_sim"] == 30.0


def test_api_pallets(client):
    test_client, _ = client
    moves = test_client.get("/api/pallets").json()["moves"]
    assert moves[0]["pallet_id"] == "PALLET_001"


def test_start_cycle_goes_into_mailbox(client):
    test_client, db_path = client
    queued = test_client.post("/api/start_cycle",
                              json={"scenario_id": "DEMO_HARVEST_01"}).json()
    row_id = queued["queued_id"]

    row = test_client.get(f"/api/web_command/{row_id}").json()
    assert row["kind"] == "START_CYCLE"
    assert row["state"] == "PENDING"
    assert json.loads(row["payload"])["scenario_id"] == "DEMO_HARVEST_01"

    # recorder 가 집어 가면 상태가 바뀐다.
    store = MonitorStore(db_path)
    taken = store.take_pending_web_command()
    store.finish_web_command(int(taken["id"]), "ACCEPTED",
                             {"accepted": True, "task_id": TASK})
    store.close()

    row = test_client.get(f"/api/web_command/{row_id}").json()
    assert row["state"] == "ACCEPTED"
    assert test_client.get("/api/web_command/99999").status_code == 404


def test_label_is_saved(client):
    test_client, _ = client
    response = test_client.post("/api/label", json={
        "task_id": TASK, "command_id": TASK + "-CMD-003",
        "label": "ANOMALY", "note": "도킹 횡 오차가 허용에 붙음"})
    assert response.json() == {"ok": True}

    labels = test_client.get(f"/api/cycle/{TASK}").json()["labels"]
    assert labels[0]["label"] == "ANOMALY"
    assert "허용" in labels[0]["note"]


def test_map_png_served(client):
    test_client, _ = client
    response = test_client.get("/api/map.png")
    if response.status_code == 404:
        pytest.skip("지도 이미지가 이 기기에 없음")
    assert response.headers["content-type"] == "image/png"
    assert response.content[:8] == b"\x89PNG\r\n\x1a\n"
