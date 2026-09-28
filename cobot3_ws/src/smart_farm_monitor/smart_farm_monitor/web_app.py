"""관제 웹 서비스. DB 파일을 읽기 전용으로 조회해 화면과 API 를 제공한다.

ROS 에 의존하지 않는다. 시작 버튼을 누르면 web_command 표에 행을 넣고,
recorder 노드가 그것을 읽어 /start_cycle 서비스를 대신 호출한다.

실행:
  ros2 run smart_farm_monitor web            (기본 0.0.0.0:8080)
  python3 -m smart_farm_monitor.web_app --port 8080
"""

import argparse
import asyncio
import json
import os
import time

import yaml
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .store import MonitorStore, find_share_subdir


DEFAULT_DB_PATH = (
    "/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_monitor/data/farm.db"
)
DEFAULT_MAP_YAML = (
    "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/maps/"
    "Collected_smartfarm_v014.yaml"
)
DEFAULT_STATIONS_YAML = (
    "/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/config/"
    "stations.yaml"
)
def find_web_dir() -> str:
    """화면 파일이 있는 폴더를 찾는다. 소스 트리와 설치본 둘 다에서 동작한다."""

    here = os.path.dirname(os.path.abspath(__file__))
    source_tree = os.path.join(here, "..", "web")
    if os.path.isdir(source_tree):
        return os.path.abspath(source_tree)

    share = find_share_subdir("web")
    if share is not None:
        return os.path.abspath(share)
    return os.path.abspath(source_tree)


WEB_DIR = find_web_dir()

# 대시보드 체크리스트에 그리는 공정 순서 (팀 scenario.py 의 StepDefinition 12개)
CYCLE_STEPS = (
    "TRANSFER", "PICK_HARVEST", "NAVIGATION", "PLACE_INSPECT",
    "CONVEY_TO_INSPECT", "PREPARE_INSPECT", "MOVE_TO_INSPECT", "INSPECT",
    "CULL", "RECHECK", "RELEASE_INSPECT", "CONVEYOR_OUT",
)

# 불량으로 세는 클래스 이름 조각
DEFECT_HINTS = ("yellow", "brown")


class StartCycleBody(BaseModel):
    """시작 버튼이 보내는 몸통."""

    scenario_id: str = "DEMO_HARVEST_01"


class LabelBody(BaseModel):
    """사람이 붙이는 정상·이상 라벨."""

    task_id: str
    command_id: str = ""
    label: str = "NORMAL"
    note: str = ""


def rows_to_list(rows) -> list:
    """sqlite3.Row 목록을 JSON 으로 보낼 수 있는 딕셔너리 목록으로 바꾼다."""

    return [dict(row) for row in rows]


def load_map_meta(map_yaml: str, stations_yaml: str) -> dict:
    """지도 배경과 작업점을 화면에서 쓸 형태로 읽는다.

    map 좌표(x, y, m)를 이미지 pixel 로 바꾸는 식은 두 줄이다.
      px = (x - origin_x) / resolution
      py = height_px - (y - origin_y) / resolution      # 이미지 원점은 좌상단
    """

    meta = {"available": False}
    if not os.path.isfile(map_yaml):
        return meta

    with open(map_yaml, encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}

    image_name = document.get("image", "")
    image_path = os.path.join(os.path.dirname(map_yaml), image_name)
    width, height = png_size(image_path)

    meta = {
        "available": width is not None,
        "image_url": "/api/map.png",
        "resolution": float(document.get("resolution", 0.05)),
        "origin_x": float((document.get("origin") or [0, 0, 0])[0]),
        "origin_y": float((document.get("origin") or [0, 0, 0])[1]),
        "width_px": width,
        "height_px": height,
        "image_path": image_path,
        "stations": [],
        "initial_pose": None,
    }

    if os.path.isfile(stations_yaml):
        with open(stations_yaml, encoding="utf-8") as handle:
            stations_doc = yaml.safe_load(handle) or {}
        for name, pose in (stations_doc.get("stations") or {}).items():
            meta["stations"].append({
                "name": name,
                "x": float(pose.get("x", 0.0)),
                "y": float(pose.get("y", 0.0)),
                "yaw_deg": float(pose.get("yaw_deg", 0.0)),
            })
        initial = stations_doc.get("initial_pose")
        if initial:
            meta["initial_pose"] = {
                "x": float(initial.get("x", 0.0)),
                "y": float(initial.get("y", 0.0)),
                "yaw_deg": float(initial.get("yaw_deg", 0.0)),
            }
    return meta


def png_size(path: str):
    """PNG 헤더에서 가로·세로 pixel 을 읽는다. 없으면 (None, None)."""

    if not os.path.isfile(path):
        return None, None
    with open(path, "rb") as handle:
        header = handle.read(24)
    if len(header) < 24 or header[:8] != b"\x89PNG\r\n\x1a\n":
        return None, None
    width = int.from_bytes(header[16:20], "big")
    height = int.from_bytes(header[20:24], "big")
    return width, height


def create_app(db_path: str, map_yaml: str, stations_yaml: str) -> FastAPI:
    """웹 앱을 만든다. db_path 는 recorder 가 쓰는 파일과 같아야 한다."""

    app = FastAPI(title="스마트팜 관제", docs_url="/api/docs")
    map_meta = load_map_meta(map_yaml, stations_yaml)

    # recorder 보다 웹이 먼저 떠도 "no such table" 이 나지 않게 표를 한 번 만들어 둔다.
    # 같은 schema.sql 이고 CREATE TABLE IF NOT EXISTS 라 기존 기록에는 영향이 없다.
    MonitorStore(db_path).close()

    def open_store(write: bool = False) -> MonitorStore:
        """요청마다 새 연결을 연다. SQLite 연결은 스레드 사이에서 공유하지 않는다."""

        return MonitorStore(db_path, read_only=not write)

    # ------------------------------------------------------------------
    # 화면
    # ------------------------------------------------------------------

    def page(name: str) -> HTMLResponse:
        path = os.path.join(WEB_DIR, name)
        if not os.path.isfile(path):
            raise HTTPException(status_code=404, detail=f"{name} 이 없습니다")
        with open(path, encoding="utf-8") as handle:
            return HTMLResponse(handle.read())

    @app.get("/", response_class=HTMLResponse)
    def index():
        return page("index.html")

    @app.get("/history", response_class=HTMLResponse)
    def history():
        return page("history.html")

    @app.get("/inspection", response_class=HTMLResponse)
    def inspection():
        return page("inspection.html")

    @app.get("/dock", response_class=HTMLResponse)
    def dock():
        return page("dock.html")

    @app.get("/pallet", response_class=HTMLResponse)
    def pallet():
        return page("pallet.html")

    if os.path.isdir(WEB_DIR):
        app.mount("/static", StaticFiles(directory=WEB_DIR), name="static")

    # ------------------------------------------------------------------
    # 조회 API
    # ------------------------------------------------------------------

    def build_state() -> dict:
        """대시보드가 한 번에 받는 현재 상태."""

        store = open_store()
        try:
            live = store.live_values()
            cycle_live = {}
            if "cycle" in live:
                cycle_live = json.loads(live["cycle"]["value"])

            pose = None
            if "robot_pose" in live:
                pose = json.loads(live["robot_pose"]["value"])
                pose["updated_wall"] = live["robot_pose"]["updated_wall"]

            latest = store.latest_cycle()
            task_id = cycle_live.get("task_id") or (
                latest["task_id"] if latest is not None else "")

            steps = rows_to_list(store.steps(task_id)) if task_id else []
            return {
                "server_wall": time.time(),
                "cycle_live": cycle_live,
                "cycle": dict(latest) if latest is not None else None,
                "robot_pose": pose,
                "executors": rows_to_list(store.executor_states()),
                "steps": steps,
                "step_order": list(CYCLE_STEPS),
                "kpi": store.kpi(),
                "dock_recent": rows_to_list(store.dock_attempts(limit=5)),
            }
        finally:
            store.close()

    @app.get("/api/state")
    def api_state():
        return build_state()

    @app.get("/api/kpi")
    def api_kpi():
        store = open_store()
        try:
            return store.kpi()
        finally:
            store.close()

    @app.get("/api/cycles")
    def api_cycles(limit: int = 50):
        store = open_store()
        try:
            return {"cycles": rows_to_list(store.cycles(limit))}
        finally:
            store.close()

    @app.get("/api/cycle/{task_id}")
    def api_cycle(task_id: str):
        store = open_store()
        try:
            cycle = store.latest_cycle() if not task_id else None
            row = store.conn.execute(
                "SELECT * FROM cycle WHERE task_id = ?", (task_id,)).fetchone()
            if row is None and cycle is None:
                raise HTTPException(status_code=404, detail="사이클이 없습니다")
            return {
                "cycle": dict(row) if row is not None else None,
                "steps": rows_to_list(store.steps(task_id)),
                "verdicts": rows_to_list(store.verdicts(task_id)),
                "detections": rows_to_list(store.detections(task_id)),
                "pallet_moves": rows_to_list(store.pallet_moves(task_id)),
                "labels": rows_to_list(store.anomaly_labels(task_id)),
            }
        finally:
            store.close()

    @app.get("/api/step-durations")
    def api_step_durations():
        store = open_store()
        try:
            return {"durations": rows_to_list(store.step_durations())}
        finally:
            store.close()

    @app.get("/api/detections")
    def api_detections(task_id: str = ""):
        store = open_store()
        try:
            if not task_id:
                latest = store.latest_cycle()
                task_id = latest["task_id"] if latest is not None else ""
            return {
                "task_id": task_id,
                "detections": rows_to_list(store.detections(task_id)) if task_id else [],
                "verdicts": rows_to_list(store.verdicts(task_id)) if task_id else [],
                "defect_hints": list(DEFECT_HINTS),
            }
        finally:
            store.close()

    @app.get("/api/dock")
    def api_dock(limit: int = 200):
        store = open_store()
        try:
            return {"attempts": rows_to_list(store.dock_attempts(limit))}
        finally:
            store.close()

    @app.get("/api/pallets")
    def api_pallets(task_id: str = ""):
        store = open_store()
        try:
            moves = store.pallet_moves(task_id or None)
            return {"moves": rows_to_list(moves)}
        finally:
            store.close()

    @app.get("/api/map")
    def api_map():
        return map_meta

    @app.get("/api/map.png")
    def api_map_png():
        path = map_meta.get("image_path")
        if not path or not os.path.isfile(path):
            raise HTTPException(status_code=404, detail="지도 이미지가 없습니다")
        return FileResponse(path, media_type="image/png")

    # ------------------------------------------------------------------
    # 실시간 갱신 (SSE)
    # ------------------------------------------------------------------

    @app.get("/api/events")
    async def api_events():
        """DB 가 바뀔 때만 현재 상태를 밀어 준다."""

        async def event_stream():
            last_token = None
            # 연결이 끊기면 GeneratorExit 이 나므로 따로 정리할 것이 없다.
            while True:
                store = open_store()
                try:
                    token = store.version_token()
                finally:
                    store.close()

                if token != last_token:
                    last_token = token
                    payload = json.dumps(build_state(), ensure_ascii=False,
                                         default=str)
                    yield f"data: {payload}\n\n"
                else:
                    # 프록시가 끊지 않게 주석 줄을 보낸다.
                    yield ": keepalive\n\n"
                await asyncio.sleep(0.5)

        return StreamingResponse(
            event_stream(),
            media_type="text/event-stream",
            headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
        )

    # ------------------------------------------------------------------
    # 관제 조작 (웹 -> DB 우편함 -> recorder -> /start_cycle)
    # ------------------------------------------------------------------

    @app.post("/api/start_cycle")
    def api_start_cycle(body: StartCycleBody):
        store = open_store(write=True)
        try:
            row_id = store.queue_web_command(
                "START_CYCLE", {"scenario_id": body.scenario_id}, time.time())
            return {"queued_id": row_id}
        finally:
            store.close()

    @app.get("/api/web_command/{row_id}")
    def api_web_command(row_id: int):
        store = open_store()
        try:
            row = store.web_command(row_id)
            if row is None:
                raise HTTPException(status_code=404, detail="요청이 없습니다")
            return dict(row)
        finally:
            store.close()

    @app.post("/api/label")
    def api_label(body: LabelBody):
        store = open_store(write=True)
        try:
            store.add_anomaly_label(body.task_id, body.command_id, body.label,
                                    body.note, time.time())
            return {"ok": True}
        finally:
            store.close()

    return app


def build_parser() -> argparse.ArgumentParser:
    """실행 인자를 정의한다."""

    parser = argparse.ArgumentParser(description="스마트팜 관제 웹")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--db", default=DEFAULT_DB_PATH)
    parser.add_argument("--map-yaml", default=DEFAULT_MAP_YAML)
    parser.add_argument("--stations-yaml", default=DEFAULT_STATIONS_YAML)
    return parser


def main(argv=None) -> None:
    """웹 서버를 띄운다."""

    import uvicorn

    # ros2 run 으로 부르면 ROS 인자가 붙어 오므로 모르는 인자는 무시한다.
    args, _ = build_parser().parse_known_args(argv)

    app = create_app(args.db, args.map_yaml, args.stations_yaml)
    print(f"관제 웹: http://{args.host}:{args.port}  (DB: {args.db})")
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
