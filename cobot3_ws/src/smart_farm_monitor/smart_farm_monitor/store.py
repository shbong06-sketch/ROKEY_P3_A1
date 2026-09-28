"""관제 DB(SQLite) 읽기·쓰기. ROS 에 의존하지 않는 순수 파이썬 모듈이다.

recorder 노드는 이 모듈로 기록하고, 웹은 같은 파일을 읽기 전용으로 조회한다.
쓰기는 recorder 한 곳에서만 하고, 동시 읽기를 위해 WAL 모드를 쓴다.
"""

import json
import os
import sqlite3
from typing import Optional


def find_schema_path() -> str:
    """schema.sql 의 경로를 찾는다. 소스 트리와 설치본 둘 다에서 동작한다."""

    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, "schema.sql"),
        os.path.join(here, "..", "..", "share", "smart_farm_monitor", "sql", "schema.sql"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return os.path.abspath(path)
    raise FileNotFoundError("schema.sql 을 찾지 못했습니다: " + str(candidates))


def dump_json(value) -> str:
    """리스트나 딕셔너리를 DB 에 넣을 문자열로 바꾼다."""

    return json.dumps(list(value) if isinstance(value, tuple) else value,
                      ensure_ascii=False)


def command_sequence(command_id: str) -> Optional[int]:
    """command_id 의 순번을 뽑는다. 'TASK-...-CMD-005' -> 5.

    'CMD-<숫자>' 형태가 없으면 None 이다. task_id 처럼 끝이 숫자인 문자열을
    순번으로 잘못 읽지 않도록 'CMD' 표시를 반드시 확인한다.
    """

    parts = command_id.split("-")
    for index in range(len(parts) - 1):
        if parts[index] == "CMD" and parts[index + 1].isdigit():
            return int(parts[index + 1])
    return None


class MonitorStore:
    """공정 기록을 담는 SQLite 파일 하나를 다룬다."""

    def __init__(self, db_path: str, read_only: bool = False) -> None:
        """DB 파일을 열고 없으면 스키마를 만든다."""

        self.db_path = db_path
        self.read_only = read_only

        if not read_only:
            directory = os.path.dirname(os.path.abspath(db_path))
            os.makedirs(directory, exist_ok=True)

        self.conn = sqlite3.connect(db_path, timeout=5.0)
        self.conn.row_factory = sqlite3.Row
        self.conn.execute("PRAGMA journal_mode=WAL")
        self.conn.execute("PRAGMA synchronous=NORMAL")

        if not read_only:
            with open(find_schema_path(), encoding="utf-8") as schema_file:
                self.conn.executescript(schema_file.read())
            self.conn.commit()

    def close(self) -> None:
        """연결을 닫는다."""

        self.conn.close()

    # ------------------------------------------------------------------
    # 쓰기 — 사이클
    # ------------------------------------------------------------------

    def open_cycle(self, task_id: str, scenario_id: str,
                   wall: float, sim: float) -> bool:
        """사이클을 새로 연다. 이미 있으면 False 를 돌려주고 아무것도 하지 않는다."""

        cursor = self.conn.execute(
            "INSERT OR IGNORE INTO cycle"
            " (task_id, scenario_id, started_wall, started_sim, terminal_status)"
            " VALUES (?, ?, ?, ?, 'RUNNING')",
            (task_id, scenario_id, wall, sim),
        )
        self.conn.commit()
        return cursor.rowcount > 0

    def update_cycle(self, task_id: str, terminal_status: str,
                     failure_reason: str) -> None:
        """진행 중 사이클의 상태와 사유를 갱신한다."""

        self.conn.execute(
            "UPDATE cycle SET terminal_status = ?, failure_reason = ?"
            " WHERE task_id = ?",
            (terminal_status, failure_reason, task_id),
        )
        self.conn.commit()

    def close_cycle(self, task_id: str, final_state: str, terminal_status: str,
                    failure_reason: str, wall: float, sim: float) -> None:
        """사이클을 끝낸다. 이미 끝난 사이클은 다시 덮어쓰지 않는다."""

        self.conn.execute(
            "UPDATE cycle SET ended_wall = ?, ended_sim = ?, final_state = ?,"
            " terminal_status = ?, failure_reason = ?"
            " WHERE task_id = ? AND ended_wall IS NULL",
            (wall, sim, final_state, terminal_status, failure_reason, task_id),
        )
        self.conn.commit()

    # ------------------------------------------------------------------
    # 쓰기 — 공정 단계
    # ------------------------------------------------------------------

    def record_command(self, task_id: str, command_id: str, executor: str,
                       operation: str, recipe_id: str, pallet_id: str,
                       source: str, destination: str, target_slots,
                       cycle_state: str, wall: float, sim: float) -> None:
        """명령 발행을 기록한다. 같은 command_id 가 다시 와도 한 행만 남는다."""

        self.conn.execute(
            "INSERT OR IGNORE INTO step"
            " (task_id, command_id, seq, cycle_state, executor, operation,"
            "  recipe_id, pallet_id, source, destination, target_slots,"
            "  issued_wall, issued_sim)"
            " VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (task_id, command_id, command_sequence(command_id), cycle_state,
             executor, operation, recipe_id, pallet_id, source, destination,
             dump_json(target_slots), wall, sim),
        )
        self.conn.commit()

    def record_result(self, task_id: str, command_id: str, operation: str,
                      status: str, phase: str, reason: str,
                      reached_station: str, wall: float, sim: float) -> bool:
        """공정 결과를 기록하고 소요시간을 채운다.

        이미 결과가 들어간 단계에 또 결과가 오면 새 행에 late_result=1 로 남긴다.
        Task Manager 가 TIMEOUT 으로 끝낸 뒤 Isaac 동작이 늦게 끝나는 경우다.
        돌려주는 값은 '늦은 결과였는가' 이다.
        """

        row = self.conn.execute(
            "SELECT id, issued_wall, issued_sim, status FROM step"
            " WHERE task_id = ? AND command_id = ?",
            (task_id, command_id),
        ).fetchone()

        if row is not None and row["status"] is None:
            duration_wall = self._elapsed(row["issued_wall"], wall)
            duration_sim = self._elapsed(row["issued_sim"], sim)
            self.conn.execute(
                "UPDATE step SET result_wall = ?, result_sim = ?,"
                " duration_wall = ?, duration_sim = ?, status = ?, phase = ?,"
                " reason = ?, reached_station = ? WHERE id = ?",
                (wall, sim, duration_wall, duration_sim, status, phase, reason,
                 reached_station, row["id"]),
            )
            self.conn.commit()
            return False

        # 발행 기록이 없거나 이미 끝난 단계다. 별도 행으로 남긴다.
        issued_wall = row["issued_wall"] if row is not None else None
        issued_sim = row["issued_sim"] if row is not None else None
        self.conn.execute(
            "INSERT INTO step"
            " (task_id, command_id, seq, operation, issued_wall, issued_sim,"
            "  result_wall, result_sim, duration_wall, duration_sim,"
            "  status, phase, reason, reached_station, late_result)"
            " VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, 1)",
            (task_id, command_id + "#late", command_sequence(command_id),
             operation, issued_wall, issued_sim, wall, sim,
             self._elapsed(issued_wall, wall), self._elapsed(issued_sim, sim),
             status, phase, reason, reached_station),
        )
        self.conn.commit()
        return True

    @staticmethod
    def _elapsed(start: Optional[float], end: Optional[float]) -> Optional[float]:
        """두 시각의 차. 어느 쪽이든 없거나 되감겼으면 None."""

        if start is None or end is None or end < start:
            return None
        return round(end - start, 3)

    # ------------------------------------------------------------------
    # 쓰기 — 상태·검사·팔레트·도킹
    # ------------------------------------------------------------------

    def record_executor_status(self, executor: str, state: str, task_id: str,
                               command_id: str, operation: str, phase: str,
                               detail: str, wall: float, sim: float) -> None:
        """executor 상태를 남긴다. 값이 바뀐 순간만 부르도록 노드 쪽에서 거른다."""

        self.conn.execute(
            "INSERT INTO executor_status"
            " (recv_wall, recv_sim, executor, state, task_id, command_id,"
            "  operation, phase, detail)"
            " VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (wall, sim, executor, state, task_id, command_id, operation, phase,
             detail),
        )
        self.conn.commit()

    def record_detections(self, payload: dict, pass_no: int,
                          wall: float) -> int:
        """/inspection/detections_2d JSON 한 건을 슬롯별 행으로 나눠 넣는다."""

        header = payload.get("header") or {}
        stamp = header.get("stamp") or {}
        stamp_sim = None
        if "sec" in stamp:
            stamp_sim = float(stamp.get("sec", 0)) + float(stamp.get("nanosec", 0)) * 1e-9

        rows = []
        for item in payload.get("detections") or []:
            rows.append((
                payload.get("task_id", ""), payload.get("command_id", ""),
                payload.get("pallet_id", ""), pass_no, stamp_sim, wall,
                header.get("frame_id", ""),
                payload.get("image_width"), payload.get("image_height"),
                item.get("slot_id", ""), item.get("class_name", ""),
                item.get("confidence"), item.get("center_u"), item.get("center_v"),
                item.get("bbox_x_min"), item.get("bbox_y_min"),
                item.get("bbox_x_max"), item.get("bbox_y_max"),
            ))

        if rows:
            self.conn.executemany(
                "INSERT INTO detection"
                " (task_id, command_id, pallet_id, pass_no, stamp_sim, recv_wall,"
                "  frame_id, image_width, image_height, slot_id, class_name,"
                "  confidence, center_u, center_v, bbox_x_min, bbox_y_min,"
                "  bbox_x_max, bbox_y_max)"
                " VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                rows,
            )
            self.conn.commit()
        return len(rows)

    def record_inspection_verdict(self, task_id: str, command_id: str,
                                  pallet_id: str, operation: str,
                                  defect_slots, unknown_slots,
                                  wall: float, sim: float) -> None:
        """검사 판정(불량·미판정 슬롯)을 기록한다."""

        self.conn.execute(
            "INSERT INTO inspection_verdict"
            " (task_id, command_id, pallet_id, operation, defect_slots,"
            "  unknown_slots, recv_wall, recv_sim)"
            " VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
            (task_id, command_id, pallet_id, operation,
             dump_json(defect_slots), dump_json(unknown_slots), wall, sim),
        )
        self.conn.commit()

    def record_pallet_move(self, task_id: str, command_id: str, pallet_id: str,
                           location: str, source: str,
                           wall: float, sim: float) -> None:
        """팔레트의 논리 위치가 바뀐 것을 기록한다."""

        self.conn.execute(
            "INSERT INTO pallet_move"
            " (task_id, command_id, pallet_id, location, source, recv_wall, recv_sim)"
            " VALUES (?, ?, ?, ?, ?, ?, ?)",
            (task_id, command_id, pallet_id, location, source, wall, sim),
        )
        self.conn.commit()

    def record_dock_attempt(self, task_id: str, command_id: str, result: dict,
                            map_pose: Optional[dict],
                            wall: float, sim: float) -> None:
        """/feeder_dock/result 한 건을 기록한다. map_pose 는 없으면 None."""

        pose = map_pose or {}
        self.conn.execute(
            "INSERT INTO dock_attempt"
            " (task_id, command_id, run_id, status, reason, face_dist_m,"
            "  yaw_err_deg, lat_m, retry, map_x, map_y, map_yaw_deg,"
            "  recv_wall, recv_sim)"
            " VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (task_id, command_id, result.get("run_id", ""),
             result.get("status", ""), result.get("reason", ""),
             result.get("face_dist_m"), result.get("yaw_err_deg"),
             result.get("lat_m"), result.get("retry"),
             pose.get("x"), pose.get("y"), pose.get("yaw_deg"), wall, sim),
        )
        self.conn.commit()

    def set_live(self, key: str, value, wall: float, sim: float) -> None:
        """최신값 한 개를 덮어쓴다. 이력으로 쌓지 않는다."""

        self.conn.execute(
            "INSERT INTO live (key, value, updated_wall, updated_sim)"
            " VALUES (?, ?, ?, ?)"
            " ON CONFLICT(key) DO UPDATE SET value = excluded.value,"
            " updated_wall = excluded.updated_wall,"
            " updated_sim = excluded.updated_sim",
            (key, dump_json(value) if not isinstance(value, str) else value,
             wall, sim),
        )
        self.conn.commit()

    def add_anomaly_label(self, task_id: str, command_id: str, label: str,
                          note: str, wall: float) -> None:
        """사람이 붙인 정상·이상 라벨을 남긴다."""

        self.conn.execute(
            "INSERT INTO anomaly_label"
            " (task_id, command_id, label, note, labeled_wall)"
            " VALUES (?, ?, ?, ?, ?)",
            (task_id, command_id, label, note, wall),
        )
        self.conn.commit()

    # ------------------------------------------------------------------
    # 웹 -> ROS 우편함
    # ------------------------------------------------------------------

    def queue_web_command(self, kind: str, payload: dict, wall: float) -> int:
        """웹에서 들어온 명령을 대기 행으로 넣고 그 id 를 돌려준다."""

        cursor = self.conn.execute(
            "INSERT INTO web_command (kind, payload, requested_wall)"
            " VALUES (?, ?, ?)",
            (kind, dump_json(payload), wall),
        )
        self.conn.commit()
        return int(cursor.lastrowid)

    def take_pending_web_command(self) -> Optional[sqlite3.Row]:
        """가장 오래된 대기 명령을 SENT 로 바꾸고 돌려준다. 없으면 None."""

        row = self.conn.execute(
            "SELECT * FROM web_command WHERE state = 'PENDING'"
            " ORDER BY id LIMIT 1"
        ).fetchone()
        if row is None:
            return None
        self.conn.execute(
            "UPDATE web_command SET state = 'SENT' WHERE id = ?", (row["id"],)
        )
        self.conn.commit()
        return row

    def finish_web_command(self, command_row_id: int, state: str,
                           response: dict) -> None:
        """우편함 행에 서비스 응답을 적는다."""

        self.conn.execute(
            "UPDATE web_command SET state = ?, response = ? WHERE id = ?",
            (state, dump_json(response), command_row_id),
        )
        self.conn.commit()

    # ------------------------------------------------------------------
    # 읽기 — 웹 조회용
    # ------------------------------------------------------------------

    def latest_cycle(self) -> Optional[sqlite3.Row]:
        """가장 최근 사이클 한 건."""

        return self.conn.execute(
            "SELECT * FROM cycle ORDER BY started_wall DESC LIMIT 1"
        ).fetchone()

    def cycles(self, limit: int = 50) -> list:
        """최근 사이클 목록."""

        return self.conn.execute(
            "SELECT * FROM cycle ORDER BY started_wall DESC LIMIT ?", (limit,)
        ).fetchall()

    def steps(self, task_id: str) -> list:
        """한 사이클의 공정 단계 목록(발행 순)."""

        return self.conn.execute(
            "SELECT * FROM step WHERE task_id = ? ORDER BY id", (task_id,)
        ).fetchall()

    def last_open_step(self, executor: str) -> Optional[sqlite3.Row]:
        """해당 executor 의 결과가 아직 안 온 마지막 단계. 도킹 결과를 붙일 때 쓴다."""

        return self.conn.execute(
            "SELECT * FROM step WHERE executor = ? AND status IS NULL"
            " ORDER BY id DESC LIMIT 1",
            (executor,),
        ).fetchone()

    def step_by_command(self, task_id: str, command_id: str):
        """task_id 와 command_id 로 단계 한 건을 찾는다."""

        return self.conn.execute(
            "SELECT * FROM step WHERE task_id = ? AND command_id = ?",
            (task_id, command_id),
        ).fetchone()

    def executor_states(self) -> list:
        """executor 별 가장 최근 상태 한 줄씩."""

        return self.conn.execute(
            "SELECT executor, state, task_id, command_id, operation, phase,"
            " detail, recv_wall, recv_sim FROM executor_status"
            " WHERE id IN (SELECT MAX(id) FROM executor_status GROUP BY executor)"
            " ORDER BY executor"
        ).fetchall()

    def detections(self, task_id: str) -> list:
        """한 사이클의 검사 검출 전체."""

        return self.conn.execute(
            "SELECT * FROM detection WHERE task_id = ? ORDER BY id", (task_id,)
        ).fetchall()

    def verdicts(self, task_id: str) -> list:
        """한 사이클의 검사 판정 전체."""

        return self.conn.execute(
            "SELECT * FROM inspection_verdict WHERE task_id = ? ORDER BY id",
            (task_id,),
        ).fetchall()

    def pallet_moves(self, task_id: Optional[str] = None) -> list:
        """팔레트 위치 이력. task_id 를 주면 그 사이클만."""

        if task_id is None:
            return self.conn.execute(
                "SELECT * FROM pallet_move ORDER BY id"
            ).fetchall()
        return self.conn.execute(
            "SELECT * FROM pallet_move WHERE task_id = ? ORDER BY id", (task_id,)
        ).fetchall()

    def dock_attempts(self, limit: int = 200) -> list:
        """최근 도킹 시도 목록."""

        return self.conn.execute(
            "SELECT * FROM dock_attempt ORDER BY id DESC LIMIT ?", (limit,)
        ).fetchall()

    def live_values(self) -> dict:
        """live 표를 딕셔너리로 돌려준다."""

        rows = self.conn.execute("SELECT * FROM live").fetchall()
        return {row["key"]: dict(row) for row in rows}

    def kpi(self) -> dict:
        """대시보드 상단 지표. 표본이 없으면 값은 None 이다."""

        cycle_row = self.conn.execute(
            "SELECT COUNT(*) AS total,"
            " SUM(CASE WHEN terminal_status = 'SUCCEEDED' THEN 1 ELSE 0 END) AS ok,"
            " AVG(CASE WHEN terminal_status = 'SUCCEEDED'"
            "     THEN ended_sim - started_sim END) AS avg_sim"
            " FROM cycle WHERE ended_wall IS NOT NULL"
        ).fetchone()

        dock_row = self.conn.execute(
            "SELECT COUNT(*) AS total,"
            " SUM(CASE WHEN status = 'SUCCEEDED' THEN 1 ELSE 0 END) AS ok,"
            " AVG(retry) AS avg_retry, MAX(ABS(lat_m)) AS max_lat"
            " FROM dock_attempt"
        ).fetchone()

        defect_row = self.conn.execute(
            "SELECT COUNT(*) AS defects FROM detection"
            " WHERE class_name LIKE '%yellow%' OR class_name LIKE '%brown%'"
        ).fetchone()

        return {
            "cycle_total": cycle_row["total"] or 0,
            "cycle_succeeded": cycle_row["ok"] or 0,
            "cycle_avg_sim_sec": cycle_row["avg_sim"],
            "dock_total": dock_row["total"] or 0,
            "dock_succeeded": dock_row["ok"] or 0,
            "dock_avg_retry": dock_row["avg_retry"],
            "dock_max_lat_m": dock_row["max_lat"],
            "defect_detections": defect_row["defects"] or 0,
        }

    def step_durations(self) -> list:
        """operation 별 시뮬레이션 소요시간 통계. 사이클 타임 분석과 timeout 재설정용."""

        return self.conn.execute(
            "SELECT operation, COUNT(*) AS n, AVG(duration_sim) AS avg_sim,"
            " MIN(duration_sim) AS min_sim, MAX(duration_sim) AS max_sim"
            " FROM step WHERE duration_sim IS NOT NULL AND late_result = 0"
            " GROUP BY operation ORDER BY avg_sim DESC"
        ).fetchall()
