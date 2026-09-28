"""실험 로그 -> SQLite 간이 DB (파일 하나).

  python build_db.py ROOT [DB 경로]      (기본 DB: ROOT/results/sim_runs.sqlite)

표
  runs             실행 1회 = 1행. 조건(실험·경로·속도·사람·반복) + 구간 시간·거리·기울기·스테이션·선별 결과 (analyze.py 와 같은 계산)
  flow_events      흐름 명령·결과 (벽시계, 시뮬레이션 시각, 내용)
  station_events   비전 스테이션 공정 이벤트 (시뮬레이션 시각)
  inspection       칸별 검사·재검사 판정 (phase = first / recheck)
  culls            선별 1건 = 1행 (색, 버린 통, 비전 좌표 오차, 성공 여부, 걸린 시간)
  chassis_track    카터 궤적 1초 간격 (t, x, y, yaw, roll, pitch, 벽시계)
  tilt_samples     차체 기울기 0.5° 초과 표본 (0.25 초 간격)
  human_events     사람 돌발상황 이벤트
  run_files        실행 폴더의 원본 파일 목록 (경로·크기)
다시 돌리면 DB 를 새로 만든다(덮어씀). 원본 로그는 건드리지 않는다.
"""
import json
import sqlite3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import analyze as A  # noqa: E402

SCHEMA = """
CREATE TABLE runs (run TEXT PRIMARY KEY, kind TEXT, exp TEXT, route TEXT, speed REAL, human INTEGER, rep INTEGER,
  pattern TEXT, defects INTEGER, finished INTEGER, timeout INTEGER, pick_ok INTEGER, nav_ok INTEGER, place_ok INTEGER,
  pick_s REAL, nav_s REAL, drive_s REAL, nav_dist_m REAL, nav_eff_speed_mps REAL, place_s REAL, conveyor_s REAL,
  push_s REAL, inspect_s REAL, cull_total_s REAL, recheck_s REAL, pushback_s REAL, station_total_s REAL,
  cycle_to_discharge_s REAL, tilt_nav_deg REAL, tilt_all_deg REAL, heads_seen INTEGER, vision_correct INTEGER,
  culls INTEGER, cull_fail INTEGER, human_gap_m REAL, human_block_s REAL, run_dir TEXT, meta_json TEXT);
CREATE TABLE flow_events (run TEXT, seq INTEGER, wall_ts REAL, sim_t REAL, text TEXT);
CREATE TABLE station_events (run TEXT, seq INTEGER, t REAL, event TEXT, pallet TEXT);
CREATE TABLE inspection (run TEXT, phase TEXT, slot TEXT, head TEXT, label TEXT, truth TEXT, correct INTEGER);
CREATE TABLE culls (run TEXT, seq INTEGER, slot TEXT, colour TEXT, box TEXT, vision_xy_error_mm REAL, in_box INTEGER,
  duration_s REAL, head_final_x REAL, head_final_y REAL, head_final_z REAL);
CREATE TABLE chassis_track (run TEXT, t REAL, x REAL, y REAL, yaw REAL, roll REAL, pitch REAL, wall_ts REAL);
CREATE TABLE tilt_samples (run TEXT, t REAL, tilt_deg REAL);
CREATE TABLE human_events (run TEXT, t REAL, event TEXT, data_json TEXT);
CREATE TABLE run_files (run TEXT, path TEXT, bytes INTEGER);
CREATE INDEX i_track ON chassis_track(run, t);
CREATE VIEW v_conditions AS
  SELECT route, speed, human, COUNT(*) n, ROUND(AVG(pick_s),1) pick_s, ROUND(AVG(nav_s),1) nav_s, ROUND(AVG(nav_dist_m),2) nav_dist_m,
         ROUND(AVG(place_s),1) place_s, ROUND(AVG(conveyor_s),1) conveyor_s, ROUND(AVG(station_total_s),1) station_s,
         ROUND(AVG(cycle_to_discharge_s),1) cycle_s, ROUND(MAX(tilt_all_deg),2) tilt_max
  FROM runs WHERE kind='allinone' AND nav_ok=1 AND place_ok=1 GROUP BY route, speed, human;
CREATE VIEW v_station_by_defects AS
  SELECT defects, COUNT(*) n, ROUND(AVG(station_total_s),1) station_s, ROUND(AVG(cull_total_s),1) cull_s
  FROM runs WHERE kind='station' GROUP BY defects;
CREATE VIEW v_cull_fail AS
  SELECT r.kind, COUNT(*) culls, SUM(1 - c.in_box) fails, ROUND(1.0 * SUM(1 - c.in_box) / COUNT(*), 3) rate
  FROM culls c JOIN runs r USING(run) GROUP BY r.kind;
"""
COLS = [c.split()[0] for c in SCHEMA.split("CREATE TABLE runs (")[1].split(");")[0].replace("\n", " ").split(",")]


def b(v):
    return None if v is None else int(bool(v))


def main():
    root = Path(sys.argv[1])
    db = Path(sys.argv[2]) if len(sys.argv) > 2 else root / "results" / "sim_runs.sqlite"
    db.parent.mkdir(parents=True, exist_ok=True)
    if db.exists():
        db.unlink()
    con = sqlite3.connect(db)
    con.executescript(SCHEMA)
    n = 0
    for d in sorted(p for p in root.iterdir() if p.is_dir() and p.name[:3] in ("e0_", "e1_", "e2_", "e3_")):
        meta = A.load(d / "run_meta.json", {})
        station = d.name.startswith("e3_")
        row = A.station_run(d) if station else A.allinone_run(d)
        row.update(kind="station" if station else "allinone", run_dir=str(d), meta_json=json.dumps(meta, ensure_ascii=False),
                   exp=meta.get("exp"))
        for k in ("human", "finished", "timeout", "pick_ok", "nav_ok", "place_ok"):
            row[k] = b(row.get(k))
        con.execute(f"INSERT INTO runs ({','.join(COLS)}) VALUES ({','.join('?' * len(COLS))})", [row.get(c) for c in COLS])

        sdir = d if station else d / "station"
        mon = A.load(d / "monitor.json", {})
        track = mon.get("chassis_track", [])
        con.executemany("INSERT INTO chassis_track VALUES (?,?,?,?,?,?,?,?)",
                        [(d.name, *r[:4], *(r[4:7] if len(r) >= 7 else (None, None, None))) for r in track])
        con.executemany("INSERT INTO tilt_samples VALUES (?,?,?)", [(d.name, t, v) for t, v in mon.get("tilt_samples_over_0p5deg", [])])
        f = A.wall_to_sim(track)
        flow = A.load(d / "flow.json", {})
        con.executemany("INSERT INTO flow_events VALUES (?,?,?,?,?)",
                        [(d.name, i, w, round(f(w), 2) if f else None, txt) for i, (w, txt) in enumerate(flow.get("events", []))])
        ev = A.load(sdir / "station_events.json", [])
        con.executemany("INSERT INTO station_events VALUES (?,?,?,?,?)",
                        [(d.name, i, e["t"], e["event"], e.get("pallet")) for i, e in enumerate(ev)])
        res = (A.load(d / "result.json", {}) or {}).get("station") if station else A.load(sdir / "station_results.json", [])
        per = row.get("cull_per_head_s") or []
        for r0 in res or []:
            for phase in ("inspection", "recheck"):
                con.executemany("INSERT INTO inspection VALUES (?,?,?,?,?,?,?)",
                                [(d.name, "first" if phase == "inspection" else "recheck", x.get("slot"), x.get("head"),
                                  x.get("label"), x.get("truth"), b(x.get("label") == x.get("truth"))) for x in r0.get(phase, [])])
            for i, c in enumerate(r0.get("culls", [])):
                hf = (c.get("head_final") or [None, None, None]) + [None] * 3
                con.execute("INSERT INTO culls VALUES (?,?,?,?,?,?,?,?,?,?,?)",
                            (d.name, i, c.get("slot"), c.get("colour"), c.get("box"), c.get("vision_xy_error_mm"), b(c.get("in_box")),
                             per[i] if i < len(per) else None, hf[0], hf[1], hf[2]))
        he = A.load(sdir / "human_events.json", {}) or {}
        con.executemany("INSERT INTO human_events VALUES (?,?,?,?)",
                        [(d.name, e.get("t"), e.get("event"), json.dumps({k: v for k, v in e.items() if k not in ("t", "event")}))
                         for e in he.get("events", [])])
        con.executemany("INSERT INTO run_files VALUES (?,?,?)",
                        [(d.name, str(p.relative_to(d)), p.stat().st_size) for p in d.rglob("*") if p.is_file()])
        n += 1
    con.commit()
    for t in ("runs", "flow_events", "station_events", "inspection", "culls", "chassis_track", "tilt_samples", "human_events", "run_files"):
        print(f"{t}: {con.execute(f'SELECT COUNT(*) FROM {t}').fetchone()[0]}")
    con.close()
    print(f"db -> {db} ({n} runs, {db.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
