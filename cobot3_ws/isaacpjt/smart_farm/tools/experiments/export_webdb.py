"""2026-09-27 experiments -> normalized tables for a web DB (PostgreSQL / Supabase, or SQLite).

  python export_webdb.py [OUT]      (OUT default D:\\smartfarm-sim\\out\\webdb_2026-09-27)

Writes OUT/csv/<table>.csv (UTF-8, header), OUT/data.json (all tables), OUT/schema_postgres.sql, OUT/load_postgres.sql
(psql \\copy), OUT/smartfarm_20260927.sqlite. Reads the raw run folders, not the report pages.
Tables
  experiment         one row per experiment (grasp margin, flow validation, DES, carry speed, reference)
  run                one simulator run (folder) - parameters in params (json), validity + reason
  grasp_trial        one cull attempt (station test + 9/26 baselines)
  flow_check         full-flow runs at 12 N·m: vision verdict / cull / recheck from isaac.log
  carry_result       carry-speed runs: drive/dock time, head sway, dock error
  head_sway_sample   per-second head sway on the carried tray (monitor pallet01_heads_series)
  des_scenario       DES fleet sizing (mean / sd over 20 reps)
  media              videos and clips per run (path relative to D:\\smartfarm-sim\\out)
  reference_spec     sim asset vs real cabbage / tray specs with source URLs
"""
import csv
import json
import re
import sqlite3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from analyze import load  # noqa: E402

OUT_ROOT = Path(r"D:\smartfarm-sim\out")
GRASP = OUT_ROOT / "exp_20260927_grasp"
SPEED = OUT_ROOT / "exp_20260927_speed"
OUT = Path(sys.argv[1] if len(sys.argv) > 1 else OUT_ROOT / "webdb_2026-09-27")

# table -> [(column, type)]; types: text int real bool json (mapped per DB below)
SCHEMA = {
    "experiment": [("experiment_id", "text pk"), ("title", "text"), ("run_date", "text"), ("kind", "text"),
                   ("description", "text"), ("code_commit", "text"), ("data_path", "text")],
    "run": [("run_id", "text pk"), ("experiment_id", "text fk:experiment"), ("condition", "text"), ("rep", "int"),
            ("valid", "bool"), ("invalid_reason", "text"), ("params", "json"), ("wall_min", "real"), ("source_path", "text")],
    "grasp_trial": [("trial_id", "int pk"), ("run_id", "text fk:run"), ("source", "text"), ("grip_force_nm", "real"),
                    ("head_mass_kg", "real"), ("cull_order", "int"), ("slot", "text"), ("head_shape", "text"),
                    ("colour", "text"), ("outcome", "text"), ("in_box", "bool"), ("finger_at_lift_rad", "real"),
                    ("finger_at_via_rad", "real"), ("pick_xy_error_mm", "real"), ("head_final_z_m", "real")],
    "flow_check": [("run_id", "text pk fk:run"), ("grip_force_nm", "real"), ("max_speed_mps", "real"),
                   ("verdict_correct", "int"), ("verdict_total", "int"), ("cull_ok", "int"), ("cull_attempts", "int"),
                   ("recheck_line", "text"), ("verdict_line", "text")],
    "carry_result": [("run_id", "text pk fk:run"), ("max_speed_mps", "real"), ("accel_mps2", "real"), ("dock_args", "text"),
                     ("head_mass_kg", "real"), ("nav_ok", "bool"), ("place_ok", "bool"), ("carry_s", "real"),
                     ("dock_s", "real"), ("fast_s", "int"), ("v_peak_mps", "real"), ("a_peak_mps2", "real"),
                     ("dist_m", "real"), ("chassis_tilt_deg", "text"), ("dock_status", "text"), ("dock_face_m", "real"),
                     ("dock_yaw_deg", "real"), ("dock_lat_m", "real"), ("dock_retries", "int"), ("head_disp_mm", "real"),
                     ("head_tilt_deg", "real"), ("tray_shift_mm", "real"), ("heads_kept", "bool")],
    "head_sway_sample": [("run_id", "text pk fk:run"), ("t_sim_s", "real pk"), ("head_disp_mm", "real"),
                         ("head_tilt_deg", "real"), ("tray_x_m", "real"), ("tray_y_m", "real"), ("tray_z_m", "real")],
    "des_scenario": [("scenario_id", "int pk"), ("experiment_id", "text fk:experiment"), ("drive_cond", "text"),
                     ("robots", "int"), ("stations", "int"), ("defect_rate", "real"), ("q_name", "text"),
                     ("cull_success_q", "real"), ("policy", "text"), ("trays_per_h", "real"), ("trays_per_h_sd", "real"),
                     ("station_util", "real"), ("station_util_sd", "real"), ("robot_wait_frac", "real"),
                     ("robot_wait_frac_sd", "real"), ("escaped_per_1000_trays", "real"),
                     ("escaped_per_1000_trays_sd", "real"), ("culled", "real"), ("culled_sd", "real"), ("reps", "int")],
    "media": [("media_id", "int pk"), ("run_id", "text fk:run"), ("kind", "text"), ("rel_path", "text"), ("bytes", "int")],
    "reference_spec": [("spec_id", "int pk"), ("category", "text"), ("item", "text"), ("is_sim", "bool"), ("width_mm", "text"),
                       ("height_mm", "text"), ("mass_kg", "text"), ("density_kg_m3", "text"), ("note", "text"),
                       ("source_url", "text")],
}
rows = {t: [] for t in SCHEMA}


def num(v):
    try:
        return None if v in (None, "", "-") else float(v)
    except (TypeError, ValueError):
        return None


# ---------- experiments ----------
rows["experiment"] = [
    dict(experiment_id="biz_0926", title="사업 검토 실험 (기준선)", run_date="2026-09-26", kind="sim_allinone+station",
         description="31회. 여기서는 집기 기준선(8 N·m, 0.3 kg) 선별 시도만 참조", code_commit="bbb5e38",
         data_path=r"out\exp_20260926"),
    dict(experiment_id="grasp_margin", title="쥐는 힘 x 포기 질량", run_date="2026-09-27", kind="sim_station",
         description="스테이션 단독, 정답 라벨, 패턴 B,Y,B,Y,B,Y (회당 선별 6건)", code_commit="53fb8fb",
         data_path=r"out\exp_20260927_grasp"),
    dict(experiment_id="flow_12nm", title="전체 흐름 12 N·m 확인", run_date="2026-09-27", kind="sim_allinone",
         description="SMARTFARM_GRIP_FORCE=12, 최고속도 0.6 m/s, 유효 3회 목표", code_commit="53fb8fb",
         data_path=r"out\exp_20260927_grasp\flow_f12_*"),
    dict(experiment_id="des_fleet", title="설비 구성 DES", run_date="2026-09-27", kind="des",
         description="로봇 1~5 x 스테이션 1~2 x 불량률 5/10/20 % x 선별 성공률 x 재시도 정책, 8 h x 20회",
         code_commit=None, data_path=r"out\exp_20260927_grasp\des_fleet.csv"),
    dict(experiment_id="carry_speed", title="운반 속도 (주행 + 도킹)", run_date="2026-09-27", kind="sim_allinone",
         description="수확 -> Nav2 + feeder_dock -> 내려놓기까지. 최고속도·가감속·도킹 속도·포기 질량", code_commit="3a4ae3a",
         data_path=r"out\exp_20260927_speed"),
    dict(experiment_id="reference", title="에셋 vs 실물 규격", run_date="2026-09-27", kind="literature",
         description="공개 자료 조사 (출처 URL 은 reference_spec)", code_commit=None, data_path=None),
]

# ---------- grasp runs + trials ----------
for d in sorted(p for p in GRASP.iterdir() if p.is_dir() and re.match(r"f\d+_m", p.name)):
    res = load(d / "result.json", {})
    rows["run"].append(dict(run_id=d.name, experiment_id="grasp_margin", condition=d.name, rep=1, valid=True,
                            invalid_reason=None, params=json.dumps({k: res.get(k) for k in ("grip_force", "head_mass",
                                                                                            "pattern", "truth_labels")}),
                            wall_min=round(res.get("wall_seconds", 0) / 60, 1) or None,
                            source_path=str(d.relative_to(OUT_ROOT))))
trials = json.load(open(GRASP / "trials.json"))
known = {r["run_id"] for r in rows["run"]}
for t in trials:
    if t["run"] not in known:        # 9/26 baseline runs referenced by trials
        known.add(t["run"])
        rows["run"].append(dict(run_id=t["run"], experiment_id="biz_0926", condition=t["run"], rep=None, valid=True,
                                invalid_reason=None, params=json.dumps({"grip_force": 8.0, "head_mass": 0.3,
                                                                        "mode": t["source"]}),
                                wall_min=None, source_path=str(Path("exp_20260926") / t["run"])))
for i, t in enumerate(trials, 1):
    rows["grasp_trial"].append(dict(trial_id=i, run_id=t["run"], source=t["source"], grip_force_nm=t["grip_force"],
                                    head_mass_kg=t["head_mass"], cull_order=t["cull_order"], slot=t["slot"],
                                    head_shape=t["shape"], colour=t["colour"], outcome=t["outcome"],
                                    in_box=bool(t["in_box"]), finger_at_lift_rad=t["finger_at_lift"],
                                    finger_at_via_rad=t["finger_at_via"], pick_xy_error_mm=t["pick_xy_error_mm"],
                                    head_final_z_m=t["head_final_z"]))

# ---------- flow validation ----------
for d in sorted(GRASP.glob("flow_f12_*")):
    done = load(d / "done.json", {})
    log = (d / "isaac.log").read_text(encoding="utf-8", errors="replace") if (d / "isaac.log").exists() else ""
    verdict = next(iter(re.findall(r"\[비전\] Pallet_01 판정: (.*)", log)), "")
    recheck = next(iter(re.findall(r"\[비전\] Pallet_01 재검사: (.*)", log)), "")
    slots = re.findall(r"\d\d=(\S+)", verdict)
    invalid = None if done.get("valid") else (done.get("note") or "카메라 검은 화면: 판정 전부 UNKNOWN")
    rows["run"].append(dict(run_id=d.name, experiment_id="flow_12nm", condition="f12_v0.6",
                            rep=int(d.name.rsplit("_r", 1)[1]), valid=bool(done.get("valid")), invalid_reason=invalid,
                            params=json.dumps({"grip_force": 12.0, "max_speed": 0.6, "route": "default"}),
                            wall_min=done.get("wall_min"), source_path=str(d.relative_to(OUT_ROOT))))
    rows["flow_check"].append(dict(run_id=d.name, grip_force_nm=12.0, max_speed_mps=0.6,
                                   verdict_correct=sum(1 for s in slots if "정답" not in s and "UNKNOWN" not in s),
                                   verdict_total=len(slots), cull_ok=len(re.findall(r"버림 성공", log)),
                                   cull_attempts=len(re.findall(r"SLOT_\d\d 버림", log)), recheck_line=recheck,
                                   verdict_line=verdict))

# ---------- carry speed ----------
speed = {r["run"]: r for r in json.load(open(SPEED / "speed_runs.json"))}
for d in sorted(p for p in SPEED.iterdir() if p.is_dir() and (p / "done.json").exists()):
    done = load(d / "done.json", {})
    cond, rep = d.name.rsplit("_r", 1)
    rows["run"].append(dict(run_id=d.name, experiment_id="carry_speed", condition=cond, rep=int(rep),
                            valid=not done.get("invalid"), invalid_reason=done.get("invalid"),
                            params=json.dumps({"max_speed": num(done.get("speed")), "accel": num(done.get("accel")) or 0.3,
                                               "dock_args": done.get("dock") or "", "head_mass": num(done.get("head_mass")) or 0.3}),
                            wall_min=done.get("wall_min"), source_path=str(d.relative_to(OUT_ROOT))))
    r = speed.get(d.name)
    if r:
        rows["carry_result"].append(dict(
            run_id=d.name, max_speed_mps=num(r["speed"]), accel_mps2=num(r["accel"]), dock_args=r["dock"],
            head_mass_kg=r["head_mass"], nav_ok=bool(r["nav_ok"]), place_ok=bool(r["place_ok"]), carry_s=r.get("nav_s"),
            dock_s=r.get("dock_s"), fast_s=r.get("fast_s"), v_peak_mps=r.get("v_peak"), a_peak_mps2=r.get("a_peak"),
            dist_m=r.get("dist_m"), chassis_tilt_deg=str(r.get("chassis_tilt_deg")), dock_status=r.get("dock_status"),
            dock_face_m=r.get("dock_face_m"), dock_yaw_deg=r.get("dock_yaw_deg"), dock_lat_m=r.get("dock_lat_m"),
            dock_retries=r.get("dock_retries"), head_disp_mm=r.get("head_disp_mm"), head_tilt_deg=r.get("head_tilt_deg"),
            tray_shift_mm=r.get("tray_shift_mm"), heads_kept=bool(r.get("kept"))))
    for s in load(d / "monitor.json", {}).get("pallet01_heads_series", []):
        rows["head_sway_sample"].append(dict(run_id=d.name, t_sim_s=s[0], head_disp_mm=s[1], head_tilt_deg=s[2],
                                             tray_x_m=s[3], tray_y_m=s[4], tray_z_m=s[5]))

# ---------- DES ----------
for i, r in enumerate(csv.DictReader(open(GRASP / "des_fleet.csv", encoding="utf-8")), 1):
    rows["des_scenario"].append(dict(scenario_id=i, experiment_id="des_fleet", drive_cond=r["cond"], robots=int(r["robots"]),
                                     stations=int(r["stations"]), defect_rate=float(r["defect_rate"]), q_name=r["q_name"],
                                     cull_success_q=float(r["q"]), policy=r["policy"], reps=20,
                                     **{k: float(r[k]) for k in ("trays_per_h", "trays_per_h_sd", "station_util",
                                                                 "station_util_sd", "robot_wait_frac", "robot_wait_frac_sd",
                                                                 "escaped_per_1000_trays", "escaped_per_1000_trays_sd",
                                                                 "culled", "culled_sd")}))

# ---------- media ----------
mid = 0
for base in (SPEED,):
    for d in sorted(p for p in base.iterdir() if p.is_dir()):
        for f in sorted(d.glob("*.mp4")):
            mid += 1
            rows["media"].append(dict(media_id=mid, run_id=d.name, kind=f.stem, rel_path=str(f.relative_to(OUT_ROOT)),
                                      bytes=f.stat().st_size))

# ---------- reference specs ----------
REF = [
    ("head", "시뮬 에셋 포기 A/B/C", True, "85~90", "56~61", "0.3", "≈1280", "편구 부피 약 235 cm³", "build_info.json"),
    ("head", "미니 양배추 Tiara", False, None, None, "0.45~0.9", None, "정식 후 63일",
     "https://www.johnnyseeds.com/vegetables/cabbage/fresh-market-cabbage/tiara-f1-cabbage-seed-3626.html"),
    ("head", "미니 양배추 Gonzales", False, "100~150", None, "≈0.45", None, None,
     "https://www.gardeningknowhow.com/edible/vegetables/cabbage/how-to-grow-gonzales-cabbage.htm"),
    ("head", "양배추 유메부타이 실측", False, "≈200", "≈126", "1.40~1.62", "510~644", "밀도군 3종",
     "https://www.jstage.jst.go.jp/article/fstr/15/1/15_1_11/_pdf"),
    ("head", "KAMIS 조사 등급", False, None, None, "1.5~4.0", None, "하 1.5~2 / 중 2~3 / 상 3~4 kg",
     "https://www.kamis.or.kr/customer/price/knowhow/knowhow.do?action=search&search_itemcategorycode=200"),
    ("tray", "시뮬 트레이 cabbage_pallet_6", True, "552 x 252", "80", "1.0", None, "2 x 3구, 간격 150 x 189 mm",
     "build_info.json"),
    ("tray", "육묘 트레이 (범농)", False, "540 x 280", None, None, None, "32~512구, 128구 = 8 x 16",
     "http://www.bumnong.com/v2/kr/products/index2_07.htm"),
    ("tray", "농산물 표준 플라스틱 상자", False, "550 x 366", "155~350", None, None, "허용오차 ±3 mm (별표 2)",
     "https://www.law.go.kr/flDownload.do?flSeq=155174507"),
    ("trade", "가락시장 거래 단위", False, None, None, "8 (망당 3개)", None, "실제 10 kg 초과 많음",
     "https://www.nongmin.com/article/20200102318434"),
    ("friction", "양배추-벨트 마찰", False, None, None, None, None, "μ 0.364", "https://doi.org/10.5424/sjar/2023211-19979"),
    ("friction", "배추-컨베이어 벨트 마찰", False, None, None, None, None, "μ 0.81",
     "https://pmc.ncbi.nlm.nih.gov/articles/PMC12084294/"),
    ("gripper", "RG2 완전 개방 패드 간격 (Isaac 실측)", True, "99.6", None, None, None, "docs/cabbage_asset_and_allinone.md",
     None),
]
for i, (cat, item, sim, w, h, m, rho, note, url) in enumerate(REF, 1):
    rows["reference_spec"].append(dict(spec_id=i, category=cat, item=item, is_sim=sim, width_mm=w, height_mm=h, mass_kg=m,
                                       density_kg_m3=rho, note=note, source_url=url))

# ---------- write ----------
(OUT / "csv").mkdir(parents=True, exist_ok=True)
for t, cols in SCHEMA.items():
    names = [c for c, _ in cols]
    with open(OUT / "csv" / f"{t}.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=names)
        w.writeheader()
        for r in rows[t]:
            w.writerow({k: ("true" if v is True else "false" if v is False else v) for k, v in r.items()})
json.dump(rows, open(OUT / "data.json", "w", encoding="utf-8"), ensure_ascii=False, indent=1)


def ddl(dialect):
    tmap = {"postgres": {"text": "TEXT", "int": "INTEGER", "real": "DOUBLE PRECISION", "bool": "BOOLEAN", "json": "JSONB"},
            "sqlite": {"text": "TEXT", "int": "INTEGER", "real": "REAL", "bool": "INTEGER", "json": "TEXT"}}[dialect]
    out = []
    for t, cols in SCHEMA.items():
        lines, pk, fks = [], [], []
        for c, spec in cols:
            parts = spec.split()
            lines.append(f"  {c} {tmap[parts[0]]}")
            if "pk" in parts:
                pk.append(c)
            for p in parts:
                if p.startswith("fk:"):
                    ref = p[3:]
                    fks.append(f"  FOREIGN KEY ({c}) REFERENCES {ref}({SCHEMA[ref][0][0]})")
        lines.append(f"  PRIMARY KEY ({', '.join(pk)})")
        out.append(f"CREATE TABLE {t} (\n" + ",\n".join(lines + fks) + "\n);")
    out.append("CREATE INDEX idx_run_experiment ON run(experiment_id);")
    out.append("CREATE INDEX idx_trial_run ON grasp_trial(run_id);")
    out.append("CREATE INDEX idx_media_run ON media(run_id);")
    out.append("""CREATE VIEW v_grasp_by_condition AS
  SELECT source, grip_force_nm, head_mass_kg, COUNT(*) AS n, SUM(CASE WHEN in_box THEN 1 ELSE 0 END) AS ok,
         SUM(CASE WHEN outcome = 'lift_slip' THEN 1 ELSE 0 END) AS lift_slip,
         SUM(CASE WHEN outcome = 'drop_out' THEN 1 ELSE 0 END) AS drop_out, AVG(finger_at_lift_rad) AS finger_lift_mean
  FROM grasp_trial GROUP BY source, grip_force_nm, head_mass_kg;""")
    out.append("""CREATE VIEW v_carry_by_condition AS
  SELECT r.condition, c.max_speed_mps, c.accel_mps2, c.dock_args, c.head_mass_kg, COUNT(*) AS n,
         SUM(CASE WHEN c.nav_ok AND c.place_ok THEN 1 ELSE 0 END) AS ok, AVG(c.carry_s) AS carry_s_mean,
         MAX(c.head_disp_mm) AS head_disp_max_mm
  FROM carry_result c JOIN run r ON r.run_id = c.run_id WHERE r.valid GROUP BY r.condition, c.max_speed_mps,
         c.accel_mps2, c.dock_args, c.head_mass_kg;""")
    return "\n\n".join(out) + "\n"


(OUT / "schema_postgres.sql").write_text("-- smartfarm experiments 2026-09-27 (PostgreSQL / Supabase)\n" + ddl("postgres"),
                                          encoding="utf-8")
order = ["experiment", "run", "grasp_trial", "flow_check", "carry_result", "head_sway_sample", "des_scenario", "media",
         "reference_spec"]
(OUT / "load_postgres.sql").write_text(
    "-- psql -d DB -f schema_postgres.sql && psql -d DB -f load_postgres.sql   (run in this folder)\n"
    + "\n".join(f"\\copy {t} FROM 'csv/{t}.csv' WITH (FORMAT csv, HEADER true, ENCODING 'UTF8')" for t in order) + "\n",
    encoding="utf-8")
db = OUT / "smartfarm_20260927.sqlite"
if db.exists():
    db.unlink()
con = sqlite3.connect(db)
con.execute("PRAGMA foreign_keys = ON")
con.executescript(ddl("sqlite"))
for t in order:
    names = [c for c, _ in SCHEMA[t]]
    con.executemany(f"INSERT INTO {t} VALUES ({','.join('?' * len(names))})",
                    [[(int(v) if isinstance(v, bool) else v) for v in (r.get(c) for c in names)] for r in rows[t]])
con.commit()
for t in order:
    print(f"{t:18s} {len(rows[t]):6d}")
print("fk violations:", con.execute("PRAGMA foreign_key_check").fetchall()[:5])
