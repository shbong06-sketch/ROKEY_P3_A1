"""Grasp-margin experiment (2026-09-27) -> SQLite + summary.

  python grasp_analyze.py [ROOT]      (ROOT default D:\\smartfarm-sim\\out\\exp_20260927_grasp)

Rows = one cull attempt. Sources: this experiment (station test, --truth-labels, pattern B,Y,B,Y,B,Y, factors
grip force N·m x head mass kg) + the 2026-09-26 business runs (force 8, mass 0.3; full flow vs station) for
comparison. Outcome classes:
  ok          head ended inside the SortBin
  lift_slip   finger closed to >= 1.0 rad by the VIA stage = hand empty = head slipped out while lifting
  drop_out    head was carried (finger < 1.0 at VIA) but ended outside the bin
"""
import json
import os
import sqlite3
import statistics as st
import sys
from pathlib import Path

ROOT = Path(sys.argv[1] if len(sys.argv) > 1 else r"D:\smartfarm-sim\out\exp_20260927_grasp")
OLD = Path(r"D:\smartfarm-sim\out\exp_20260926")
SHAPE = {"01": "A", "02": "B", "03": "C", "04": "A", "05": "B", "06": "C"}     # cabbage_pallet_6 build_info slots


def trials(run_dir, source, force, mass, recs):
    out = []
    for r in recs or []:
        colour = {x["slot"]: x.get("truth") for x in r.get("inspection", [])}
        for i, c in enumerate(r.get("culls", [])):
            tr = {t["stage"]: t for t in c.get("trace", [])}
            f_lift = tr.get("LIFT", {}).get("finger")
            f_via = tr.get("VIA", {}).get("finger")
            slip = f_via is not None and f_via >= 1.0
            cls = "ok" if c.get("in_box") else ("lift_slip" if slip else "drop_out")
            out.append(dict(run=run_dir.name, source=source, grip_force=force, head_mass=mass, cull_order=i + 1,
                            slot=c["slot"], shape=SHAPE.get(c["slot"][-2:]), colour=colour.get(c["slot"]),
                            outcome=cls, in_box=int(bool(c.get("in_box"))), finger_at_lift=f_lift, finger_at_via=f_via,
                            pick_xy_error_mm=c.get("pick_xy_error_mm"), head_final_z=(c.get("head_final") or [None] * 3)[2]))
    return out


rows = []
for d in sorted(p for p in ROOT.iterdir() if p.is_dir()):
    f = d / "result.json"
    if not f.exists():
        continue
    res = json.load(open(f, encoding="utf-8"))
    rows += trials(d, "grasp_exp", res.get("grip_force"), res.get("head_mass"), res.get("station"))
for d in sorted(p for p in OLD.iterdir() if p.is_dir() and p.name[:3] in ("e0_", "e1_", "e2_", "e3_")):
    if (d / "station" / "station_results.json").exists():
        rows += trials(d, "0926_flow", 8.0, 0.3, json.load(open(d / "station" / "station_results.json", encoding="utf-8")))
    elif (d / "result.json").exists():
        rows += trials(d, "0926_station", 8.0, 0.3, json.load(open(d / "result.json", encoding="utf-8")).get("station"))

db = ROOT / "grasp.sqlite"
if db.exists():
    db.unlink()
con = sqlite3.connect(db)
cols = list(rows[0].keys())
con.execute(f"CREATE TABLE trials ({', '.join(cols)})")
con.executemany(f"INSERT INTO trials VALUES ({','.join('?' * len(cols))})", [[r[c] for c in cols] for r in rows])
con.executescript("""
CREATE VIEW v_by_condition AS
  SELECT source, grip_force, head_mass, COUNT(*) n, SUM(in_box) ok, SUM(outcome='lift_slip') lift_slip,
         SUM(outcome='drop_out') drop_out, ROUND(AVG(finger_at_lift),3) finger_lift_mean, ROUND(MIN(finger_at_lift),3) finger_lift_min
  FROM trials GROUP BY source, grip_force, head_mass;
CREATE VIEW v_by_finger AS
  SELECT ROUND(finger_at_lift, 2) finger_bin, COUNT(*) n, SUM(in_box) ok FROM trials WHERE finger_at_lift IS NOT NULL
  GROUP BY finger_bin;
""")
con.commit()

print(f"trials {len(rows)} -> {db}")
print("source       force  mass   n  ok  slip  drop  finger@LIFT mean/min")
for r in con.execute("SELECT * FROM v_by_condition ORDER BY source, head_mass, grip_force"):
    print(f"{r[0]:<12} {r[1]:>5} {r[2]:>5} {r[3]:>3} {r[4]:>3} {r[5]:>5} {r[6]:>5}   {r[7]} / {r[8]}")
ok_f = [r["finger_at_lift"] for r in rows if r["in_box"] and r["finger_at_lift"] is not None]
bad_f = [r["finger_at_lift"] for r in rows if r["outcome"] == "lift_slip" and r["finger_at_lift"] is not None]
if ok_f and bad_f:
    print(f"finger@LIFT: success {min(ok_f):.3f}..{max(ok_f):.3f} (mean {st.mean(ok_f):.3f}) | "
          f"slip {min(bad_f):.3f}..{max(bad_f):.3f} (mean {st.mean(bad_f):.3f})")
by_order = {}
for r in rows:
    if r["source"] == "grasp_exp":
        by_order.setdefault(r["cull_order"], []).append(r["in_box"])
print("success by cull order (grasp_exp):", {k: f"{sum(v)}/{len(v)}" for k, v in sorted(by_order.items())})
by_shape = {}
for r in rows:
    if r["source"] == "grasp_exp":
        by_shape.setdefault(r["shape"], []).append(r["in_box"])
print("success by head shape (grasp_exp):", {k: f"{sum(v)}/{len(v)}" for k, v in sorted(by_shape.items())})
json.dump(rows, open(ROOT / "trials.json", "w"), indent=1)
