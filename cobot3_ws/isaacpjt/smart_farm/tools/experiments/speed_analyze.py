"""Driving-speed experiment (2026-09-27): does a faster carry (top speed / accel / docking speeds) keep the heads on the tray?

  python speed_analyze.py [ROOT]      (ROOT default D:\\smartfarm-sim\\out\\exp_20260927_speed)

Per run (harvest -> Nav2 + feeder_dock -> place, cut after the place):
  nav_s        navigation command -> result (Nav2 drive + docking)
  drive_s      moving part of that window, dock_s = from the dock SETTLE log to DONE (feeder_dock wall stamps -> sim)
  carry window pick result -> place command (tray on the lift fork)
  head_disp_mm / head_tilt_deg  max over the carry window of the Pallet_01 heads relative to the tray
                                (monitor pallet01_heads_series; 0.25 s sampling, 1 s windows)
  tray_shift_mm                 tray centre drift on the fork (chassis frame) over the carry window
  a_peak       peak |dv/dt| of the chassis from the 1 s track (a lower bound of the real peak)
  dock result  face distance / yaw / lateral error from feeder_dock [DONE]
Criterion (kept): every head within 10 mm of its seat (seat depth ~9-10 mm, build_info h_seat) and place SUCCEEDED.
"""
import json
import math
import re
import sqlite3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from analyze import load, wall_to_sim  # noqa: E402

ROOT = Path(sys.argv[1] if len(sys.argv) > 1 else r"D:\smartfarm-sim\out\exp_20260927_speed")
KEEP_MM = 10.0


def run_row(d):
    done, flow, mon = load(d / "done.json", {}), load(d / "flow.json", {}), load(d / "monitor.json", {})
    row = {"run": d.name, "cond": done.get("cond"), "speed": done.get("speed"), "accel": done.get("accel") or "0.3",
           "dock": done.get("dock") or "", "head_mass": float(done.get("head_mass") or 0.3), "placed": bool(done.get("placed"))}
    track = mon.get("chassis_track", [])
    f = wall_to_sim(track)
    evs = flow.get("events", [])

    def t_of(pat, nth=0):
        hits = [w for w, txt in evs if re.search(pat, txt)]
        return f(hits[nth]) if f and len(hits) > nth else None

    pr = t_of(r"sim result", 0)
    nc, nr = t_of(r"navigation command"), t_of(r"navigation result")
    lc = t_of(r"sim command PLACE_INSPECT")
    row["nav_ok"] = bool(re.search(r"navigation result SUCCEEDED", " ".join(t for _, t in evs)))
    row["place_ok"] = (flow.get("place") or {}).get("status") == "SUCCEEDED" or bool(
        re.search(r"PLACE_INSPECT.*\n.*sim result SUCCEEDED", "\n".join(t for _, t in evs)))
    row["nav_s"] = round(nr - nc, 1) if nc is not None and nr is not None else None
    if nc is not None and nr is not None:
        seg = [r for r in track if nc <= r[0] <= nr]
        moving = [b[0] for a, b in zip(seg, seg[1:]) if math.hypot(b[1] - a[1], b[2] - a[2]) > 0.2]   # > 0.2 m/s
        row["fast_s"] = len(moving)
        vs = [(b[0], math.hypot(b[1] - a[1], b[2] - a[2]) / max(b[0] - a[0], 1e-6)) for a, b in zip(seg, seg[1:])]
        row["v_peak"] = round(max((v for _, v in vs), default=0.0), 2)
        row["a_peak"] = round(max((abs(b[1] - a[1]) / max(b[0] - a[0], 1e-6) for a, b in zip(vs, vs[1:])), default=0.0), 2)
        row["dist_m"] = round(sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(seg, seg[1:])), 2)
        tilts = [s[1] for s in mon.get("tilt_samples_over_0p5deg", []) if nc <= s[0] <= nr]
        row["chassis_tilt_deg"] = round(max(tilts), 2) if tilts else "<0.5"
    # docking phases from feeder_dock.log (ROS stamps are wall seconds)
    dl = (d / "feeder_dock.log").read_text(encoding="utf-8", errors="replace") if (d / "feeder_dock.log").exists() else ""
    stamps = {}
    for m in re.finditer(r"\[INFO\] \[(\d+\.\d+)\] \[feeder_dock\]: \[(\w+)\]", dl):
        stamps.setdefault(m.group(2), float(m.group(1)))
    if f and "SETTLE" in stamps and "DONE" in stamps:
        row["dock_s"] = round(f(stamps["DONE"]) - f(stamps["SETTLE"]), 1)
    m = re.search(r"\[DONE\] (\{.*\})", dl)
    if m:
        res = json.loads(m.group(1))
        row.update(dock_status=res.get("status"), dock_face_m=res.get("face_dist_m"), dock_yaw_deg=res.get("yaw_err_deg"),
                   dock_lat_m=res.get("lat_m"))
    row["dock_retries"] = len(re.findall(r"\[BACKOFF\]|retry", dl))
    ser = mon.get("pallet01_heads_series", [])
    end = lc if lc is not None else (nr + 20 if nr is not None else None)     # 주행 실패 회차: 실패 후 20 s 까지
    if pr is not None and end is not None and ser:
        w = [s for s in ser if pr + 1 <= s[0] <= end]
        if w:
            row["head_disp_mm"] = round(max(s[1] for s in w), 2)
            row["head_tilt_deg"] = round(max(s[2] for s in w), 2)
            x0 = w[0][3:6]
            row["tray_shift_mm"] = round(max(math.dist(s[3:6], x0) for s in w) * 1000, 1)
    row["kept"] = row.get("head_disp_mm") is not None and row["head_disp_mm"] <= KEEP_MM
    return row


rows = [run_row(d) for d in sorted(ROOT.iterdir()) if d.is_dir() and (d / "done.json").exists()
        and not load(d / "done.json", {}).get("invalid")]          # invalid = 실험과 무관한 기동 실패 (Nav2 bringup 등)
cols = sorted({k for r in rows for k in r}, key=lambda k: list(rows[0].keys()).index(k) if k in rows[0] else 99)
db = ROOT / "speed.sqlite"
if db.exists():
    db.unlink()
con = sqlite3.connect(db)
con.execute(f"CREATE TABLE runs ({', '.join(cols)})")
con.executemany(f"INSERT INTO runs VALUES ({','.join('?' * len(cols))})", [[r.get(c) for c in cols] for r in rows])
con.commit()
json.dump(rows, open(ROOT / "speed_runs.json", "w"), indent=1)
show = ["run", "head_mass", "nav_ok", "place_ok", "nav_s", "dock_s", "fast_s", "v_peak", "a_peak", "chassis_tilt_deg", "head_disp_mm",
        "head_tilt_deg", "tray_shift_mm", "dock_face_m", "dock_yaw_deg", "dock_lat_m", "dock_retries"]
print(" | ".join(show))
for r in rows:
    print(" | ".join(str(r.get(k, "-")) for k in show))
