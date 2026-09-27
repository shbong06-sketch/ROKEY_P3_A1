"""Old vs new YOLO on the station's own camera frames from the 2026-09-26 business experiments (31 runs).

  (PYTHONPATH as train_eval_yolo.py)  D:\\isaacsim\\kit\\python\\python.exe eval_station_frames.py OUT_JSON W1.pt W2.pt ...

Frames: <run>/station/<Pallet>_{0,1,2}.png (first inspection) and _recheck_{0,1,2}.png, the raw RGB the station fed to
YOLO. Truth per frame = colour counts of the heads on the tray (DB table 'inspection': first = all 6 slots' truth,
recheck = slots that were not culled). Image-level score (no boxes needed): predicted colour counts (conf >= 0.35)
== truth counts, plus per-colour count error and the confidence of detections.
"""
import json
import sqlite3
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(r"D:\smartfarm-sim\out\exp_20260926")
DB = ROOT / "results" / "sim_runs.sqlite"
SHORT = {"lettuce_dark_green": "green", "lettuce_yellow": "yellow", "lettuce_brown": "brown"}
CONF_MIN = 0.35

con = sqlite3.connect(DB)
truth = {}
for run, phase, slot, t in con.execute("SELECT run, phase, slot, truth FROM inspection WHERE phase = 'first'"):
    truth.setdefault((run, "first"), {})[slot] = t
# recheck: the DB keeps every slot's original colour; the culled heads are no longer on the tray
# a failed cull (in_box = 0) may leave the head on the tray or drop it elsewhere -> that run's recheck is skipped
culled, failed = {}, set()
for run, slot, in_box in con.execute("SELECT run, slot, in_box FROM culls"):
    culled.setdefault(run, set()).add(slot)
    if not in_box:
        failed.add(run)
for (run, _), t in list(truth.items()):
    if run not in failed:
        truth[(run, "recheck")] = {k: v for k, v in t.items() if k not in culled.get(run, set())}
frames = []
for run_dir in sorted(p for p in ROOT.iterdir() if p.is_dir() and p.name[:3] in ("e0_", "e1_", "e2_", "e3_")):
    st = run_dir / "station" if (run_dir / "station").exists() else run_dir
    for png in sorted(st.glob("*.png")):
        phase = "recheck" if "_recheck_" in png.name else "first"
        t = truth.get((run_dir.name, phase))
        if t:
            frames.append((png, Counter(v for v in t.values() if v)))
print(f"{len(frames)} frames with truth", flush=True)

from ultralytics import YOLO  # noqa: E402

out = {}
for w in sys.argv[2:]:
    m = YOLO(w)
    r = {"frames": len(frames), "exact": 0, "abs_count_error": Counter(), "conf": [], "low_conf_0.35_0.6": 0,
         "worst": []}
    for png, tc in frames:
        res = m.predict(str(png), imgsz=640, conf=CONF_MIN, verbose=False)[0]
        pc = Counter(SHORT.get(m.names[int(c)], m.names[int(c)]) for c in res.boxes.cls.tolist())
        confs = [float(s) for s in res.boxes.conf.tolist()]
        r["conf"] += confs
        r["low_conf_0.35_0.6"] += sum(1 for s in confs if s < 0.6)
        err = {c: pc.get(c, 0) - tc.get(c, 0) for c in ("green", "yellow", "brown")}
        for c, e in err.items():
            r["abs_count_error"][c] += abs(e)
        if all(e == 0 for e in err.values()):
            r["exact"] += 1
        elif len(r["worst"]) < 8:
            r["worst"].append({"frame": str(png.relative_to(ROOT)), "truth": dict(tc), "pred": dict(pc)})
    c = sorted(r.pop("conf"))
    r["exact_rate"] = round(r["exact"] / max(1, len(frames)), 4)
    r["abs_count_error"] = dict(r["abs_count_error"])
    r["detections"] = len(c)
    r["conf_mean"] = round(sum(c) / max(1, len(c)), 3)
    r["conf_p10"] = round(c[len(c) // 10], 3) if c else None
    out[w] = r
    print(w, json.dumps(r, ensure_ascii=False), flush=True)
json.dump(out, open(sys.argv[1], "w"), indent=1, ensure_ascii=False)
