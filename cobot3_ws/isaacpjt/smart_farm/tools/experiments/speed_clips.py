"""Cut the drive + dock + place part out of each speed run's navplace captures -> <run>/clip_navplace.mp4 (4x speed).

  python speed_clips.py [ROOT]      (needs WSL ffmpeg; frames = captures/cap_navplace_<t_sim>.jpg every 0.5 s)
"""
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from analyze import load, wall_to_sim  # noqa: E402

ROOT = Path(sys.argv[1] if len(sys.argv) > 1 else r"D:\smartfarm-sim\out\exp_20260927_speed")


def wsl(p):
    p = str(p).replace("\\", "/")
    return "/mnt/" + p[0].lower() + p[2:]


for d in sorted(p for p in ROOT.iterdir() if (p / "captures").is_dir() and (p / "done.json").exists()):
    if (d / "clip_navplace.mp4").exists():
        continue
    mon, flow = load(d / "monitor.json", {}), load(d / "flow.json", {})
    f = wall_to_sim(mon.get("chassis_track", []))
    evs = flow.get("events", [])
    nav = [f(w) for w, t in evs if "navigation command" in t]
    if not f or not nav:
        continue
    t0, t1 = nav[0] - 2.0, (f(evs[-1][0]) if evs else nav[0] + 60) + 25.0
    frames = sorted((float(m.group(1)), p) for p in (d / "captures").glob("cap_navplace_*.jpg")
                    if (m := re.search(r"_(\d+\.\d)\.jpg$", p.name)) and t0 <= float(m.group(1)) <= t1)
    if not frames:
        continue
    tmp = Path(tempfile.mkdtemp(dir=d))
    for i, (_, p) in enumerate(frames):
        shutil.copy(p, tmp / f"f{i:05d}.jpg")
    out = d / "clip_navplace.mp4"
    subprocess.run(["wsl", "-d", "Ubuntu-24.04", "--", "ffmpeg", "-y", "-loglevel", "error", "-framerate", "8", "-i",
                    wsl(tmp) + "/f%05d.jpg", "-vf", "scale=960:-2,format=yuv420p", "-c:v", "libx264", "-crf", "27",
                    wsl(out)], check=True)
    shutil.rmtree(tmp)
    print(f"{d.name}: {len(frames)} frames t {frames[0][0]:.1f}-{frames[-1][0]:.1f} -> {out.name}")
