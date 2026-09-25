"""Cut the all-in-one process-camera captures into one video per process (+ one combined video).

  python 10_make_process_videos.py RUN_DIR [FPS]        (needs numpy + opencv + pillow: pylib / pylib_yolo)

RUN_DIR = run_allinone_live.ps1 -OutDir (captures/cap_<cam>_<t>.jpg, monitor.json, station/station_events.json,
station/*_yolo.jpg). Windows (sim time):
  1 harvest      cam1  start of arm motion at the rack  -> 6 s after the chassis starts driving
  2 nav2+place   cam2  chassis starts driving           -> the tray arrives in the vision room
  3 inspection   cam3  (wrist RealSense) push done      -> inspection result, + YOLO result frames, + recheck
  4 cull         cam4  tray arrives                     -> tray pushed back + 8 s
"""
import glob, json, os, re, sys
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

RUN = sys.argv[1]
FPS = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0      # captures are 5 per sim-second -> 2x speed
OUT = os.path.join(RUN, "videos")
os.makedirs(OUT, exist_ok=True)
FONT = ImageFont.truetype(r"C:\Windows\Fonts\malgunbd.ttf", 30)
FONT_S = ImageFont.truetype(r"C:\Windows\Fonts\malgun.ttf", 22)
SIZE = (960, 540)


def frames(cam):
    out = []
    for p in glob.glob(os.path.join(RUN, "captures", f"cap_{cam}_*.jpg")):
        t = float(re.search(r"_(\d+\.\d)\.jpg$", p).group(1))
        out.append((t, p))
    return sorted(out)


def label(img, title, sub=None):
    im = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    d = ImageDraw.Draw(im, "RGBA")
    d.rectangle([0, 0, SIZE[0], 50 if sub is None else 82], fill=(0, 0, 0, 150))
    d.text((16, 6), title, font=FONT, fill=(255, 255, 255, 255))
    if sub:
        d.text((16, 48), sub, font=FONT_S, fill=(255, 220, 120, 255))
    return cv2.cvtColor(np.asarray(im), cv2.COLOR_RGB2BGR)


def fit(img):
    h, w = img.shape[:2]
    if (w, h) == SIZE:
        return img
    scale = min(SIZE[0] / w, SIZE[1] / h)
    img = cv2.resize(img, (int(w * scale), int(h * scale)))
    canvas = np.zeros((SIZE[1], SIZE[0], 3), np.uint8)
    y, x = (SIZE[1] - img.shape[0]) // 2, (SIZE[0] - img.shape[1]) // 2
    canvas[y:y + img.shape[0], x:x + img.shape[1]] = img
    return canvas


def title_card(text, sub, seconds=2.0):
    im = Image.new("RGB", SIZE, (18, 22, 30))
    d = ImageDraw.Draw(im)
    d.text((60, 210), text, font=ImageFont.truetype(r"C:\Windows\Fonts\malgunbd.ttf", 46), fill=(255, 255, 255))
    d.text((60, 290), sub, font=FONT_S, fill=(200, 210, 230))
    return [cv2.cvtColor(np.asarray(im), cv2.COLOR_RGB2BGR)] * int(seconds * FPS)


def write(name, imgs):
    path = os.path.join(OUT, name)
    vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, SIZE)
    for im in imgs:
        vw.write(im)
    vw.release()
    print(f"{name}: {len(imgs)} frames, {len(imgs) / FPS:.1f} s")
    return path


def motion_start(fr, thresh=2.0):
    """first capture that differs from the first one (the arm starts moving)"""
    ref = cv2.imread(fr[0][1], cv2.IMREAD_GRAYSCALE).astype(np.float32)
    for t, p in fr:
        if np.abs(cv2.imread(p, cv2.IMREAD_GRAYSCALE).astype(np.float32) - ref).mean() > thresh:
            return t
    return fr[0][0]


mon = json.load(open(os.path.join(RUN, "monitor.json")))
track = mon["chassis_track"]
x0, y0 = track[0][1], track[0][2]
drive_start = next((t for t, x, y, _ in track if np.hypot(x - x0, y - y0) > 0.05), track[-1][0])
events = json.load(open(os.path.join(RUN, "station", "station_events.json"), encoding="utf-8"))
ev = {}
for e in events:
    if e["pallet"] == "Pallet_01":
        ev.setdefault(e["event"], e["t"])
culls = [e for e in events if e["event"].startswith("CULL_DONE") and e["pallet"] == "Pallet_01"]
t_arrive = ev["ARRIVED_PUSH_START"]
t_push = ev["PUSH_DONE_INSPECT_MOVE"]
t_insp = ev["INSPECTION_DONE"]
t_recheck = ev.get("RECHECK_DONE", t_insp)
t_back = ev.get("PUSH_BACK_DONE_RELEASED", t_recheck)
print("windows: drive_start", drive_start, "arrive", t_arrive, "push", t_push, "inspection", t_insp, "back", t_back)

results = json.load(open(os.path.join(RUN, "station", "station_results.json"), encoding="utf-8"))
rec = next(r for r in results if r["pallet"] == "Pallet_01")
verdict = " ".join(f"{r['slot'][-2:]}={r['label']}" for r in rec["inspection"])
removed = ", ".join(f"{c['slot']}({c['colour']})→{c['box']}" for c in rec.get("culls", []) if c.get("in_box"))

videos = []
# 1 harvest
f1 = frames("cam1_harvest")
s1 = max(f1[0][0], motion_start(f1) - 1.0)
clip1 = [label(fit(cv2.imread(p)), "① 랙에서 양배추 팔레트 수확 (PICK_HARVEST)", f"t = {t:.1f} s")
         for t, p in f1 if s1 <= t <= drive_start + 6.0]
videos.append(("01_harvest_rack.mp4", clip1, "① 수확", "랙의 Pallet_01 을 포크로 들어 올림"))
# 2 nav2 + place
f2 = frames("cam2_nav2place")
clip2 = [label(fit(cv2.imread(p)), "② Nav2 주행 → 컨베이어 앞 도킹 → 팔레트 내려놓기", f"t = {t:.1f} s")
         for t, p in f2 if drive_start - 1.0 <= t <= t_arrive]
videos.append(("02_nav2_drive_place.mp4", clip2, "② 이동·내려놓기", "Nav2 FEEDER_DOCK + PLACE_INSPECT + 컨베이어 반송"))
# 3 inspection (wrist camera) + YOLO frames
f3 = frames("cam3_inspection")
clip3 = [label(fit(cv2.imread(p)), "③ 비전룸 검사 — 손목 RealSense 화면", f"t = {t:.1f} s")
         for t, p in f3 if t_push <= t <= t_insp + 1.0]
for p in sorted(glob.glob(os.path.join(RUN, "station", "Pallet_01_[0-9]_yolo.jpg")))[:1]:
    clip3 += [label(fit(cv2.imread(p)), "③ YOLO(best.pt) 판정 결과", verdict)] * int(4 * FPS)
for p in sorted(glob.glob(os.path.join(RUN, "station", "Pallet_01_recheck_[0-9]_yolo.jpg")))[:1]:
    clip3 += [label(fit(cv2.imread(p)), "③ 솎아내기 후 재검사", "불량(노랑·갈색) 칸이 비었는지 확인")] * int(3 * FPS)
videos.append(("03_vision_inspection.mp4", clip3, "③ 비전 검사", "YOLO 로 칸별 색 판정 (노랑·갈색 = 제거 대상)"))
# 4 cull pick and place
f4 = frames("cam4_cullpickplace")
clip4 = []
for t, p in f4:
    if t_arrive - 2.0 <= t <= t_back + 8.0:
        step = ("푸셔가 트레이를 로봇 앞으로" if t < t_push else "검사 자세" if t < t_insp else
                "솎아내기 → SortBin 1/2 번갈아 버리기" if t < t_recheck else "재검사 · 푸셔가 벨트로 되돌림 · 배출")
        clip4.append(label(fit(cv2.imread(p)), "④ 비전룸 픽앤플레이스 (불량 제거)", f"t = {t:.1f} s · {step}"))
videos.append(("04_cull_pick_place.mp4", clip4, "④ 픽앤플레이스", f"제거: {removed}"))

combined = []
for name, clip, title, sub in videos:
    write(name, clip)
    combined += title_card(title, sub) + clip
write("00_all_processes.mp4", combined)
json.dump({"windows": {"drive_start": drive_start, "arrive": t_arrive, "push_done": t_push, "inspection": t_insp,
                       "recheck": t_recheck, "push_back": t_back},
           "verdict": verdict, "removed": removed}, open(os.path.join(OUT, "videos.json"), "w", encoding="utf-8"),
          ensure_ascii=False, indent=1)
