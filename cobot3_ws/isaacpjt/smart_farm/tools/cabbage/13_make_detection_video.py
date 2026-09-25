"""Vision detection video: every wrist-RealSense capture of the station window run through best.pt, drawn like an
rqt_image_view window on /inspection/annotated (boxes + class + confidence, live over the whole inspect -> cull).

  (ultralytics python) 13_make_detection_video.py RUN_DIR [WEIGHTS] [FPS]
RUN_DIR = run_allinone_live.ps1 -OutDir with captures/cap_cam3_inspection_*.jpg and station/station_events.json
out: RUN_DIR/videos/06_vision_detection_boxes.mp4
"""
import glob, json, os, re, sys
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO

RUN = sys.argv[1]
WEIGHTS = sys.argv[2] if len(sys.argv) > 2 else r"C:\Users\kangm\Downloads\romaine3_v012_640sq_yolo11n_best.pt"
FPS = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
W, H = 960, 760                                      # window: title 36 + toolbar 44 + image 640 + status 40
COLORS = {"lettuce_dark_green": (60, 170, 60), "lettuce_yellow": (0, 200, 230), "lettuce_brown": (40, 80, 150)}
SHORT = {"lettuce_dark_green": "green (정상)", "lettuce_yellow": "yellow (제거)", "lettuce_brown": "brown (제거)"}
F = lambda s, b=False: ImageFont.truetype(r"C:\Windows\Fonts\malgunbd.ttf" if b else r"C:\Windows\Fonts\malgun.ttf", s)

events = json.load(open(os.path.join(RUN, "station", "station_events.json"), encoding="utf-8"))
ev = {}
for e in events:
    if e["pallet"] == "Pallet_01":
        ev.setdefault(e["event"], e["t"])
t0, t1 = ev["PUSH_DONE_INSPECT_MOVE"], ev.get("PUSH_BACK_DONE_RELEASED", 1e9)
frames = []
for p in glob.glob(os.path.join(RUN, "captures", "cap_cam3_inspection_*.jpg")):
    t = float(re.search(r"_(\d+\.\d)\.jpg$", p).group(1))
    if t0 - 0.5 <= t <= t1:
        frames.append((t, p))
frames.sort()
model = YOLO(WEIGHTS)
os.makedirs(os.path.join(RUN, "videos"), exist_ok=True)
out = os.path.join(RUN, "videos", "06_vision_detection_boxes.mp4")
vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
step_of = lambda t: ("검사 (INSPECT)" if t < ev.get("INSPECTION_DONE", 0) + 0.5 else
                     "솎아내기 (CULL)" if t < ev.get("RECHECK_DONE", 1e9) - 5 else "재검사 (RECHECK)")
for t, p in frames:
    bgr = cv2.imread(p)
    r = model.predict(bgr, imgsz=640, conf=0.35, verbose=False)[0]
    img = Image.fromarray(cv2.cvtColor(cv2.resize(bgr, (640, 640)), cv2.COLOR_BGR2RGB))
    d = ImageDraw.Draw(img)
    sx, sy = 640 / bgr.shape[1], 640 / bgr.shape[0]
    counts = {}
    for (x0, y0, x1, y1), c, s in zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist(), r.boxes.conf.tolist()):
        name = model.names[int(c)]
        counts[name] = counts.get(name, 0) + 1
        col = COLORS.get(name, (255, 255, 255))[::-1]          # BGR tuple -> RGB
        box = [x0 * sx, y0 * sy, x1 * sx, y1 * sy]
        d.rectangle(box, outline=col, width=3)
        label = f"{name.replace('lettuce_', '')} {s:.2f}"
        tw = d.textlength(label, font=F(15, True))
        d.rectangle([box[0], box[1] - 21, box[0] + tw + 8, box[1]], fill=col)
        d.text((box[0] + 4, box[1] - 20), label, font=F(15, True), fill=(0, 0, 0) if "yellow" in name else (255, 255, 255))
    win = Image.new("RGB", (W, H), (236, 236, 236))
    wd = ImageDraw.Draw(win)
    wd.rectangle([0, 0, W, 36], fill=(48, 50, 56))
    wd.text((12, 6), "rqt_image_view  —  /inspection/annotated", font=F(18, True), fill=(235, 235, 235))
    wd.rectangle([10, 44, 520, 72], outline=(150, 150, 150), fill=(255, 255, 255))
    wd.text((16, 46), "/inspection/annotated  (sensor_msgs/Image, best.pt)", font=F(16), fill=(30, 30, 30))
    wd.text((540, 46), f"sim t = {t:6.1f} s   ·   {step_of(t)}", font=F(16, True), fill=(30, 30, 30))
    win.paste(img, (10, 80))
    # side panel: counts and legend
    wd.rectangle([660, 80, W - 10, 720], fill=(250, 250, 250), outline=(200, 200, 200))
    wd.text((676, 92), "검출 수", font=F(20, True), fill=(20, 20, 20))
    y = 130
    for name in ("lettuce_dark_green", "lettuce_yellow", "lettuce_brown"):
        col = COLORS[name][::-1]
        wd.rectangle([676, y + 4, 696, y + 24], fill=col)
        wd.text((706, y), f"{SHORT[name]} : {counts.get(name, 0)}", font=F(18), fill=(20, 20, 20))
        y += 40
    wd.text((676, 270), "판정 규칙", font=F(20, True), fill=(20, 20, 20))
    wd.text((676, 305), "green  = 정상 (남김)\nyellow = 불량 (제거)\nbrown  = 불량 (제거)\n\nconf ≥ 0.35 · 3프레임 다수결\n칸 배정: 카메라 모델로\n포기 투영 후 가장 가까운 칸",
            font=F(16), fill=(40, 40, 40))
    wd.rectangle([0, 724, W, H], fill=(225, 225, 225))
    wd.text((12, 732), f"{os.path.basename(WEIGHTS)}   ·   640x640   ·   wrist RealSense D455 (color)", font=F(15), fill=(60, 60, 60))
    vw.write(cv2.cvtColor(np.asarray(win), cv2.COLOR_RGB2BGR))
vw.release()
print(out, len(frames), "frames")
