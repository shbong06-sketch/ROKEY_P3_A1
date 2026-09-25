"""바닥 선(EnvDressing/Merged 의 yellow_*, tape_y_*, tape_w_*) 메쉬 점을 월드 좌표로 뽑아 json + 그림으로 저장.
wsl/lane_planner.py 의 차선 좌표(중앙 x -0.42, y -1.54)는 이 결과(floor_lines_zoom.png)에서 읽었다. 씬이 바뀌면 다시 돌린다.

  <Isaac>\\python.bat floor_lines.py SCENE.usd OUT_DIR [MAP.png]
  -> OUT_DIR/floor_lines.json, floor_lines_zoom.png (x -3.2~0.6, y -3.6~3.0, 1 m 격자), [floor_lines_on_map.png]
  MAP.png 는 Nav2 지도 이미지 (smart_farm_navigation/maps/Collected_smartfarm_v011.png, origin (-4.525, -10.025), 0.05 m/px)
"""
import json
import sys
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})      # Isaac 의 USD 로 씬(바이너리 .usd)을 읽는다
from pxr import Usd, UsdGeom  # noqa: E402

scene, out = sys.argv[1], Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=True)
st = Usd.Stage.Open(scene)
xc = UsdGeom.XformCache(0)
res = {}
for p in st.TraverseAll():
    n = p.GetName()
    if p.IsA(UsdGeom.Mesh) and "/EnvDressing/Merged/" in str(p.GetPath()) and n.split("_")[0] in ("yellow", "tape"):
        m = xc.GetLocalToWorldTransform(p)
        pts = UsdGeom.Mesh(p).GetPointsAttr().Get() or []
        w = [m.Transform(v) for v in pts]
        mesh = UsdGeom.Mesh(p)
        res[n] = {"z": [round(min(v[2] for v in w), 3), round(max(v[2] for v in w), 3)],
                  "pts": [[round(v[0], 3), round(v[1], 3)] for v in w],
                  "counts": list(mesh.GetFaceVertexCountsAttr().Get() or []),
                  "idx": list(mesh.GetFaceVertexIndicesAttr().Get() or [])}
(out / "floor_lines.json").write_text(json.dumps(res), encoding="utf-8")
print({k: (len(v["pts"]), v["z"]) for k, v in res.items()})

if True:   # 확대 그림: x -3.2~0.6, y -3.6~3.0, 1 m = 120 px, 선 면을 채워 그린다
    from PIL import Image, ImageDraw

    X0, X1, Y0, Y1, K = -3.2, 0.6, -3.6, 3.0, 120
    im2 = Image.new("RGB", (int((X1 - X0) * K), int((Y1 - Y0) * K)), (70, 70, 70))
    d2 = ImageDraw.Draw(im2)

    def q(x, y):
        return ((x - X0) * K, (Y1 - y) * K)

    for gx in range(-3, 1):
        d2.line([q(gx, Y0), q(gx, Y1)], fill=(110, 110, 110))
    for gy in range(-3, 3):
        d2.line([q(X0, gy), q(X1, gy)], fill=(110, 110, 110))
    for k, v in res.items():
        col = (240, 200, 0) if k.startswith(("yellow", "tape_y")) else (255, 255, 255)
        i = 0
        for c in v["counts"]:
            poly = [q(*v["pts"][j]) for j in v["idx"][i:i + c]]
            i += c
            if len(poly) >= 3:
                d2.polygon(poly, fill=col)
    for name, (x, y) in {"RACK_DOCK": (-0.421, 1.006), "CORRIDOR_EXIT": (-0.421, -1.2), "FEEDER_APPROACH": (-2.19, -1.55),
                         "FEEDER_DOCK": (-2.19, -2.75)}.items():
        a, b = q(x, y)
        d2.ellipse((a - 7, b - 7, a + 7, b + 7), outline=(255, 60, 60), width=3)
        d2.text((a + 9, b - 7), name, fill=(255, 90, 90))
    im2.save(out / "floor_lines_zoom.png")

if len(sys.argv) > 3:
    from PIL import Image, ImageDraw

    im = Image.open(sys.argv[3]).convert("RGB")
    ox, oy, r = -4.525, -10.025, 0.05
    S = 4
    im = im.resize((im.width * S, im.height * S), Image.NEAREST)
    d = ImageDraw.Draw(im)
    H = im.height

    def px(x, y):
        return ((x - ox) / r * S, H - (y - oy) / r * S)

    for k, v in res.items():
        col = (230, 190, 0) if k.startswith(("yellow", "tape_y")) else (120, 120, 255)
        for x, y in v["pts"]:
            a, b = px(x, y)
            d.point((a, b), fill=col)
    for name, (x, y) in {"RACK_DOCK": (-0.421, 1.006), "FEEDER_APPROACH": (-2.19, -1.55), "FEEDER_DOCK": (-2.19, -2.75)}.items():
        a, b = px(x, y)
        d.ellipse((a - 5, b - 5, a + 5, b + 5), outline=(255, 0, 0), width=2)
        d.text((a + 6, b - 6), name, fill=(255, 0, 0))
    im.save(out / "floor_lines_on_map.png")

app.close()
