"""[navigation 2026-09-28] 이름으로 prim 을 찾아 world 위치와 크기를 출력한다.

카메라를 놓을 자리를 정할 때 쓴다. 눈대중 대신 좌표로 정하기 위한 도구다.
"""

import argparse

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom  # noqa: E402

DEFAULT_SCENE = (
    "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/"
    "Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd"
)


def main() -> None:
    """키워드가 들어간 prim 의 world 경계 상자를 출력한다."""

    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default=DEFAULT_SCENE)
    parser.add_argument("--keywords", nargs="+", required=True)
    parser.add_argument("--depth", type=int, default=5, help="경로 깊이 제한")
    args = parser.parse_args()

    stage = Usd.Stage.Open(args.scene)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])

    seen = set()
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.count("/") > args.depth:
            continue
        low = path.lower()
        if not any(k.lower() in low for k in args.keywords):
            continue
        parent = path.rsplit("/", 1)[0]
        if parent in seen:
            continue
        seen.add(path)
        try:
            box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if box.IsEmpty():
                continue
            mn, mx = box.GetMin(), box.GetMax()
            print(f"### {path}", flush=True)
            print(f"###     x {mn[0]:7.2f}~{mx[0]:7.2f}  y {mn[1]:7.2f}~{mx[1]:7.2f}  "
                  f"z {mn[2]:6.2f}~{mx[2]:6.2f}", flush=True)
        except Exception:
            continue
    print("### end", flush=True)
    app.close()


if __name__ == "__main__":
    main()
