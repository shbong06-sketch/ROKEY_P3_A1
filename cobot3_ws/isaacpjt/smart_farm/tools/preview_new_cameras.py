"""[navigation 2026-09-28] 새 카메라 자리를 시험해 본다.

위치와 바라볼 지점을 주면 임시 카메라를 만들어 한 장씩 렌더한다.
씬 파일은 고치지 않는다(메모리 안의 stage 에만 만든다).

사용:
  export DISPLAY=:96
  python.sh preview_new_cameras.py --out <폴더> \
      --cam "이름,px,py,pz,tx,ty,tz" --cam "..."
"""

import argparse
import math
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": False, "width": 1920, "height": 1080})

import omni.usd  # noqa: E402
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

DEFAULT_SCENE = (
    "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/"
    "Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd"
)


def look_at_rotation(eye, target):
    """카메라가 target 을 보도록 하는 (yaw, pitch) 를 도 단위로 준다.

    USD 카메라는 기본으로 -z 를 본다. z 축 회전(yaw)과 x 축 회전(pitch)만 쓰면
    평면 위의 롱샷에는 충분하다.
    """
    dx, dy, dz = (target[i] - eye[i] for i in range(3))
    yaw = math.degrees(math.atan2(dy, dx)) - 90.0
    pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
    return yaw, pitch


def main() -> None:
    """지정한 카메라들을 만들고 한 장씩 캡처한다."""

    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--scene", default=DEFAULT_SCENE)
    parser.add_argument("--focal", type=float, default=16.0)
    parser.add_argument("--settle", type=int, default=60)
    parser.add_argument("--cam", action="append", default=[],
                        help="이름,px,py,pz,tx,ty,tz (월드 고정 카메라)")
    parser.add_argument("--povcam", action="append", default=[],
                        help="이름,부모prim,ox,oy,oz,rx,ry,rz (움직이는 prim 에 붙이는 시점뷰)")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    omni.usd.get_context().open_stage(args.scene)
    for _ in range(120):
        app.update()

    stage = omni.usd.get_context().get_stage()
    viewport = get_active_viewport()

    for spec in args.povcam:
        parts = spec.split(",")
        name = parts[0].strip()
        parent = parts[1].strip()
        offset = [float(v) for v in parts[2:5]]
        rotation = [float(v) for v in parts[5:8]]
        if not stage.GetPrimAtPath(parent).IsValid():
            print(f"### SKIP {name}: 부모 prim 없음 {parent}", flush=True)
            continue
        path = f"{parent}/{name}"
        camera = UsdGeom.Camera.Define(stage, path)
        camera.GetFocalLengthAttr().Set(args.focal)
        xform = UsdGeom.Xformable(camera.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*offset))
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
        viewport.camera_path = path
        for _ in range(args.settle):
            app.update()
        capture_viewport_to_file(viewport, os.path.join(args.out, f"new_{name}.png"))
        for _ in range(20):
            app.update()
        print(f"### SAVED {name} parent={parent} offset={offset} rot={rotation}", flush=True)

    for spec in args.cam:
        parts = spec.split(",")
        name = parts[0].strip()
        eye = [float(v) for v in parts[1:4]]
        target = [float(v) for v in parts[4:7]]

        path = f"/World/PreviewCameras/{name}"
        camera = UsdGeom.Camera.Define(stage, path)
        camera.GetFocalLengthAttr().Set(args.focal)
        xform = UsdGeom.Xformable(camera.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*eye))
        yaw, pitch = look_at_rotation(eye, target)
        # USD 카메라는 -z 를 보므로 x 축으로 90도 세운 뒤 pitch 를 더한다.
        xform.AddRotateXYZOp().Set(Gf.Vec3f(90.0 + pitch, 0.0, yaw))

        viewport.camera_path = path
        for _ in range(args.settle):
            app.update()
        out = os.path.join(args.out, f"new_{name}.png")
        capture_viewport_to_file(viewport, out)
        for _ in range(20):
            app.update()
        print(f"### SAVED {name} eye={eye} target={target} yaw={yaw:.1f} pitch={pitch:.1f}", flush=True)

    print("### end", flush=True)
    app.close()


if __name__ == "__main__":
    main()
