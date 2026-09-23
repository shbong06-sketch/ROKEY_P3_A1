"""검사 비전 기능 시험용 Isaac Sim 실행기 (단일 파일, 외부 모듈 없음).

best.pt 학습 조건을 재현해 `/rgb` 를 내보낸다. 조건 출처는
`romaine3_v011_yolo11n_handoff.zip` README 와
`scripts/62_v011_tray_inspect_dataset.py`.
1280x720 · 손목 RealSense · 6구 트레이 · 거리 0.50~0.72 m · 앙각 50~88°
· 비전룸 패널 + 그리퍼 링 라이트.

    --keep-pose    씬에 저장된 팔 자세 사용 (IK 자산 불필요)
    (기본)         학습 분포 안에서 IK 로 자세를 찾음
    --sweep        팔은 두고 트레이만 흔들어 ROI 여유 측정
    --rig-camera   팔 대신 트레이 위 수직 카메라 사용

    ~/isaacsim/python.sh runtime/vision_functest.py --headless --keep-pose
    ... --sweep --sweep-offsets="-60,0;60,0"   # 음수 시작이면 = 를 붙인다

로그 접두사: [시작] 씬 · [조건] 설정 · [자세]/[확인] 실측 · [스윕] 자세
변경 · [준비] 발행 시작 · [진행] 살아 있음
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

# ── 경로 ──────────────────────────────────────────────
RUNTIME_DIR = Path(__file__).resolve().parent
PROJECT_DIR = RUNTIME_DIR.parent
DEFAULT_SCENE = (PROJECT_DIR / "scenes" / "Collected_smartfarm_v013"
                 / "Collected_smartfarm_v013.usd")
DEFAULT_URDF = (PROJECT_DIR.parent / "M0609" / "doosan-robot2" / "urdf"
                / "m0609_isaac_sim.urdf")
DEFAULT_DESC = Path.home() / (
    "Downloads/smartfarm_v008_vision/Collected_smartfarm_v008/lula/"
    "m0609_robot_description.yaml")

# ── 씬 prim 경로 ──────────────────────────────────────
PLACED = "/World/SmartFarm/Placed"
ARM_ART = f"{PLACED}/M0609/Asset/root_joint"
ARM_ROOT = f"{PLACED}/M0609/Asset"
BRACKET = f"{PLACED}/M0609/Asset/onrobot_rg2ft/angle_bracket"
COLOR = f"{BRACKET}/realsense_d455/RSD455/Camera_OmniVision_OV9782_Color"
TOOL0 = f"{PLACED}/M0609/Asset/link_6/tool0"
RING = f"{BRACKET}/vision_ring_light"
PANEL = "/World/VisionRoom/Lights/Panel_0"
RENDER_PRODUCT = "/World/SmartFarm/Graph/ROS_Camera/RenderProduct"
RIG_CAMERA = "/World/VisionRig/InspectCam"
TRAYS = [f"{PLACED}/Pallet_Inspect{s}" for s in ("", "_01", "_02", "_03")]
MAIN_TRAY = TRAYS[0]

# ── 데이터셋 스크립트의 실측 상수 ─────────────────────
ROLLER_TOP = 0.769
TRAY_Z = ROLLER_TOP + 0.02592913 + 0.001
LOOK_Z = ROLLER_TOP + 0.10   # 트레이 판이 아니라 포기 높이 근처를 본다
PANEL_INTENSITY, RING_INTENSITY = 9000.0, 32000.0
PHYSICS_DT, SETTLE_STEPS, SWEEP_SETTLE_STEPS = 1.0 / 60.0, 90, 30
ARM_DOF = 6

# IK 후보. 팔이 못 닿는 자세를 피해 학습 분포 안에서 훑는다.
IK_ELEVATIONS = (70.0, 65.0, 60.0, 75.0, 55.0, 80.0, 50.0, 85.0)
IK_DISTANCES = (0.60, 0.55, 0.50, 0.65, 0.72)
IK_AZIMUTHS = (0.0, -20.0, 20.0, -30.0, 30.0)
SEED_Q_DEG = [85.0, 50.0, 30.0, 0.0, 100.9, 0.0]

# 컨베이어 정지 산포를 흉내 내는 기본 트레이 오프셋 (m).
DEFAULT_SWEEP_OFFSETS = [(dx, dy) for dy in (-0.02, 0.0, 0.02)
                         for dx in (-0.03, 0.0, 0.03)]
RIG_FOCAL, RIG_APERTURE = 24.0, 20.955   # 리그 카메라 광학


# ── 명령행 인자 ───────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    g = p.add_argument_group("씬")
    g.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    g.add_argument("--headless", action="store_true")
    g.add_argument("--width", type=int, default=1280)
    g.add_argument("--height", type=int, default=720)

    g = p.add_argument_group("트레이")
    g.add_argument("--tray-x", type=float, default=-0.675)
    g.add_argument("--tray-y", type=float, default=-6.70)
    g.add_argument("--tray-yaw", type=float, default=None,
                   help="yaw 를 이 각도로 덮어쓴다 (도). 180도 시험용.")

    g = p.add_argument_group("팔 자세")
    g.add_argument("--keep-pose", action="store_true",
                   help="IK 를 건너뛰고 씬에 저장된 자세를 쓴다.")
    g.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    g.add_argument("--desc", type=Path, default=DEFAULT_DESC)
    g.add_argument("--dist", type=float, default=0.60)
    g.add_argument("--elev", type=float, default=70.0)
    g.add_argument("--azim-offset", type=float, default=0.0)
    g.add_argument("--roll", type=float, default=0.0)

    g = p.add_argument_group("리그 카메라 / 스윕")
    g.add_argument("--rig-camera", action="store_true",
                   help="트레이 위 수직 카메라로 RenderProduct 를 돌린다.")
    g.add_argument("--rig-height", type=float, default=0.70)
    g.add_argument("--sweep", action="store_true")
    g.add_argument("--sweep-hold", type=float, default=6.0)
    g.add_argument("--sweep-offsets", default=None,
                   help="mm 단위 'dx,dy;dx,dy;...'")
    return p.parse_known_args()


# ── SimulationApp 부트스트랩 ──────────────────────────
# pxr·omni·isaacsim 을 쓰는 어떤 import 보다 먼저 SimulationApp 을 만들어야
# 한다. 그래서 이 구간만 모듈 최상위에 있다.
def configure_ros_environment(marker="FUNCTEST_REEXEC"):
    """Isaac 번들 ROS 2 를 브리지가 찾게 한다.

    LD_LIBRARY_PATH 는 프로세스가 뜬 뒤 바꿔도 링커가 다시 읽지 않는다.
    경로를 넣고 execv 로 자기를 재실행한다. marker 가 재실행을 한 번으로
    막는다.
    """
    root = Path(os.environ.get("ISAAC_PATH") or os.path.abspath(os.path.join(
        os.path.dirname(os.path.abspath(sys.executable)), "..", "..", "..")))
    lib = root / "exts" / "isaacsim.ros2.bridge" / "jazzy" / "lib"
    if not lib.is_dir():
        raise RuntimeError(f"ROS 2 번들을 찾지 못했습니다: {lib}")
    paths = [p for p in os.environ.get("LD_LIBRARY_PATH", "").split(":") if p]
    if str(lib) in paths or os.environ.get(marker) == "1":
        return
    os.environ["LD_LIBRARY_PATH"] = ":".join([*paths, str(lib)])
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    os.environ[marker] = "1"
    print(f"[ROS2] LD_LIBRARY_PATH 에 {lib} 추가 후 재실행", flush=True)
    os.execv(sys.executable, [sys.executable, *sys.argv])


args, kit_args = parse_args()
configure_ros_environment()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "extra_args": kit_args})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, UsdGeom  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.robot_motion.motion_generation import (  # noqa: E402
    LulaKinematicsSolver)


# ── 공통 도우미 ───────────────────────────────────────
def v3(vec, digits=3):
    """벡터 세 성분을 '(+0.000, +0.000, +0.000)' 로."""
    return "({:+.{d}f}, {:+.{d}f}, {:+.{d}f})".format(*vec[:3], d=digits)


def require_prim(stage, path, label):
    """없으면 바로 실패시킨다. 나중에 None 으로 터지는 것보다 낫다."""
    prim = stage.GetPrimAtPath(path)
    if not prim:
        raise RuntimeError(f"{label} 을(를) 찾지 못했습니다: {path}")
    return prim


def translate_op(prim):
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            return op
    raise RuntimeError(f"{prim.GetPath()} 에 translate op 이 없습니다.")


def world_xform(stage, path):
    return UsdGeom.Xformable(
        stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)


# ── 씬 설정 ───────────────────────────────────────────
def open_scene(path):
    """ROS 2 브리지를 켜고 씬을 연다. 로딩이 끝날 때까지 기다린다."""
    enable_extension("isaacsim.ros2.bridge")
    app.update()
    omni.usd.get_context().open_stage(str(path))
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        app.update()
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError(f"씬을 열지 못했습니다: {path}")
    print(f"[시작] {path}", flush=True)
    return stage


def set_render_product(stage, width, height):
    prim = require_prim(stage, RENDER_PRODUCT, "RenderProduct")
    before = (prim.GetAttribute("inputs:width").Get(),
              prim.GetAttribute("inputs:height").Get())
    prim.GetAttribute("inputs:width").Set(width)
    prim.GetAttribute("inputs:height").Set(height)
    print(f"[조건] RenderProduct {before} -> ({width}, {height})", flush=True)


def set_tray_yaw(prim, yaw_deg):
    """트레이 yaw 를 덮어쓴다. 180도 뒤집힘을 재현할 때 쓴다."""
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        name = op.GetOpName()
        # rotateXYZ / rotateZYX 모두 벡터는 (X, Y, Z) 각도다. 회전 순서만
        # 다르다. 트레이는 Z 만 쓰므로 세 번째 성분을 바꾸면 된다.
        if name in ("xformOp:rotateXYZ", "xformOp:rotateZYX"):
            before = op.Get()
            op.Set(Gf.Vec3f(before[0], before[1], float(yaw_deg)))
            print(f"[조건] 트레이 yaw {before[2]:+.1f}° -> {yaw_deg:+.1f}° "
                  f"({name.split(':')[-1]})", flush=True)
            return
        if name == "xformOp:orient":
            op.Set(Gf.Quatf(
                Gf.Rotation(Gf.Vec3d(0, 0, 1), float(yaw_deg)).GetQuat()))
            print(f"[조건] 트레이 yaw -> {yaw_deg:+.1f}° (orient)", flush=True)
            return
    raise RuntimeError("트레이에 회전 op 이 없습니다.")


def place_trays(stage, tray_x, tray_y, tray_yaw=None):
    """검사 트레이 한 장만 벨트 검사 위치에 두고 나머지는 치운다.

    데이터셋 스크립트도 주차 트레이 네 장을 숨기고 검사 대상만 올린다.
    """
    main = require_prim(stage, MAIN_TRAY, "검사 트레이")
    before = world_xform(stage, MAIN_TRAY).ExtractTranslation()
    translate_op(main).Set(Gf.Vec3d(tray_x, tray_y, TRAY_Z))
    if tray_yaw is not None:
        set_tray_yaw(main, tray_yaw)
    print(f"[조건] 검사 트레이 {v3(before)} -> "
          f"{v3((tray_x, tray_y, TRAY_Z))}", flush=True)

    for path in TRAYS[1:]:
        prim = stage.GetPrimAtPath(path)
        if not prim:
            continue
        translate_op(prim).Set(Gf.Vec3d(0.0, 0.0, -20.0))
        UsdGeom.Imageable(prim).MakeInvisible()
    print(f"[조건] 대기 트레이 {len(TRAYS) - 1}장 숨김", flush=True)


def set_lights(stage):
    """비전룸 천장 패널과 그리퍼 링 라이트를 학습 기준값으로."""
    for path, value in ((PANEL, PANEL_INTENSITY), (RING, RING_INTENSITY)):
        prim = stage.GetPrimAtPath(path)
        attr = prim.GetAttribute("inputs:intensity") if prim else None
        if not attr:
            print(f"[조건] 조명 없음: {path}", flush=True)
            continue
        attr.Set(float(value))
        print(f"[조건] {path.rsplit('/', 1)[-1]} intensity={value:.0f}",
              flush=True)


def make_rig_camera(stage, tray_x, tray_y, height):
    """트레이 위 수직 카메라를 만들고 RenderProduct 를 돌린다.

    물리 객체는 건드리지 않는다. 트레이를 로봇 쪽으로 옮기면 그리퍼와
    겹쳐 물리가 터진다. 카메라만 새로 두는 쪽이 안전하다.
    """
    UsdGeom.Xform.Define(stage, RIG_CAMERA.rsplit("/", 1)[0])
    cam = UsdGeom.Camera.Define(stage, RIG_CAMERA)
    # 회전 없음 = 로컬 -Z 가 월드 -Z. 곧바로 아래를 내려다본다.
    cam.AddTranslateOp().Set(Gf.Vec3d(tray_x, tray_y, TRAY_Z + height))
    cam.CreateFocalLengthAttr(RIG_FOCAL)
    cam.CreateHorizontalApertureAttr(RIG_APERTURE)
    cam.CreateVerticalApertureAttr(RIG_APERTURE)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))

    rel = require_prim(stage, RENDER_PRODUCT,
                       "RenderProduct").GetRelationship("inputs:cameraPrim")
    before = [str(x) for x in rel.GetTargets()]
    rel.SetTargets([Sdf.Path(RIG_CAMERA)])

    fov = 2 * math.degrees(math.atan(RIG_APERTURE / (2 * RIG_FOCAL)))
    span = 2 * height * math.tan(math.radians(fov / 2))
    print(f"[조건] 리그 카메라 z={TRAY_Z + height:.3f} · FOV {fov:.1f}° · "
          f"화면 폭 {span * 1000:.0f} mm", flush=True)
    print(f"[조건] RenderProduct 카메라 교체: {before} -> [{RIG_CAMERA}]",
          flush=True)


# ── 자세 보고 ─────────────────────────────────────────
def rpy_deg(quat):
    """USD quaternion -> roll·pitch·yaw (도)."""
    w = quat.GetReal()
    x, y, z = quat.GetImaginary()
    return (math.degrees(math.atan2(2 * (w * x + y * z),
                                    1 - 2 * (x * x + y * y))),
            math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))),
            math.degrees(math.atan2(2 * (w * z + x * y),
                                    1 - 2 * (y * y + z * z))))


def report_pose(stage, arm, camera_path, target_x, target_y):
    """팔과 카메라 자세를 찍는다.

    `/rgb` 한 장이 어떤 자세에서 나왔는지 나중에 재현하려면 이 값이
    필요하다. 관절 각도만 있으면 씬에 그대로 되박을 수 있다.
    """
    print("[자세] ── M0609 + RealSense ──────────────", flush=True)
    names = list(arm.dof_names)
    pos = np.array(arm.get_joint_positions(), dtype=float)
    deg = [math.degrees(pos[names.index(f"joint_{i}")])
           for i in range(1, ARM_DOF + 1)]
    print("[자세] 관절(도)  " + "  ".join(
        f"j{i + 1}={v:+8.3f}" for i, v in enumerate(deg)), flush=True)
    print(f"[자세] 관절(rad) {[round(math.radians(v), 6) for v in deg]}",
          flush=True)

    for label, path in (("베이스", ARM_ROOT), ("tool0", TOOL0),
                        ("카메라", camera_path)):
        if not stage.GetPrimAtPath(path):
            print(f"[자세] {label} 없음: {path}", flush=True)
            continue
        xf = world_xform(stage, path)
        q = xf.ExtractRotationQuat()
        print(f"[자세] {label:<6s} pos={v3(xf.ExtractTranslation(), 4)} m  "
              f"quat(wxyz)=({q.GetReal():+.5f}, "
              f"{q.GetImaginary()[0]:+.5f}, {q.GetImaginary()[1]:+.5f}, "
              f"{q.GetImaginary()[2]:+.5f})  rpy={v3(rpy_deg(q), 2)}",
              flush=True)

    xf = world_xform(stage, camera_path)
    rot = xf.ExtractRotationMatrix()
    eye = xf.ExtractTranslation()
    # USD 카메라 로컬축: +X 오른쪽, +Y 위, -Z 시선.
    right = Gf.Vec3d(*rot[0]).GetNormalized()
    up = Gf.Vec3d(*rot[1]).GetNormalized()
    fwd = Gf.Vec3d(-rot[2][0], -rot[2][1], -rot[2][2]).GetNormalized()
    print(f"[자세] 카메라축 forward={v3(fwd, 4)} up={v3(up, 4)} "
          f"right={v3(right, 4)}", flush=True)

    cache = UsdGeom.XformCache()
    mc = Gf.Matrix4d(cache.GetLocalToWorldTransform(
        stage.GetPrimAtPath(COLOR))).RemoveScaleShear()
    mt = Gf.Matrix4d(cache.GetLocalToWorldTransform(
        stage.GetPrimAtPath(TOOL0))).RemoveScaleShear()
    print(f"[자세] tool0 기준 카메라 오프셋 "
          f"{v3((mc * mt.GetInverse()).ExtractTranslation(), 4)} m",
          flush=True)

    cam = UsdGeom.Camera(stage.GetPrimAtPath(camera_path))
    focal = cam.GetFocalLengthAttr().Get()
    ah, av = (cam.GetHorizontalApertureAttr().Get(),
              cam.GetVerticalApertureAttr().Get())
    product = stage.GetPrimAtPath(RENDER_PRODUCT)
    print(f"[자세] 광학 focal={focal:.3f} aperture={ah:.3f}x{av:.3f} -> FOV "
          f"{2 * math.degrees(math.atan(ah / (2 * focal))):.1f}° x "
          f"{2 * math.degrees(math.atan(av / (2 * focal))):.1f}°", flush=True)
    print(f"[자세] 렌더 {product.GetAttribute('inputs:width').Get()}x"
          f"{product.GetAttribute('inputs:height').Get()} · "
          f"prim {camera_path}", flush=True)

    to_target = Gf.Vec3d(target_x - eye[0], target_y - eye[1], LOOK_Z - eye[2])
    angle = math.degrees(math.acos(
        max(-1.0, min(1.0, to_target.GetNormalized() * fwd))))
    print(f"[확인] 카메라 {v3(eye)} 시선 {v3(fwd)}", flush=True)
    print(f"[확인] 검사 지점까지 거리 {to_target.GetLength():.3f} m · "
          f"시선각 {angle:.1f}°", flush=True)


# ── 팔 자세 (IK) ──────────────────────────────────────
def pose_arm_with_ik(stage, arm):
    """IK 로 손목 카메라를 검사 지점에 겨냥시킨다.

    먼저 요청받은 조합을 쓰고, 안 풀리면 학습 분포 전체를 훑는다.
    데이터셋 스크립트도 한 장면마다 최대 8회 다시 뽑는다.
    """
    if not args.desc.is_file() or not args.urdf.is_file():
        raise RuntimeError(f"IK 자산 없음: {args.desc} / {args.urdf}")
    ik = LulaKinematicsSolver(robot_description_path=str(args.desc),
                              urdf_path=str(args.urdf))
    base = world_xform(stage, ARM_ROOT)
    bt, bq = base.ExtractTranslation(), base.ExtractRotationQuat()
    ik.set_robot_base_pose(np.array([bt[0], bt[1], bt[2]]),
                           np.array([bq.GetReal(), *bq.GetImaginary()]))

    cache = UsdGeom.XformCache()
    mc = Gf.Matrix4d(cache.GetLocalToWorldTransform(
        stage.GetPrimAtPath(COLOR))).RemoveScaleShear()
    mt = Gf.Matrix4d(cache.GetLocalToWorldTransform(
        stage.GetPrimAtPath(TOOL0))).RemoveScaleShear()
    tool_in_cam = (mc * mt.GetInverse()).GetInverse()

    look = np.array([args.tray_x, args.tray_y, LOOK_Z])
    to_arm = math.atan2(bt[1] - look[1], bt[0] - look[0])
    seed = np.deg2rad(SEED_Q_DEG)

    candidates = [(args.dist, args.elev, args.azim_offset)]
    candidates += [(d, e, a) for e in IK_ELEVATIONS
                   for d in IK_DISTANCES for a in IK_AZIMUTHS]

    for dist, elev, azim in candidates:
        az, el = to_arm + math.radians(azim), math.radians(elev)
        eye = look + dist * np.array([math.cos(el) * math.cos(az),
                                      math.cos(el) * math.sin(az),
                                      math.sin(el)])
        cam = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*look),
                                      Gf.Vec3d(0, 0, 1)).GetInverse()
        cam = Gf.Matrix4d().SetRotate(
            Gf.Rotation(Gf.Vec3d(0, 0, 1), args.roll)) * cam
        tool = tool_in_cam * cam
        q, t = tool.ExtractRotationQuat(), tool.ExtractTranslation()
        sol, ok = ik.compute_inverse_kinematics(
            frame_name="tool0",
            target_position=np.array([t[0], t[1], t[2]]),
            target_orientation=np.array([q.GetReal(), *q.GetImaginary()]),
            warm_start=seed)
        if not ok:
            continue

        joints = np.asarray(getattr(sol, "joint_positions", sol), dtype=float)
        current = np.array(arm.get_joint_positions(), dtype=float)
        names = list(arm.dof_names)
        for i in range(ARM_DOF):      # 그리퍼 관절은 건드리지 않는다
            current[names.index(f"joint_{i + 1}")] = joints[i]
        arm.set_joint_positions(current)

        print(f"[조건] IK 해: 거리 {dist:.2f} m · 앙각 {elev:.0f}° · "
              f"az {azim:+.0f}° · roll {args.roll:.0f}° "
              f"(학습: 0.50~0.72 m, 50~88°, ±25°)", flush=True)
        print(f"[조건] 카메라 eye={v3(eye)} look={v3(look)}", flush=True)
        print(f"[조건] 관절(도) {np.rad2deg(joints).round(1).tolist()}",
              flush=True)
        return

    raise RuntimeError(f"IK 실패: {len(candidates)}개 조합 모두 안 풀림. "
                       f"look={look.round(3)} 트레이 위치를 옮겨 보세요.")


# ── 실행 모드 ─────────────────────────────────────────
def parse_sweep_offsets(text):
    """'dx,dy;dx,dy' (mm) 를 (dx, dy) 미터 목록으로."""
    if not text:
        return DEFAULT_SWEEP_OFFSETS
    pairs = [tuple(float(v) / 1000.0 for v in item.split(","))
             for item in text.split(";") if item.strip()]
    if not pairs:
        raise RuntimeError(f"오프셋을 해석하지 못했습니다: {text}")
    return pairs


def run_sweep(stage, world):
    """팔은 그대로 두고 트레이만 옮겨 가며 잡을 시간을 준다.

    실제로도 팔은 고정 검사 자세로 가고 트레이가 컨베이어를 타고 와서
    선다. 흔들리는 쪽은 트레이다. ROI 여유도 그 기준으로 재야 한다.
    """
    op = translate_op(stage.GetPrimAtPath(MAIN_TRAY))
    hold = max(1, int(args.sweep_hold / PHYSICS_DT))
    offsets = parse_sweep_offsets(args.sweep_offsets)

    for index, (dx, dy) in enumerate(offsets, start=1):
        x, y = args.tray_x + dx, args.tray_y + dy
        op.Set(Gf.Vec3d(x, y, TRAY_Z))
        for _ in range(SWEEP_SETTLE_STEPS):
            world.step(render=True)
        print(f"[스윕] {index}/{len(offsets)} dx={dx:+.3f} dy={dy:+.3f} "
              f"-> ({x:+.3f}, {y:+.3f})", flush=True)
        for _ in range(hold):
            world.step(render=True)
            if not app.is_running():
                return
    print("[스윕] 완료", flush=True)


def main():
    stage = open_scene(args.scene.resolve())

    # 물리 객체 배치는 world.reset() 전에 끝낸다. 재생 뒤 xform 을 바꾸면
    # 물리가 그 자리를 모른다.
    set_render_product(stage, args.width, args.height)
    place_trays(stage, args.tray_x, args.tray_y, args.tray_yaw)
    set_lights(stage)
    if args.rig_camera:
        make_rig_camera(stage, args.tray_x, args.tray_y, args.rig_height)

    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT,
                  rendering_dt=PHYSICS_DT)
    world.reset()
    arm = SingleArticulation(prim_path=ARM_ART, name="m0609")
    arm.initialize()

    if args.rig_camera:
        print("[조건] 리그 카메라 모드 — 팔을 건드리지 않는다.", flush=True)
    elif args.keep_pose:
        print("[조건] IK 건너뜀 — 씬에 저장된 팔 자세를 그대로 쓴다.",
              flush=True)
    else:
        pose_arm_with_ik(stage, arm)

    world.play()
    for _ in range(SETTLE_STEPS):
        world.step(render=True)

    report_pose(stage, arm, RIG_CAMERA if args.rig_camera else COLOR,
                args.tray_x, args.tray_y)
    print(f"[준비] 재생 중. ROS_DOMAIN_ID="
          f"{os.environ.get('ROS_DOMAIN_ID', '0')} 에서 "
          f"/rgb {args.width}x{args.height} 발행", flush=True)

    if args.sweep:
        run_sweep(stage, world)
        return
    steps = 0
    while app.is_running():
        world.step(render=True)
        steps += 1
        if steps % 600 == 0:
            print(f"[진행] {steps} 스텝", flush=True)


try:
    main()
except KeyboardInterrupt:
    pass
except Exception:
    # app.close() 가 트레이스백보다 먼저 프로세스를 끝내므로 먼저 찍는다.
    import traceback
    traceback.print_exc()
    sys.stdout.flush()
    sys.stderr.flush()
finally:
    app.close()
