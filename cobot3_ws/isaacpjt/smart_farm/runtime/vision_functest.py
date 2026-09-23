"""검사 비전 기능 시험용 Isaac Sim 실행기.

best.pt 학습 조건을 재현해 `/rgb` 를 내보낸다. 조건 출처는
`romaine3_v011_yolo11n_handoff.zip` README 와
`scripts/62_v011_tray_inspect_dataset.py`.
1280x720 · 손목 RealSense · 6구 트레이 · 거리 0.50~0.72 m · 앙각 50~88°
· 비전룸 패널 + 그리퍼 링 라이트.

    --keep-pose    씬에 저장된 팔 자세 사용 (IK 자산 불필요)
    (기본)         학습 분포 안에서 IK 로 자세를 찾음
    --sweep        팔은 두고 트레이만 옮겨 ROI 여유 측정 (물리 컨베이어 시험 아님)
    --rig-camera   팔 대신 트레이 위 수직 카메라 사용
    --bridge       /inspection/detections_2d 를 받아 불량(DEFECT) 슬롯만
                   base 좌표로 바꿔 /inspection/targets_3d 로 낸다
                   (같은 폴더 vision_bridge.py + object_detection.yaml 필요)
    --strict       자세가 학습 분포를 벗어나거나 대체 자세로 넘어가면 중단
    --azim-ref     IK 방위각 기준: arm(트레이->로봇, 기본) / scene(씬 카메라 시선)

로봇을 옮겨도 멈추지 않게: 재생 직후 베이스 실제 위치를 재서 씬과 다르면 그
값을 쓰고, IK 해를 적용해 보고 충돌·범위 밖이면 다음 해로, 해가 없으면 씬에
저장된 자세로 대체한다 (모두 [검증] WARN 으로 기록, --strict 면 중단).

네이티브:
    ~/isaacsim/python.sh runtime/vision_functest.py --headless --keep-pose
Docker (호스트에서):
    ./runtime/run_isaac_docker.sh --keep-pose
    ./runtime/run_isaac_docker.sh --keep-pose --bridge
    ... --sweep --sweep-offsets="-60,0;60,0"   # 음수 시작이면 = 를 붙인다

경로는 환경변수로 덮어쓸 수 있다 (Docker 용):
    FUNCTEST_SCENE · M0609_URDF · M0609_LULA_DESC
    컨테이너에서는 --headless 가 강제된다 (X 포워딩을 쓰면 FUNCTEST_ALLOW_GUI=1)

로그 접두사: [시작] 씬 · [조건] 설정 · [점검] 그래프 · [자세]/[확인] 실측 ·
[검증] 학습 분포 판정 · [스윕] 자세 변경 · [브리지] 좌표 변환 ·
[준비] 발행 시작 · [진행] 살아 있음
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

# ── 경로 ──────────────────────────────────────────────
RUNTIME_DIR = Path(__file__).resolve().parent
PROJECT_DIR = RUNTIME_DIR.parent
IN_CONTAINER = (Path("/.dockerenv").exists()
                or os.environ.get("container") in ("docker", "podman"))


def _env_path(name, default):
    value = os.environ.get(name)
    return Path(value).expanduser() if value else default


DEFAULT_SCENE = _env_path("FUNCTEST_SCENE", PROJECT_DIR / "scenes"
                          / "Collected_smartfarm_v013"
                          / "Collected_smartfarm_v013.usd")
DEFAULT_URDF = _env_path("M0609_URDF", PROJECT_DIR.parent / "M0609"
                         / "doosan-robot2" / "urdf" / "m0609_isaac_sim.urdf")
DEFAULT_INSPECTION_CONFIG = _env_path(
    "INSPECTION_CONFIG", PROJECT_DIR.parent.parent / "src"
    / "smart_farm_vision" / "config" / "object_detection.yaml")
DEFAULT_DESC = _env_path("M0609_LULA_DESC", Path.home() / (
    "Downloads/smartfarm_v008_vision/Collected_smartfarm_v008/lula/"
    "m0609_robot_description.yaml"))

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
GRAPH_ROOT = "/World/SmartFarm/Graph"
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

# 학습 분포. dist·elev·azim 은 docstring/IK 로그 기준.
# look_err·roll 허용치는 임시값 — 데이터셋 스크립트 확인 후 바꿀 것.
DATASET_RANGE = {"dist": (0.50, 0.72), "elev": (50.0, 88.0),
                 "azim": (-25.0, 25.0), "look_err": (0.0, 10.0),
                 "roll": (-10.0, 10.0)}
FK_USD_TOL_MM = 5.0          # FK 와 USD 가 이보다 다르면 USD 자세를 믿지 않는다
SETTLE_TOL_DEG = 1.0         # settle 후 팔 관절 오차 경고 기준
BASE_TOL_MM = 10.0           # 물리 루트 링크와 씬(USD) 베이스가 이보다 다르면 물리 값 사용
BASE_PROBE_STEPS = 10        # 베이스 실측 전 물리 스텝
AZIM_REF_WARN_DEG = 10.0     # 방위각 기준 두 가지가 이보다 다르면 경고

# IK 후보. 학습 분포(±25°) 안에서만 훑는다.
IK_ELEVATIONS = (70.0, 65.0, 60.0, 75.0, 55.0, 80.0, 50.0, 85.0)
IK_DISTANCES = (0.60, 0.55, 0.50, 0.65, 0.72)
IK_AZIMUTHS = (0.0, -20.0, 20.0, -25.0, 25.0)
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

    g = p.add_argument_group("ROS 환경")
    g.add_argument("--no-ros-env", action="store_true",
                   help="번들 ROS 2 LD_LIBRARY_PATH 재설정을 건너뛴다 (A/B 시험).")

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
    g.add_argument("--strict", action="store_true",
                   help="[검증] 에 WARN 이 있거나 대체 자세로 넘어가면 중단한다.")
    g.add_argument("--azim-ref", choices=("arm", "scene"), default="arm",
                   help="IK 방위각 0° 기준. arm=트레이->로봇 방향(기본), "
                        "scene=씬에 저장된 카메라가 보던 방향 (로봇을 옮겨도 "
                        "화면 방향 유지)")
    g.add_argument("--max-pose-tries", type=int, default=6,
                   help="IK 해를 실제로 적용해 보고 충돌·범위 밖이면 다음 "
                        "해로 넘어가는 최대 횟수")

    g = p.add_argument_group("리그 카메라 / 스윕")
    g.add_argument("--rig-camera", action="store_true",
                   help="트레이 위 수직 카메라로 RenderProduct 를 돌린다.")
    g.add_argument("--rig-height", type=float, default=0.70)
    g.add_argument("--view-camera", action="store_true",
                   help="GUI 뷰포트를 검사 카메라 화면으로 바꾼다 (GUI 전용).")
    g.add_argument("--sweep", action="store_true")
    g.add_argument("--sweep-hold", type=float, default=6.0)
    g.add_argument("--sweep-offsets", default=None,
                   help="mm 단위 'dx,dy;dx,dy;...'")

    g = p.add_argument_group("브리지 (검출 -> base 좌표)")
    g.add_argument("--bridge", action="store_true")
    g.add_argument("--det-topic", default="/inspection/detections_2d")
    g.add_argument("--depth-topic", default="",
                   help="깊이 영상 토픽 (비우면 수평면 교점만 쓴다).")
    g.add_argument("--target-topic", default="/inspection/targets_3d")
    g.add_argument("--plane-z", type=float, default=LOOK_Z,
                   help="깊이가 없을 때 쓰는 월드 수평면 높이 (m).")
    g.add_argument("--inspection-config", type=Path,
                   default=DEFAULT_INSPECTION_CONFIG,
                   help="Inspection Node 와 같은 object_detection.yaml. "
                        "이 파일로 판정을 재현해 불량(DEFECT) 슬롯 좌표만 낸다.")
    return p.parse_known_args()


# ── SimulationApp 부트스트랩 ──────────────────────────
# pxr·omni·isaacsim 을 쓰는 어떤 import 보다 먼저 SimulationApp 을 만들어야
# 한다. 그래서 이 구간만 모듈 최상위에 있다.
def bundled_ros_dir():
    """Isaac 번들 ROS 2 Jazzy 폴더 (exts/isaacsim.ros2.bridge/jazzy)."""
    root = Path(os.environ.get("ISAAC_PATH") or os.path.abspath(os.path.join(
        os.path.dirname(os.path.abspath(sys.executable)), "..", "..", "..")))
    return root / "exts" / "isaacsim.ros2.bridge" / "jazzy"


def configure_ros_environment(marker="FUNCTEST_REEXEC"):
    """Isaac 번들 ROS 2 를 브리지가 찾게 한다.

    LD_LIBRARY_PATH 는 프로세스가 뜬 뒤 바꿔도 링커가 다시 읽지 않는다.
    번들 lib 를 맨 앞에 두고 시스템 ROS 경로를 걷어낸 뒤 execv 로 자기를
    재실행한다. marker 는 재실행된 자식에서만 1 이며, 확인 즉시 지운다.
    """
    if os.environ.pop(marker, None) == "1":
        return
    lib = str(bundled_ros_dir() / "lib")
    if not Path(lib).is_dir():
        raise RuntimeError(f"ROS 2 번들을 찾지 못했습니다: {lib}")
    old = os.environ.get("LD_LIBRARY_PATH", "")
    rest = [p for p in old.split(":")
            if p and p != lib and "/opt/ros/" not in p]
    new = ":".join([lib, *rest])
    if new == old:
        return
    os.environ["LD_LIBRARY_PATH"] = new
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    os.environ[marker] = "1"
    print(f"[ROS2] LD_LIBRARY_PATH 선두에 {lib} 배치 후 재실행", flush=True)
    sys.stdout.flush()
    os.execv(sys.executable, [sys.executable, *sys.argv])


def strip_system_ros_pythonpath():
    """PYTHONPATH 에서 시스템 ROS 2 경로를 걷어낸다.

    호스트에서 `source /opt/ros/jazzy/setup.bash` 를 한 셸로 Isaac 을 띄우면,
    Isaac 내장 Python(3.11) 이 시스템 ROS 의 python3.12 트리를 먼저 보다가
    `_rclpy_pybind11` 을 못 찾고 rclpy 를 포기한다. 그러면 ROS 2 브리지가
    죽어 `/rgb` 가 한 프레임도 안 나간다. 증상이 조용해서 찾기 어렵다.
    /opt/ros 밖의 python3.12 워크스페이스 경로도 같은 문제를 내므로 경고한다.
    """
    raw = os.environ.get("PYTHONPATH", "")
    if not raw:
        return
    parts = [p for p in raw.split(":") if p]
    kept = [p for p in parts if "/opt/ros/" not in p]
    for p in kept:
        if "python3.12" in p:
            print(f"[ROS2][경고] PYTHONPATH 에 python3.12 경로가 남아 "
                  f"있습니다: {p}", flush=True)
    if len(kept) == len(parts):
        return
    if kept:
        os.environ["PYTHONPATH"] = ":".join(kept)
    else:
        os.environ.pop("PYTHONPATH", None)
    print("[ROS2] PYTHONPATH 에서 시스템 ROS 경로를 제거했습니다 "
          "(Isaac 내장 rclpy 를 쓰기 위함).", flush=True)


args, kit_args = parse_args()
strip_system_ros_pythonpath()
if not args.no_ros_env:
    configure_ros_environment()
if (IN_CONTAINER and not args.headless
        and os.environ.get("FUNCTEST_ALLOW_GUI") != "1"):
    print("[시작] 컨테이너 안에서 GUI 없이 실행합니다 (--headless 강제).",
          flush=True)
    args.headless = True

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "extra_args": kit_args})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.robot_motion.motion_generation import (  # noqa: E402
    LulaKinematicsSolver)


# ── 공통 도우미 ───────────────────────────────────────
def v3(vec, digits=3):
    """벡터 세 성분을 '(+0.000, +0.000, +0.000)' 로."""
    return "({:+.{d}f}, {:+.{d}f}, {:+.{d}f})".format(*vec[:3], d=digits)


def require_prim(stage, path, label, schema=None):
    """없거나 기대한 타입이 아니면 바로 실패시킨다."""
    prim = stage.GetPrimAtPath(path)
    if not prim:
        raise RuntimeError(f"{label} 을(를) 찾지 못했습니다: {path}")
    if schema is not None and not prim.IsA(schema):
        raise RuntimeError(f"{label} 의 타입이 {prim.GetTypeName()} 입니다 "
                           f"({schema.__name__} 필요): {path}")
    return prim


def translate_op(prim):
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            return op
    raise RuntimeError(f"{prim.GetPath()} 에 translate op 이 없습니다.")


def world_xform(stage, path):
    """USD 월드 행렬 (스케일 제거). 재생 중에는 물리보다 낡았을 수 있다."""
    return Gf.Matrix4d(UsdGeom.Xformable(
        stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)
    ).RemoveScaleShear()


def fk_matrix(pos, rot):
    """Lula FK (위치, 3x3 회전; 열벡터 규약) -> Gf 행벡터 4x4 행렬."""
    r = np.asarray(rot, dtype=float).reshape(3, 3)
    m = Gf.Matrix4d(1.0)
    m.SetRotateOnly(Gf.Matrix3d(*r.T.flatten().tolist()))
    m.SetTranslateOnly(Gf.Vec3d(*[float(c) for c in np.ravel(pos)[:3]]))
    return m


def camera_axes(m):
    """카메라 월드 행렬 -> (eye, forward, up, right). USD: -Z 시선, +Y 위."""
    rot = m.ExtractRotationMatrix()
    right = Gf.Vec3d(*rot[0]).GetNormalized()
    up = Gf.Vec3d(*rot[1]).GetNormalized()
    fwd = Gf.Vec3d(-rot[2][0], -rot[2][1], -rot[2][2]).GetNormalized()
    return m.ExtractTranslation(), fwd, up, right


def view_metrics(cam_m, look, base_xy):
    """카메라 자세를 학습 분포 지표로 바꾼다.

    dist: 눈 -> 검사 지점 거리. elev: 검사 지점에서 본 눈의 앙각.
    azim: 트레이->베이스 방향 기준 트레이->눈 수평 방향의 부호 있는 각.
    look_err: 광축과 검사 지점 방향 사이 각. roll: 광축 둘레 회전
    (월드 Z 를 화면에 투영한 방향과 카메라 up 사이 부호 있는 각).
    """
    eye, fwd, up, _ = camera_axes(cam_m)
    look = Gf.Vec3d(*look)
    to_target = look - eye
    horiz = math.hypot(eye[0] - look[0], eye[1] - look[1])
    elev = math.degrees(math.atan2(eye[2] - look[2], horiz))
    a = (base_xy[0] - look[0], base_xy[1] - look[1])
    b = (eye[0] - look[0], eye[1] - look[1])
    azim = math.degrees(math.atan2(a[0] * b[1] - a[1] * b[0],
                                   a[0] * b[0] + a[1] * b[1]))
    cos_look = max(-1.0, min(1.0, to_target.GetNormalized() * fwd))
    world_up = Gf.Vec3d(0, 0, 1)
    ref = world_up - fwd * (world_up * fwd)
    roll = float("nan")
    if ref.GetLength() > 1e-6:
        ref = ref.GetNormalized()
        # 부호는 --roll 과 같게 맞춘다 (--roll 15 -> roll +15).
        roll = math.degrees(math.atan2((up ^ ref) * fwd, ref * up))
    return {"dist": to_target.GetLength(), "elev": elev, "azim": azim,
            "look_err": math.degrees(math.acos(cos_look)), "roll": roll}


# ── 씬 설정 ───────────────────────────────────────────
def set_viewport_camera(camera_path):
    """GUI 뷰포트를 지정한 카메라로 바꾼다. 헤드리스면 조용히 넘어간다."""
    try:
        from omni.kit.viewport.utility import get_active_viewport
    except ImportError:
        print("[뷰포트] viewport 유틸을 쓸 수 없습니다.", flush=True)
        return
    viewport = get_active_viewport()
    if viewport is None:
        print("[뷰포트] 활성 뷰포트가 없습니다 (헤드리스).", flush=True)
        return
    viewport.set_active_camera(camera_path)
    print(f"[뷰포트] 화면을 {camera_path.rsplit('/', 1)[-1]} 로 바꿨습니다.",
          flush=True)


def open_scene(path):
    """ROS 2 브리지를 켜고 씬을 연다. 로딩이 끝날 때까지 기다린다."""
    if not path.is_file():
        hint = (" (컨테이너라면 프로젝트 폴더가 마운트됐는지 확인)"
                if IN_CONTAINER else "")
        raise RuntimeError(f"씬 파일이 없습니다: {path}{hint}")
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


def check_camera_graph(stage):
    """/rgb 를 내는 그래프가 씬에 있는지 참고용으로 찍는다.

    그래프 노드 값이 USD 에 저장되지 않는 구성이면 못 찾을 수 있다.
    최종 판정은 호스트의 `ros2 topic hz /rgb` 다.
    """
    root = stage.GetPrimAtPath(GRAPH_ROOT)
    topics = []
    if root:
        for prim in Usd.PrimRange(root):
            attr = prim.GetAttribute("inputs:topicName")
            if not attr or attr.Get() is None:
                continue
            kind = prim.GetAttribute("inputs:type")
            topics.append(str(attr.Get()))
            print(f"[점검] {prim.GetPath()} topic={attr.Get()} "
                  f"type={kind.Get() if kind else '?'}", flush=True)
    rel = stage.GetPrimAtPath(RENDER_PRODUCT).GetRelationship(
        "inputs:cameraPrim")
    targets = [str(t) for t in rel.GetTargets()] if rel else []
    print(f"[점검] RenderProduct 카메라 {targets}", flush=True)
    for t in targets:
        require_prim(stage, t, "RenderProduct 대상 카메라", UsdGeom.Camera)
    if not any(t.strip("/").endswith("rgb") for t in topics):
        print("[점검][경고] 그래프에서 rgb 토픽 노드를 찾지 못했습니다. "
              "호스트에서 ros2 topic list 로 직접 확인하세요.", flush=True)


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


def is_rigid(prim):
    return bool(prim) and prim.HasAPI(UsdPhysics.RigidBodyAPI)


def place_trays(stage, tray_x, tray_y, tray_yaw=None):
    """검사 트레이 한 장만 벨트 검사 위치에 두고 나머지는 치운다.

    데이터셋 스크립트도 주차 트레이 네 장을 숨기고 검사 대상만 올린다.
    치운 트레이가 강체면 kinematic 으로 바꿔 무한 낙하를 막는다.
    """
    main = require_prim(stage, MAIN_TRAY, "검사 트레이")
    before = world_xform(stage, MAIN_TRAY).ExtractTranslation()
    translate_op(main).Set(Gf.Vec3d(tray_x, tray_y, TRAY_Z))
    if tray_yaw is not None:
        set_tray_yaw(main, tray_yaw)
    print(f"[조건] 검사 트레이 {v3(before)} -> "
          f"{v3((tray_x, tray_y, TRAY_Z))} (강체={is_rigid(main)})",
          flush=True)

    for path in TRAYS[1:]:
        prim = stage.GetPrimAtPath(path)
        if not prim:
            continue
        translate_op(prim).Set(Gf.Vec3d(0.0, 0.0, -20.0))
        UsdGeom.Imageable(prim).MakeInvisible()
        if is_rigid(prim):
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
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
    cam.CreateVerticalApertureAttr(RIG_APERTURE * args.height / args.width)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))

    rel = require_prim(stage, RENDER_PRODUCT,
                       "RenderProduct").GetRelationship("inputs:cameraPrim")
    before = [str(x) for x in rel.GetTargets()]
    rel.SetTargets([Sdf.Path(RIG_CAMERA)])

    fov = 2 * math.degrees(math.atan(RIG_APERTURE / (2 * RIG_FOCAL)))
    span = 2 * height * math.tan(math.radians(fov / 2))
    print(f"[조건] 리그 카메라 z={TRAY_Z + height:.3f} · 수평 FOV {fov:.1f}° · "
          f"화면 폭 {span * 1000:.0f} mm", flush=True)
    print(f"[조건] RenderProduct 카메라 교체: {before} -> [{RIG_CAMERA}]",
          flush=True)


# ── IK 도우미 ─────────────────────────────────────────
def make_ik(stage):
    """Lula 솔버. 자산이 없으면 None (keep-pose 에서는 FK 검증만 빠진다)."""
    if not args.desc.is_file() or not args.urdf.is_file():
        print(f"[검증] IK 자산 없음 — FK 검증 생략 "
              f"(desc={args.desc.is_file()}, urdf={args.urdf.is_file()})",
              flush=True)
        return None
    ik = LulaKinematicsSolver(robot_description_path=str(args.desc),
                              urdf_path=str(args.urdf))
    base = world_xform(stage, ARM_ROOT)
    bt, bq = base.ExtractTranslation(), base.ExtractRotationQuat()
    ik.set_robot_base_pose(np.array([bt[0], bt[1], bt[2]]),
                           np.array([bq.GetReal(), *bq.GetImaginary()]))
    print(f"[조건] Lula 관절 {list(ik.get_joint_names())}", flush=True)
    return ik


def articulation_action(joints, idx):
    """팔 관절만 드라이브 목표로. 그리퍼(mimic) 관절에는 목표를 걸지 않는다."""
    try:
        from isaacsim.core.utils.types import ArticulationAction
    except ImportError:          # 모듈 위치가 바뀐 버전 대비
        from isaacsim.core.api.controllers.articulation_controller import (
            ArticulationAction)
    return ArticulationAction(joint_positions=joints, joint_indices=idx)


def arm_indices(arm, ik):
    """Lula 관절 순서 -> articulation DOF 인덱스."""
    names = list(arm.dof_names)
    ik_names = (list(ik.get_joint_names()) if ik is not None
                else [f"joint_{i}" for i in range(1, ARM_DOF + 1)])
    missing = [n for n in ik_names if n not in names]
    if missing:
        raise RuntimeError(f"articulation 에 없는 관절: {missing} "
                           f"(DOF: {names})")
    return ik_names, np.array([names.index(n) for n in ik_names])


BASE_OVERRIDE = None   # 물리에서 잰 베이스 (씬 USD 와 다를 때만 설정)
AZIM_REF = None        # IK 에 쓴 방위각 0° 방향 (라디안). 검증도 같은 기준
RANGE_EPS = 0.5        # 경계값 부동소수 오차 허용 (도·cm 단위 공통으로 충분히 작음)


def base_world(stage):
    """로봇 베이스 월드 행렬. 물리 실측값이 있으면 그것을 쓴다."""
    if BASE_OVERRIDE is not None:
        return BASE_OVERRIDE
    return world_xform(stage, ARM_ROOT)


def probe_base(stage, world, arm, ik):
    """재생 직후 로봇 루트 링크의 실제 위치를 재서 씬 값과 비교한다.

    로봇을 옮겼는데 root_joint(바닥 고정 관절)가 예전 자리를 붙잡고 있으면
    재생하자마자 로봇이 옮겨진다. 그 경우 이후 IK·좌표 계산은 물리 실측
    베이스를 쓴다. 실측이 안 되면 씬 값을 그대로 쓴다.
    """
    global BASE_OVERRIDE
    for _ in range(BASE_PROBE_STEPS):
        world.step(render=False)
    try:
        pos, quat = arm.get_world_pose()
        pos = np.asarray(pos, dtype=float)
        quat = np.asarray(quat, dtype=float)
    except Exception as exc:        # API 차이 대비
        print(f"[검증] 베이스 실측 불가 ({type(exc).__name__}) — 씬 값 사용",
              flush=True)
        return
    usd = world_xform(stage, ARM_ROOT).ExtractTranslation()
    diff_mm = float(np.linalg.norm(pos - np.array(usd))) * 1000
    verdict = "PASS" if diff_mm <= BASE_TOL_MM else "WARN"
    print(f"[검증] 베이스 위치 {verdict} 물리 {v3(pos)} vs 씬 {v3(usd)} "
          f"차이 {diff_mm:.1f} mm", flush=True)
    if verdict == "PASS":
        return
    m = Gf.Matrix4d(1.0)
    m.SetRotateOnly(Gf.Quatd(float(quat[0]),
                             Gf.Vec3d(*[float(c) for c in quat[1:4]])))
    m.SetTranslateOnly(Gf.Vec3d(*[float(c) for c in pos]))
    BASE_OVERRIDE = m
    if ik is not None:
        ik.set_robot_base_pose(pos, quat)
    print("[검증] 이후 IK·좌표는 물리 실측 베이스로 계산합니다 "
          "(로봇 이동 후 root_joint 고정 위치가 예전 값일 수 있음).",
          flush=True)


def azimuth_reference(stage, scene_cam, look):
    """IK 방위각 0° 방향(라디안). 두 기준을 모두 계산해 차이를 알린다."""
    b = base_world(stage).ExtractTranslation()
    ref_arm = math.atan2(b[1] - look[1], b[0] - look[0])
    fwd = camera_axes(scene_cam)[1]
    ref_scene = None
    if math.hypot(fwd[0], fwd[1]) > 0.05:
        ref_scene = math.atan2(-fwd[1], -fwd[0])
    if ref_scene is not None:
        diff = math.degrees(math.atan2(math.sin(ref_scene - ref_arm),
                                       math.cos(ref_scene - ref_arm)))
        print(f"[조건] 방위각 기준: arm {math.degrees(ref_arm):+.1f}° · "
              f"scene {math.degrees(ref_scene):+.1f}° (차이 {diff:+.1f}°) "
              f"-> {args.azim_ref} 사용", flush=True)
        if abs(diff) > AZIM_REF_WARN_DEG:
            print("[검증] 방위각 WARN — 로봇이 옆으로 옮겨진 것 같습니다. "
                  "화면 속 트레이 방향이 ROI 보정 때와 다를 수 있습니다. "
                  "debug_image 로 확인하고, 씬에 저장된 시선을 유지하려면 "
                  "--azim-ref scene", flush=True)
    global AZIM_REF
    AZIM_REF = (ref_scene if args.azim_ref == "scene" and ref_scene is not None
                else ref_arm)
    return AZIM_REF


def azim_anchor(stage):
    """view_metrics 방위각 기준점: IK 와 같은 기준 방향 위의 한 점."""
    if AZIM_REF is None:
        b = base_world(stage).ExtractTranslation()
        return (b[0], b[1])
    return (args.tray_x + math.cos(AZIM_REF), args.tray_y + math.sin(AZIM_REF))


def out_of_range(metrics):
    bad = []
    for k, (lo, hi) in DATASET_RANGE.items():
        eps = RANGE_EPS if k != "dist" else 1e-4
        v = metrics[k]
        if math.isnan(v) or not lo - eps <= v <= hi + eps:
            bad.append(k)
    return bad


def ik_candidates(stage, ik, cam_in_tool, scene_cam):
    """학습 분포 안에서 IK 가 수렴한 해를 차례로 낸다.

    (dist, elev, azim, eye, look, joints) — joints 는 Lula 관절 순서.
    """
    look = np.array([args.tray_x, args.tray_y, LOOK_Z])
    ref = azimuth_reference(stage, scene_cam, look)
    tool_in_cam = cam_in_tool.GetInverse()
    seed = np.deg2rad(SEED_Q_DEG)
    elev0 = min(max(args.elev, 1.0), 89.0)
    candidates = [(args.dist, elev0, args.azim_offset)]
    candidates += [(d, e, a) for e in IK_ELEVATIONS
                   for d in IK_DISTANCES for a in IK_AZIMUTHS
                   if (d, e, a) != candidates[0]]
    for dist, elev, azim in candidates:
        az, el = ref + math.radians(azim), math.radians(elev)
        eye = look + dist * np.array([math.cos(el) * math.cos(az),
                                      math.cos(el) * math.sin(az),
                                      math.sin(el)])
        cam = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*look),
                                      Gf.Vec3d(0, 0, 1)).GetInverse()
        cam = Gf.Matrix4d().SetRotate(
            Gf.Rotation(Gf.Vec3d(0, 0, 1), args.roll)) * cam
        tool = tool_in_cam * cam
        q, t = tool.ExtractRotationQuat(), tool.ExtractTranslation()
        try:
            sol, ok = ik.compute_inverse_kinematics(
                frame_name="tool0",
                target_position=np.array([t[0], t[1], t[2]]),
                target_orientation=np.array([q.GetReal(), *q.GetImaginary()]),
                warm_start=seed)
        except Exception as exc:     # 솔버 예외는 해 없음으로 본다
            print(f"[조건] IK 예외 ({type(exc).__name__}) — 다음 조합",
                  flush=True)
            continue
        if ok:
            joints = np.asarray(getattr(sol, "joint_positions", sol),
                                dtype=float)
            yield dist, elev, azim, eye, look, joints


def apply_arm(arm, idx, joints):
    """팔 관절만 순간 배치 + 드라이브 목표 설정 (그리퍼는 그대로)."""
    current = np.array(arm.get_joint_positions(), dtype=float)
    current[idx] = joints
    arm.set_joint_positions(current)
    arm.apply_action(articulation_action(joints, idx))


def settle(world, steps=SETTLE_STEPS, render=False):
    for _ in range(steps):
        world.step(render=render)


def fk_camera(ik, cam_in_tool, joints):
    pos, rot = ik.compute_forward_kinematics("tool0", joints)
    return cam_in_tool * fk_matrix(pos, rot)


def pose_bad_keys(stage, cam_m):
    metrics = view_metrics(cam_m, (args.tray_x, args.tray_y, LOOK_Z),
                           azim_anchor(stage))
    return out_of_range(metrics), metrics


def restore_scene_pose(arm, idx, scene_q, reason):
    print(f"[검증] 대체 자세 WARN — {reason}. 씬에 저장된 자세를 씁니다.",
          flush=True)
    apply_arm(arm, idx, scene_q[idx])
    if args.strict:
        raise RuntimeError(f"대체 자세로 넘어감: {reason} (--strict)")


def choose_arm_pose(stage, world, arm, ik, cam_in_tool, scene_q, scene_cam,
                    mover):
    """IK 해를 실제로 적용·안정화해 보고 통과하는 첫 해를 쓴다.

    통과 조건: settle 후 관절 오차 <= 허용치 (충돌·도달 불가 없음) 이고
    카메라 자세가 학습 분포 안. 실패하면 트레이를 원위치하고 다음 해.
    모두 실패하면 가장 나은 해, 해가 없으면 씬 자세로 대체한다 (에러 없이).
    반환: (idx, joints) 또는 None(씬 자세)
    """
    _, idx = arm_indices(arm, ik)
    if ik is None:
        restore_scene_pose(arm, idx, scene_q, "IK 자산 없음")
        return None
    tries = []
    for n, (dist, elev, azim, eye, look, joints) in enumerate(
            ik_candidates(stage, ik, cam_in_tool, scene_cam), start=1):
        joints = joints[:len(idx)]
        apply_arm(arm, idx, joints)
        settle(world)
        err = math.degrees(float(np.abs(
            np.array(arm.get_joint_positions())[idx] - joints).max()))
        bad, metrics = pose_bad_keys(stage, fk_camera(ik, cam_in_tool,
                                                      joints))
        ok = err <= SETTLE_TOL_DEG and not bad
        print(f"[자세 시도 {n}] 거리 {dist:.2f} · 앙각 {elev:.0f}° · "
              f"az {azim:+.0f}° -> 관절오차 {err:.2f}° · "
              f"범위밖 {bad or '없음'} -> {'채택' if ok else '다음'}",
              flush=True)
        tries.append((len(bad), err, (dist, elev, azim), eye, look, joints))
        if ok:
            break
        mover.reset()
        if len(tries) >= max(1, args.max_pose_tries):
            break
    if not tries:
        restore_scene_pose(arm, idx, scene_q, "학습 범위 안 IK 해 없음")
        return None
    best = min(tries, key=lambda t: (t[0], t[1]))
    n_bad, err, (dist, elev, azim), eye, look, joints = best
    if best is not tries[-1] or n_bad or err > SETTLE_TOL_DEG:
        apply_arm(arm, idx, joints)
        mover.reset()
        settle(world)
        if n_bad or err > SETTLE_TOL_DEG:
            print("[검증] 자세 WARN — 모든 시도가 기준 미달이라 가장 나은 해를 "
                  "씁니다.", flush=True)
            if args.strict:
                raise RuntimeError("기준을 만족하는 IK 자세 없음 (--strict)")
    print(f"[조건] IK 해: 거리 {dist:.2f} m · 앙각 {elev:.0f}° · "
          f"az {azim:+.0f}° · roll {args.roll:.0f}° "
          f"(학습: 0.50~0.72 m, 50~88°, ±25°)", flush=True)
    print(f"[조건] 카메라 eye={v3(eye)} look={v3(look)}", flush=True)
    print(f"[조건] 관절(도) {np.rad2deg(joints).round(1).tolist()}",
          flush=True)
    return idx, joints


# ── 자세 보고 · 검증 ──────────────────────────────────
def rpy_deg(quat):
    """USD quaternion -> roll·pitch·yaw (도)."""
    w = quat.GetReal()
    x, y, z = quat.GetImaginary()
    return (math.degrees(math.atan2(2 * (w * x + y * z),
                                    1 - 2 * (x * x + y * y))),
            math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))),
            math.degrees(math.atan2(2 * (w * z + x * y),
                                    1 - 2 * (y * y + z * z))))


def print_matrix(label, m):
    q = m.ExtractRotationQuat()
    print(f"[자세] {label:<8s} pos={v3(m.ExtractTranslation(), 4)} m  "
          f"quat(wxyz)=({q.GetReal():+.5f}, {q.GetImaginary()[0]:+.5f}, "
          f"{q.GetImaginary()[1]:+.5f}, {q.GetImaginary()[2]:+.5f})  "
          f"rpy={v3(rpy_deg(q), 2)}", flush=True)


def camera_pose(stage, arm, ik, cam_in_tool):
    """검사 카메라의 월드 행렬과 출처('fk'|'usd')를 정한다.

    재생 중 USD 가 물리 결과를 따라가지 않을 수 있으므로, IK 자산이
    있으면 FK(tool0) x 고정 오프셋(tool0->카메라)을 쓰고 USD 와 비교한다.
    """
    if args.rig_camera:
        return world_xform(stage, RIG_CAMERA), "usd"
    usd_cam = world_xform(stage, COLOR)
    if ik is None:
        print("[검증] 카메라 자세 출처: USD (FK 교차검증 불가)", flush=True)
        return usd_cam, "usd"

    ik_names, idx = arm_indices(arm, ik)
    q = np.array(arm.get_joint_positions(), dtype=float)[idx]
    fk_pos, fk_rot = ik.compute_forward_kinematics("tool0", q)
    tool_fk = fk_matrix(fk_pos, fk_rot)
    tool_usd = world_xform(stage, TOOL0)
    diff_mm = (tool_fk.ExtractTranslation()
               - tool_usd.ExtractTranslation()).GetLength() * 1000
    base_mm = world_xform(stage, ARM_ROOT).ExtractTranslation().GetLength() * 1000
    print_matrix("tool0 FK", tool_fk)
    print_matrix("tool0 USD", tool_usd)
    verdict = "PASS" if diff_mm <= FK_USD_TOL_MM else "WARN"
    print(f"[검증] tool0 FK-USD 차이 {verdict} {diff_mm:.1f} mm "
          f"(허용 {FK_USD_TOL_MM} mm)", flush=True)
    if verdict == "WARN":
        if abs(diff_mm - base_mm) < 20.0:
            print("[검증] 차이가 베이스 위치 크기와 비슷합니다 — FK 가 base "
                  "기준 좌표를 돌려주는 것일 수 있습니다. 결과 해석 전에 "
                  "확인하세요.", flush=True)
        else:
            print("[검증] USD 자세가 물리 상태를 따라가지 않을 가능성 — "
                  "카메라 자세는 FK 기준을 씁니다.", flush=True)
    return cam_in_tool * tool_fk, "fk"


def report_pose(stage, arm, ik, cam_in_tool, target_x, target_y):
    """팔과 카메라 자세를 찍고 학습 분포와 비교한다.

    반환: (카메라 월드 행렬, WARN 항목 목록)
    """
    camera_path = RIG_CAMERA if args.rig_camera else COLOR
    print("[자세] ── M0609 + RealSense ──────────────", flush=True)
    names = list(arm.dof_names)
    pos = np.array(arm.get_joint_positions(), dtype=float)
    _, idx = arm_indices(arm, ik)
    deg = [math.degrees(pos[i]) for i in idx]
    print("[자세] 관절(도)  " + "  ".join(
        f"j{i + 1}={v:+8.3f}" for i, v in enumerate(deg)), flush=True)
    print(f"[자세] 관절(rad) {[round(math.radians(v), 6) for v in deg]}",
          flush=True)
    print(f"[자세] DOF {names}", flush=True)
    print_matrix("베이스", base_world(stage))

    cam_m, source = camera_pose(stage, arm, ik, cam_in_tool)
    print_matrix(f"카메라({source})", cam_m)
    eye, fwd, up, right = camera_axes(cam_m)
    print(f"[자세] 카메라축 forward={v3(fwd, 4)} up={v3(up, 4)} "
          f"right={v3(right, 4)}", flush=True)
    print(f"[자세] tool0 기준 카메라 오프셋 "
          f"{v3(cam_in_tool.ExtractTranslation(), 4)} m", flush=True)

    cam = UsdGeom.Camera(stage.GetPrimAtPath(camera_path))
    focal = cam.GetFocalLengthAttr().Get()
    ah = cam.GetHorizontalApertureAttr().Get()
    product = stage.GetPrimAtPath(RENDER_PRODUCT)
    w = product.GetAttribute("inputs:width").Get()
    h = product.GetAttribute("inputs:height").Get()
    # 정사각 픽셀: 수직 FOV 는 수평 조리개와 해상도 비율로 정해진다고 본다.
    av_eff = ah * h / w
    print(f"[자세] 광학 focal={focal:.3f} aperture(h)={ah:.3f} -> FOV "
          f"{2 * math.degrees(math.atan(ah / (2 * focal))):.1f}° x "
          f"{2 * math.degrees(math.atan(av_eff / (2 * focal))):.1f}° "
          f"· 렌더 {w}x{h} · prim {camera_path}", flush=True)

    if args.rig_camera:
        print("[검증] 리그 카메라 — 학습 분포 판정 생략", flush=True)
        return cam_m, []

    metrics = view_metrics(cam_m, (target_x, target_y, LOOK_Z),
                           azim_anchor(stage))
    bad = out_of_range(metrics)
    for key, (lo, hi) in DATASET_RANGE.items():
        ok = key not in bad
        print(f"[검증] {key:<8s} {'PASS' if ok else 'WARN'} {metrics[key]:+.3f} "
              f"(범위 {lo}~{hi})", flush=True)
    if (w, h) != (args.width, args.height):
        bad.append("resolution")
        print(f"[검증] resolution WARN {w}x{h}", flush=True)
    return cam_m, bad


# ── 브리지 (검출 -> base 좌표) ────────────────────────
class DetectionBridge:
    """Isaac 프로세스 안에서 rclpy 로 검출을 받아 base 좌표를 낸다.

    카메라 행렬은 settle 뒤 한 번 정한다 (팔이 고정 검사 자세라는 전제).
    """

    def __init__(self, stage, cam_matrix):
        sys.path.insert(0, str(RUNTIME_DIR))
        # Isaac 내장 Python(3.11)은 번들 rclpy 를 sys.path 앞에 둬야 한다
        # (standalone_app.py 의 configure_ros_environment 와 같은 처리).
        ros_python = bundled_ros_dir() / "rclpy"
        if ros_python.is_dir() and str(ros_python) not in sys.path:
            sys.path.insert(0, str(ros_python))
        import vision_bridge as vb
        import rclpy
        from sensor_msgs.msg import Image
        from std_msgs.msg import String

        self.vb, self.rclpy = vb, rclpy
        self.judge = None
        try:
            self.judge = vb.load_inspection_config(args.inspection_config)
            print(f"[브리지] 판정표 {self.judge[1]} "
                  f"(출처 {args.inspection_config})", flush=True)
        except (OSError, RuntimeError, ValueError, KeyError) as exc:
            print(f"[브리지][경고] 검사 설정을 못 읽음: {exc} — 판정 없이 "
                  f"모든 슬롯 좌표를 냅니다 (status=UNJUDGED).", flush=True)
        camera_path = RIG_CAMERA if args.rig_camera else COLOR
        self.model = vb.CameraModel(stage, camera_path, RENDER_PRODUCT)
        self.cam_matrix = cam_matrix
        self.base_matrix = base_world(stage)
        self.depth = None
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("vision_functest_bridge")
        self.pub = self.node.create_publisher(String, args.target_topic, 10)
        self.node.create_subscription(String, args.det_topic,
                                      self.on_detections, 10)
        if args.depth_topic:
            self.node.create_subscription(Image, args.depth_topic,
                                          self.on_depth, 1)
        self.String = String
        print(f"[브리지] {args.det_topic} -> {args.target_topic} "
              f"(깊이: {args.depth_topic or '없음, 수평면 z='}"
              f"{'' if args.depth_topic else f'{args.plane_z:.3f}'}) · "
              f"fx={self.model.fx:.1f} · FOV "
              f"{self.model.fov_deg()[0]:.1f}°x{self.model.fov_deg()[1]:.1f}°",
              flush=True)

    def on_depth(self, msg):
        try:
            self.depth = self.vb.decode_depth(msg)
        except RuntimeError as exc:
            print(f"[브리지][경고] {exc}", flush=True)

    def on_detections(self, msg):
        """검출 -> (판정 재현) -> 불량 슬롯 base 좌표 발행.

        판정은 Inspection Node 와 같은 규칙·같은 YAML 로 다시 계산한다.
        UNKNOWN 슬롯이 하나라도 있으면 Node 도 검사 FAILED 를 내므로 좌표를
        내지 않는다 (Task Manager 도 이때 CULL 을 보내지 않는다).
        """
        vb = self.vb
        try:
            parsed = vb.parse_detections(msg.data)
            if not vb.check_resolution(parsed, self.model):
                print("[브리지][경고] 검출 메시지에 해상도 정보 없음 — "
                      "카메라 해상도로 가정", flush=True)
                parsed["image_width"] = self.model.width
                parsed["image_height"] = self.model.height
        except (ValueError, RuntimeError) as exc:
            print(f"[브리지][오류] {exc}", flush=True)
            return

        out = {"task_id": parsed["task_id"],
               "command_id": parsed["command_id"],
               "pallet_id": parsed["pallet_id"],
               "stamp": parsed["stamp"],
               "frame": (ARM_ROOT if BASE_OVERRIDE is None
                         else f"{ARM_ROOT} (물리 루트 링크 실측)"),
               "targets": {}}

        if self.judge is not None:
            result = vb.assess_slots(parsed, *self.judge)
            out["defect_slots"] = result["defect"]
            out["unknown_slots"] = result["unknown"]
            states = "  ".join(f"{k[-2:]}={v}"
                               for k, v in result["states"].items())
            print(f"[브리지] 판정 {states}", flush=True)
            if result["unknown"]:
                out.update(status="FAILED", reason="UNKNOWN_SLOT")
                print(f"[브리지] 검사 FAILED (UNKNOWN {result['unknown']}) — "
                      f"좌표를 내지 않습니다.", flush=True)
                self._publish(out)
                return
            out.update(status="SUCCEEDED", reason="NONE")
            chosen = {s: result["items"][s] for s in result["defect"]}
        else:
            out.update(status="UNJUDGED", reason="NO_CONFIG")
            chosen = parsed["slots"]

        for slot, item in sorted(chosen.items()):
            center = vb.extract_center(item)
            if center is None:
                print(f"[브리지][경고] {slot}: 중심 필드를 못 찾음 "
                      f"(키: {sorted(item)})", flush=True)
                continue
            target = vb.pixel_to_base(
                self.model, self.cam_matrix, self.base_matrix, *center,
                depth_image=self.depth, plane_z=args.plane_z)
            if target is None:
                print(f"[브리지][경고] {slot}: 3D 변환 실패 {center}",
                      flush=True)
                continue
            target["class_name"] = item.get("class_name")
            target["confidence"] = item.get("confidence")
            out["targets"][slot] = target
            label = "불량" if self.judge is not None else "미판정"
            print(f"[브리지] {label} {slot} {target['class_name']} "
                  f"px={target['pixel']} base={target['xyz_base']} "
                  f"world={target['xyz_world']} ({target['source']})",
                  flush=True)
        if out["status"] == "SUCCEEDED" and not chosen:
            print("[브리지] 불량 슬롯 없음 — 좌표 없음 (정상)", flush=True)
        self._publish(out)

    def _publish(self, out):
        self.pub.publish(self.String(data=json.dumps(out,
                                                     ensure_ascii=False)))

    def spin_once(self):
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def close(self):
        try:
            self.node.destroy_node()
            if self.rclpy.ok():
                self.rclpy.shutdown()
        except Exception:   # 종료 중 오류는 삼킨다
            pass


def start_bridge(stage, cam_matrix):
    try:
        return DetectionBridge(stage, cam_matrix)
    except Exception as exc:     # import·rclpy.init 실패 모두
        print(f"[브리지][오류] 시작 실패: {type(exc).__name__}: {exc} — "
              f"rclpy 번들 또는 vision_bridge.py 위치를 확인하세요. "
              f"브리지 없이 /rgb 발행만 계속합니다.", flush=True)
        return None


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


class TrayMover:
    """검사 트레이를 옮긴다. 강체면 물리 API, 아니면 USD translate."""

    def __init__(self, stage):
        prim = stage.GetPrimAtPath(MAIN_TRAY)
        self.rigid = is_rigid(prim)
        self.op = translate_op(prim)
        self.body = None
        if self.rigid:
            from isaacsim.core.prims import SingleRigidPrim
            self.body = SingleRigidPrim(prim_path=MAIN_TRAY,
                                        name="inspect_tray")
            self.body.initialize()
        self.stage = stage
        q = world_xform(stage, MAIN_TRAY).ExtractRotationQuat()
        self.orient = np.array([q.GetReal(), *q.GetImaginary()])

    def reset(self):
        """검사 위치로 되돌린다 (자세 시도 중 팔에 밀렸을 수 있음)."""
        self.move(args.tray_x, args.tray_y)

    def move(self, x, y):
        if self.body is not None:
            self.body.set_world_pose(position=np.array([x, y, TRAY_Z]),
                                     orientation=self.orient)
            self.body.set_linear_velocity(np.zeros(3))
            self.body.set_angular_velocity(np.zeros(3))
        else:
            self.op.Set(Gf.Vec3d(x, y, TRAY_Z))

    def position(self):
        if self.body is not None:
            return self.body.get_world_pose()[0]
        return world_xform(self.stage, MAIN_TRAY).ExtractTranslation()


def run_sweep(stage, tick, mover):
    """팔은 그대로 두고 트레이만 옮겨 가며 잡을 시간을 준다.

    이미지 위치 흔들림 시험이다. 물리 컨베이어 동작 검증이 아니다.
    """
    print(f"[스윕] 트레이 이동 방식: "
          f"{'물리 API (강체)' if mover.rigid else 'USD translate'}",
          flush=True)
    hold = max(1, int(args.sweep_hold / PHYSICS_DT))
    offsets = parse_sweep_offsets(args.sweep_offsets)

    for index, (dx, dy) in enumerate(offsets, start=1):
        x, y = args.tray_x + dx, args.tray_y + dy
        mover.move(x, y)
        for _ in range(SWEEP_SETTLE_STEPS):
            if not tick():
                return
        actual = mover.position()
        err = math.hypot(actual[0] - x, actual[1] - y) * 1000
        print(f"[스윕] {index}/{len(offsets)} dx={dx:+.3f} dy={dy:+.3f} "
              f"-> 요청 ({x:+.3f}, {y:+.3f}) 실측 {v3(actual)} "
              f"오차 {err:.1f} mm", flush=True)
        for _ in range(hold):
            if not tick():
                return
    print("[스윕] 완료", flush=True)


def main():
    print(f"[시작] 컨테이너={IN_CONTAINER} · ROS_DOMAIN_ID="
          f"{os.environ.get('ROS_DOMAIN_ID', '0')} · RMW="
          f"{os.environ.get('RMW_IMPLEMENTATION', '?')} · FASTDDS 전송="
          f"{os.environ.get('FASTDDS_BUILTIN_TRANSPORTS', '기본')}",
          flush=True)
    stage = open_scene(args.scene.resolve())

    # 물리 객체 배치는 world.reset() 전에 끝낸다. 재생 뒤 xform 을 바꾸면
    # 물리가 그 자리를 모른다 (스윕은 TrayMover 가 따로 처리).
    set_render_product(stage, args.width, args.height)
    place_trays(stage, args.tray_x, args.tray_y, args.tray_yaw)
    set_lights(stage)
    if args.rig_camera:
        make_rig_camera(stage, args.tray_x, args.tray_y, args.rig_height)
    check_camera_graph(stage)

    # tool0 -> 카메라는 강체 고정이므로 재생 전 USD 에서 한 번 잰다.
    require_prim(stage, COLOR, "손목 카메라", UsdGeom.Camera)
    require_prim(stage, TOOL0, "tool0")
    scene_cam = world_xform(stage, COLOR)        # 씬에 저장된 카메라 자세
    cam_in_tool = scene_cam * world_xform(stage, TOOL0).GetInverse()

    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT,
                  rendering_dt=PHYSICS_DT)
    world.reset()
    arm = SingleArticulation(prim_path=ARM_ART, name="m0609")
    arm.initialize()
    ik = make_ik(stage)
    scene_q = np.array(arm.get_joint_positions(), dtype=float)
    mover = TrayMover(stage)
    probe_base(stage, world, arm, ik)

    commanded = None
    if args.rig_camera:
        print("[조건] 리그 카메라 모드 — 팔을 건드리지 않는다.", flush=True)
    elif args.keep_pose:
        print("[조건] IK 건너뜀 — 씬에 저장된 팔 자세를 그대로 쓴다.",
              flush=True)
    else:
        commanded = choose_arm_pose(stage, world, arm, ik, cam_in_tool,
                                    scene_q, scene_cam, mover)

    world.play()
    settle(world, render=True)

    if commanded is not None:
        idx, target = commanded
        err = np.abs(np.array(arm.get_joint_positions())[idx] - target)
        worst = math.degrees(float(err.max()))
        verdict = "PASS" if worst <= SETTLE_TOL_DEG else "WARN"
        print(f"[검증] settle 후 팔 관절 최대 오차 {verdict} {worst:.2f}° "
              f"(허용 {SETTLE_TOL_DEG}°; 크면 드라이브·중력·충돌 의심)",
              flush=True)

    if args.view_camera:
        if args.headless:
            print("[뷰포트] 헤드리스라 생략 — 호스트에서 "
                  "`ros2 run rqt_image_view rqt_image_view /rgb` 로 보세요.",
                  flush=True)
        else:
            set_viewport_camera(RIG_CAMERA if args.rig_camera else COLOR)

    cam_m, bad = report_pose(stage, arm, ik, cam_in_tool,
                             args.tray_x, args.tray_y)
    if bad and args.strict:
        raise RuntimeError(f"학습 분포 밖: {bad} (--strict)")

    bridge = start_bridge(stage, cam_m) if args.bridge else None
    print(f"[준비] 재생 중. ROS_DOMAIN_ID="
          f"{os.environ.get('ROS_DOMAIN_ID', '0')} 에서 "
          f"/rgb {args.width}x{args.height} 발행", flush=True)

    def tick():
        world.step(render=True)
        if bridge is not None:
            bridge.spin_once()
        return app.is_running()

    try:
        if args.sweep:
            run_sweep(stage, tick, mover)
            return
        steps = 0
        while tick():
            steps += 1
            if steps % 600 == 0:
                print(f"[진행] {steps} 스텝", flush=True)
    finally:
        if bridge is not None:
            bridge.close()


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
