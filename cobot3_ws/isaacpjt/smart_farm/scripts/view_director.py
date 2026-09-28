"""[navigation 2026-09-28] 녹화용 시점 연출. 공정과 대상 위치를 보고 뷰포트 카메라를 고른다.

씬 파일을 고치지 않는다. 필요한 카메라는 실행 중에 만든다.
`SMARTFARM_VIEW_FOLLOW=1` 일 때만 동작하므로 평소 실행에는 영향이 없다.

카메라 자리는 눈대중이 아니라 2026-09-28 에 후보를 렌더해 비교하고 정했다
(tools/preview_new_cameras.py, results/log_media/camera_try_20260928_1643/).
"""

import math
import os

CARTER = "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS/chassis_link"
FORK_LINK = "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS/m0609_with_fork/link_6"
PALLET = "/World/SmartFarm/Placed/Pallet_01"
PROCESS_CAMS = "/World/ProcessCameras"

# 씬에 이미 있는 카메라 (2026-09-28 렌더로 화각 확인함)
CAM_HARVEST = PROCESS_CAMS + "/Cam1_Harvest"          # 랙 + 카터 + 포크팔 풀샷
CAM_NAV_PLACE = PROCESS_CAMS + "/Cam2_Nav2Place"      # 주행 경로 + 턴테이블 롱샷
CAM_CULL = PROCESS_CAMS + "/Cam4_CullPickPlace"       # 비전룸 풀샷
CAM_PUSHER = PROCESS_CAMS + "/Cam5_Pusher"            # 컨베이어 베드 정면

# 실행 중에 만드는 카메라. (경로, 눈 위치, 바라볼 점, 초점거리)
NEW_CAMS = (
    # 턴테이블 벨트 ~ 피더 방향전환 ~ 비전룸 입구를 한 화면에. 3번이 비어 있어 그 번호를 쓴다.
    (PROCESS_CAMS + "/Cam3_FeederEntry", (-2.20, -2.30, 2.40), (-2.20, -7.00, 0.60), 14.0),
    # 비전룸 출구 ~ 반출 컨베이어 ~ 맵 밖까지.
    (PROCESS_CAMS + "/Cam6_Outfeed", (2.50, -3.80, 3.40), (7.00, -6.70, 0.60), 14.0),
)

# 움직이는 prim 에 붙이는 시점뷰. (부모 prim, 카메라 이름, 로컬 위치, 로컬 회전 XYZ, 초점거리)
POV_CAMS = (
    (FORK_LINK, "ForkPOV", (0.0, 0.0, 0.12), (90.0, 0.0, 0.0), 12.0),
    (PALLET, "PalletPOV", (0.0, 0.0, 0.35), (75.0, 0.0, 0.0), 10.0),
)

# 공정 단계별 기본 카메라
BY_OPERATION = {
    "TRANSFER": CAM_HARVEST,
    "PICK_HARVEST": CAM_HARVEST,
    "PLACE_INSPECT": CAM_NAV_PLACE,
    "CONVEY_TO_INSPECT": PROCESS_CAMS + "/Cam3_FeederEntry",
    "PREPARE_INSPECT": CAM_CULL,
    "MOVE_TO_INSPECT": CAM_CULL,
    "CULL": CAM_CULL,
    "RELEASE_INSPECT": CAM_PUSHER,
    "CONVEYOR_OUT": CAM_PUSHER,
}

# 랙 통로 남단. 카터가 이 선을 넘으면 주행 롱샷으로 바꾼다(stations.yaml 의 CORRIDOR_EXIT 기준).
CORRIDOR_EXIT_Y = -1.20
# 팔레트가 이 x 를 넘으면 반출 카메라로 바꾼다(비전룸 동쪽 끝 x 0.24 를 지난 뒤).
OUTFEED_X = 0.60


def enabled() -> bool:
    """녹화용 시점 연출을 켤지 여부."""

    return os.environ.get("SMARTFARM_VIEW_FOLLOW", "").strip() in ("1", "true", "True")


def _world_xyz(stage, path):
    """prim 의 world 위치. 없으면 None."""

    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return None
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = matrix.ExtractTranslation()
    return (t[0], t[1], t[2])


def _look_at_rotation(eye, target):
    """eye 에서 target 을 보게 하는 (pitch, yaw). USD 카메라는 -z 를 본다."""

    dx, dy, dz = (target[i] - eye[i] for i in range(3))
    yaw = math.degrees(math.atan2(dy, dx)) - 90.0
    pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
    return 90.0 + pitch, yaw


def ensure_cameras(stage) -> None:
    """녹화에 필요한 카메라를 만든다. 이미 있으면 그대로 둔다."""

    from pxr import Gf, UsdGeom

    for path, eye, target, focal in NEW_CAMS:
        if stage.GetPrimAtPath(path).IsValid():
            continue
        camera = UsdGeom.Camera.Define(stage, path)
        camera.GetFocalLengthAttr().Set(focal)
        xform = UsdGeom.Xformable(camera.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*eye))
        pitch, yaw = _look_at_rotation(eye, target)
        xform.AddRotateXYZOp().Set(Gf.Vec3f(pitch, 0.0, yaw))
        print(f"[화면] 카메라 생성 {path}", flush=True)

    for parent, name, offset, rotation, focal in POV_CAMS:
        if not stage.GetPrimAtPath(parent).IsValid():
            print(f"[화면] 부모 prim 이 없어 {name} 을 건너뜀: {parent}", flush=True)
            continue
        path = f"{parent}/{name}"
        if stage.GetPrimAtPath(path).IsValid():
            continue
        camera = UsdGeom.Camera.Define(stage, path)
        camera.GetFocalLengthAttr().Set(focal)
        xform = UsdGeom.Xformable(camera.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*offset))
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
        print(f"[화면] 시점뷰 카메라 생성 {path}", flush=True)


def decide(stage, operation: str) -> str:
    """지금 보여 줄 카메라를 고른다.

    공정 이름만으로는 부족한 두 구간을 위치로 보완한다.
      - NAVIGATION 은 Isaac 이 명령을 받지 않는다. 카터가 통로를 벗어나면 주행 롱샷으로 바꾼다.
      - CONVEYOR_OUT 은 팔레트가 비전룸을 나간 뒤 반출 카메라로 바꾼다.
    """

    if operation == "CONVEYOR_OUT":
        pallet = _world_xyz(stage, PALLET)
        if pallet is not None and pallet[0] > OUTFEED_X:
            return PROCESS_CAMS + "/Cam6_Outfeed"
        return CAM_PUSHER

    chosen = BY_OPERATION.get(operation)
    if chosen:
        return chosen

    # 명령이 없는 구간(NAVIGATION, INSPECT, RECHECK 등)은 카터 위치로 판단한다.
    carter = _world_xyz(stage, CARTER)
    if carter is not None and carter[1] <= CORRIDOR_EXIT_Y:
        return CAM_NAV_PLACE
    return CAM_HARVEST


class ViewDirector:
    """지금 카메라를 기억해 두고 바뀔 때만 뷰포트를 옮긴다."""

    def __init__(self, set_camera, every: int = 30) -> None:
        """set_camera 는 카메라 경로를 받아 뷰포트를 바꾸는 함수다."""

        self.set_camera = set_camera
        self.every = max(1, every)
        self.current = ""
        self.ready = False
        self.ticks = 0

    def update(self, stage, operation: str) -> None:
        """매 프레임 불러도 되게 싸게 만들었다. every 프레임마다 한 번만 판단한다."""

        if not enabled() or stage is None:
            return
        if not self.ready:
            ensure_cameras(stage)
            self.ready = True

        self.ticks += 1
        if self.ticks % self.every:
            return

        chosen = decide(stage, operation or "")
        if chosen and chosen != self.current:
            self.current = chosen
            print(f"[화면] 공정 {operation or '(명령없음)'} -> {chosen}", flush=True)
            self.set_camera(chosen)
