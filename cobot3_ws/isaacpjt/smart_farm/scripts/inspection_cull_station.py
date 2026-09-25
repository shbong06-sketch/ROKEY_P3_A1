"""[올인원 2026-09-25] 비전룸 검사 · 솎아내기 스테이션.

컨베이어(conveyor.py)가 카메라 앞에 세우고 고정한 팔레트를 받아서
비전 M0609 손목 RealSense + YOLO(best.pt) 로 칸별 색을 판정하고, 불량 칸 포기를
팀 CullMotion(RMPflow, cull_motion.py 무수정) 으로 집어 SortBin_1 / SortBin_2 에 번갈아 버린 뒤
팔레트를 벨트로 돌려보낸다. 컨베이어는 시간이 아니라 이 스테이션이 끝났을 때(inspection_done) 다시 돈다.

    station = install(stage, world, m0609_dir)       # world.reset() 전 (이송 프레임도 여기서 세운다)
    world.reset(); conveyor.attach(); station.attach(conveyor)
    매 스텝: conveyor.update(dt); station.update(dt)
    Stop -> Play: station.reset() -> world.reset() -> station.attach(conveyor)

팔레트 한 장의 순서 (state)
    IDLE          컨베이어가 카메라 앞에 세운 팔레트가 있는지 본다
    PUSH_IN       이송 프레임(build_transfer_frame)이 트레이 위로 와서 내려앉고, PlateN 이 트레이를 로봇 쪽으로 민다.
                  컨베이어 줄(y -6.73)은 팔 도달거리 밖이다. 위에서 집기 + 접근 18 cm 로 닿는 트레이 중심은
                  base 에서 약 0.48 m 까지라 그 자리(y -7.00)로 옮긴다. 검사·솎아내기 동안 프레임이 트레이를 잡아 둔다.
    MOVE_INSPECT  검사 자세 (카메라가 팔 쪽 위에서 트레이 중심을 내려다봄)
    CAPTURE       컬러 프레임 FRAMES 장 -> YOLO -> 칸 배정(카메라 모델로 포기 투영) -> 다수결
    CULL / HOME   CULL_CLASSES 칸마다 OPEN-APPROACH-DESCEND-GRASP-LIFT-VIA-PLACE-RELEASE-RETREAT, 끝나면 검사 자세로
    RECHECK       다시 찍어 불량이 남았는지 기록
    PUSH_OUT      PlateS 가 트레이를 벨트 줄로 되밀고 프레임이 올라간 뒤 conveyor.inspection_done()

양배추 판정 (2026-09-25 결정): dark_green = 정상, yellow · brown = 불량 -> 둘 다 솎아낸다 (CULL_CLASSES).
조정할 값은 아래 상수 묶음에 모여 있다 (도달거리 TRAY_REACH, 프레임 치수·속도, 버리는 높이, 검사 자세 ...).
"""

import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

ROBOT_PATH = "/World/SmartFarm/Placed/M0609/Asset"
BASE_PATH = f"{ROBOT_PATH}/base_link"
EE_PATH = f"{ROBOT_PATH}/link_6"
GRIPPER_ROOT_PATH = f"{ROBOT_PATH}/onrobot_rg2ft"
CAMERA_PATH = f"{GRIPPER_ROOT_PATH}/angle_bracket/realsense_d455/RSD455/Camera_OmniVision_OV9782_Color"
# 버리는 상자. 장면의 SortBox_1/2 는 속이 찬 큐브라 포기가 뚜껑 위에 얹혔다 — 양배추 씬 복사본(06_make_cabbage_scene)에서
# 같은 자리·크기의 윗면이 열린 통 SortBin_1/2 로 바꿨다. 통이 없는 씬이면 SortBox 를 쓴다.
SORT_BOX_PATHS = ("/World/SmartFarm/RobotZone/SortBin_1", "/World/SmartFarm/RobotZone/SortBin_2")
SORT_BOX_FALLBACK = ("/World/SmartFarm/RobotZone/SortBox_1", "/World/SmartFarm/RobotZone/SortBox_2")
GUIDE_S_PATH = "/World/ConveyorGuide/GuideS"       # conveyor.py 의 남쪽 옆가이드

ARM_JOINTS = tuple(f"joint_{index}" for index in range(1, 7))
READY_JOINTS_DEG = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)          # cull_standalone 과 같음
GRIPPER_JOINTS = ("finger_joint", "right_inner_knuckle_joint")
GRIPPER_OPEN, GRIPPER_CLOSE = 0.0, 1.18
ARM_DRIVE = (1.0e8, 1.0e4, 1.0e8)                              # cull_standalone 과 같음
# 팀 기본 1e4 N·m 는 손가락이 포기를 관통한다. 8 N·m 에서 6칸 모두 파지 성공(05_m0609_pick_place_test).
GRIPPER_DRIVE = (1.0e5, 1.0e3, 8.0)
TOOL_Q = (0.0, 1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0), 0.0)   # cull_standalone 의 위에서 집기 자세
TCP_OFFSET = (0.0, 0.0, 0.19671)

CULL_CLASSES = ("lettuce_brown", "lettuce_yellow")
CLASS_SHORT = {"lettuce_dark_green": "green", "lettuce_yellow": "yellow", "lettuce_brown": "brown"}
CONF_MIN = 0.35
FRAMES = 3

# 트레이(원점) 기준 높이 — 양배추 에셋 build_info: 포기 원점 +45 mm, 파지 밴드 +35 mm
GRIP_ABOVE_TRAY = 0.078        # RG2 패드 중심을 밴드 중심 2 mm 아래
AIM_ABOVE_TRAY = 0.090         # 비전 픽셀을 바닥면과 만나게 할 높이 (포기 윗부분)
HEAD_TOP_ABOVE_TRAY = 0.105

TRAY_REACH = 0.48              # base -> 트레이 중심 수평거리 (dbg_reach: 0.48 이면 6칸 모두 접근·집기 IK 가능)
TRANSFER_Y_LIMIT = (-7.06, -6.60)   # 트레이 폭 0.252, 롤러 y -7.20 ~ -6.30 안

# 가로 이송 프레임 (로우폴리, install() 때 씬에 세운다 — 구조는 build_transfer_frame 설명 참고)
#   PlateN(주황) 이 트레이를 로봇 쪽으로 밀고, PlateS(빨강) 가 벨트 줄로 되민다. 평소에는 벨트 위에 올려 둔다.
PUSHER_ROOT = "/World/VisionTransfer"
STATION_X = -0.686             # 트레이가 서는 x (컨베이어 vision_x -0.69 + 관성)
LANE_Y = -6.73                 # 컨베이어 가로줄기 트레이 중심 y (옆가이드 사이)
TRAY_HALF_W = 0.126            # 트레이 반폭 (y)
TRAY_HALF_L = 0.276            # 트레이 반길이 (x)
FRAME_GAP = 0.012              # 판·팔 안쪽 면과 트레이 사이 여유
FRAME_T = 0.02                 # 판·팔 두께
FRAME_H = 0.06                 # 판·팔 높이
FRAME_Z_DOWN = 0.805           # 내린 높이(중심): 바닥 0.775 > 롤러 윗면 0.769, 윗면 0.835 < 포기 바닥 0.84
FRAME_Z_UP = 0.965             # 올린 높이(중심): 바닥 0.935 > 포기 꼭대기 0.90 -> 트레이가 아래로 지나감
COLUMN_Y = -6.02               # 기둥·캐리지 y (컨베이어 바깥쪽 끝 -6.175 과 방 북쪽 벽 -5.70 사이)
ROD_LEN = 0.90                 # 로드 길이 (N판에서 북쪽으로, 캐리지를 관통)
PUSH_SPEED = 0.08              # m/s (눈으로 보이게 천천히)
COLOR_N, COLOR_S, COLOR_ARM = (0.95, 0.55, 0.10), (0.85, 0.15, 0.10), (0.30, 0.32, 0.36)
COLOR_ROD, COLOR_BODY = (0.75, 0.76, 0.78), (0.20, 0.21, 0.24)
INSPECT_EYE = (0.0, -0.20, 0.36)    # 트레이 중심(포기 윗면 높이) 기준 카메라 위치: 팔 쪽 0.20 m, 위 0.36 m
DROP_ABOVE_BOX_TOP = 0.10      # SortBox 윗면 위 TCP 높이 (0.20 에서는 먼저 버린 포기에 튀어 상자 밖으로)
# 같은 상자에 여러 번 버릴 때 상자 긴 변(0.57 m)을 따라 자리를 바꾼다. 같은 자리면 먼저 버린 포기 위에 떨어져 굴러 나갔다.
DROP_SPREAD = (0.0, -0.15, 0.15, -0.075, 0.075)
VIA_RADIUS = 0.45              # 집은 자리 -> SortBox 사이에 거치는 base 둘레 중간점 반지름
CAMERA_RES = (640, 640)
SETTLE_FRAMES = 20

DEFAULT_WEIGHTS = r"C:\Users\kangm\Downloads\romaine3_v012_640sq_yolo11n_best.pt"
DEFAULT_PYTHON = r"D:\isaacsim\kit\python\python.exe"
DEFAULT_PYTHONPATH = (r"D:\smartfarm-sim\team_ref\pylib_yolo;"
                      r"D:\isaacsim\exts\omni.isaac.ml_archive\pip_prebundle;"
                      r"D:\smartfarm-sim\team_ref\pylib")


def _log(message):
    print(message, flush=True)


# ── 작은 수학 ───────────────────────────────────────────
def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw])


def _qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _qmat(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _mat_to_q(m):
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
    q = np.zeros(4)
    q[0] = (m[k, j] - m[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (m[j, i] + m[i, j]) / s
    q[1 + k] = (m[k, i] + m[i, k]) / s
    return q / np.linalg.norm(q)


def _world_matrix(stage, path):
    m = Gf.Matrix4d(UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    return np.array(m.RemoveScaleShear()).T          # 열벡터 규약


# ── 설치 ────────────────────────────────────────────────
def install(stage, world, m0609_dir, out_dir=None):
    """world.reset() 전에 부른다. 팔·그리퍼 드라이브를 세션 값으로 바꾸고 articulation 을 등록한다."""
    for name in ARM_JOINTS:
        drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{ROBOT_PATH}/joints/{name}"), "angular")
        drive.GetStiffnessAttr().Set(ARM_DRIVE[0])
        drive.GetDampingAttr().Set(ARM_DRIVE[1])
        drive.GetMaxForceAttr().Set(ARM_DRIVE[2])
    finger = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{GRIPPER_ROOT_PATH}/joints/finger_joint"), "angular")
    finger.GetStiffnessAttr().Set(GRIPPER_DRIVE[0])
    finger.GetDampingAttr().Set(GRIPPER_DRIVE[1])
    finger.GetMaxForceAttr().Set(GRIPPER_DRIVE[2])
    base_y = _world_matrix(stage, BASE_PATH)[1, 3]
    target_y = float(np.clip(base_y + TRAY_REACH, *TRANSFER_Y_LIMIT))
    build_transfer_frame(stage)
    return VisionCullStation(stage, world, Path(m0609_dir), out_dir, target_y)


def _box(stage, path, size, offset, color, collide=False):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*offset))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def build_transfer_frame(stage):
    """가로 이송 프레임 (로우폴리 박스). world.reset() 전에 한 번.

    Frame (kinematic 강체) = 트레이를 둘러싸는 사각 틀
        PlateN (주황) : 트레이 북쪽 면을 밀어 로봇 쪽으로 보냄
        PlateS (빨강) : 트레이 남쪽 면을 밀어 벨트 줄로 되돌림 (검사·솎아내기 중에는 멈춤판)
        ArmW / ArmE   : 두 판을 잇는 양 끝 팔
        Rod0 / Rod1   : PlateN 에서 북쪽 캐리지로 가는 로드 (보이기만)
    Carriage (보이기만) : 컨베이어 바깥 두 기둥 사이 가로보. 프레임과 같이 오르내린다.
    Column0 / Column1  : 고정 기둥 (y COLUMN_Y, 컨베이어 바깥)

    치수는 컨베이어를 뚫지 않게 잡았다: 판 바닥 0.775 > 롤러 윗면 0.769, 판은 양 레일(y -7.19 / -6.31) 안쪽에서만
    움직이고, 로드 바닥 0.835 > 레일 윗면 0.798, 기둥은 컨베이어 바깥 끝(-6.175)보다 북쪽.
    """
    inner_y = TRAY_HALF_W + FRAME_GAP                      # 판 안쪽 면까지 (트레이 중심 기준)
    inner_x = TRAY_HALF_L + FRAME_GAP
    plate_len = 2 * (inner_x + FRAME_T)
    stage.DefinePrim(PUSHER_ROOT, "Xform")
    frame = UsdGeom.Xform.Define(stage, f"{PUSHER_ROOT}/Frame")
    frame.ClearXformOpOrder()
    frame.AddTranslateOp().Set(Gf.Vec3d(STATION_X, LANE_Y, FRAME_Z_UP))
    UsdPhysics.RigidBodyAPI.Apply(frame.GetPrim()).CreateKinematicEnabledAttr().Set(True)
    UsdPhysics.MassAPI.Apply(frame.GetPrim()).CreateMassAttr().Set(8.0)
    f = f"{PUSHER_ROOT}/Frame"
    _box(stage, f"{f}/PlateN", (plate_len, FRAME_T, FRAME_H), (0, inner_y + FRAME_T / 2, 0), COLOR_N, collide=True)
    _box(stage, f"{f}/PlateS", (plate_len, FRAME_T, FRAME_H), (0, -inner_y - FRAME_T / 2, 0), COLOR_S, collide=True)
    for name, sx in (("ArmW", -1), ("ArmE", 1)):
        _box(stage, f"{f}/{name}", (FRAME_T, 2 * inner_y, FRAME_H), (sx * (inner_x + FRAME_T / 2), 0, 0), COLOR_ARM,
             collide=True)
    rod_y0 = inner_y + FRAME_T                             # PlateN 바깥 면
    for i, dx in enumerate((-0.18, 0.18)):
        _box(stage, f"{f}/Rod{i}", (0.03, ROD_LEN, 0.03), (dx, rod_y0 + ROD_LEN / 2, FRAME_H / 2 + 0.015), COLOR_ROD)
    carriage = UsdGeom.Xform.Define(stage, f"{PUSHER_ROOT}/Carriage")
    carriage.ClearXformOpOrder()
    carriage.AddTranslateOp().Set(Gf.Vec3d(STATION_X, COLUMN_Y, FRAME_Z_UP))
    _box(stage, f"{PUSHER_ROOT}/Carriage/Beam", (0.60, 0.08, 0.08), (0, 0, FRAME_H / 2 + 0.015), COLOR_BODY)
    for i, dx in enumerate((-0.33, 0.33)):
        _box(stage, f"{PUSHER_ROOT}/Column{i}", (0.06, 0.06, 1.20), (STATION_X + dx, COLUMN_Y, 0.60), COLOR_BODY)


class _YoloClient:
    """inspection_yolo_worker.py 하위 프로세스. 시작은 바로, 준비 확인은 처음 쓸 때."""

    def __init__(self, out_dir):
        weights = os.environ.get("SMARTFARM_YOLO_WEIGHTS", DEFAULT_WEIGHTS)
        python = os.environ.get("SMARTFARM_YOLO_PYTHON", DEFAULT_PYTHON)
        env = dict(os.environ)
        env["PYTHONPATH"] = os.environ.get("SMARTFARM_YOLO_PYTHONPATH", DEFAULT_PYTHONPATH)
        env.pop("PYTHONHOME", None)
        worker = Path(__file__).with_name("inspection_yolo_worker.py")
        self.weights = weights
        self.names = None
        self.error = None
        self._stderr = open(Path(out_dir) / "inspection_yolo_worker.err", "w") if out_dir else subprocess.DEVNULL
        try:
            self._proc = subprocess.Popen([python, str(worker), weights, str(CONF_MIN)], stdin=subprocess.PIPE,
                                          stdout=subprocess.PIPE, stderr=self._stderr, text=True, env=env, bufsize=1)
        except OSError as error:
            self._proc, self.error = None, f"YOLO 워커 실행 실패: {error}"
        _log(f"[비전] YOLO 워커 시작: {weights}")

    def _read(self):
        while True:
            line = self._proc.stdout.readline()
            if not line:
                raise RuntimeError("YOLO 워커가 종료되었습니다 (inspection_yolo_worker.err 확인)")
            line = line.strip()
            if line.startswith("{"):
                return json.loads(line)

    def ready(self):
        if self.names is None and self.error is None:
            try:
                self.names = self._read()["names"]
                _log(f"[비전] YOLO 준비 완료 — 클래스 {self.names}")
            except Exception as error:  # noqa: BLE001
                self.error = str(error)
        if self.error:
            raise RuntimeError(self.error)

    def detect(self, npy, annotated=None, raw=None):
        self.ready()
        self._proc.stdin.write(json.dumps({"npy": str(npy), "annotated": annotated, "raw": raw}) + "\n")
        self._proc.stdin.flush()
        return self._read()["detections"]

    def close(self):
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()


class VisionCullStation:
    def __init__(self, stage, world, m0609_dir, out_dir=None, target_y=-7.0):
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.robot.manipulators.grippers import ParallelGripper

        self._target_y = target_y      # 이송 프레임이 트레이 중심을 밀어 놓는 y
        self._stage = stage
        self._world = world
        self._m0609_dir = m0609_dir
        self._out = Path(out_dir or os.environ.get("SMARTFARM_STATION_OUT", "") or Path.cwd() / "station_out")
        self._out.mkdir(parents=True, exist_ok=True)
        self.robot = world.scene.add(SingleArticulation(prim_path=ROBOT_PATH, name="vision_m0609"))
        self.gripper = ParallelGripper(
            end_effector_prim_path=EE_PATH, joint_prim_names=list(GRIPPER_JOINTS),
            joint_opened_positions=np.array([GRIPPER_OPEN] * 2), joint_closed_positions=np.array([GRIPPER_CLOSE] * 2),
            action_deltas=None)
        self._yolo = _YoloClient(self._out)
        self._rgb = None
        self._conveyor = None
        self._drop_index = 0          # SortBox_1 / SortBox_2 번갈아
        self.results = []
        self.events = []
        self._clock = 0.0             # update() 로 누적한 물리 시간 (aio_wrapper 캡처 시각과 같은 기준)
        self.reset()

    # ── 생명주기 ──
    def _event(self, name):
        """시뮬레이션 시각과 함께 공정 이벤트를 남긴다 (영상 구간 자르기·보고용)."""
        self.events.append({"t": round(self._clock, 2), "event": name,
                            "pallet": self._record["pallet"] if self._record else None})
        with open(self._out / "station_events.json", "w", encoding="utf-8") as f:
            json.dump(self.events, f, ensure_ascii=False, indent=1)

    def reset(self, keep_frame=False):
        if getattr(self, "_tray", None) is not None:
            self._set_guide(True)          # Stop 으로 끊겨도 옆가이드는 원래대로
        if not keep_frame and self._stage.GetPrimAtPath(f"{PUSHER_ROOT}/Frame"):
            self._set_frame((STATION_X, LANE_Y, FRAME_Z_UP))   # 대기: 벨트 줄 위에 올려 둔 자리
        self._moves = []
        self._move_key = None
        self._tray = None
        self.state = "IDLE"
        self._pallet = None
        self._timer = 0
        self._motion = None
        self._queue = []
        self._frames = []
        self._record = None
        self._via = None

    def attach(self, conveyor):
        """world.reset() 뒤. 팔을 준비 자세로 두고 RMPflow 를 만든다."""
        import omni.replicator.core as rep
        from isaacsim.core.utils.xforms import get_world_pose

        if str(self._m0609_dir / "rmpflow") not in sys.path:
            sys.path.insert(0, str(self._m0609_dir / "rmpflow"))
        from m0609_rmpflow_controller import RMPFlowController

        self._conveyor = conveyor
        self._get_world_pose = get_world_pose
        self.robot.initialize(physics_sim_view=self._world.physics_sim_view)
        self.gripper.initialize(
            physics_sim_view=self._world.physics_sim_view, articulation_apply_action_func=self.robot.apply_action,
            get_joint_positions_func=self.robot.get_joint_positions, set_joint_positions_func=self.robot.set_joint_positions,
            dof_names=self.robot.dof_names)
        self.gripper.set_default_state(np.array([GRIPPER_OPEN] * 2))
        q = np.zeros(self.robot.num_dof)
        for name, deg in zip(ARM_JOINTS, READY_JOINTS_DEG):
            q[self.robot.get_dof_index(name)] = np.deg2rad(deg)
        self.robot.set_joints_default_state(positions=q, velocities=np.zeros(self.robot.num_dof))
        self.robot.set_joint_positions(q)
        self.robot.set_joint_velocities(np.zeros(self.robot.num_dof))
        self._controller = RMPFlowController(
            name="vision_m0609_rmpflow", robot_articulation=self.robot, physics_dt=1.0 / 60.0,
            urdf_path=str(self._m0609_dir / "doosan-robot2/urdf/m0609_isaac_sim.urdf"),
            robot_description_path=str(self._m0609_dir / "rmpflow/m0609_description.yaml"),
            rmpflow_config_path=str(self._m0609_dir / "rmpflow/m0609_rmpflow_common.yaml"),
            end_effector_frame_name="link_6")
        if self._rgb is None:
            product = rep.create.render_product(CAMERA_PATH, CAMERA_RES)
            self._rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            self._rgb.attach([product])
        base_p, base_q = self._base_pose()
        self._base_p, self._base_q = base_p, base_q
        self._boxes = []
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        self._box_paths = SORT_BOX_PATHS if self._stage.GetPrimAtPath(SORT_BOX_PATHS[0]) else SORT_BOX_FALLBACK
        for path in self._box_paths:
            rng = cache.ComputeWorldBound(self._stage.GetPrimAtPath(path)).ComputeAlignedRange()
            lo, hi = np.array(rng.GetMin()), np.array(rng.GetMax())
            drop = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, hi[2] + DROP_ABOVE_BOX_TOP])
            self._boxes.append((path.rsplit("/", 1)[-1], drop, hi[2]))
        _log(f"[솎아내기] 비전 M0609 base ({base_p[0]:+.3f}, {base_p[1]:+.3f}, {base_p[2]:.3f}) · "
             + " · ".join(f"{n} 투하점 ({d[0]:+.3f}, {d[1]:+.3f}) 수평 {np.hypot(*(d[:2] - base_p[:2])):.2f} m"
                          for n, d, _ in self._boxes))
        self.reset()

    def close(self):
        self._yolo.close()

    # ── 조회 ──
    def _base_pose(self):
        p, q = self._get_world_pose(BASE_PATH)
        return np.asarray(p, float), np.asarray(q, float)

    def to_base(self, world_point):
        return _qmat(self._base_q).T @ (np.asarray(world_point, float) - self._base_p)

    @property
    def busy(self):
        return self.state != "IDLE"

    # ── 매 스텝 ──
    def update(self, dt, render=None):
        """render: 카메라 프레임을 새로 그리게 하는 함수 (world.render). 없으면 world.render."""
        self._render = render or self._world.render
        self._clock += dt
        try:
            getattr(self, f"_state_{self.state.lower()}")(dt)
        except Exception as error:  # noqa: BLE001 - 스테이션 실패가 올인원을 죽이지 않게
            _log(f"[솎아내기] 실패 ({self.state}): {error} — 팔레트를 벨트로 돌려보냅니다")
            if self._record is not None:
                self._record["error"] = f"{self.state}: {error}"
            self._motion = None
            if self.state == "PUSH_OUT":          # 되돌리기도 실패하면 그 자리에서 풀어 준다
                self._set_guide(True)
                self._conveyor.inspection_done()
                self.reset()
                return
            self._begin_transfer_out()

    def _state_idle(self, dt):
        if self._conveyor is None:
            return
        path = self._conveyor.inspecting
        if path is None:
            return
        self._pallet = path
        root = self._stage.GetPrimAtPath(path)
        bodies = [p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))
                  if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        depth = min(len(str(p.GetPath()).split("/")) for p in bodies)
        self._tray = [p for p in bodies if len(str(p.GetPath()).split("/")) == depth][0]
        self._heads = sorted((p for p in bodies if p is not self._tray and p.GetName().startswith("Cabbage_")),
                             key=lambda p: p.GetName())
        pallet = self._conveyor._find(path)
        self._rigid = pallet.rigid
        body_paths = list(pallet.body_paths)
        self._tray_index = body_paths.index(str(self._tray.GetPath()))
        self._head_index = {p.rsplit("/", 1)[-1]: i for i, p in enumerate(body_paths) if i != self._tray_index}
        positions, _ = self._rigid.get_world_poses()
        positions = np.asarray(positions)
        tray_p = positions[self._tray_index]
        self._lane_y = float(tray_p[1])
        self._seat_offsets = {name: positions[i] - tray_p for name, i in self._head_index.items()}
        self._record = {"pallet": path.rsplit("/", 1)[-1], "stop_xy": [round(float(tray_p[0]), 3), round(float(tray_p[1]), 3)],
                        "sim_start": time.time()}
        _log(f"[솎아내기] {self._record['pallet']} 도착 ({tray_p[0]:+.3f}, {tray_p[1]:+.3f}) — 이송 프레임이 내려와 "
             f"로봇 앞으로 {(self._target_y - tray_p[1]) * 1000:+.0f} mm 밀어 줍니다 (트레이 중심 수평거리 "
             f"{np.hypot(tray_p[0] - self._base_p[0], self._target_y - self._base_p[1]):.2f} m)")
        # 컨베이어 lock() 은 트레이·포기의 시뮬레이션을 끈다. 다시 켜서(가로줄기는 멈춰 있음) 프레임이 물리로 민다.
        # 민 자리의 트레이·먼 줄 포기가 보이지 않는 남쪽 옆가이드(y -6.95~-6.93)와 겹쳐 가이드가 포기를 칸에서
        # 밀어냈었다 — 검사 중에만 그 가이드 충돌을 끈다.
        self._event("ARRIVED_PUSH_START")
        self._set_guide(False)
        self._rigid.enable_rigid_body_physics()
        self._rigid.set_velocities(np.zeros((len(body_paths), 6), dtype=np.float32))
        x, y = float(tray_p[0]), float(tray_p[1])
        self._frame_x = x
        self._moves = [(x, y, FRAME_Z_UP),              # 올린 채 트레이 바로 위로 맞춤
                       (x, y, FRAME_Z_DOWN),            # 트레이를 감싸도록 내림 (판·팔은 트레이 바깥)
                       (x, self._target_y - FRAME_GAP, FRAME_Z_DOWN)]   # PlateN 이 밀어 트레이 중심 = 목표 y
        self._timer = 0
        self.state = "PUSH_IN"

    def _set_guide(self, on):
        prim = self._stage.GetPrimAtPath(GUIDE_S_PATH)
        if prim and prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(bool(on))

    # ── 이송 프레임 ──
    def _frame_op(self):
        op = UsdGeom.Xformable(self._stage.GetPrimAtPath(f"{PUSHER_ROOT}/Frame")).GetOrderedXformOps()[0]
        return op, op.Get()

    def _set_frame(self, xyz):
        """프레임(kinematic 목표)과 캐리지(높이만 따라감)를 옮긴다."""
        op, cur = self._frame_op()
        op.Set(type(cur)(*map(float, xyz)))
        c_op = UsdGeom.Xformable(self._stage.GetPrimAtPath(f"{PUSHER_ROOT}/Carriage")).GetOrderedXformOps()[0]
        c = c_op.Get()
        c_op.Set(type(c)(float(xyz[0]), c[1], float(xyz[2])))

    def _step_moves(self, moves, dt):
        """moves 앞에서부터 하나씩 프레임을 목표 (x, y, z) 로 부드럽게 옮긴다. 다 끝나면 True."""
        if not moves:
            return True
        goal = np.array(moves[0], dtype=float)
        if getattr(self, "_move_key", None) != tuple(goal):
            self._move_key, self._move_from, self._move_t = tuple(goal), np.array(self._frame_op()[1], float), 0.0
        distance = float(np.linalg.norm(goal - self._move_from))
        self._move_t = min(1.0, self._move_t + dt * PUSH_SPEED / max(distance, 1e-3))
        s = self._move_t * self._move_t * (3 - 2 * self._move_t)
        self._set_frame(self._move_from + (goal - self._move_from) * s)
        if self._move_t >= 1.0:
            moves.pop(0)
            self._move_key = None
        return not moves

    def _state_push_in(self, dt):
        if not self._step_moves(self._moves, dt):
            return
        self._timer += 1
        if self._timer < 20:                  # 트레이·포기가 멈출 때까지
            return
        positions, _ = self._rigid.get_world_poses()
        positions = np.asarray(positions)
        tray_p = positions[self._tray_index]
        seat = max(float(np.linalg.norm(positions[i] - tray_p - self._seat_offsets[n]))
                   for n, i in self._head_index.items()) * 1000
        self._record["pushed_xy"] = [round(float(tray_p[0]), 3), round(float(tray_p[1]), 3)]
        self._record["seat_error_after_push_mm"] = round(seat, 1)
        _log(f"[솎아내기] 프레임 이송 완료 — 트레이 ({tray_p[0]:+.3f}, {tray_p[1]:+.3f}), 포기 칸 이탈 최대 {seat:.1f} mm")
        self._event("PUSH_DONE_INSPECT_MOVE")
        self._timer = 0
        self._start_inspect_move()
        self.state = "MOVE_INSPECT"

    # ── 팔 이동 (팀 CullMotion 을 계획만 바꿔 씀) ──
    def _new_motion(self, tool_q_world, target_path=None):
        from cull_motion import CullMotion, CullPickConfig

        tool_q_base = _qmul(_qconj(self._base_q), tool_q_world)
        get_target = (lambda: self._get_world_pose(target_path)) if target_path else None
        return CullMotion(
            robot=self.robot, gripper=self.gripper, arm_controller=self._controller,
            get_end_effector_world_pose=lambda: self._get_world_pose(EE_PATH),
            get_base_world_pose=self._base_pose, get_target_world_pose=get_target,
            config=CullPickConfig(approach_clearance=0.18, lift_clearance=0.22, tcp_offset_local=TCP_OFFSET,
                                  tool_orientation_base=tuple(tool_q_base / np.linalg.norm(tool_q_base)),
                                  min_target_rise=0.05))

    def _run_plan(self, motion, plan):
        from cull_motion import CullStep  # noqa: F401 - 계획 원소 형식

        self._controller.reset()
        motion.start_pick(plan[1].position_base if plan[0].position_base is None else plan[0].position_base)
        motion._plan = tuple(plan)       # 팀 CullMotion 은 Pick 계획만 만든다. 같은 실행기에 전체 계획을 넣는다.
        self._motion = motion

    def _inspect_pose(self):
        tray = _world_matrix(self._stage, str(self._tray.GetPath()))
        centre = tray[:3, 3] + np.array([0.0, 0.0, HEAD_TOP_ABOVE_TRAY])
        away = centre[:2] - self._base_p[:2]
        away /= np.linalg.norm(away)
        eye = centre.copy()
        eye[:2] += away * INSPECT_EYE[1]      # 음수 = 팔 쪽
        eye[2] += INSPECT_EYE[2]
        view = centre - eye
        view /= np.linalg.norm(view)
        up = np.array([away[0], away[1], 0.0])
        up = up - view * (up @ view)
        up /= np.linalg.norm(up)
        # 카메라는 link_6 의 +Z 를 보고, 카메라 +Y = link_6 +Y, 카메라 +X = link_6 -X (dbg_camera_mount)
        z6, y6 = view, up
        x6 = np.cross(y6, z6)
        r6 = np.column_stack([x6, y6, z6])
        cam_in_link6 = np.array([-0.0115, 0.045, 0.0525])
        flange = eye - r6 @ cam_in_link6
        tcp = flange + r6 @ np.array(TCP_OFFSET)
        return tcp, _mat_to_q(r6), eye

    def _start_inspect_move(self, via=None):
        tcp, q_world, eye = self._inspect_pose()
        from cull_motion import CullStep

        motion = self._new_motion(q_world)
        steps = ([CullStep("VIA_BACK", tuple(via))] if via is not None else []) + \
            [CullStep("INSPECT_MOVE", tuple(self.to_base(tcp)))]
        self._run_plan(motion, tuple(steps))
        _log(f"[비전] 검사 자세로 이동 — 카메라 ({eye[0]:+.3f}, {eye[1]:+.3f}, {eye[2]:.3f})")

    def _motion_step(self):
        """True = 끝남. 실패는 예외 (관절각·TCP 를 같이 남긴다)."""
        try:
            self._motion.update()
            if self._motion.is_failed:
                raise RuntimeError(self._motion.error)
        except Exception:
            joints = np.rad2deg(self.robot.get_joint_positions()[:6]).round(1).tolist()
            ee_p, ee_q = self._get_world_pose(EE_PATH)
            tcp = np.asarray(ee_p) + _qmat(ee_q) @ np.array(TCP_OFFSET)
            _log(f"[솎아내기:진단] 단계 {self._motion.current_stage} 관절(deg) {joints} TCP 월드 {tcp.round(3).tolist()}")
            raise
        return self._motion.is_done

    def _state_move_inspect(self, dt):
        if not self._motion_step():
            return
        self._timer += 1
        if self._timer < SETTLE_FRAMES:
            return
        self._frames = []
        self._timer = 0
        self.state = "CAPTURE"

    # ── 촬영·판정 ──
    def _camera_model(self):
        cam = UsdGeom.Camera(self._stage.GetPrimAtPath(CAMERA_PATH))
        fx = CAMERA_RES[0] * cam.GetFocalLengthAttr().Get() / cam.GetHorizontalApertureAttr().Get()
        return fx, CAMERA_RES[0] / 2.0, CAMERA_RES[1] / 2.0, _world_matrix(self._stage, CAMERA_PATH)

    def _project(self, model, point):
        fx, cx, cy, m = model
        p = m[:3, :3].T @ (np.asarray(point, float) - m[:3, 3])
        if p[2] >= -1e-6:
            return None
        return np.array([cx + fx * p[0] / -p[2], cy - fx * p[1] / -p[2]])

    def _ray_plane(self, model, u, v, plane_z):
        fx, cx, cy, m = model
        d = m[:3, :3] @ np.array([(u - cx) / fx, -(v - cy) / fx, -1.0])
        eye = m[:3, 3]
        t = (plane_z - eye[2]) / d[2]
        return eye + t * d

    def _state_capture(self, dt):
        self._timer += 1
        if self._timer < 4:           # 렌더 몇 번 돌려 새 자세의 프레임을 받는다
            self._render()
            return
        self._render()
        frame = self._rgb.get_data()
        if frame is None or getattr(frame, "size", 0) == 0:
            self._timer = 0
            return
        index = len(self._frames)
        name = f"{self._record['pallet']}_{'recheck_' if self.state == 'RECHECK' else ''}{index}"
        npy = self._out / f"{name}.npy"
        np.save(npy, np.asarray(frame))
        detections = self._yolo.detect(npy, annotated=str(self._out / f"{name}_yolo.jpg"),
                                       raw=str(self._out / f"{name}.png"))
        npy.unlink(missing_ok=True)
        self._frames.append((self._camera_model(), detections))
        self._timer = 0
        if len(self._frames) < FRAMES:
            return
        self._judge()

    def _truth(self, head):
        vs = head.GetVariantSets()
        return vs.GetVariantSelection("condition") if vs.HasVariantSet("condition") else "?"

    def _assign(self):
        """프레임마다 검출을 가장 가까운 포기 투영점에 배정하고, 칸별 다수결."""
        tray_z = _world_matrix(self._stage, str(self._tray.GetPath()))[2, 3]
        per_head = {h.GetName(): [] for h in self._heads}
        aims = {h.GetName(): [] for h in self._heads}
        for model, detections in self._frames:
            centres = {}
            for head in self._heads:
                hp = _world_matrix(self._stage, str(head.GetPath()))[:3, 3]
                if hp[2] < tray_z:                 # 이미 버린 포기
                    continue
                uv = self._project(model, hp + np.array([0, 0, 0.035]))
                if uv is not None:
                    centres[head.GetName()] = uv
            best = {}
            for det in detections:
                if det["conf"] < CONF_MIN:
                    continue
                x0, y0, x1, y1 = det["xyxy"]
                uv = np.array([(x0 + x1) / 2, (y0 + y1) / 2])
                if not centres:
                    break
                name, dist = min(((n, float(np.linalg.norm(uv - c))) for n, c in centres.items()), key=lambda t: t[1])
                if dist > max(x1 - x0, y1 - y0):   # 박스 크기보다 멀면 다른 물체
                    continue
                if name not in best or det["conf"] > best[name][0]["conf"]:
                    best[name] = (det, uv)
            for name, (det, uv) in best.items():
                per_head[name].append(det["class_name"])
                aims[name].append(self._ray_plane(model, uv[0], uv[1], tray_z + AIM_ABOVE_TRAY))
        labels = {}
        for name, votes in per_head.items():
            if not votes or len(votes) * 2 <= len(self._frames):
                labels[name] = ("UNKNOWN", None)
                continue
            top = max(set(votes), key=votes.count)
            labels[name] = (top if votes.count(top) * 2 > len(votes) else "UNKNOWN",
                            np.mean(aims[name], axis=0) if aims[name] else None)
        return labels

    def _judge(self):
        labels = self._assign()
        tray_z = _world_matrix(self._stage, str(self._tray.GetPath()))[2, 3]
        rows = []
        for head in self._heads:
            name = head.GetName()
            label, _ = labels.get(name, ("UNKNOWN", None))
            removed = _world_matrix(self._stage, str(head.GetPath()))[2, 3] < tray_z
            rows.append({"slot": "SLOT_" + name.split("_")[-1], "head": name,
                         "label": "removed" if removed else CLASS_SHORT.get(label, label), "truth": self._truth(head)})
        tag = "재검사" if self.state == "RECHECK" else "판정"
        self._event("RECHECK_DONE" if tag == "재검사" else "INSPECTION_DONE")
        _log(f"[비전] {self._record['pallet']} {tag}: " + " ".join(
            f"{r['slot'][-2:]}={r['label']}{'' if r['label'] in (r['truth'], 'removed') else '(정답 ' + r['truth'] + ')'}"
            for r in rows))
        self._record.setdefault("images", []).extend(
            sorted(p.name for p in self._out.glob(f"{self._record['pallet']}_{'recheck_' if tag == '재검사' else ''}?_yolo.jpg")))
        if self.state == "RECHECK":
            self._record["recheck"] = rows
            self._begin_transfer_out()
            return
        self._record["inspection"] = rows
        self._queue = []
        for head in self._heads:
            label, aim = labels.get(head.GetName(), ("UNKNOWN", None))
            if label in CULL_CLASSES and aim is not None:
                self._queue.append((head, aim, CLASS_SHORT[label]))
        self._record["culls"] = []
        _log(f"[솎아내기] 대상 {len(self._queue)}개: " + ", ".join(
            f"SLOT_{h.GetName()[-2:]}({c})" for h, _, c in self._queue))
        self._next_cull()

    # ── 솎아내기 ──
    @staticmethod
    def _via_point(a_base, b_base, z):
        a0 = math.atan2(a_base[1], a_base[0])
        a1 = math.atan2(b_base[1], b_base[0])
        delta = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
        mid = a0 + delta / 2
        return np.array([VIA_RADIUS * math.cos(mid), VIA_RADIUS * math.sin(mid), z])

    def _next_cull(self):
        from cull_motion import CullConfig, CullStep, build_cull_plan

        if not self._queue:
            self._start_recheck()
            return
        head, vision_aim, colour = self._queue.pop(0)
        tray_m = _world_matrix(self._stage, str(self._tray.GetPath()))
        tray_z = tray_m[2, 3]
        truth = _world_matrix(self._stage, str(head.GetPath()))[:3, 3]
        # 집는 좌표 = 트레이 자세(컨베이어·이송기가 아는 값) + 6구 칸 배치. 비전은 '어느 칸' 을 정하고 좌표는 대조만 한다.
        # 비전 좌표를 그대로 쓰면 오차 2~10 mm 에서 RG2 여유(한쪽 10.8 mm)를 넘겨 들다 놓친 적이 있다.
        aim = tray_m[:3, 3] + self._seat_offsets[head.GetName()]
        aim[2] = vision_aim[2]
        box_index = self._drop_index % len(self._boxes)
        box_name, drop, box_top = self._boxes[box_index]
        use = self._drop_index // len(self._boxes)
        self._drop_index += 1
        drop = drop + np.array([0.0, DROP_SPREAD[use % len(DROP_SPREAD)], 0.0])   # 상자 긴 변 = y
        aim_base = self.to_base(aim)
        drop_base = self.to_base(drop)
        pick_z_offset = float((tray_z + GRIP_ABOVE_TRAY) - aim[2])
        plan = list(build_cull_plan(tuple(aim_base), CullConfig(
            place_position_base=tuple(drop_base), approach_clearance=0.18, transit_clearance=0.18,
            pick_z_offset=pick_z_offset)))
        # 팀 계획은 LIFT -> PLACE_APPROACH 가 직선이라 로봇 base 바로 위를 지난다. 그때 팔이 꺾이며 포기를
        # 떨어뜨렸다(받침대 앞 바닥). base 둘레 반지름 VIA_RADIUS 의 중간점을 한 번 거친다.
        lift_z = plan[4].position_base[2]
        self._via = self._via_point(aim_base, drop_base, lift_z)
        plan.insert(5, CullStep("VIA", tuple(self._via)))
        motion = self._new_motion(_qmul(self._base_q, np.array(TOOL_Q)), str(head.GetPath()))
        self._run_plan(motion, plan)
        err = np.linalg.norm(vision_aim[:2] - truth[:2]) * 1000
        self._current = {"slot": "SLOT_" + head.GetName()[-2:], "colour": colour, "box": box_name,
                         "vision_xy_error_mm": round(float(err), 1),
                         "pick_xy_error_mm": round(float(np.linalg.norm(aim[:2] - truth[:2]) * 1000), 1),
                         "head": head, "box_top": box_top,
                         "box_path": self._box_paths[box_index]}
        _log(f"[솎아내기] SLOT_{head.GetName()[-2:]} ({colour}) -> {box_name} · 비전 좌표 대조 {err:.1f} mm")
        self.state = "CULL"

    def _state_cull(self, dt):
        stage = self._motion.current_stage
        if stage != self._current.get("_stage"):
            self._current["_stage"] = stage
            ee_p, ee_q = self._get_world_pose(EE_PATH)
            tcp = np.asarray(ee_p) + _qmat(ee_q) @ np.array(TCP_OFFSET)
            hp = _world_matrix(self._stage, str(self._current["head"].GetPath()))[:3, 3]
            self._current.setdefault("trace", []).append(
                {"stage": stage, "head_to_tcp_mm": round(float(np.linalg.norm(hp - tcp)) * 1000, 1),
                 "finger": round(float(self.robot.get_joint_positions()[self.robot.get_dof_index("finger_joint")]), 3)})
        if not self._motion_step():
            return
        self._timer += 1
        if self._timer < 45:                  # 떨어진 포기가 상자 안에 가라앉을 때까지
            return
        self._timer = 0
        cur = self._current
        hp = _world_matrix(self._stage, str(cur["head"].GetPath()))[:3, 3]
        rng = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]) \
            .ComputeWorldBound(self._stage.GetPrimAtPath(cur["box_path"])).ComputeAlignedRange()
        lo, hi = rng.GetMin(), rng.GetMax()
        inside = lo[0] <= hp[0] <= hi[0] and lo[1] <= hp[1] <= hi[1] and hp[2] <= cur["box_top"]
        entry = {k: v for k, v in cur.items() if k not in ("head", "box_top", "box_path", "_stage")}
        entry["in_box"] = bool(inside)
        entry["head_final"] = [round(float(v), 3) for v in hp]
        self._record["culls"].append(entry)
        _log(f"[솎아내기] {cur['slot']} 버림 {'성공' if inside else '실패'} — {cur['box']} "
             f"({hp[0]:+.3f}, {hp[1]:+.3f}, {hp[2]:.3f})")
        self._event(f"CULL_DONE_{cur['slot']}_{cur['box']}")
        if not self._queue:
            self._start_recheck()
            return
        # 다음 포기는 첫 번째를 집었던 같은 자세(검사 자세)에서 시작한다. 올 때와 같은 중간점을 거친다.
        self._start_inspect_move(via=self._via)
        self.state = "HOME"

    def _state_home(self, dt):
        if not self._motion_step():
            return
        self._next_cull()

    def _start_recheck(self):
        self._start_inspect_move(via=getattr(self, "_via", None))
        self.state = "RECHECK_MOVE"

    def _state_recheck_move(self, dt):
        if not self._motion_step():
            return
        self._timer += 1
        if self._timer < SETTLE_FRAMES:
            return
        self._timer = 0
        self._frames = []
        self.state = "RECHECK"

    _state_recheck = _state_capture

    # ── 되돌려 보내기 ──
    def _begin_transfer_out(self):
        if self._tray is None or self._conveyor is None:
            self.reset()
            return
        # PlateS 가 트레이를 도착했던 줄로 되민다 (프레임 중심을 여유만큼 더 북쪽으로 -> 트레이 중심 = 줄 중심).
        # 그다음 프레임을 올려 트레이가 아래로 빠져나가게 한다.
        x = getattr(self, "_frame_x", STATION_X)
        self._moves = [(x, self._lane_y + FRAME_GAP, FRAME_Z_DOWN),
                       (x, self._lane_y + FRAME_GAP, FRAME_Z_UP),
                       (STATION_X, LANE_Y, FRAME_Z_UP)]
        self._timer = 0
        self.state = "PUSH_OUT"

    def _state_push_out(self, dt):
        if not self._step_moves(self._moves, dt):
            return
        self._timer += 1
        if self._timer < 10:
            return
        self._set_guide(True)                 # 트레이가 다시 가이드 사이로 돌아왔다
        positions, _ = self._rigid.get_world_poses()
        tray_p = np.asarray(positions)[self._tray_index]
        self._record["returned_xy"] = [round(float(tray_p[0]), 3), round(float(tray_p[1]), 3)]
        self._record["elapsed_wall_s"] = round(time.time() - self._record.pop("sim_start"), 1)
        self.results.append(self._record)
        with open(self._out / "station_results.json", "w", encoding="utf-8") as f:
            json.dump(self.results, f, ensure_ascii=False, indent=1)
        culled = [c["slot"] for c in self._record.get("culls", []) if c.get("in_box")]
        _log(f"[솎아내기] {self._record['pallet']} 완료 — 제거 {culled} · 벨트 줄 복귀 "
             f"({tray_p[0]:+.3f}, {tray_p[1]:+.3f}), 배출")
        self._event("PUSH_BACK_DONE_RELEASED")
        self._conveyor.inspection_done()
        self.reset(keep_frame=True)
