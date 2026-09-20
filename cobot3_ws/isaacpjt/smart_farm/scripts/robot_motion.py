"""
리프트로 높이 맞추기 → 팔레트 집기 → 인양 → 인출 → 놓기 : 월드 좌표 목표를 IK로 풀어 실행

동작은 두 묶음입니다.
  PICK_STAGES  : 집을 팔레트를 기준으로 한 좌표
  PLACE_STAGES : 놓을 자리를 기준으로 한 좌표
두 묶음이 같은 오프셋 표를 씁니다. 기준점만 다릅니다.

수정할 곳
  치수/여유 값      : 포크나 팔레트가 바뀔 때
  BASE_* 허용 범위  : AMR 도킹 오차를 어디까지 받아 줄지
  JOINT_SPEED_DEG_S : 관절 명령 속도

주의
  - IK는 충돌을 피하지 않습니다. 경로가 랙에 닿는지는 화면으로 확인하세요.
  - 관절 보간은 중간 경로의 완전한 직선을 보장하지 않습니다.
"""

from typing import NamedTuple, Optional

import numpy as np
from pxr import UsdPhysics

from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationAction

EE_FRAME = "link_6"
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
WHEEL_JOINT_NAMES = ["joint_wheel_left", "joint_wheel_right"]


# ── 포크 자세와 치수 ─────────────────────────────────────
# 포크가 수평이고 월드 -x 방향을 향하는 자세 (w, x, y, z).
#   link_6 로컬 +Z = 월드 -x  (포크가 뻗는 방향)
#   link_6 로컬 +Y = 월드 +z  (포크 판의 얇은 방향이 위아래)
FORK_QUAT = np.array([0.5, 0.5, -0.5, -0.5])

# 포크: link_6 원점에서 잰 거리 (충돌 박스 실측값)
# 주의: fork_tool prim 자체에 scale (1.5, 1, 1.2) 가 걸려 있습니다.
#       아래 값은 그 배율까지 반영한 '진짜' 치수입니다.
#       USD 에서 Cube 크기만 보고 적으면 갈래 길이가 49 mm 짧게 나옵니다.
FORK_TINE_TIP = 0.294          # 갈래 끝        (0.135 + 0.220/2) x 1.2
FORK_PLATE_FRONT = 0.030       # 판 앞면        (0.0125 + 0.025/2) x 1.2

# 팔레트: prim 원점에서 잰 거리 (simple_pallet.usd 깊이 0.30 기준 실측)
PALLET_FRONT = 0.150           # 앞면(로봇 쪽)
PALLET_POCKET_CENTER = 0.040   # 포크 틈의 가운데 높이


# ── 여유 값 (동작을 조정할 때 여기를 바꿉니다) ───────────
PLATE_CLEARANCE = 0.010        # 포크 판과 팔레트 앞면 사이 여유
APPROACH_GAP = 0.037           # 진입 직전, 갈래 끝과 앞면 사이
READY_GAP = 0.187              # 대기 위치, 갈래 끝과 앞면 사이
PALLET_LIFT = 0.060            # 인양 높이
ENTRY_RISE = 0.150             # 랙 앞에서 뜨는 높이 (HOME 과 작업 높이를 잇는 경유점)
                               # 더 키우면 높은 층에서 IK 자세가 뒤집힙니다
RETRACT_DISTANCE = 0.328       # 인출 거리 (팔레트 뒷면이 선반 앞 끝을 3cm 넘어섬)


# ── 위 값에서 계산되는 목표 (기준점 기준 좌표) ───────────
# 앞쪽(로봇 쪽) +x, 위 +z. 계산식이 그대로 의미입니다.
READY_X = PALLET_FRONT + FORK_TINE_TIP + READY_GAP
APPROACH_X = PALLET_FRONT + FORK_TINE_TIP + APPROACH_GAP
DOCK_X = PALLET_FRONT + FORK_PLATE_FRONT + PLATE_CLEARANCE
RETRACT_X = DOCK_X + RETRACT_DISTANCE

FORK_Z = PALLET_POCKET_CENTER
LIFTED_Z = FORK_Z + PALLET_LIFT
ENTRY_Z = FORK_Z + ENTRY_RISE

# 검사와 연결된 단계 이름은 상수로 둡니다. 오타로 검사가 빠지는 것을 막습니다.
STAGE_HOME = "HOME"
STAGE_PALLET_UP = "PALLET_UP"       # 이 단계 끝에서 '인양 확인'
STAGE_PALLET_DOWN = "PALLET_DOWN"   # 이 단계 끝에서 '안착 확인'

# 집기: 집을 팔레트가 기준
#
# ENTRY 가 맨 앞에 있는 이유:
#   HOME 은 팔을 세운 자세, READY 는 랙 앞 낮은 자세입니다. 둘을 바로 이으면
#   관절 보간이 큰 호를 그려서 포크가 랙을 쓸고 내려옵니다.
#   랙 앞 '높은 곳'을 한 번 거치면 그 뒤로는 앞에서 내려오므로 랙에 닿지 않습니다.
#   경유점의 앞뒤 위치를 RETRACT_X 로 잡은 이유: READY_X 는 팔 베이스에서 30 cm 밖에
#   안 떨어진 좁은 구역이라 높이를 더하면 IK 가 풀리지 않습니다. RETRACT_X 는
#   팔레트를 들고 오르내리는 자리라 이 높이대가 이미 검증돼 있습니다.
PICK_STAGES = [
    ("ENTRY",          [RETRACT_X,  0.0, ENTRY_Z]),  # 랙 앞 위쪽 (HOME 에서 여기로 먼저)
    ("READY",          [READY_X,    0.0, FORK_Z]),   # 수직으로 내려와 랙 앞에서 대기
    ("APPROACH",       [APPROACH_X, 0.0, FORK_Z]),   # 팔레트 앞까지 접근
    ("DOCK",           [DOCK_X,     0.0, FORK_Z]),   # 판이 앞면에 닿기 직전까지 삽입
    (STAGE_PALLET_UP,  [DOCK_X,     0.0, LIFTED_Z]), # 인양
    ("RETRACT",        [RETRACT_X,  0.0, LIFTED_Z]), # 인양 높이를 유지한 채 인출
]

# 놓기: 놓을 자리가 기준 (같은 오프셋을 그대로 씁니다)
PLACE_STAGES = [
    ("DESCEND",          [RETRACT_X, 0.0, LIFTED_Z]), # 놓을 높이의 통로 위치
    ("PLACE_IN",         [DOCK_X,    0.0, LIFTED_Z]), # 선반 안으로 삽입
    (STAGE_PALLET_DOWN,  [DOCK_X,    0.0, FORK_Z]),   # 내려서 선반에 안착
    ("FORK_OUT",         [RETRACT_X, 0.0, FORK_Z]),   # 빈 포크만 빼기
    ("EXIT",             [RETRACT_X, 0.0, ENTRY_Z]),  # 수직으로 올라간 뒤 HOME 으로 (들어올 때의 반대)
]

class Task(NamedTuple):
    pallet_path: str
    destination_shelf_top: Optional[float]


MAX_SEGMENT_M = 0.06           # 이보다 긴 구간은 잘라서 간다

# 시작할 때 먼저 지나가는 고정 자세. 매번 같은 곳에서 출발하게 합니다.
# joint_1 만 180도 = 팔을 세운 채 랙 쪽(월드 -x)을 보게 돌린 자세라, 장면 시작
# 자세에서 안전하게 갈 수 있고 첫 목표까지의 관절 변화도 작습니다.
# 팔 베이스가 돌아가 있으면 home_joints_deg() 가 그만큼 빼 줍니다.
# joint_2 는 0 으로 둡니다. 기울이면 팔이 '마스트 쪽'으로 눕습니다.
#   joint_2 = -20 일 때 손목과 마스트 사이 1 mm (실측) — 사실상 파고듭니다
#   joint_2 =   0 일 때 마스트 64 mm / 랙 68 mm 여유
# 이 리그는 자기충돌이 꺼져 있어 겹쳐도 시뮬레이터가 알려주지 않습니다. 눈으로 보거나
# 따로 계산해야 하므로, 여유가 넉넉한 값을 씁니다.
HOME_JOINTS_DEG = [180.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# ── 시간·검사 기준 ───────────────────────────────────────
JOINT_SPEED_DEG_S = 20.0       # 관절 명령 속도. 올리면 추종 오차가 커집니다
MIN_MOVE_SECONDS = 0.5         # 짧은 구간도 최소 이만큼은 씁니다
START_WAIT_SECONDS = 2.0

JOINT_REACHED_TOL_DEG = 1.0
JOINT_TRACKING_LIMIT_DEG = 8.0
HOLD_SECONDS = 0.3             # 목표에 닿은 뒤 이만큼 머물러야 '도달'로 봅니다
REACH_TIMEOUT_SECONDS = 5.0

IK_POSITION_TOL = 0.002        # m,   IK 결과와 목표의 거리
IK_ANGLE_TOL_DEG = 0.5         # deg, 포크 자세 오차
IK_JUMP_LIMIT_DEG = 20.0       # deg, 이웃 점 사이 관절 변화 (자세 뒤집힘 검출)
FIRST_MOVE_LIMIT_DEG = 150.0   # deg, HOME에서 첫 목표까지 허용하는 관절 변화

PALLET_PUSH_TOL = 0.015        # 인양 전에 이만큼 움직이면 중단
MIN_PALLET_RISE = 0.010        # 인양 후 실제 상승량 최소값
PALLET_SLIP_TOL = 0.030        # 운반 중 손목 기준 상대 위치 변화
MIN_PALLET_DROP = 0.010        # 안착 후 실제 하강량 최소값
PALLET_STAY_TOL = 0.020        # 포크를 뺄 때 팔레트가 따라 나오면 중단
LOG_INTERVAL_SECONDS = 1.0

# ── AMR 도킹 허용 범위 ───────────────────────────────────
# 여기는 '누가 봐도 잘못 선 경우'를 거르는 안전선입니다.
# 실제로 팔이 닿는지는 그 뒤 IK 가 판단합니다 (범위 안이어도 IK 가 거부할 수 있음).
#
# 기준을 월드 좌표가 아니라 '집을 팔레트'로 잡습니다.
# 그래야 AMR 이 어느 랙 앞에 서든 같은 값으로 검사할 수 있습니다.
# 검증된 자리는 팔레트 원점에서 앞으로 0.932 m, 옆으로 +0.018 m 입니다.
BASE_TO_PALLET_X = (0.89, 1.05)     # m,   앞뒤 (작을수록 팔레트에 가까움)
BASE_TO_PALLET_Y = (-0.21, 0.34)    # m,   좌우 어긋남
BASE_YAW_LIMIT_DEG = 25.0           # deg, 기준 방향에서 얼마나 틀어져도 되는가
CHASSIS_FACING_DEG = 90.0           # deg, 차체가 랙을 향하는 기준 방향
                                    #   0  = 앞면이 랙을 봄
                                    #   90 = 옆면이 랙을 봄 (v004 배치)
                                    # 팔 베이스가 아니라 '차체' 기준입니다. 팔 베이스는
                                    # 리그에 비스듬히 붙어 있을 수 있어 각도 판단에 못 씁니다.
BASE_HEIGHT_BAND = (-0.50, 0.15)    # m,   '집을 팔레트가 있는 선반 윗면' 대비 베이스 높이

BASE_STILL_TOL = 0.002              # m,   이보다 적게 움직이면 정지로 봅니다
BASE_STILL_SECONDS = 0.5            # 이만큼 계속 멈춰 있어야 시작합니다
BASE_WAIT_NOTICE_SECONDS = 3.0      # 대기 중 안내를 찍는 주기

# 관절 Drive 강성. USD 기본값은 약해서 팔레트를 들면 팔이 처집니다.
DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8

# 카터 바퀴를 지금 각도에 붙잡는 강성 (주차 브레이크).
WHEEL_BRAKE_STIFFNESS = 1e5


# ── 로봇 읽기·쓰기 ───────────────────────────────────────
def robot_indices(robot):
    return np.array([robot.get_dof_index(name) for name in JOINT_NAMES])


def read_joints_deg(robot, indices):
    """실제 관절 각도를 degree로 반환합니다."""
    return np.rad2deg(robot.get_joint_positions()[indices])


def command_joints_deg(robot, indices, target_deg):
    """관절 목표를 radian으로 바꿔 적용합니다."""
    robot.apply_action(
        ArticulationAction(
            joint_positions=np.deg2rad(target_deg),
            joint_indices=indices,
        )
    )


def setup_arm_drives(stage, arm_path):
    """팔 관절의 Drive를 강화합니다. 메모리 안의 장면만 바뀌고 USD 파일은 그대로입니다."""
    for name in JOINT_NAMES:
        drive = UsdPhysics.DriveAPI.Get(
            stage.GetPrimAtPath(f"{arm_path}/joints/{name}"), "angular"
        )
        drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
        drive.GetDampingAttr().Set(DRIVE_DAMPING)
        drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
    print(f"[Drive] {len(JOINT_NAMES)}개 강화")


def brake_wheels(stage, rig_path):
    """
    카터 바퀴를 지금 각도에 붙잡습니다 (주차 브레이크).

    팔이 팔레트를 밀고 당기면 카터가 굴러가고, 그러면 IK 의 기준점이 흔들립니다.
    이 스크립트는 주행을 하지 않으므로 시작 각도(0)에 그대로 묶어 둡니다.
    주행까지 하는 스크립트에서는 팀원의 rig_mode.py 로 주행/작업을 전환하세요.
    """
    for name in WHEEL_JOINT_NAMES:
        drive = UsdPhysics.DriveAPI.Get(
            stage.GetPrimAtPath(f"{rig_path}/{name}"), "angular"
        )
        drive.GetStiffnessAttr().Set(WHEEL_BRAKE_STIFFNESS)
        drive.GetTargetPositionAttr().Set(0.0)
        drive.GetTargetVelocityAttr().Set(0.0)
    print(f"[브레이크] 카터 바퀴 {len(WHEEL_JOINT_NAMES)}개 고정")


def joint_limits_deg(stage, arm_path):
    """USD에 적힌 관절 한계를 degree로 읽습니다."""
    lower, upper = [], []
    for name in JOINT_NAMES:
        joint = UsdPhysics.RevoluteJoint(
            stage.GetPrimAtPath(f"{arm_path}/joints/{name}")
        )
        if not joint:
            raise RuntimeError(f"USD 관절을 찾지 못했습니다: {name}")
        lower.append(joint.GetLowerLimitAttr().Get())
        upper.append(joint.GetUpperLimitAttr().Get())
    return np.array(lower), np.array(upper)


# ── AMR 베이스 확인 ─────────────────────────────────────
def tine_tip_position(robot):
    """포크 갈래 끝의 월드 좌표. 손목에서 포크가 뻗는 방향으로 FORK_TINE_TIP 만큼."""
    position, quaternion = robot.end_effector.get_world_pose()
    forward = quat_to_rot_matrix(quaternion) @ np.array([0.0, 0.0, FORK_TINE_TIP])
    return np.array(position, dtype=float) + forward


def yaw_deg(quaternion):
    """베이스가 z축으로 몇 도 돌아가 있는지 (w, x, y, z)"""
    rotation = quat_to_rot_matrix(quaternion)
    return float(np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0])))


def wrap_deg(angle):
    """각도를 -180 ~ +180 으로 접습니다."""
    return (angle + 180.0) % 360.0 - 180.0


def home_joints_deg(base_quaternion):
    """
    지금 팔 베이스가 향한 방향에 맞춘 HOME 관절값.

    joint_1 은 베이스 좌표계 기준이라, 베이스가 돌아간 만큼 빼 줘야
    팔이 늘 같은 세계 방향(랙 쪽)을 봅니다.

    빼고 나서 -180~180 으로 접습니다. 접지 않으면 베이스가 180도 돌아 선 배치에서
    joint_1 이 360도가 되어 한계(±360)에 붙어 버립니다. 접어도 문제없는 이유는
    solve_plan 이 IK 결과를 '앞 자세에 가장 가까운 표현'으로 맞춰 주기 때문입니다.
    """
    joints = np.array(HOME_JOINTS_DEG, dtype=float)
    joints[0] = wrap_deg(joints[0] - yaw_deg(base_quaternion))
    return joints


class BaseWatcher:
    """
    팔 베이스가 실제로 멈췄는지 직접 재서 확인합니다.

    상위에서 '도착했다' 고 알려 주는 것과, 실제로 정지한 것은 다릅니다.
    카터 주행뿐 아니라 리프트 승강이 끝났는지도 이 한 군데서 같이 걸러집니다.
    매 물리 스텝마다 update 를 부르고, settled 가 True 가 되면 계획을 만듭니다.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.last_position = None
        self.still_seconds = 0.0
        self.waited_seconds = 0.0
        self.notice_seconds = 0.0

    def update(self, arm_base, dt):
        position = np.array(arm_base.get_world_pose()[0], dtype=float)
        moved = (
            self.last_position is None
            or float(np.linalg.norm(position - self.last_position)) > BASE_STILL_TOL
        )
        self.last_position = position
        self.still_seconds = 0.0 if moved else self.still_seconds + dt

        self.waited_seconds += dt
        self.notice_seconds += dt
        if self.notice_seconds >= BASE_WAIT_NOTICE_SECONDS:
            self.notice_seconds = 0.0
            print(f"[대기] 팔 베이스 정지를 기다리는 중 ({self.waited_seconds:.1f}초)")

    @property
    def settled(self):
        return self.still_seconds >= BASE_STILL_SECONDS


def base_offset_from_pallet(base_position, pallet_position, pallet_quaternion):
    """팔레트 좌표계에서 본 팔 베이스 위치. (앞뒤, 좌우, 위아래)"""
    rotation = quat_to_rot_matrix(pallet_quaternion)
    return rotation.T @ (np.array(base_position, float) - np.array(pallet_position, float))


def check_base_pose(arm_position, chassis_quaternion, pallet_position, pallet_quaternion):
    """
    도킹 위치가 안전선 안인지 확인합니다.

    기준은 '집을 팔레트'입니다. 어느 랙 앞이든 같은 값으로 검사됩니다.
    위치는 '팔 베이스'로, 각도는 '차체'로 봅니다.
      - 팔이 닿는지는 팔 베이스가 팔레트에서 얼마나 떨어져 있느냐로 정해집니다.
      - 팔 베이스는 리그에 비스듬히 붙어 있을 수 있어(에셋 값) 각도 판단에 쓰면
        안 됩니다. AMR 이 삐뚤게 섰는지는 차체 방향으로 봐야 합니다.
        (팔 베이스가 돌아간 건 home_joints_deg() 가 알아서 보정합니다)

    여기를 통과했다고 작업이 가능한 것은 아닙니다. 실제 도달 여부는 IK 가 봅니다.
    벗어나면 무엇이 얼마나 벗어났는지 알려 주고 계획을 만들지 않습니다.
    """
    forward, sideways, _ = base_offset_from_pallet(
        arm_position, pallet_position, pallet_quaternion
    )
    yaw = wrap_deg(yaw_deg(chassis_quaternion) - yaw_deg(pallet_quaternion)
                   - CHASSIS_FACING_DEG)
    height = float(arm_position[2]) - float(pallet_position[2])   # 팔레트 원점 = 선반 윗면

    if not BASE_TO_PALLET_X[0] <= forward <= BASE_TO_PALLET_X[1]:
        raise RuntimeError(
            f"팔레트와의 앞뒤 거리가 범위 밖입니다: {forward:.3f} m "
            f"(허용 {BASE_TO_PALLET_X[0]} ~ {BASE_TO_PALLET_X[1]}). 다시 도킹하세요."
        )
    if not BASE_TO_PALLET_Y[0] <= sideways <= BASE_TO_PALLET_Y[1]:
        raise RuntimeError(
            f"팔레트와의 좌우 어긋남이 범위 밖입니다: {sideways:+.3f} m "
            f"(허용 {BASE_TO_PALLET_Y[0]} ~ {BASE_TO_PALLET_Y[1]}). 다시 도킹하세요."
        )
    if abs(yaw) > BASE_YAW_LIMIT_DEG:
        raise RuntimeError(
            f"차체가 팔레트를 정면으로 보고 있지 않습니다: {yaw:+.1f}° "
            f"(허용 ±{BASE_YAW_LIMIT_DEG}°). 다시 도킹하세요."
        )
    if not BASE_HEIGHT_BAND[0] <= height <= BASE_HEIGHT_BAND[1]:
        raise RuntimeError(
            f"베이스 높이가 집을 선반과 맞지 않습니다: 선반 윗면 대비 {height:+.3f} m "
            f"(허용 {BASE_HEIGHT_BAND[0]:+.2f} ~ {BASE_HEIGHT_BAND[1]:+.2f}). "
            "리프트 높이를 조정하세요."
        )

    print(
        f"[도킹 확인] 팔레트 기준 앞뒤 {forward:.3f} m, 좌우 {sideways:+.3f} m, "
        f"높이 {height:+.3f} m, 차체 틀어짐 {yaw:+.1f}° — 범위 안"
    )


# ── 계획의 한 줄 ────────────────────────────────────────
class Step(NamedTuple):
    """
    계획 한 줄. 목표 좌표와 그 좌표를 푼 관절값을 같이 들고 다닙니다.

    stage  : 원래 단계 이름 (예: PALLET_UP) — 검사에 씁니다
    name   : 화면에 찍는 이름 (예: PALLET_UP_2)
    target : 월드 좌표. HOME 처럼 좌표가 없는 단계는 None
    joints : 관절 목표 (degree, 6개)
    last   : 이 줄이 그 단계의 마지막 조각인가
    """

    stage: str
    name: str
    target: Optional[np.ndarray]
    joints: np.ndarray
    last: bool


def check_stage_tables():
    """표에 오타가 있으면 시작 전에 알려 줍니다."""
    names = [name for name, _ in PICK_STAGES + PLACE_STAGES]
    if len(set(names)) != len(names):
        raise ValueError("단계 이름이 중복되어 있습니다.")
    if names.count(STAGE_PALLET_UP) != 1:
        raise ValueError(f"{STAGE_PALLET_UP} 단계가 정확히 하나 있어야 합니다 (인양 확인).")
    if names.count(STAGE_PALLET_DOWN) != 1:
        raise ValueError(f"{STAGE_PALLET_DOWN} 단계가 정확히 하나 있어야 합니다 (안착 확인).")


# ── 계획 만들기 ──────────────────────────────────────────
def _transform_stage_points(origin, quaternion, stages):
    """기준점 상대 단계 좌표를 월드 좌표로 바꿉니다."""
    rotation = quat_to_rot_matrix(quaternion)
    origin = np.array(origin, dtype=float)
    return [
        (name, origin + rotation @ np.array(offset, dtype=float))
        for name, offset in stages
    ]


def pick_stage_points(pallet_position, pallet_quaternion):
    """집을 당시 팔레트 pose를 기준으로 Pick 단계 좌표를 만듭니다."""
    return _transform_stage_points(pallet_position, pallet_quaternion, PICK_STAGES)


def place_stage_points(pallet_position, pallet_quaternion, destination_shelf_top):
    """집을 당시 팔레트 pose와 놓을 높이로 Place 단계 좌표를 만듭니다."""
    place_origin = np.array(pallet_position, dtype=float).copy()
    if destination_shelf_top is not None:
        place_origin[2] = destination_shelf_top     # 팔레트 원점 높이 = 선반 윗면
    return _transform_stage_points(place_origin, pallet_quaternion, PLACE_STAGES)


def split_segments(points, start_point=None):
    """
    긴 구간을 MAX_SEGMENT_M 이하로 자릅니다.

    한 단계가 여러 조각으로 잘리므로, 조각마다 마지막인지(last)를 표시합니다.
    인양·안착 확인은 마지막 조각에서만 합니다.
    관절값은 아직 모르므로 빈 배열로 두고, solve_plan 이 채웁니다.
    """
    empty = np.zeros(len(JOINT_NAMES))
    result = []
    previous = None if start_point is None else np.array(start_point, dtype=float)

    for stage, goal in points:
        if previous is None:
            result.append(Step(stage, stage, goal, empty, True))
            previous = goal
            continue

        start = previous
        count = max(1, int(np.ceil(np.linalg.norm(goal - start) / MAX_SEGMENT_M)))
        for k in range(1, count + 1):
            name = stage if count == 1 else f"{stage}_{k}"
            point = start + (goal - start) * k / count
            result.append(Step(stage, name, point, empty, k == count))
        previous = goal
    return result


def solve_plan(solver, segments, lower_deg, upper_deg, start_joints_deg,
               first_move_limit_deg=FIRST_MOVE_LIMIT_DEG):
    """
    각 목표 좌표를 IK로 풀어 관절값 표를 만듭니다.

    바로 앞 단계의 해를 다음 계산의 출발점(warm start)으로 넘겨서,
    팔이 갑자기 다른 자세로 뒤집히지 않게 합니다.
    첫 작업은 HOME, 후속 작업은 현재 관절 자세를 기준으로 계산·검사합니다.
    """
    want_rotation = quat_to_rot_matrix(FORK_QUAT)
    warm = np.deg2rad(start_joints_deg)
    plan = []

    for index, segment in enumerate(segments):
        name, target = segment.name, segment.target
        joints, solved = solver.compute_inverse_kinematics(
            EE_FRAME, target, FORK_QUAT, warm
        )
        if not solved:
            raise RuntimeError(
                f"{name}: 지금 도킹 위치에서는 목표 {np.round(target, 3)} 에 닿지 않습니다. "
                "AMR 위치나 리프트 높이를 조정해 다시 시도하세요."
            )

        reached, rotation = solver.compute_forward_kinematics(EE_FRAME, joints)
        position_error = float(np.linalg.norm(reached - target))
        angle_error = float(np.degrees(np.arccos(np.clip(
            (np.trace(want_rotation.T @ rotation) - 1.0) / 2.0, -1.0, 1.0
        ))))
        if position_error > IK_POSITION_TOL or angle_error > IK_ANGLE_TOL_DEG:
            raise RuntimeError(
                f"{name}: IK 정확도 부족 (위치 {position_error * 1000:.1f} mm, "
                f"자세 {angle_error:.1f}°)."
            )

        # 첫 목표는 시작 자세에서 오는 이동이라 변화량 기준을 따로 둡니다.
        previous = start_joints_deg if index == 0 else plan[-1].joints

        # IK 는 같은 자세를 +360 / -360 도 다르게 표현해 돌려주기도 합니다.
        # 그대로 두면 실제로는 제자리인데 '360도 휘두른다'고 읽히고, 보간도 한 바퀴
        # 돌아갑니다. 앞 자세에서 가장 가까운 표현으로 맞춰 둡니다.
        joints_deg = previous + wrap_deg(np.rad2deg(joints) - previous)

        over = np.where((joints_deg < lower_deg) | (joints_deg > upper_deg))[0]
        if len(over):
            detail = ", ".join(
                f"{JOINT_NAMES[i]} {joints_deg[i]:+.1f}° "
                f"(한계 {lower_deg[i]:+.0f} ~ {upper_deg[i]:+.0f})" for i in over
            )
            raise RuntimeError(f"{name}: 관절 한계를 벗어났습니다 — {detail}")
        limit = first_move_limit_deg if index == 0 else IK_JUMP_LIMIT_DEG
        jump = float(np.max(np.abs(joints_deg - previous)))
        if jump > limit:
            raise RuntimeError(
                f"{name}: 앞 자세 대비 관절이 {jump:.1f}° 바뀝니다 (한계 {limit:.0f}°). "
                "자세가 뒤집히거나 크게 휘두르는 경로입니다."
            )

        plan.append(segment._replace(joints=joints_deg))
        warm = joints

    return plan


def print_plan(plan):
    """계획을 표로 찍습니다. HOME은 좌표가 없는 고정 자세입니다."""
    print("── 계획 ──")
    for step in plan:
        where = "고정 자세" if step.target is None else str(np.round(step.target, 3).tolist())
        print(f"  {step.name:12s} 목표 {where:28s} 관절 {np.round(step.joints, 1).tolist()}")


# ── 실행 ────────────────────────────────────────────────
class _PalletTracker:
    """Pick과 Place 사이에 팔레트 pose와 안전검사 기준을 유지합니다."""

    def __init__(self, robot, pallet):
        self.robot = robot
        self.pallet = pallet
        position, quaternion = pallet.get_world_pose()
        self.pick_position = np.array(position, dtype=float)
        self.pick_quaternion = np.array(quaternion, dtype=float)
        self.pick_end_target = None

        self.phase = "BEFORE_LIFT"
        self.pallet_start = None      # 시작 위치 (밀림 확인용)
        self.lift_start_z = None      # 인양 직전 높이
        self.support_offset = None    # 손목 기준 팔레트 위치 (운반 중 확인용)
        self.carry_z = None           # 운반 중 높이 (안착 확인용)
        self.placed_position = None   # 안착 위치 (포크 뺄 때 확인용)

    def pallet_position(self):
        return np.array(self.pallet.get_world_pose()[0], dtype=float)

    def relative_position(self):
        """손목 좌표계에서 본 팔레트 위치. 팔이 돌아가도 값이 유지됩니다."""
        position, quaternion = self.robot.end_effector.get_world_pose()
        return quat_to_rot_matrix(quaternion).T @ (self.pallet_position() - position)

    def begin_wait(self):
        if self.pallet_start is None:
            self.pallet_start = self.pallet_position()

    def begin_stage(self, stage):
        if stage == STAGE_PALLET_UP and self.lift_start_z is None:
            self.lift_start_z = self.pallet_position()[2]
        if stage == STAGE_PALLET_DOWN and self.carry_z is None:
            self.carry_z = self.pallet_position()[2]

    def confirm_stage_end(self, stage, last):
        if stage == STAGE_PALLET_UP and last:
            rise = self.pallet_position()[2] - self.lift_start_z
            if rise < MIN_PALLET_RISE:
                raise RuntimeError(f"인양 실패: 실제 상승량 {rise * 1000:.1f} mm")
            self.support_offset = self.relative_position()
            self.phase = "CARRYING"
            print(f"[인양 확인] 실제 상승량 {rise * 1000:.1f} mm")

        if stage == STAGE_PALLET_DOWN and last:
            drop = self.carry_z - self.pallet_position()[2]
            if drop < MIN_PALLET_DROP:
                raise RuntimeError(
                    f"안착 실패: 실제 하강량 {drop * 1000:.1f} mm. "
                    "팔레트가 아직 포크에 얹혀 있습니다."
                )
            self.placed_position = self.pallet_position()
            self.phase = "PLACED"
            print(f"[안착 확인] 실제 하강량 {drop * 1000:.1f} mm")

    def check(self, stage, name):
        if self.pallet_start is None:
            return

        if self.phase == "CARRYING":
            # PALLET_DOWN 은 일부러 내려놓는 단계입니다. 팔레트가 선반에 닿은 뒤에도
            # 포크는 포켓 안에서 더 내려가므로, 손목 기준 위치가 바뀌는 게 정상입니다.
            if stage == STAGE_PALLET_DOWN:
                return
            slip = float(np.linalg.norm(self.relative_position() - self.support_offset))
            if slip > PALLET_SLIP_TOL:
                raise RuntimeError(
                    f"{name}: 운반 중 팔레트가 {slip * 1000:.1f} mm 미끄러졌습니다."
                )

        elif self.phase == "PLACED":
            moved = float(np.linalg.norm(self.pallet_position() - self.placed_position))
            if moved > PALLET_STAY_TOL:
                raise RuntimeError(
                    f"{name}: 포크를 빼는 중 팔레트가 {moved * 1000:.1f} mm 따라왔습니다."
                )

        elif stage != STAGE_PALLET_UP:      # 아직 들기 전
            moved = float(np.linalg.norm(self.pallet_position() - self.pallet_start))
            if moved > PALLET_PUSH_TOL:
                raise RuntimeError(
                    f"{name}: 인양 전 팔레트가 {moved * 1000:.1f} mm 움직였습니다."
                )


class JointSequence:
    """
    계획된 관절값을 한 단계씩 실행하고, 팔레트 상태를 실제로 확인합니다.

    팔레트 상태는 세 시기로 나뉩니다.
      BEFORE_LIFT : 아직 들기 전. 팔레트가 움직이면 밀고 있는 것 → 중단
      CARRYING    : 들고 있는 중. 손목 기준 상대 위치가 변하면 미끄러진 것 → 중단
      PLACED      : 내려놓은 뒤. 포크를 뺄 때 팔레트가 따라오면 → 중단
    """

    def __init__(self, robot, indices, plan, pallet_tracker=None,
                 done_message="[DONE] 동작을 확인했습니다.",
                 start_wait_seconds=START_WAIT_SECONDS):
        self.robot = robot
        self.indices = indices
        self.plan = plan
        self.pallet_tracker = pallet_tracker
        self.done_message = done_message

        self.index = -1
        self.stage = "WAIT"
        self.name = "WAIT"
        self.last = True
        self.elapsed = 0.0
        self.reached_seconds = 0.0
        self.done = False

        self.start = read_joints_deg(robot, indices)
        self.goal = self.start.copy()
        self.duration = start_wait_seconds
        if start_wait_seconds <= 0.0:
            if self.pallet_tracker is not None:
                self.pallet_tracker.begin_wait()
            self.begin_next_stage()

    def pallet_position(self):
        return None if self.pallet_tracker is None else self.pallet_tracker.pallet_position()

    # ── 단계 진행 ───────────────────────────────────
    def begin_next_stage(self):
        self.index += 1
        self.elapsed = 0.0
        self.reached_seconds = 0.0

        if self.index == len(self.plan):
            self.stage = self.name = "DONE"
            self.done = True
            print(self.done_message)
            return

        step = self.plan[self.index]
        self.stage, self.name, self.last = step.stage, step.name, step.last
        self.start = read_joints_deg(self.robot, self.indices)
        self.goal = np.asarray(step.joints, dtype=float)

        largest_move = float(np.max(np.abs(self.goal - self.start)))
        self.duration = max(MIN_MOVE_SECONDS, largest_move / JOINT_SPEED_DEG_S)

        if self.pallet_tracker is not None:
            self.pallet_tracker.begin_stage(self.stage)

        print(f"[{self.name}] 보간 시간 {self.duration:.1f}초")

    def confirm_stage_end(self):
        """단계가 끝난 순간에만 하는 확인 (인양 성공, 안착 성공)"""
        if self.pallet_tracker is not None:
            self.pallet_tracker.confirm_stage_end(self.stage, self.last)

    # ── 팔레트 상태 확인 ────────────────────────────
    def check_pallet(self):
        if self.pallet_tracker is not None:
            self.pallet_tracker.check(self.stage, self.name)

    # ── 매 물리 스텝 ────────────────────────────────
    def update(self, dt):
        self.check_pallet()

        if self.done:
            command_joints_deg(self.robot, self.indices, self.goal)
            return

        if self.name == "WAIT":
            command_joints_deg(self.robot, self.indices, self.goal)
            self.elapsed += dt
            if self.elapsed >= self.duration:
                if self.pallet_tracker is not None:
                    self.pallet_tracker.begin_wait()
                self.begin_next_stage()
            return

        self.elapsed += dt
        alpha = min(1.0, self.elapsed / self.duration)
        target = self.start + alpha * (self.goal - self.start)

        actual = read_joints_deg(self.robot, self.indices)
        tracking_error = float(np.max(np.abs(target - actual)))
        if tracking_error > JOINT_TRACKING_LIMIT_DEG:
            raise RuntimeError(
                f"{self.name}: 관절 추종 오차 {tracking_error:.1f}°. "
                "충돌 또는 구동 설정을 확인하세요."
            )

        command_joints_deg(self.robot, self.indices, target)

        if alpha < 1.0:
            return

        goal_error = float(np.max(np.abs(self.goal - actual)))
        if goal_error <= JOINT_REACHED_TOL_DEG:
            self.reached_seconds += dt
        else:
            self.reached_seconds = 0.0

        if self.reached_seconds >= HOLD_SECONDS:
            self.confirm_stage_end()
            self.begin_next_stage()
        elif self.elapsed > self.duration + REACH_TIMEOUT_SECONDS:
            raise RuntimeError(
                f"{self.name}: 도달 시간 초과, 관절 오차 {goal_error:.1f}°"
            )


def build_sequence(solver, robot, indices, lower_deg, upper_deg, points,
                   start_joints_deg, pallet_tracker=None, start_point=None,
                   home_joints=None, first_move_limit_deg=FIRST_MOVE_LIMIT_DEG,
                   done_message="[DONE] 동작을 확인했습니다.",
                   start_wait_seconds=START_WAIT_SECONDS):
    """주어진 Pick 또는 Place 좌표를 기존 IK와 JointSequence로 만듭니다."""
    segments = split_segments(points, start_point=start_point)
    plan = solve_plan(
        solver,
        segments,
        lower_deg,
        upper_deg,
        start_joints_deg,
        first_move_limit_deg=first_move_limit_deg,
    )
    if home_joints is not None:
        plan = [Step(STAGE_HOME, STAGE_HOME, None, home_joints, True)] + plan

    print_plan(plan)
    return JointSequence(
        robot,
        indices,
        plan,
        pallet_tracker=pallet_tracker,
        done_message=done_message,
        start_wait_seconds=start_wait_seconds,
    )


class RobotMotion:
    """기존 IK와 JointSequence로 Home, Pick, Place를 실행합니다."""

    def __init__(self, robot, arm_base, solver, stage, arm_path):
        self._robot = robot
        self._arm_base = arm_base
        self._solver = solver
        self._stage = stage
        self._arm_path = arm_path

        self._indices = None
        self._lower_deg = None
        self._upper_deg = None
        self._sequence = None
        self._pallet_tracker = None
        self._hold_target = None
        self._log_elapsed = 0.0
        self._transfer_place_pending = False
        self._transfer_destination = None
        self._initialized = False

    def initialize(self):
        """관절 인덱스와 한계를 한 번 읽고 팔 관절 Drive를 준비합니다."""
        if self._initialized:
            return

        check_stage_tables()
        self._lower_deg, self._upper_deg = joint_limits_deg(
            self._stage, self._arm_path
        )
        setup_arm_drives(self._stage, self._arm_path)
        self._indices = robot_indices(self._robot)
        self._hold_target = read_joints_deg(self._robot, self._indices)
        self._initialized = True

    def start_home(self):
        """현재 베이스 방향에 맞는 HOME 자세로 이동합니다."""
        self._require_initialized()
        self._require_idle()
        if self._is_carrying:
            raise RuntimeError("팔레트를 운반 중에는 HOME 동작을 시작할 수 없습니다.")

        _, base_quaternion = self._arm_base.get_world_pose()
        goal = home_joints_deg(base_quaternion)
        plan = [Step(STAGE_HOME, STAGE_HOME, None, goal, True)]
        print_plan(plan)
        self._pallet_tracker = None
        self._sequence = JointSequence(
            self._robot,
            self._indices,
            plan,
            done_message="[DONE] HOME 자세에 도달했습니다.",
        )
        self._reset_start_state()

    def start_pick(self, pallet, start_from_home=True):
        """PICK_STAGES만 계획하고 팔레트를 CARRYING 상태까지 인출합니다."""
        self._require_initialized()
        self._require_idle()
        if self._is_carrying:
            raise RuntimeError("이미 팔레트를 운반 중이므로 새 Pick을 시작할 수 없습니다.")

        tracker = _PalletTracker(self._robot, pallet)
        base_position, base_quaternion = self._set_solver_base_pose()
        _, chassis_quaternion = self._robot.get_world_pose()
        check_base_pose(
            base_position,
            chassis_quaternion,
            tracker.pick_position,
            tracker.pick_quaternion,
        )
        print(f"[팔 베이스] {np.round(base_position, 3).tolist()} "
              f"(요 {yaw_deg(base_quaternion):+.1f}° → HOME joint_1 "
              f"{180.0 - yaw_deg(base_quaternion):+.1f}°)")
        print(f"[팔레트] {np.round(tracker.pick_position, 3).tolist()}")

        start_deg = (
            home_joints_deg(base_quaternion)
            if start_from_home
            else read_joints_deg(self._robot, self._indices)
        )
        points = pick_stage_points(tracker.pick_position, tracker.pick_quaternion)
        sequence = build_sequence(
            self._solver,
            self._robot,
            self._indices,
            self._lower_deg,
            self._upper_deg,
            points,
            start_deg,
            pallet_tracker=tracker,
            home_joints=start_deg if start_from_home else None,
            done_message="[DONE] 집기·인양·인출까지 확인했습니다.",
        )
        tracker.pick_end_target = np.array(points[-1][1], dtype=float)
        self._pallet_tracker = tracker
        self._sequence = sequence
        self._reset_start_state()

    def start_place(self, destination_shelf_top):
        """CARRYING 팔레트에 대해 PLACE_STAGES만 계획하고 실행합니다."""
        self._require_initialized()
        self._require_idle()
        if not self._is_carrying:
            raise RuntimeError("Place는 Pick이 끝난 CARRYING 상태에서만 시작할 수 있습니다.")

        self._set_solver_base_pose()
        tracker = self._pallet_tracker
        points = place_stage_points(
            tracker.pick_position,
            tracker.pick_quaternion,
            destination_shelf_top,
        )
        start_deg = read_joints_deg(self._robot, self._indices)
        self._sequence = build_sequence(
            self._solver,
            self._robot,
            self._indices,
            self._lower_deg,
            self._upper_deg,
            points,
            start_deg,
            pallet_tracker=tracker,
            start_point=tracker.pick_end_target,
            first_move_limit_deg=IK_JUMP_LIMIT_DEG,
            done_message="[DONE] 놓기·안착·포크 인출까지 확인했습니다.",
            start_wait_seconds=0.0,
        )
        self._reset_start_state()

    def start_transfer(self, pallet, destination_shelf_top, start_from_home=True):
        """start_pick() 완료 후 start_place()를 자동으로 이어 실행합니다."""
        self.start_pick(pallet, start_from_home=start_from_home)
        self._transfer_place_pending = True
        self._transfer_destination = destination_shelf_top

    def update(self, dt):
        """현재 JointSequence를 물리 한 스텝만큼 진행합니다."""
        self._require_initialized()
        if not self.is_running:
            return

        sequence = self._sequence
        sequence.update(dt)
        self._log_elapsed += dt
        if self._log_elapsed >= LOG_INTERVAL_SECONDS:
            status = (
                f"[{sequence.name}] "
                f"관절 {np.round(read_joints_deg(self._robot, self._indices), 1)}"
            )
            pallet_position = sequence.pallet_position()
            if pallet_position is not None:
                status += f", 팔레트 {np.round(pallet_position, 3)}"
            print(status)
            self._log_elapsed = 0.0

        if sequence.done:
            self._hold_target = np.asarray(sequence.goal, dtype=float).copy()
            if self._transfer_place_pending:
                destination = self._transfer_destination
                self._transfer_place_pending = False
                self._transfer_destination = None
                self.start_place(destination)

    def hold(self):
        """현재 위치 또는 마지막으로 완료한 안전한 관절 목표를 유지합니다."""
        self._require_initialized()
        if self._pallet_tracker is not None:
            self._pallet_tracker.check("HOLD", "HOLD")
        if self.is_running:
            self._hold_target = read_joints_deg(self._robot, self._indices)
        command_joints_deg(self._robot, self._indices, self._hold_target)

    def cancel(self):
        """진행 중 동작을 버리고 현재 위치를 유지합니다."""
        self._require_initialized()
        if self.is_running:
            self._hold_target = read_joints_deg(self._robot, self._indices)
        elif self.is_done:
            self._hold_target = np.asarray(self._sequence.goal, dtype=float).copy()

        self._sequence = None
        self._pallet_tracker = None
        self._log_elapsed = 0.0
        self._transfer_place_pending = False
        self._transfer_destination = None
        command_joints_deg(self._robot, self._indices, self._hold_target)

    @property
    def is_running(self):
        return self._sequence is not None and not self._sequence.done

    @property
    def is_done(self):
        return self._sequence is not None and self._sequence.done

    @property
    def current_stage(self):
        return "IDLE" if self._sequence is None else self._sequence.name

    def _reset_start_state(self):
        self._log_elapsed = 0.0

    def _require_initialized(self):
        if not self._initialized:
            raise RuntimeError("RobotMotion.initialize()를 먼저 호출하세요.")

    def _require_idle(self):
        if self.is_running:
            raise RuntimeError(
                f"팔 동작이 이미 실행 중입니다: {self.current_stage}"
            )

    @property
    def _is_carrying(self):
        return (
            self._pallet_tracker is not None
            and self._pallet_tracker.phase == "CARRYING"
        )

    def _set_solver_base_pose(self):
        base_position, base_quaternion = self._arm_base.get_world_pose()
        self._solver.set_robot_base_pose(
            robot_position=base_position,
            robot_orientation=base_quaternion,
        )
        return base_position, base_quaternion
