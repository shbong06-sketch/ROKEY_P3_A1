"""
Pick & Place — 상태 기계로 순서 만들기

지금까지 배운 것을 하나로 엮는다.
  TCP 오프셋으로 손가락 끝을 보내고 (2단계)
  그리퍼를 열고 닫으며 (3단계)
  구간을 보간해 이동하고 (4단계)
  씬 구성은 Task 가 맡는다 (5단계)
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import sys

import numpy as np
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.xforms import get_world_pose
from isaacsim.robot.manipulators.grippers import ParallelGripper


# Standalone 실행에서는 USD의 ROS 2 Action Graph를 읽기 전에 Bridge를 켠다.
enable_extension("isaacsim.ros2.bridge")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
THIS_DIR  = Path(__file__).resolve().parent
M0609_DIR = THIS_DIR.parent

USD_PATH = str(M0609_DIR / "Collected_m0609_color_zone/m0609_color_zone.usd")
URDF_PATH = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
RMPFLOW_DIR = M0609_DIR / "rmpflow"
RMPFLOW_DESCRIPTION_PATH = str(RMPFLOW_DIR / "m0609_description.yaml")
RMPFLOW_CONFIG_PATH = str(RMPFLOW_DIR / "m0609_rmpflow_common.yaml")

# 기존 M0609 RMPflow controller를 그대로 import한다.
if str(RMPFLOW_DIR) not in sys.path:
    sys.path.insert(0, str(RMPFLOW_DIR))
from m0609_rmpflow_controller import RMPFlowController

WRIST_RGB_CAMERA_PATH = (
    "/World/m0609/onrobot_rg2ft/angle_bracket/realsense_d455/"
    "RSD455/Camera_OmniVision_OV9782_Color"
)
# 수집된 메인 USD가 x=112.6458로 덮어쓴 값을 원본 D455 값으로 복구한다.
WRIST_RGB_CAMERA_TRANSLATION = Gf.Vec3d(0.0, 0.0115, 0.0)

# 단계 2 제어 주기. ROS2 publisher는 이 단계에서 사용하지 않는다.
SIMULATION_RENDER_HZ = 60.0


# ══════════════════════════════════════════════════════════════
#  로봇 설정
# ══════════════════════════════════════════════════════════════
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME = "link_6"
EE_PRIM_PATH = f"{ROBOT_PRIM_PATH}/{EE_LINK_NAME}"

# Drive 는 팔 6축에만 적용한다
ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

# 시작 자세 — 그리퍼가 아래를 향하도록 미리 굽혀 둔다
READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


# ══════════════════════════════════════════════════════════════
#  그리퍼 설정
# ══════════════════════════════════════════════════════════════
# finger_joint 가 구동 관절이고 나머지 5개는 Mimic 으로 따라온다
# 두 번째 이름은 ParallelGripper 가 요구하는 형식상 필요하다
GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]

# finger_joint 절대 목표값 (라디안)
#   Physics Inspector 는 도로 표시한다.  0.0 ~ 67.609 deg = 0.0 ~ 1.18 rad
GRIPPER_OPEN_POS = 0.0
GRIPPER_CLOSE_POS = 1.18

# 물체 접촉 후 중력과 이동 가속도를 버티도록 구동력을 명시한다.
GRIPPER_DRIVE_STIFFNESS = 1.0e5
GRIPPER_DRIVE_DAMPING = 1.0e3
GRIPPER_DRIVE_MAX_FORCE = 1.0e4


# ══════════════════════════════════════════════════════════════
#  랜덤 큐브 설정
# ══════════════════════════════════════════════════════════════
CUBE_PRIM_PATH = "/World/RandomPickCube"
CUBE_NAME = "random_pick_cube"
CUBE_SIZE = 0.020
GROUND_Z = 0.0
CUBE_STATIC_FRICTION = 1.5
CUBE_DYNAMIC_FRICTION = 1.5

# None이면 프로그램을 실행할 때마다 다른 결과를 사용한다.
# 정수로 바꾸면 같은 위치와 색상을 재현할 수 있다.
RANDOM_SEED = None
CUBE_X_RANGE = (0.30, 0.42)
CUBE_Y_RANGE = (-0.18, 0.18)
CUBE_COLORS = {
    "green": np.array([0.0, 1.0, 0.0]),
    "blue": np.array([0.0, 0.0, 1.0]),
}

# /color_id에 따른 World 좌표 Place 중심.
# 사용자 지정: 1=파란 마커, 2=초록 마커.
COLOR_ID_NONE = 0
COLOR_ID_BLUE_MARKER = 1
COLOR_ID_GREEN_MARKER = 2
PLACE_CUBE_CENTER_BY_COLOR_ID = {
    COLOR_ID_BLUE_MARKER: np.array(
        [0.40, -0.20, GROUND_Z + CUBE_SIZE / 2.0]
    ),
    COLOR_ID_GREEN_MARKER: np.array(
        [0.40, 0.20, GROUND_Z + CUBE_SIZE / 2.0]
    ),
}
PLACE_NAME_BY_COLOR_ID = {
    COLOR_ID_BLUE_MARKER: "BLUE_MARKER",
    COLOR_ID_GREEN_MARKER: "GREEN_MARKER",
}


# ══════════════════════════════════════════════════════════════
#  기본 조명 설정
# ══════════════════════════════════════════════════════════════
DEFAULT_DOME_LIGHT_PATH = "/World/DefaultDomeLight"
DEFAULT_SUN_LIGHT_PATH = "/World/DefaultDistantLight"
DEFAULT_DOME_INTENSITY = 500.0
DEFAULT_SUN_INTENSITY = 1500.0


# ══════════════════════════════════════════════════════════════
#  TCP 오프셋
# ══════════════════════════════════════════════════════════════
# link_6 로컬 좌표계에서 손가락 패드 끝까지의 거리 (실측)
#   손가락 패드 범위  0.13632 ~ 0.19671
#   링크 원점 0.14155 는 관절 위치이지 파지면이 아니다
FINGER_PAD_TIP_Z = 0.19671
TCP_OFFSET = np.array([0.0, 0.0, FINGER_PAD_TIP_Z])


# ══════════════════════════════════════════════════════════════
#  고정 Pick & Place 목표
# ══════════════════════════════════════════════════════════════
# 6_pick_place.py에서 검증한 TCP 높이.
# PICK_TCP_Z는 큐브 상단에서 손가락이 옆면을 물 수 있는 높이다.
PICK_TCP_Z = 0.04
PLACE_TCP_Z = 0.055
APPROACH_TCP_Z = 0.250
LIFT_TCP_Z = 0.230
RETREAT_TCP_Z = 0.230

# Lift 완료 후 큐브 중심이 이 높이 이상이면 실제 물리 파지 성공으로 본다.
GRASP_SUCCESS_MIN_CUBE_Z = 0.080

# time.sleep 없이 simulation frame으로 기다린다.
GRIPPER_CLOSE_WAIT_FRAMES = 180
GRIPPER_OPEN_WAIT_FRAMES = 120

# 매 simulation frame마다 갱신되는 TCP 보간 설정
TCP_SPEED = 0.004
MIN_STEPS = 60
MAX_STEPS = 600

# 보간 종료 후 실제 TCP가 목표에 수렴했는지 확인한다.
TARGET_POSITION_TOLERANCE = 0.005
TARGET_SETTLE_FRAMES = 15
MAX_TARGET_WAIT_FRAMES = 600

# 접근 방향 — 툴(link_6 로컬 +Z)이 어디를 향할지
#   roll  pitch      방향
#    180      0      바닥
#    180     90      +x 수평
#    180    -90      -x 수평
#     90      0      -y 수평
#    -90      0      +y 수평
#      0      0      하늘
APPROACH_ROLL_DEG  = 180.0
APPROACH_PITCH_DEG = 0.0

# 툴축 회전 — 접근 방향은 그대로, 손가락(로컬 +X)만 돌아간다
GRIPPER_YAW_DEG = 0.0


# ══════════════════════════════════════════════════════════════
#  회전 유틸
# ══════════════════════════════════════════════════════════════
def quat_mul(a, b):
    """쿼터니언 곱. 순서는 (w, x, y, z)"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_from_axis(axis, deg):
    """회전축과 각도(도)로 쿼터니언을 만든다"""
    half = np.radians(deg) / 2.0
    a = np.array(axis, dtype=float)
    a = a / np.linalg.norm(a)
    return np.concatenate([[np.cos(half)], a * np.sin(half)])


def make_target_quat(roll_deg, pitch_deg, yaw_deg):
    """
    각도 세 개로 목표 자세를 만든다.

    roll, pitch 로 접근 방향을 정한 뒤 yaw 를 마지막에 곱한다.
    마지막에 곱하면 툴 로컬 Z축 회전이 되므로
    접근 방향은 유지되고 손가락 방향만 바뀐다.
    """
    q = quat_mul(quat_from_axis([1, 0, 0], roll_deg),
                 quat_from_axis([0, 1, 0], pitch_deg))
    q = quat_mul(q, quat_from_axis([0, 0, 1], yaw_deg))
    return q / np.linalg.norm(q)


def quat_to_matrix(q):
    """
    쿼터니언을 회전행렬로 바꾼다.
    각 열이 로컬 축의 월드 방향이다.
      1열 = 로컬 +X (손가락 방향)
      3열 = 로컬 +Z (툴 방향)
    """
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


# ══════════════════════════════════════════════════════════════
#  TCP 변환
# ══════════════════════════════════════════════════════════════
def tcp_to_flange(tcp_pos, quat):
    """
    손가락 끝 목표를 플랜지 목표로 바꾼다.

    오프셋은 link_6 로컬 좌표이므로 목표 자세만큼 회전시킨 뒤 빼야 한다.
    """
    R = quat_to_matrix(quat)
    return np.array(tcp_pos) - R @ TCP_OFFSET


def get_tcp_pose():
    """link_6 월드 pose로부터 손가락 TCP의 World 위치를 계산한다."""
    pos, quat = get_world_pose(EE_PRIM_PATH)
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


# ══════════════════════════════════════════════════════════════
#  궤적 보간
# ══════════════════════════════════════════════════════════════
def steps_for(start, goal):
    """구간 길이를 속도로 나눠 스텝 수를 정한다"""
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def lerp(start, goal, alpha):
    """시작점에서 목표점까지 선형 보간"""
    return start + alpha * (goal - start)


class ColorIdSubscriber(Node):
    """외부 색상 감지 노드의 /color_id 최신값을 보관한다."""

    def __init__(self):
        super().__init__("pick_place_color_selector")
        self.latest_id = COLOR_ID_NONE
        self._last_logged_id = None
        self.create_subscription(
            Int32,
            "/color_id",
            self._callback,
            QoSProfile(depth=10),
        )
        self.get_logger().info("waiting for /color_id (1=blue, 2=green)")

    def _callback(self, message):
        color_id = int(message.data)
        self.latest_id = (
            color_id if color_id in PLACE_CUBE_CENTER_BY_COLOR_ID
            else COLOR_ID_NONE
        )
        if self.latest_id != self._last_logged_id:
            self.get_logger().info(f"/color_id={self.latest_id}")
            self._last_logged_id = self.latest_id

    def reset(self):
        self.latest_id = COLOR_ID_NONE
        self._last_logged_id = None


class PickPlaceFSM:
    """RMPflow에 한 프레임씩 목표를 공급하는 물리 Pick & Place FSM."""

    NAMES = [
        "PICK_APPROACH",
        "PICK_DESCEND",
        "GRIPPER_CLOSE",
        "WAIT_COLOR",
        "LIFT",
        "PLACE_APPROACH",
        "PLACE_DESCEND",
        "GRIPPER_OPEN",
        "RETREAT",
        "DONE",
    ]
    GRIPPER_STATES = {2: "close", 7: "open"}
    WAIT_COLOR_STATE = 3
    DONE_STATE = 9

    def __init__(self, robot, pick_cube_center, color_id_getter):
        self._robot = robot
        self._pick_cube_center = np.array(pick_cube_center, dtype=float)
        self._color_id_getter = color_id_getter
        self.reset()

    def _build_pick_waypoints(self):
        pick_x, pick_y = self._pick_cube_center[:2]
        lift = np.array([pick_x, pick_y, LIFT_TCP_Z])
        self.waypoints = [
            np.array([pick_x, pick_y, APPROACH_TCP_Z]),
            np.array([pick_x, pick_y, PICK_TCP_Z]),
            np.array([pick_x, pick_y, PICK_TCP_Z]),
            np.array([pick_x, pick_y, PICK_TCP_Z]),
            lift,
            lift.copy(),
            lift.copy(),
            lift.copy(),
            lift.copy(),
        ]

    def _set_place_waypoints(self, color_id):
        place_x, place_y = PLACE_CUBE_CENTER_BY_COLOR_ID[color_id][:2]
        self.waypoints[5] = np.array([place_x, place_y, LIFT_TCP_Z])
        self.waypoints[6] = np.array([place_x, place_y, PLACE_TCP_Z])
        self.waypoints[7] = np.array([place_x, place_y, PLACE_TCP_Z])
        self.waypoints[8] = np.array([place_x, place_y, RETREAT_TCP_Z])

    def reset(self, pick_cube_center=None):
        if pick_cube_center is not None:
            self._pick_cube_center = np.array(pick_cube_center, dtype=float)
        self._build_pick_waypoints()
        self.state = 0
        self.step = 0
        self.start = None
        self.goal = self.waypoints[0]
        self.n_steps = MIN_STEPS
        self.settled_frames = 0
        self.gripper = "open"
        self.selected_color_id = COLOR_ID_NONE
        self._wait_color_logged = False
        print("   motion reset  state=PICK_APPROACH gripper=open")

    def _enter_state(self):
        self.start = get_tcp_pose()
        self.goal = self.waypoints[self.state]
        self.gripper = self.GRIPPER_STATES.get(self.state, self.gripper)

        if self.state == 2:
            self.n_steps = GRIPPER_CLOSE_WAIT_FRAMES
            distance = 0.0
        elif self.state == 7:
            self.n_steps = GRIPPER_OPEN_WAIT_FRAMES
            distance = 0.0
        elif self.state == self.WAIT_COLOR_STATE:
            self.n_steps = 1
            distance = 0.0
        else:
            self.n_steps, distance = steps_for(self.start, self.goal)

        print(
            f"   [{self.state}] {self.NAMES[self.state]:14s} "
            f"target={vec(self.goal)} distance={distance:.4f} m "
            f"frames={self.n_steps} gripper={self.gripper}"
        )

    def current_target(self):
        if self.state >= self.DONE_STATE:
            return self.waypoints[-1]
        if self.start is None:
            self._enter_state()
        alpha = min(1.0, self.step / float(self.n_steps))
        return lerp(self.start, self.goal, alpha)

    def advance(self):
        if self.state >= self.DONE_STATE:
            return

        if self.state == self.WAIT_COLOR_STATE:
            color_id = int(self._color_id_getter())
            if color_id not in PLACE_CUBE_CENTER_BY_COLOR_ID:
                if not self._wait_color_logged:
                    print("   WAIT_COLOR     holding cube; waiting for /color_id 1 or 2")
                    self._wait_color_logged = True
                return

            self.selected_color_id = color_id
            self._set_place_waypoints(color_id)
            center = PLACE_CUBE_CENTER_BY_COLOR_ID[color_id]
            print(
                f"   color route   id={color_id} "
                f"{PLACE_NAME_BY_COLOR_ID[color_id]} center={vec(center)}"
            )
            completed_state = self.state
            self.state += 1
            self.step = 0
            self.start = None
            self.settled_frames = 0
            return completed_state

        self.step += 1
        if self.step < self.n_steps:
            return

        if self.state not in self.GRIPPER_STATES:
            position_error = float(np.linalg.norm(get_tcp_pose() - self.goal))
            if position_error <= TARGET_POSITION_TOLERANCE:
                self.settled_frames += 1
            else:
                self.settled_frames = 0
            if self.settled_frames < TARGET_SETTLE_FRAMES:
                if self.step >= self.n_steps + MAX_TARGET_WAIT_FRAMES:
                    raise RuntimeError(
                        f"{self.NAMES[self.state]} did not converge: "
                        f"position error={position_error:.4f} m"
                    )
                return

        completed_state = self.state
        print(f"   [{completed_state}] {self.NAMES[completed_state]} complete")
        if completed_state == 2:
            finger = self._robot.get_joint_positions()[
                self._robot.get_dof_index("finger_joint")
            ]
            print(
                f"   gripper close wait complete; finger={finger:+.4f}; "
                "cube motion remains physical"
            )

        self.state += 1
        self.step = 0
        self.settled_frames = 0
        self.start = None
        if self.state >= self.DONE_STATE:
            print(f"   [{self.DONE_STATE}] DONE")
        return completed_state

    @property
    def is_done(self):
        return self.state >= self.DONE_STATE


# ══════════════════════════════════════════════════════════════
#  씬 구성 — Task
# ══════════════════════════════════════════════════════════════
def camera_frame_skip_count():
    """렌더 주기에서 목표 카메라 발행 주기에 필요한 skip 수를 계산한다."""
    if CAMERA_PUBLISH_HZ <= 0.0 or CAMERA_PUBLISH_HZ > SIMULATION_RENDER_HZ:
        raise ValueError(
            "CAMERA_PUBLISH_HZ must be greater than 0 and no greater than "
            "SIMULATION_RENDER_HZ"
        )

    render_frames_per_message = SIMULATION_RENDER_HZ / CAMERA_PUBLISH_HZ
    rounded = round(render_frames_per_message)
    if not np.isclose(render_frames_per_message, rounded):
        raise ValueError(
            "SIMULATION_RENDER_HZ must be an integer multiple of "
            "CAMERA_PUBLISH_HZ when frameSkipCount is used"
        )
    return int(rounded) - 1


def find_prim_path(root_path, name):
    """USD 계층에서 이름으로 prim 경로를 찾는다"""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None

    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


class M0609Task(BaseTask):
    """
    set_up_scene 은 BaseTask 가 정한 이름이다. World 가 이 이름으로 부른다.
    _ 로 시작하는 메서드는 우리가 나눈 것이라 이름을 바꿔도 된다.
    """

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None
        self._gripper = None
        self._cube = None
        self._rng = np.random.default_rng(RANDOM_SEED)
        self._pick_cube_center, self._cube_color_name = self._sample_cube_state()

    # ── 프레임워크 규약 ──────────────────────────────────
    def set_up_scene(self, scene):
        """world.reset() 안에서 자동으로 불린다"""
        super().set_up_scene(scene)
        self._load_usd()
        self._setup_default_lighting()
        self._configure_wrist_camera()
        self._setup_arm_drives()
        self._register_robot(scene)
        self._spawn_cube(scene)
        print("   scene        ready")

    # ── 우리가 나눈 단계 ─────────────────────────────────
    def _load_usd(self):
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()

        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()

        # USD에 이미 존재하는 카메라 publisher를 재사용한다.
        ros_graph = stage.GetPrimAtPath("/World/Graph/camera_graph")
        if not ros_graph.IsValid():
            raise RuntimeError("ROS2 camera graph not found: /World/Graph/camera_graph")
        ros_graph.SetActive(True)

        print("   USD          loaded")
        print("   ROS2 graph   /World/Graph/camera_graph enabled")

    def _setup_default_lighting(self):
        """USD 환경과 관계없이 사용할 기본 흰색 조명을 만든다."""
        stage = omni.usd.get_context().get_stage()

        dome = UsdLux.DomeLight.Define(stage, DEFAULT_DOME_LIGHT_PATH)
        dome.CreateIntensityAttr(DEFAULT_DOME_INTENSITY)
        dome.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))

        sun = UsdLux.DistantLight.Define(stage, DEFAULT_SUN_LIGHT_PATH)
        sun.CreateIntensityAttr(DEFAULT_SUN_INTENSITY)
        sun.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
        sun.CreateAngleAttr(1.0)
        UsdGeom.XformCommonAPI(sun.GetPrim()).SetRotate(
            Gf.Vec3f(315.0, 0.0, 315.0)
        )
        print("   lighting      default dome + distant light")

    def _configure_wrist_camera(self):
        """잘못 저장된 RGB Camera 로컬 pose를 원본 D455 값으로 복구한다."""
        stage = omni.usd.get_context().get_stage()
        camera_prim = stage.GetPrimAtPath(WRIST_RGB_CAMERA_PATH)
        if not camera_prim.IsValid() or not camera_prim.IsA(UsdGeom.Camera):
            raise RuntimeError(f"RGB Camera Prim not found: {WRIST_RGB_CAMERA_PATH}")

        camera_prim.GetAttribute("xformOp:translate").Set(
            WRIST_RGB_CAMERA_TRANSLATION
        )
        print(f"   RGB camera   {WRIST_RGB_CAMERA_PATH}")

    def _setup_arm_drives(self):
        """IK 결과를 로봇이 따라가도록 팔 관절의 Drive 를 강화한다"""
        stage = omni.usd.get_context().get_stage()
        count = 0

        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            if prim.GetName() not in ARM_JOINTS:
                continue
            for drive_type in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                    count += 1

        finger_prim = None
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            if prim.GetName() == "finger_joint":
                finger_prim = prim
                break
        if finger_prim is None:
            raise RuntimeError("finger_joint Prim not found")

        finger_drive = UsdPhysics.DriveAPI.Get(finger_prim, "angular")
        if not finger_drive:
            raise RuntimeError("angular drive not found on finger_joint")
        finger_drive.GetStiffnessAttr().Set(GRIPPER_DRIVE_STIFFNESS)
        finger_drive.GetDampingAttr().Set(GRIPPER_DRIVE_DAMPING)
        finger_drive.GetMaxForceAttr().Set(GRIPPER_DRIVE_MAX_FORCE)

        print(f"   arm drives   {count}")
        print(
            f"   grip drive   stiffness={GRIPPER_DRIVE_STIFFNESS:g} "
            f"damping={GRIPPER_DRIVE_DAMPING:g} "
            f"max_force={GRIPPER_DRIVE_MAX_FORCE:g}"
        )

    def _register_robot(self, scene):
        """Articulation root만 Scene에 등록해 child link pose reset 경고를 피한다."""
        ee_path = find_prim_path(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if ee_path != EE_PRIM_PATH:
            raise RuntimeError(
                f"Expected end effector at {EE_PRIM_PATH}, found: {ee_path}"
            )

        self._gripper = ParallelGripper(
            end_effector_prim_path=ee_path,
            joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=np.array([GRIPPER_OPEN_POS] * 2),
            joint_closed_positions=np.array([GRIPPER_CLOSE_POS] * 2),
            action_deltas=None,
        )
        self._robot = scene.add(
            SingleArticulation(
                prim_path=ROBOT_PRIM_PATH,
                name="m0609_robot",
            )
        )
        print(f"   articulation {ROBOT_PRIM_PATH}")
        print(f"   EE frame     {ee_path}")

    def _sample_cube_state(self):
        """도달 가능한 범위에서 실행별 위치와 색상을 하나 선택한다."""
        position = np.array([
            self._rng.uniform(*CUBE_X_RANGE),
            self._rng.uniform(*CUBE_Y_RANGE),
            GROUND_Z + CUBE_SIZE / 2.0,
        ])
        color_name = self._rng.choice(tuple(CUBE_COLORS))
        return position, str(color_name)

    def _spawn_cube(self, scene):
        """랜덤 Pick 위치와 색상으로 큐브 하나를 생성한다."""
        self._cube = scene.add(
            DynamicCuboid(
                prim_path=CUBE_PRIM_PATH,
                name=CUBE_NAME,
                position=self._pick_cube_center,
                size=CUBE_SIZE,
                color=CUBE_COLORS[self._cube_color_name],
            )
        )
        cube_physics_material = PhysicsMaterial(
            prim_path=f"{CUBE_PRIM_PATH}/GraspPhysicsMaterial",
            name="cube_grasp_physics_material",
            static_friction=CUBE_STATIC_FRICTION,
            dynamic_friction=CUBE_DYNAMIC_FRICTION,
            restitution=0.0,
        )
        self._cube.apply_physics_material(cube_physics_material)
        self._cube.set_default_state(
            position=self._pick_cube_center,
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            linear_velocity=np.zeros(3),
            angular_velocity=np.zeros(3),
        )
        print(f"   cube prim     {CUBE_PRIM_PATH}")
        print(f"   cube color    {self._cube_color_name}")
        print(f"   cube pick     {vec(self._pick_cube_center)}")
        for color_id, center in PLACE_CUBE_CENTER_BY_COLOR_ID.items():
            print(
                f"   place id={color_id}  "
                f"{PLACE_NAME_BY_COLOR_ID[color_id]:12s} {vec(center)}"
            )

    def report_grasp_result(self):
        """Lift 후 cube 높이로 실제 물리 파지 여부를 보고한다."""
        cube_position, _ = self._cube.get_world_pose()
        success = float(cube_position[2]) >= GRASP_SUCCESS_MIN_CUBE_Z
        result = "SUCCESS" if success else "FAILED"
        print(
            f"   physical grasp {result}  cube={vec(cube_position)} "
            f"threshold_z={GRASP_SUCCESS_MIN_CUBE_Z:.3f}"
        )
        return success

    def reset_cube(self):
        """Stop→Play마다 기존 cube Prim의 위치와 색상을 다시 추첨한다."""
        self._pick_cube_center, self._cube_color_name = self._sample_cube_state()
        self._cube.set_default_state(
            position=self._pick_cube_center,
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            linear_velocity=np.zeros(3),
            angular_velocity=np.zeros(3),
        )
        self._cube.post_reset()
        self._cube.set_linear_velocity(np.zeros(3))
        self._cube.set_angular_velocity(np.zeros(3))
        self._cube.get_applied_visual_material().set_color(
            CUBE_COLORS[self._cube_color_name]
        )
        print(f"   cube random   color={self._cube_color_name}")
        print(f"   cube reset    {vec(self._pick_cube_center)}")

    @property
    def pick_cube_center(self):
        return self._pick_cube_center.copy()

    @property
    def cube_color_name(self):
        return self._cube_color_name

    @property
    def robot(self):
        return self._robot

    @property
    def gripper(self):
        return self._gripper


def init_gripper(gripper, robot, world):
    """ParallelGripper를 articulation의 관절 함수와 연결한다."""
    gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    gripper.set_default_state(np.array([GRIPPER_OPEN_POS] * 2))


def set_ready_pose(robot):
    """팔과 그리퍼의 시작 자세를 현재값과 reset 기본값에 함께 저장한다."""
    q = np.zeros(robot.num_dof)
    for joint_name, joint_deg in zip(ARM_JOINTS, READY_JOINTS_DEG):
        q[robot.get_dof_index(joint_name)] = np.deg2rad(joint_deg)

    robot.set_joints_default_state(
        positions=q,
        velocities=np.zeros(robot.num_dof),
    )
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros(robot.num_dof))


def print_base_frame_info(robot):
    """실제 articulation base의 World pose와 좌표계 일치 여부를 출력한다."""
    base_position, base_orientation = robot.get_world_pose()
    identity_position = np.zeros(3)
    identity_orientation = np.array([1.0, 0.0, 0.0, 0.0])
    same_origin = np.allclose(base_position, identity_position, atol=1e-6)
    same_axes = np.allclose(
        np.abs(np.dot(base_orientation, identity_orientation)), 1.0, atol=1e-6
    )

    print(f"   base position {vec(base_position, 6)}")
    print(f"   base quat     {vec(base_orientation, 6)}  (w x y z)")
    if same_origin and same_axes:
        print("   base/world    aligned; World targets equal base-frame targets")
    else:
        print(
            "   base/world    differ; RMPflow uses this base World pose "
            "to transform World targets"
        )


def create_rmpflow_controller(robot):
    """프로젝트의 기존 M0609 RMPflow policy/controller를 생성한다."""
    return RMPFlowController(
        name="m0609_rmpflow",
        robot_articulation=robot,
        physics_dt=1.0 / SIMULATION_RENDER_HZ,
        urdf_path=URDF_PATH,
        robot_description_path=RMPFLOW_DESCRIPTION_PATH,
        rmpflow_config_path=RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    )


# ══════════════════════════════════════════════════════════════
#  출력
# ══════════════════════════════════════════════════════════════
def section(title):
    print(f"\n{'─' * 66}")
    print(f" {title}")
    print(f"{'─' * 66}")


def vec(v, digits=3):
    """벡터를 고정폭으로 찍는다"""
    return "[" + " ".join(f"{x:+.{digits}f}" for x in v) + "]"


def print_target_info(target_quat, pick_cube_center, cube_color_name):
    """이번 실행에서 선택된 Pick & Place 목표를 출력한다."""
    rotation = quat_to_matrix(target_quat)

    section("PLAN")
    print(f"   cube color   {cube_color_name}")
    print(f"   pick center  {vec(pick_cube_center)}  World")
    for color_id, center in PLACE_CUBE_CENTER_BY_COLOR_ID.items():
        print(
            f"   place id={color_id}  {PLACE_NAME_BY_COLOR_ID[color_id]:12s} "
            f"{vec(center)} World"
        )
    print(f"   pick TCP z   {PICK_TCP_Z:.3f}")
    print(f"   place TCP z  {PLACE_TCP_Z:.3f}")
    print(f"   tcp offset   {vec(TCP_OFFSET)}  link_6 local")
    print(f"   tcp speed    {TCP_SPEED} m/frame")
    print(f"   gripper      open {GRIPPER_OPEN_POS}  close {GRIPPER_CLOSE_POS}")
    print(
        f"   grip wait    close={GRIPPER_CLOSE_WAIT_FRAMES} "
        f"open={GRIPPER_OPEN_WAIT_FRAMES} frames"
    )
    print(f"   tool +Z      {vec(rotation @ np.array([0, 0, 1]))}")
    print(f"   finger +X    {vec(rotation @ np.array([1, 0, 0]))}")


def print_dof_info(robot):
    """Articulation DOF 이름과 용도를 출력한다."""
    section("DOF")
    for index, name in enumerate(robot.dof_names):
        tag = "arm" if name in ARM_JOINTS else "gripper"
        print(f"   [{index:2d}] {name:28s} {tag}")
    print(f"   finger index {robot.get_dof_index('finger_joint')}")
    print(f"   num_dof      {robot.num_dof}")


def print_status(robot, fsm, target_tcp):
    """현재 상태, 실제 TCP, 그리퍼 구동 관절 위치를 주기적으로 출력한다."""
    name = fsm.NAMES[min(fsm.state, fsm.DONE_STATE)]
    tcp = get_tcp_pose()
    finger = robot.get_joint_positions()[robot.get_dof_index("finger_joint")]
    print(
        f"   {name:14s} target={vec(target_tcp)} "
        f"tcp={vec(tcp)} finger={finger:+.4f}"
    )


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
LOG_INTERVAL = 60


def main():
    world = World(
        physics_dt=1.0 / SIMULATION_RENDER_HZ,
        rendering_dt=1.0 / SIMULATION_RENDER_HZ,
        stage_units_in_meters=1.0,
    )

    section("SCENE")
    task = M0609Task(name="m0609_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    gripper = task.gripper
    robot.initialize(physics_sim_view=world.physics_sim_view)
    init_gripper(gripper, robot, world)
    set_ready_pose(robot)

    print_dof_info(robot)

    section("BASE FRAME")
    print_base_frame_info(robot)

    section("RMPFLOW")
    controller = create_rmpflow_controller(robot)
    print(f"   description   {RMPFLOW_DESCRIPTION_PATH}")
    print(f"   policy        {RMPFLOW_CONFIG_PATH}")
    print(f"   end effector  {EE_LINK_NAME}")

    target_quat = make_target_quat(
        APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG
    )
    print_target_info(
        target_quat, task.pick_cube_center, task.cube_color_name
    )

    section("RUN")
    print("   random-cube physical Pick & Place; /color_id selects Place")
    print("   cube remains dynamic; no pose-follow or Place teleport is used")
    print("   grasp result is inferred from cube height after Lift")
    print("   press Stop, then Play to reset cube, robot, RMPflow, and motion state")

    if not rclpy.ok():
        rclpy.init(args=[])
    color_subscriber = ColorIdSubscriber()
    fsm = PickPlaceFSM(
        robot, task.pick_cube_center, lambda: color_subscriber.latest_id
    )
    was_playing = world.is_playing()
    control_step = 0

    try:
        while simulation_app.is_running():
            world.step(render=True)
            rclpy.spin_once(color_subscriber, timeout_sec=0.0)
            is_playing = world.is_playing()
    
            if is_playing and not was_playing:
                world.reset()
                robot.initialize(physics_sim_view=world.physics_sim_view)
                init_gripper(gripper, robot, world)
                set_ready_pose(robot)
                task.reset_cube()
                controller.reset()
                color_subscriber.reset()
                fsm.reset(task.pick_cube_center)
                control_step = 0
                print("   restart       controller and sequence initialized")
    
            if is_playing:
                target_tcp = fsm.current_target()
                flange_target = tcp_to_flange(target_tcp, target_quat)
    
                arm_action = controller.forward(
                    target_end_effector_position=flange_target,
                    target_end_effector_orientation=target_quat,
                )
                robot.apply_action(arm_action)
                robot.apply_action(gripper.forward(action=fsm.gripper))
                completed_state = fsm.advance()

                if completed_state == 2:
                    # 파지 전에 수신한 판정은 사용하지 않는다.
                    # Close 대기가 끝난 뒤 새로 들어온 /color_id만 확정한다.
                    color_subscriber.reset()
                    print("   color reset   waiting for post-grasp /color_id")

                if completed_state == 4:
                    task.report_grasp_result()
    
                if control_step % LOG_INTERVAL == 0:
                    print_status(robot, fsm, target_tcp)
                control_step += 1
    
            was_playing = is_playing
    finally:
        color_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    finally:
        # Python 예외나 창 닫기에서도 ROS camera graph와 renderer를
        # SimulationApp 생명주기 안에서 정리한다.
        simulation_app.close()
