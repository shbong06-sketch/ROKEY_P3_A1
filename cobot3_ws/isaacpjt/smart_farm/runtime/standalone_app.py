"""Isaac Sim 안에서 Sim Task ROS 노드와 팔레트 이송을 함께 실행한다.

실행 예:
    # Task Manager 명령 대기
    ~/isaacsim/python.sh runtime/standalone_app.py --autoplay

    # robot_motion_standalone.py처럼 TRANSFER를 즉시 검증
    ~/isaacsim/python.sh runtime/standalone_app.py --demo
"""

import argparse
import json
import math
import os
import sys
import traceback
from dataclasses import dataclass
from pathlib import Path


RUNTIME_DIR = Path(__file__).resolve().parent
PROJECT_DIR = RUNTIME_DIR.parent
SCRIPTS_DIR = PROJECT_DIR / "scripts"

DEFAULT_SCENE_PATH = (
    PROJECT_DIR
    / "scenes"
    / "Collected_smartfarm_v011"
    / "Collected_smartfarm_v011.usd"
)
# [올인원 2026-09-25] 양배추 씬 v014(공유 zip 을 scenes/Collected_smartfarm_v014 에 푼 것)가 있으면 --scene 없이도 그 씬을 연다.
CABBAGE_SCENE_PATH = (
    PROJECT_DIR
    / "scenes"
    / "Collected_smartfarm_v014"
    / "Collected_smartfarm_v014_room_core_cabbage.usd"
)
if CABBAGE_SCENE_PATH.exists():
    DEFAULT_SCENE_PATH = CABBAGE_SCENE_PATH

PHYSICS_DT = 1.0 / 60.0
BASE_SETTLE_SECONDS = 0.5           # [navigation 2026-09-23] Place 전 차체가 멈춰 있어야 하는 시간
BASE_SETTLE_TIMEOUT = 15.0          # [navigation 2026-09-23] 이 시간까지 안 멈추면 실패로 본다
RENDER_EVERY = 3
SETTLE_STEPS = 120
CARRY_ROTATE_DEG = 90.0
TRAVEL_BASE_HEIGHT = 1.0388
TRAVEL_HEIGHT_TOL = 0.015

RIG_PATH = "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS"
ROBOT_PATH = f"{RIG_PATH}/chassis_link"
ARM_PATH = f"{RIG_PATH}/m0609_with_fork"
ARM_BASE_PATH = f"{ARM_PATH}/base_link"
LIFT_JOINT_PATH = f"{RIG_PATH}/lift_v3_physics/lift_prismatic_joint"
LIFT_JOINT_NAME = "lift_prismatic_joint"
TURNTABLE_PATH = "/World/SmartFarm/Placed/Conveyor/TurnTable"
TURNTABLE_SURFACE_PATH = (
    f"{TURNTABLE_PATH}/Geometry/SM_ConveyorBelt_A08_Roller35_01"
)

RACK_FRONT_X = -1.205
BASE_BELOW_SHELF = 0.213
# v008은 팔레트 원점이 밑면보다 25.9 mm 위에 있다.
PALLET_ORIGIN_ABOVE_BOTTOM = 0.0259   # [place-fix 2026-09-25] TurnTable 놓기 높이에도 같은 규칙을 쓴다
# [place-fix 2026-09-25] TurnTable 은 롤러 컨베이어다. 놓은 뒤 포크를 뺄 때 포크판이 롤러 사이에 걸리지 않도록
# 포크판(DOCK_X)이 TurnTable 가장자리보다 이만큼 바깥에 오게 놓는다. 랙에서는 선반 앞 끝보다 39 mm 바깥이다.
PLACE_PLATE_EDGE_CLEARANCE = 0.035
SHELF_TOP = {
    1: 0.7388,
    2: 1.0388,
    3: 1.3388,
    4: 1.6388,
    5: 1.9388,
}

PALLET_ASSET_NAME = (
    "Cube_011_001"
)
# [올인원 2026-09-25] 컨베이어(scripts/conveyor.py, refactor/lift-robot-motion)가 감시할 팔레트 루트.
# 벨트에 이미 올라와 있는 검사 트레이도 모두 넣어야 한다(감시 밖 강체도 롤러가 밀어낸다).
# 랙 팔레트는 로봇이 들고 돌리므로 요 고정을 벨트에 올라온 순간 건다(carried).
CONVEYOR_CARRIED_PALLETS = tuple(f"/World/SmartFarm/Placed/Pallet_0{i}" for i in (1, 2, 3))
CONVEYOR_BELT_PALLETS = tuple(
    f"/World/SmartFarm/Placed/Pallet_Inspect{suffix}" for suffix in ("", "_01", "_02", "_03")
)
# 비전 노드가 아직 붙지 않았으므로 카메라 앞에서 이 시간 뒤 스스로 내보낸다. 비전 연동 시 None.
CONVEYOR_AUTO_RESUME_SECONDS = 10.0
# [올인원 2026-09-25] 비전룸 검사·솎아내기 스테이션(scripts/inspection_cull_station.py)을 켜면 스테이션이 끝낼 때
# inspection_done() 을 부르므로 자동 배출은 끈다. 트레이를 비전 M0609 base 와 같은 x 에 세운다(도달거리).
VISION_STATION_STOP_X = -0.69
# 장면의 Pallet_Inspect* 는 시험용 배치(y -7.0 / -6.58)라 conveyor.py 가 세우는 옆가이드(y -6.56 / -6.94)에
# 걸친다. 그대로 두면 가이드가 트레이를 관통한 채 생겨 트레이와 작물이 튕겨 나간다.
# conveyor_standalone.py 와 같이 줄기 서쪽 빈 바닥에 세워 두고 시작한다(세션 레이어, 장면 파일은 그대로).
CONVEYOR_PARK = (-3.40, -5.60, 0.03, 0.60)   # x, 첫 y, z, 간격

PALLET_1_PATH = f"/World/SmartFarm/Placed/Pallet_01/{PALLET_ASSET_NAME}"
PALLET_2_PATH = f"/World/SmartFarm/Placed/Pallet_02/{PALLET_ASSET_NAME}"
PALLET_3_PATH = f"/World/SmartFarm/Placed/Pallet_03/{PALLET_ASSET_NAME}"

M0609_DIR = PROJECT_DIR.parent / "M0609"
URDF_PATH = M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf"
DESCRIPTION_PATH = M0609_DIR / "descriptor/m0609_description.yaml"


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene",
        type=Path,
        default=DEFAULT_SCENE_PATH,
        help="통합에 사용할 USD Scene",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--autoplay",
        action="store_true",
        help="timeline을 자동 재생하고 ROS 명령을 기다림",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="timeline 재생 후 검증용 TRANSFER 명령을 자동 발행",
    )
    parser.add_argument(
        "--no-conveyor",
        action="store_true",
        help="[올인원 2026-09-25] 컨베이어 반송(scripts/conveyor.py)을 끄고 실행",
    )
    parser.add_argument(
        "--no-vision-station",
        action="store_true",
        help="[올인원 2026-09-25] 비전 검사·솎아내기 스테이션을 끄고 실행 (컨베이어가 10초 뒤 스스로 배출)",
    )
    parser.add_argument(
        "--human-crossing",
        action="store_true",
        help="[올인원 2026-09-25] 사람 돌발상황: 카터가 통로를 나올 때 작업자가 앞을 막았다가 비킨다 (scripts/human_crossing.py)",
    )
    return parser.parse_known_args()


def configure_ros_environment():
    """Isaac Sim 내장 ROS 2 Jazzy 라이브러리와 Python 모듈을 준비한다."""

    isaac_root = Path(
        os.environ.get("ISAAC_PATH")
        or os.path.abspath(
            os.path.join(
                os.path.dirname(os.path.abspath(sys.executable)),
                "..",
                "..",
                "..",
            )
        )
    )
    bridge_dir = isaac_root / "exts" / "isaacsim.ros2.bridge"
    ros_bundle = bridge_dir / "jazzy"
    ros_lib = ros_bundle / "lib"
    ros_python = ros_bundle / "rclpy"

    if not ros_lib.is_dir() or not ros_python.is_dir():
        raise RuntimeError(
            "Isaac Sim ROS 2 Jazzy bundle을 찾지 못했습니다: "
            f"{ros_bundle}"
        )

    current_paths = [
        item
        for item in os.environ.get("LD_LIBRARY_PATH", "").split(":")
        if item
    ]

    # 시스템 ROS와 Isaac 번들의 같은 이름 라이브러리가 섞이면 rclpy가
    # import되어도 Node 생성 시 ABI 충돌로 종료될 수 있다.
    system_ros = [
        item for item in current_paths if item.startswith("/opt/ros/")
    ]
    wanted = [str(ros_lib)] + [
        item
        for item in current_paths
        if item != str(ros_lib) and not item.startswith("/opt/ros/")
    ]

    if current_paths != wanted:
        os.environ["LD_LIBRARY_PATH"] = ":".join(wanted)
        os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

        if os.environ.get("SMARTFARM_ROS_REEXEC") == "1":
            print(
                "[ROS2] 경고 — LD_LIBRARY_PATH를 바로잡지 못했습니다. "
                "ROS를 source하지 않은 터미널에서 실행해 보세요.",
                flush=True,
            )
        else:
            os.environ["SMARTFARM_ROS_REEXEC"] = "1"
            if system_ros:
                print(
                    f"[ROS2] 시스템 ROS 경로 {len(system_ros)}개를 빼고 "
                    "Isaac 번들을 씁니다.",
                    flush=True,
                )
            print(
                f"[ROS2] LD_LIBRARY_PATH 맨 앞에 {ros_lib}를 두고 "
                "다시 실행합니다.",
                flush=True,
            )
            os.execv(sys.executable, [sys.executable, *sys.argv])

    # 시스템 Jazzy는 Python 3.12용이므로 Isaac Sim Python 3.11에서는
    # 반드시 Isaac Sim에 포함된 rclpy를 먼저 import해야 한다.
    sys.path.insert(0, str(ros_python))


args, kit_args = parse_args()
if os.environ.get("HEADLESS") == "1":
    args.headless = True
args.autoplay = args.autoplay or args.headless or args.demo

configure_ros_environment()

from isaacsim import SimulationApp  # noqa: E402


app = SimulationApp(
    {
        "headless": args.headless,
        "extra_args": kit_args,
    }
)

import omni.usd  # noqa: E402
import rclpy  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import (  # noqa: E402
    SingleArticulation,
    SingleRigidPrim,
    SingleXFormPrim,
)
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.robot.manipulators.manipulators import (  # noqa: E402
    SingleManipulator,
)
from isaacsim.robot_motion.motion_generation import (  # noqa: E402
    LulaKinematicsSolver,
)

sys.path.insert(0, str(SCRIPTS_DIR))

from lift import LiftController, check_fork_clear_of_rack  # noqa: E402
from pallet_transfer import (  # noqa: E402
    PalletTransferController,
    TransferState,
)
from robot_motion import (  # noqa: E402
    DOCK_X,
    BaseWatcher,
    EE_FRAME,
    RobotMotion,
    Task,
    brake_wheels,
    brake_wheels_at_current_position,
    release_wheels,
    tine_tip_position,
)
from sim_task_node import SimTaskNode  # noqa: E402


EE_PATH = f"{ARM_PATH}/{EE_FRAME}"


@dataclass(frozen=True)
class TransferUnit:
    """한 번의 팔레트 PICK/PLACE와 결과에 기록할 논리 ID."""

    task: Task
    result_id: str


TRANSFER_UNITS = (
    TransferUnit(
        task=Task(PALLET_2_PATH, SHELF_TOP[2]),
        result_id="PALLET_002:RACK_L3:RACK_L2",
    ),
    TransferUnit(
        task=Task(PALLET_3_PATH, SHELF_TOP[3]),
        result_id="PALLET_003:RACK_L4:RACK_L3",
    ),
)


HARVEST_TASK = Task(PALLET_1_PATH, None, pick_only=True)


class ArticulationLinkManipulator(SingleManipulator):
    """비루트 end-effector 링크를 독립 강체처럼 reset하지 않는다."""

    def post_reset(self):
        # SingleManipulator.post_reset()은 link_6에 world transform과
        # velocity를 직접 적용한다. link_6는 articulation이 구동하므로
        # articulation root/joint만 reset하고 end-effector는 pose 조회용으로
        # 유지한다.
        SingleArticulation.post_reset(self)


@dataclass
class SimulationRuntime:
    """Standalone 루프가 소유하는 Isaac Sim 객체."""

    world: World
    stage: object
    robot: ArticulationLinkManipulator
    lift: LiftController
    motion: RobotMotion
    transfer: PalletTransferController
    pallets: dict
    harvest_phase: str = "IDLE"
    wheels_released: bool = False
    base_watcher: object = None
    arm_base: object = None
    place_phase: str = "IDLE"
    place_wait_seconds: float = 0.0
    hold_fault_logged: bool = False   # [navigation 2026-09-24] 운반 중 팔레트 감시 예외를 한 번만 기록
    conveyor: object = None           # [올인원 2026-09-25] scripts/conveyor.ConveyorController (--no-conveyor 면 None)
    station: object = None            # [올인원 2026-09-25] scripts/inspection_cull_station.VisionCullStation
    human: object = None              # [올인원 2026-09-25] scripts/human_crossing.HumanCrossing (--human-crossing)


class TransferOperation:
    """Task Manager의 TRANSFER 한 건을 내부 팔레트 이송 두 건으로 실행한다."""

    def __init__(self, controller, pallets, units):
        self._controller = controller
        self._pallets = pallets
        self._units = tuple(units)
        self._index = 0
        self._completed_units = []
        self._running = False

    @property
    def is_running(self):
        return self._running

    @property
    def completed_units(self):
        return tuple(self._completed_units)

    @property
    def phase(self):
        unit_number = min(self._index + 1, len(self._units))
        return (
            f"TRANSFER_UNIT_{unit_number:02d}/"
            f"{self._controller.state.value}"
        )

    @property
    def controller_state(self):
        return self._controller.state.value

    def reset(self):
        self._controller.cancel()
        if self._controller.state == TransferState.FAILED:
            raise RuntimeError(
                f"팔레트 이송 제어기 정지 실패: {self._controller.error}"
            )

        self._index = 0
        self._completed_units.clear()
        self._running = False

    def start(self):
        if not self._units:
            raise RuntimeError("TRANSFER 작업 목록이 비어 있습니다.")
        if self._running:
            raise RuntimeError("TRANSFER가 이미 실행 중입니다.")

        self.reset()
        self._running = True
        self._start_current_unit()

    def update(self, dt):
        if not self._running:
            return False

        self._controller.update(dt)

        if self._controller.state == TransferState.FAILED:
            raise RuntimeError(str(self._controller.error))

        if self._controller.state != TransferState.SUCCEEDED:
            return False

        unit = self._units[self._index]
        self._completed_units.append(unit.result_id)
        self._index += 1

        if self._index >= len(self._units):
            self._running = False
            return True

        # PalletTransferController는 SUCCEEDED에서 다음 start()를 허용하며,
        # completed task 수를 보존해 두 번째 작업을 현재 자세에서 이어 간다.
        self._start_current_unit()
        return False

    def cancel(self):
        self._controller.cancel()
        self._running = False
        if self._controller.state == TransferState.FAILED:
            raise RuntimeError(
                f"팔레트 이송 제어기 정지 실패: {self._controller.error}"
            )

    def _start_current_unit(self):
        unit = self._units[self._index]
        pallet = self._pallets.get(unit.task.pallet_path)
        if pallet is None:
            raise RuntimeError(
                f"팔레트 prim이 등록되지 않았습니다: {unit.task.pallet_path}"
            )

        self._controller.start(unit.task, pallet)
        if self._controller.state == TransferState.FAILED:
            raise RuntimeError(str(self._controller.error))


def validate_paths(scene_path):
    required_files = (scene_path, URDF_PATH, DESCRIPTION_PATH)
    missing = [str(path) for path in required_files if not path.is_file()]
    if missing:
        raise FileNotFoundError(
            "필수 파일을 찾지 못했습니다: " + ", ".join(missing)
        )


def open_scene(scene_path):
    validate_paths(scene_path)

    enable_extension("isaacsim.ros2.bridge")
    app.update()

    omni.usd.get_context().open_stage(str(scene_path))
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        app.update()

    for _ in range(15):
        app.update()

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError(f"USD Scene을 열지 못했습니다: {scene_path}")

    stage.SetEditTarget(stage.GetSessionLayer())

    # 한 프레임의 일부 각도가 아니라 360도 point cloud를 발행하게 한다.
    # 세션 레이어만 수정하므로 원본 USD 파일은 변경되지 않는다.
    try:
        helper_count = 0
        for prim in stage.Traverse():
            if (
                prim.GetTypeName() == "OmniGraphNode"
                and str(prim.GetAttribute("node:type").Get() or "").endswith(
                    "ROS2RtxLidarHelper"
                )
                and str(prim.GetAttribute("inputs:type").Get() or "")
                == "point_cloud"
            ):
                prim.GetAttribute("inputs:fullScan").Set(True)
                helper_count += 1
        print(
            f"[라이다] 3D 라이다 fullScan=True ({helper_count}개 helper)",
            flush=True,
        )
    except Exception as error:  # noqa: BLE001
        print(f"[라이다] fullScan 설정 실패 (무시): {error}", flush=True)

    return stage


def require_prims(stage, paths):
    missing = []
    for path in paths:
        is_valid = stage.GetPrimAtPath(path).IsValid()
        state = "OK" if is_valid else "MISSING"
        print(f"[시작] prim 확인 [{state}]: {path}", flush=True)
        if not is_valid:
            missing.append(path)
            parent_path = path.rsplit("/", 1)[0]
            parent = stage.GetPrimAtPath(parent_path)
            if parent.IsValid():
                children = ", ".join(
                    f"{child.GetName()}(type={child.GetTypeName()}, "
                    f"rigid={child.HasAPI(UsdPhysics.RigidBodyAPI)})"
                    for child in parent.GetChildren()
                )
                print(
                    f"[시작] {parent_path} 하위 prim: {children}",
                    flush=True,
                )

    if missing:
        raise RuntimeError(
            "Scene에서 필수 prim을 찾지 못했습니다: " + ", ".join(missing)
        )


def turntable_place_pose(stage, arm_base_position=None):
    """TurnTable 표면 중심(팔레트 원점 높이)과 놓을 방향을 world pose로 반환합니다.

    arm_base_position 을 주면 놓을 방향을 '놓을 자리 -> 팔 베이스' 쪽으로 맞춘다(place-fix 2026-09-25).
    """
    surface_prim = stage.GetPrimAtPath(TURNTABLE_SURFACE_PATH)
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_],
        useExtentsHint=True,
    )
    surface_range = bbox_cache.ComputeWorldBound(
        surface_prim
    ).ComputeAlignedRange()
    if surface_range.IsEmpty():
        raise RuntimeError(
            f"TurnTable 표면 bound를 계산하지 못했습니다: "
            f"{TURNTABLE_SURFACE_PATH}"
        )

    minimum = surface_range.GetMin()
    maximum = surface_range.GetMax()
    # [place-fix 2026-09-25] place 단계 좌표의 z 는 팔레트 '원점' 높이다(랙의 SHELF_TOP 과 같은 규칙).
    # 팔레트 원점은 밑면보다 25.9 mm 위이므로 벨트 윗면에 그만큼 더해야 벨트를 파고들지 않는다.
    position = (
        float((minimum[0] + maximum[0]) * 0.5),
        float((minimum[1] + maximum[1]) * 0.5),
        float(maximum[2]) + PALLET_ORIGIN_ABOVE_BOTTOM,
    )

    turntable_prim = stage.GetPrimAtPath(TURNTABLE_PATH)
    transform = UsdGeom.Xformable(
        turntable_prim
    ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    # [place-fix 2026-09-25] TurnTable 행렬에는 배율이 섞여 있어 ExtractRotationQuat 가 정규화되지 않은
    # 값(0.507, 0, 0, 0)을 준다. 그 값이 그대로 IK 목표 자세에 곱해지므로 정규화한다.
    rotation = transform.ExtractRotationQuat()
    imaginary = rotation.GetImaginary()
    w, x, y, z = (
        float(rotation.GetReal()),
        float(imaginary[0]),
        float(imaginary[1]),
        float(imaginary[2]),
    )
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    quaternion = (w / norm, x / norm, y / norm, z / norm)

    if arm_base_position is not None:
        # [place-fix 2026-09-25] PLACE_STAGES 는 '+x = 놓을 자리에서 로봇 쪽'을 가정한다(랙 팔레트 좌표와 같다).
        # TurnTable 의 +x(동쪽)를 그대로 쓰면 북쪽(yaw 90)에서 도킹한 로봇과 90° 어긋나 DESCEND_1 에서
        # IK 가 뒤집혔다(관절 110° 변화, 로메인 원본 장면에서도 재현). 놓을 자리에서 팔 베이스를 향하는
        # 방향을 TurnTable 축 기준 90° 단위로 맞춰 쓴다: 팔레트가 벨트와 나란히 놓이고, 도킹 방향이
        # 바뀌어도 그대로 동작한다.
        turntable_yaw = 2.0 * math.atan2(quaternion[3], quaternion[0])
        to_arm = math.atan2(
            float(arm_base_position[1]) - position[1],
            float(arm_base_position[0]) - position[0],
        )
        quarter = round((to_arm - turntable_yaw) / (math.pi / 2.0))
        yaw = turntable_yaw + quarter * (math.pi / 2.0)
        quaternion = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))

        # 로봇 쪽 TurnTable 가장자리까지의 거리(접근 방향 기준). 롤러 가운데(-3.905)에 놓으면 포크판이
        # 가장자리(-3.597)보다 12 cm 안쪽에서 롤러 높이까지 내려가, 빼낼 때 롤러 사이에 걸렸다
        # (fork_tool 고정 조인트가 늘어나며 EXIT 에서 팔레트가 20 mm 따라 올라옴).
        ux, uy = math.cos(yaw), math.sin(yaw)
        table_range = bbox_cache.ComputeWorldBound(turntable_prim).ComputeAlignedRange()
        corners = [
            (cx, cy)
            for cx in (table_range.GetMin()[0], table_range.GetMax()[0])
            for cy in (table_range.GetMin()[1], table_range.GetMax()[1])
        ]
        edge = max((cx - position[0]) * ux + (cy - position[1]) * uy for cx, cy in corners)
        shift = edge - DOCK_X + PLACE_PLATE_EDGE_CLEARANCE
        position = (position[0] + shift * ux, position[1] + shift * uy, position[2])

    return position, quaternion


def create_simulation_runtime(scene_path):
    human_skel = None
    if args.human_crossing:
        # [올인원 2026-09-25] 사람은 장면을 열기 전에 넣어야 걷기 애니메이션이 붙는다 -> 원래 장면을 감싼 .usda 를 연다
        import tempfile
        import human_crossing

        scene_path, human_skel = human_crossing.prepare_scene(
            scene_path, Path(tempfile.gettempdir()) / "smartfarm_human", app.update)
        print(f"[사람] 작업자를 넣은 장면: {scene_path}", flush=True)
    stage = open_scene(scene_path)
    print("[시작] USD Scene 로딩이 완료되었습니다.", flush=True)

    pallet_paths = (
        *(unit.task.pallet_path for unit in TRANSFER_UNITS),
        HARVEST_TASK.pallet_path,
    )
    require_prims(
        stage,
        (
            ROBOT_PATH,
            ARM_BASE_PATH,
            EE_PATH,
            LIFT_JOINT_PATH,
            TURNTABLE_PATH,
            TURNTABLE_SURFACE_PATH,
            *pallet_paths,
        ),
    )

    print("[시작] 필수 prim 검증이 완료되었습니다.", flush=True)
    brake_wheels(stage, RIG_PATH)

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=PHYSICS_DT,
        rendering_dt=PHYSICS_DT,
        physics_prim_path="/physicsScene",
    )
    print("[시작] World를 생성했습니다.", flush=True)
    robot = world.scene.add(
        ArticulationLinkManipulator(
            prim_path=ROBOT_PATH,
            name="carter_m0609",
            end_effector_prim_path=EE_PATH,
        )
    )
    arm_base = world.scene.add(
        SingleXFormPrim(
            prim_path=ARM_BASE_PATH,
            name="arm_base",
        )
    )

    pallets = {}
    for index, pallet_path in enumerate(dict.fromkeys(pallet_paths)):
        pallets[pallet_path] = world.scene.add(
            SingleRigidPrim(
                prim_path=pallet_path,
                name=f"transfer_pallet_{index}",
            )
        )

    print("[시작] Scene 객체 등록이 완료되었습니다.", flush=True)
    conveyor = None
    if not args.no_conveyor:
        # [올인원 2026-09-25] conveyor.install() 은 world.reset() 전에, attach() 는 뒤에 (conveyor.py 계약)
        from conveyor import install as install_conveyor

        park_x, park_y, park_z, park_gap = CONVEYOR_PARK
        for index, path in enumerate(CONVEYOR_BELT_PALLETS):
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid() or not prim.IsActive():   # 양배추 씬 복사본에서는 시험 트레이를 지웠다
                continue
            for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                if op.GetOpName() == "xformOp:translate":
                    op.Set(type(op.Get())(park_x, park_y + index * park_gap, park_z))
                    print(f"[컨베이어] {prim.GetName()} 시험 배치를 벨트 밖에 세움 "
                          f"({park_x:+.2f}, {park_y + index * park_gap:+.2f})", flush=True)

        watched = [
            path
            for path in CONVEYOR_CARRIED_PALLETS + CONVEYOR_BELT_PALLETS
            if stage.GetPrimAtPath(path).IsValid() and stage.GetPrimAtPath(path).IsActive()
        ]
        use_station = not args.no_vision_station
        conveyor = install_conveyor(
            stage,
            watched,
            auto_resume=None if use_station else CONVEYOR_AUTO_RESUME_SECONDS,
            carried_paths=CONVEYOR_CARRIED_PALLETS,
            **({"vision_x": VISION_STATION_STOP_X} if use_station else {}),
        )
    station = None
    if conveyor is not None and not args.no_vision_station:
        # [올인원 2026-09-25] 비전 M0609 + 손목 RealSense + YOLO(best.pt) + 팀 CullMotion. world.reset() 전에 등록.
        enable_extension("omni.replicator.core")
        import inspection_cull_station

        station = inspection_cull_station.install(stage, world, M0609_DIR)
    world.reset()
    if conveyor is not None:
        conveyor.attach()
    if station is not None:
        station.attach(conveyor)
    print("[시작] World reset이 완료되었습니다.", flush=True)
    world.pause()

    lift = LiftController(
        robot,
        stage,
        arm_base,
        LIFT_JOINT_PATH,
        LIFT_JOINT_NAME,
    )
    solver = LulaKinematicsSolver(
        robot_description_path=str(DESCRIPTION_PATH),
        urdf_path=str(URDF_PATH),
    )
    motion = RobotMotion(
        robot,
        arm_base,
        solver,
        stage,
        ARM_PATH,
    )
    motion.initialize()
    print("[시작] 모션 제어기 초기화가 완료되었습니다.", flush=True)

    base_watcher = BaseWatcher()
    # [navigation 2026-09-23] Place 전 검사용 감시기. transfer 가 쓰는 base_watcher 와 섞지 않는다.
    place_watcher = BaseWatcher()
    transfer = PalletTransferController(
        lift,
        motion,
        arm_base,
        base_watcher,
        BASE_BELOW_SHELF,
        lambda: check_fork_clear_of_rack(
            tine_tip_position(robot)[0],
            RACK_FRONT_X,
        ),
    )

    human = None
    if human_skel is not None:
        import human_crossing

        human = human_crossing.HumanCrossing(
            human_skel, lambda: robot.get_world_pose()[0], os.environ.get("SMARTFARM_STATION_OUT"))

    return SimulationRuntime(
        human=human,
        base_watcher=place_watcher,
        arm_base=arm_base,
        world=world,
        stage=stage,
        robot=robot,
        lift=lift,
        motion=motion,
        transfer=transfer,
        pallets=pallets,
        conveyor=conveyor,
        station=station,
    )


def initialize_scene(runtime, transfer_operation, step_world):
    """Stop 후 Play를 포함해 Scene과 제어기를 초기 상태로 맞춘다."""

    transfer_operation.reset()
    runtime.harvest_phase = "IDLE"
    runtime.place_phase = "IDLE"
    runtime.place_wait_seconds = 0.0
    runtime.wheels_released = False
    if runtime.conveyor is not None:
        runtime.conveyor.reset()        # Stop -> Play: reset() -> world.reset() -> attach() (conveyor.py 계약)
    if runtime.station is not None:
        runtime.station.reset()
    runtime.world.reset()
    if runtime.conveyor is not None:
        runtime.conveyor.attach()
    if runtime.station is not None:
        runtime.station.attach(runtime.conveyor)
    brake_wheels(runtime.stage, RIG_PATH)

    print(f"[장면] 안정화를 위해 {SETTLE_STEPS} physics step을 진행합니다.")
    for _ in range(SETTLE_STEPS):
        step_world()

    runtime.lift.calibrate()


def report_dock_pose(runtime):
    """[navigation 2026-09-23] Place 직전 카터 본체가 실제로 어디에 멈췄는지 기록한다.

    Navigation 이 보고한 값과 Isaac 안의 실제 위치를 대조하기 위한 자료이며,
    허용 범위 판정은 실측 자료가 쌓인 뒤 도입한다. 지금은 기록만 한다.
    """
    chassis_position, _ = runtime.robot.get_world_pose()
    place_position, _ = turntable_place_pose(runtime.stage)
    chassis_x = float(chassis_position[0])
    chassis_y = float(chassis_position[1])
    offset_x = float(place_position[0]) - chassis_x
    offset_y = float(place_position[1]) - chassis_y
    distance = (offset_x * offset_x + offset_y * offset_y) ** 0.5
    print(
        f"[도킹] 카터 본체 world ({chassis_x:.3f}, {chassis_y:.3f}), "
        f"place 대상까지 x {offset_x:+.3f} m, y {offset_y:+.3f} m, "
        f"직선 {distance:.3f} m",
        flush=True,
    )


def start_place_motion(runtime, node):
    """[navigation 2026-09-23] 차체가 멈춘 것을 확인한 뒤 브레이크를 걸고 팔 동작을 시작한다."""
    brake_wheels_at_current_position(
        runtime.stage,
        RIG_PATH,
        runtime.robot,
    )
    runtime.wheels_released = False
    runtime.lift.hold()
    report_dock_pose(runtime)
    arm_base_position, _ = runtime.arm_base.get_world_pose()
    position, quaternion = turntable_place_pose(runtime.stage, arm_base_position)
    # 팔 베이스(차체보다 0.20 m 뒤)에서 놓을 팔레트 원점까지의 수평 거리. 기록용(판정 없음).
    # 2026-09-25 실측: 0.755 m 에서 놓기·포크 인출 성공 (팔 베이스가 대상보다 0.24 m 높아 랙 범위와 다르다).
    reach = math.hypot(
        float(arm_base_position[0]) - position[0],
        float(arm_base_position[1]) - position[1],
    )
    print(
        f"[Place] 목표 팔레트 원점 {[round(value, 4) for value in position]}, "
        f"놓는 방향 yaw {math.degrees(2.0 * math.atan2(quaternion[3], quaternion[0])):+.1f}°, "
        f"팔 베이스까지 {reach:.3f} m",
        flush=True,
    )
    if runtime.conveyor is not None:
        runtime.conveyor.hold_stem(True)   # [올인원 2026-09-25] 놓는 동안 줄기 벨트 인터록
    runtime.motion.start_place_at_pose(position, quaternion)
    runtime.place_phase = "ARM_PLACE"
    node.set_phase(
        "PLACE_INSPECT/ARM_PLACE",
        detail=(
            f"target={TURNTABLE_SURFACE_PATH}, "
            f"position={[round(value, 4) for value in position]}"
        ),
    )


def start_operation(command, runtime, transfer_operation, node):
    if command.operation == "TRANSFER":
        if command.recipe_id not in ("", "RACK_REARRANGE_01"):
            node.fail(
                reason="INVALID_COMMAND",
                phase="COMMAND_DISPATCH",
                reset_required=False,
            )
            return

        transfer_operation.start()
        node.set_phase(
            transfer_operation.phase,
            detail="physical transfer started",
        )
        return

    if command.operation == "PLACE_INSPECT":
        if (
            command.recipe_id not in ("", "PLACE_AT_INSPECTION")
            or command.pallet_id != "PALLET_001"
            or command.source != "CARRY"
            or command.destination != "INSPECT_STATION"
        ):
            node.fail(
                reason="INVALID_COMMAND",
                phase="COMMAND_DISPATCH",
                reset_required=False,
            )
            return

        # [navigation 2026-09-23] 인터페이스 설계 검증 항목 7: 베이스 정지를 확인한 뒤 Place 를 시작한다.
        # 즉시 실패시키지 않고 BASE_SETTLE_TIMEOUT 까지 기다린다. 대기 중에도 진행 상황을 남긴다.
        runtime.place_phase = "WAIT_BASE_SETTLED"
        runtime.place_wait_seconds = 0.0
        print(
            f"[Place] 차체 정지를 확인합니다 "
            f"(현재 {runtime.base_watcher.still_seconds:.2f}초 / 필요 {BASE_SETTLE_SECONDS:.2f}초)",
            flush=True,
        )
        node.set_phase(
            "PLACE_INSPECT/WAIT_BASE_SETTLED",
            detail=f"still={runtime.base_watcher.still_seconds:.2f}s",
        )
        return

    if command.operation != "PICK_HARVEST":
        node.fail(
            reason="INVALID_COMMAND",
            phase="COMMAND_DISPATCH",
            reset_required=False,
        )
        return

    if (
        command.recipe_id not in ("", "HARVEST_RACK_L1")
        or command.pallet_id != "PALLET_001"
        or command.source != "RACK_L1"
        or command.destination != "CARRY"
    ):
        node.fail(
            reason="INVALID_COMMAND",
            phase="COMMAND_DISPATCH",
            reset_required=False,
        )
        return

    pallet = runtime.pallets.get(HARVEST_TASK.pallet_path)
    if pallet is None:
        raise RuntimeError(
            f"팔레트 prim이 등록되지 않았습니다: {HARVEST_TASK.pallet_path}"
        )

    runtime.harvest_phase = "PICK"
    runtime.transfer.start(HARVEST_TASK, pallet)
    if runtime.transfer.state == TransferState.FAILED:
        raise RuntimeError(str(runtime.transfer.error))

    node.set_phase(
        f"PICK_HARVEST/{runtime.transfer.state.value}",
        detail="harvest pick started",
    )


def verify_carrying_clear(runtime):
    """팔레트를 안정적으로 들고 포크가 랙 밖에 있는지 확인한다."""

    if not runtime.motion.is_carrying:
        raise RuntimeError("팔레트가 CARRYING 상태가 아닙니다.")
    runtime.motion.hold()
    check_fork_clear_of_rack(
        tine_tip_position(runtime.robot)[0],
        RACK_FRONT_X,
    )


def verify_transport_ready(runtime):
    """운반 자세와 travel 높이가 Navigation에 안전한지 확인한다."""

    verify_carrying_clear(runtime)
    runtime.lift.hold()
    height_error = abs(runtime.lift.base_height() - TRAVEL_BASE_HEIGHT)
    if height_error > TRAVEL_HEIGHT_TOL:
        raise RuntimeError(
            f"travel 높이 오차가 큽니다: {height_error * 1000:.1f} mm"
        )


def update_operation(node, runtime, transfer_operation):
    command = node.active_command
    if command is None:
        raise RuntimeError("active command is missing")

    if command.operation == "PLACE_INSPECT":
        # [navigation 2026-09-23] 차체가 멈출 때까지 기다린 뒤 팔을 움직인다.
        if runtime.place_phase == "WAIT_BASE_SETTLED":
            runtime.lift.hold()
            runtime.place_wait_seconds += PHYSICS_DT

            if runtime.base_watcher.still_seconds >= BASE_SETTLE_SECONDS:
                print(
                    f"[Place] 차체 정지 확인 ({runtime.place_wait_seconds:.1f}초 대기)",
                    flush=True,
                )
                start_place_motion(runtime, node)
                return

            if runtime.place_wait_seconds > BASE_SETTLE_TIMEOUT:
                raise RuntimeError(
                    f"차체가 {BASE_SETTLE_TIMEOUT:.0f}초 안에 멈추지 않았습니다 "
                    f"(정지 누적 {runtime.base_watcher.still_seconds:.2f}초). "
                    "주행이 완전히 끝난 뒤 다시 지시하십시오."
                )

            node.set_phase(
                "PLACE_INSPECT/WAIT_BASE_SETTLED",
                detail=f"still={runtime.base_watcher.still_seconds:.2f}s",
            )
            return

        runtime.lift.hold()
        runtime.motion.update(PHYSICS_DT)
        node.set_phase(
            "PLACE_INSPECT/ARM_PLACE",
            detail=runtime.motion.current_stage,
        )
        if runtime.motion.is_running:
            return
        if runtime.conveyor is not None:
            runtime.conveyor.hold_stem(False)   # 포크 인출까지 끝났다 -> 벨트가 팔레트를 비전룸으로 가져간다
        if not runtime.motion.is_done or runtime.motion.is_carrying:
            raise RuntimeError("TurnTable Place 검증이 완료되지 않았습니다.")

        node.succeed(phase="RESULT")
        return

    if command.operation == "PICK_HARVEST":
        if runtime.harvest_phase == "PICK":
            runtime.transfer.update(PHYSICS_DT)
            node.set_phase(
                f"PICK_HARVEST/{runtime.transfer.state.value}",
                detail=runtime.motion.current_stage,
            )

            if runtime.transfer.state == TransferState.FAILED:
                raise RuntimeError(str(runtime.transfer.error))
            if runtime.transfer.state != TransferState.SUCCEEDED:
                return

            verify_carrying_clear(runtime)
            runtime.motion.start_carry_rotate(CARRY_ROTATE_DEG)
            runtime.harvest_phase = "CARRY_ROTATE"
            node.set_phase(
                "PICK_HARVEST/CARRY_ROTATE",
                detail=f"joint_1 +{CARRY_ROTATE_DEG:.1f} deg",
            )
            return

        if runtime.harvest_phase == "CARRY_ROTATE":
            runtime.lift.hold()
            runtime.motion.update(PHYSICS_DT)
            node.set_phase(
                "PICK_HARVEST/CARRY_ROTATE",
                detail=runtime.motion.current_stage,
            )
            if runtime.motion.is_running:
                return
            if not runtime.motion.is_done:
                raise RuntimeError("운반 자세 동작이 완료되지 않았습니다.")

            verify_carrying_clear(runtime)
            runtime.lift.start_move(TRAVEL_BASE_HEIGHT, loaded=True)
            runtime.harvest_phase = "LIFT_TO_TRAVEL"
            node.set_phase(
                "PICK_HARVEST/LIFT_TO_TRAVEL",
                detail=f"target={TRAVEL_BASE_HEIGHT:.4f}m",
            )
            return

        if runtime.harvest_phase == "LIFT_TO_TRAVEL":
            runtime.motion.hold()
            runtime.lift.update(PHYSICS_DT)
            node.set_phase(
                "PICK_HARVEST/LIFT_TO_TRAVEL",
                detail=f"height={runtime.lift.base_height():.4f}m",
            )
            if not runtime.lift.is_done:
                return

            node.set_phase(
                "PICK_HARVEST/VERIFY_TRANSPORT_READY",
                detail="checking carry pose and travel height",
            )
            verify_transport_ready(runtime)
            release_wheels(runtime.stage, RIG_PATH)
            runtime.wheels_released = True
            runtime.harvest_phase = "CARRY"
            node.succeed(phase="RESULT", safe_to_navigate=True)
            return

        raise RuntimeError(
            f"invalid PICK_HARVEST phase: {runtime.harvest_phase}"
        )

    if not transfer_operation.is_running:
        raise RuntimeError("active command has no running operation")

    node.set_phase(
        transfer_operation.phase,
        detail=transfer_operation.controller_state,
    )

    completed = transfer_operation.update(PHYSICS_DT)
    if not completed:
        return

    node.succeed(
        phase="RESULT",
        completed_units=transfer_operation.completed_units,
    )


def fail_operation(
    node,
    transfer_operation,
    *,
    reason,
    phase,
    reset_required=True,
):
    """동작 정지를 시도한 뒤 활성 명령에 반드시 실패 결과를 보낸다."""

    try:
        transfer_operation.cancel()
    except RuntimeError as error:
        node.get_logger().error(f"operation cancel failed: {error}")

    if node.has_active_command:
        node.fail(
            reason=reason,
            phase=phase,
            completed_units=transfer_operation.completed_units,
            reset_required=reset_required,
        )


def select_view_camera():
    """[navigation 2026-09-26] 화면에 보여 줄 카메라를 환경변수로 고른다.

    씬에 저장된 기본 Perspective(/OmniverseKit_Persp)가 비전룸을 비추고 있어
    녹화할 때 카터가 보이지 않는다. SMARTFARM_VIEW_CAMERA 에 카메라 prim 경로를
    주면 그 카메라로 바꾼다. 예) /World/ProcessCameras/Cam2_Nav2Place
    환경변수가 없으면 아무것도 하지 않으므로 기존 실행에는 영향이 없다.
    headless 로 띄우면 뷰포트가 없어 조용히 넘어간다.
    """
    camera = os.environ.get("SMARTFARM_VIEW_CAMERA", "").strip()
    if not camera:
        return
    try:
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        if viewport is None:
            print("[화면] 뷰포트가 없어 카메라를 바꾸지 않았습니다(headless).", flush=True)
            return
        viewport.camera_path = camera
        print(f"[화면] 뷰포트 카메라를 {camera} 로 바꿨습니다.", flush=True)
    except Exception as error:  # 녹화 편의 기능이므로 실패해도 실행을 막지 않는다
        print(f"[화면] 카메라 전환 실패 (무시): {error}", flush=True)


def run():
    print(f"[시작] Scene을 불러옵니다: {args.scene.resolve()}", flush=True)
    runtime = create_simulation_runtime(args.scene.resolve())
    print("[시작] Scene과 제어기 구성이 완료되었습니다.", flush=True)
    select_view_camera()   # [navigation 2026-09-26]
    transfer_operation = TransferOperation(
        runtime.transfer,
        runtime.pallets,
        TRANSFER_UNITS,
    )

    print("[시작] ROS 2 노드를 초기화합니다.", flush=True)
    rclpy.init()
    node = SimTaskNode(
        supported_operations={"TRANSFER", "PICK_HARVEST", "PLACE_INSPECT"},
    )
    demo_publisher = (
        node.create_publisher(String, "/sim_task/command", 10)
        if args.demo
        else None
    )
    print("[시작] ROS 2 노드 초기화가 완료되었습니다.", flush=True)

    step_count = 0
    needs_initialization = True
    stopped_handled = False
    demo_command_sent = False

    def step_world():
        nonlocal step_count
        step_count += 1
        runtime.world.step(
            render=(
                not args.headless
                and RENDER_EVERY > 0
                and step_count % RENDER_EVERY == 0
            )
        )
        if runtime.conveyor is not None:
            runtime.conveyor.update(PHYSICS_DT)   # [올인원 2026-09-25] 벨트는 Play 중 계속 돈다
        if runtime.station is not None:
            runtime.station.update(PHYSICS_DT, render=runtime.world.render)   # 카메라 앞 팔레트 검사·솎아내기
        if runtime.human is not None and runtime.world.is_playing():
            runtime.human.update(PHYSICS_DT)   # [올인원 2026-09-25] 사람 돌발상황

    try:
        if args.autoplay:
            runtime.world.play()
        else:
            print(
                "준비 완료. Isaac Sim에서 Play를 누르면 초기화 후 READY가 됩니다.",
                flush=True,
            )

        print(
            f"[시작] main loop 진입: app_running={app.is_running()}, "
            f"playing={runtime.world.is_playing()}",
            flush=True,
        )
        while app.is_running() and rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.0)
            except Exception:  # noqa: BLE001 - 종료 신호와 실행 오류를 구분한다.
                if not rclpy.ok():
                    break
                raise

            if runtime.world.is_stopped():
                if not stopped_handled:
                    stopped_handled = True
                    needs_initialization = True

                    try:
                        transfer_operation.cancel()
                    except RuntimeError as error:
                        node.get_logger().error(f"stop failed: {error}")

                    if node.has_active_command:
                        node.fail(
                            reason="RESET_REQUIRED",
                            phase="SIMULATION_STOPPED",
                            completed_units=(
                                transfer_operation.completed_units
                            ),
                        )
                    else:
                        node.mark_error(
                            "timeline stopped; press Play to reinitialize"
                        )

                runtime.world.render()
                continue

            if not runtime.world.is_playing():
                runtime.world.render()
                continue

            stopped_handled = False

            if needs_initialization:
                try:
                    initialize_scene(
                        runtime,
                        transfer_operation,
                        step_world,
                    )
                except RuntimeError as error:
                    node.mark_error(f"scene initialization failed: {error}")
                    runtime.world.pause()
                    continue

                needs_initialization = False
                ready_detail = (
                    f"{args.scene.stem} scene ready; "
                    "TRANSFER/PICK_HARVEST/PLACE_INSPECT physical profiles loaded"
                )
                node.mark_ready(ready_detail)
                print(f"[READY] {ready_detail}", flush=True)

                if args.demo and not demo_command_sent:
                    message = String()
                    message.data = json.dumps(
                        {
                            "task_id": "STANDALONE-DEMO",
                            "command_id": "STANDALONE-DEMO-CMD-001",
                            "operation": "TRANSFER",
                            "recipe_id": "RACK_REARRANGE_01",
                        },
                        separators=(",", ":"),
                    )
                    demo_publisher.publish(message)
                    demo_command_sent = True
                    print(
                        "[DEMO] 검증용 TRANSFER 명령을 발행했습니다.",
                        flush=True,
                    )
                else:
                    print(
                        "[대기] /sim_task/command의 String/JSON 명령을 "
                        "기다립니다. 독립 동작 검증은 --demo를 사용하세요.",
                        flush=True,
                    )

            command = node.take_command()
            if command is not None:
                try:
                    start_operation(
                        command,
                        runtime,
                        transfer_operation,
                        node,
                    )
                except RuntimeError as error:
                    node.get_logger().error(str(error))
                    fail_operation(
                        node,
                        transfer_operation,
                        reason="MOTION_FAILED",
                        phase="START_OPERATION",
                    )

            if node.has_active_command:
                try:
                    update_operation(
                        node,
                        runtime,
                        transfer_operation,
                    )
                except RuntimeError as error:
                    node.get_logger().error(str(error))
                    if runtime.conveyor is not None:
                        runtime.conveyor.hold_stem(False)   # [올인원 2026-09-25] 실패해도 인터록은 푼다
                    fail_operation(
                        node,
                        transfer_operation,
                        reason="MOTION_FAILED",
                        phase="EXECUTION",
                    )

            # [navigation 2026-09-23] 차체 정지 감시를 매 스텝 갱신한다.
            runtime.base_watcher.update(runtime.arm_base, PHYSICS_DT)

            if runtime.motion.is_carrying and not node.has_active_command:
                runtime.lift.hold()
                # [navigation 2026-09-24] 운반 중 팔레트 미끄러짐 검사가 예외를 던지면 앱 전체가 죽고
                # Isaac 이 종료되었다(24차: 도킹 회전 중 30.1 mm). 관절 목표는 물리 드라이브에 남아 있으므로
                # 여기서는 한 번만 기록하고 계속 돈다. 다음 PLACE 명령에서 같은 검사가 MOTION_FAILED 로 보고한다.
                try:
                    runtime.motion.hold()
                except RuntimeError as error:
                    if not runtime.hold_fault_logged:
                        node.get_logger().error(f"운반 중 팔레트 감시 예외 (계속 진행): {error}")
                        runtime.hold_fault_logged = True

            step_world()

    finally:
        try:
            if node.has_active_command:
                fail_operation(
                    node,
                    transfer_operation,
                    reason="CANCELED",
                    phase="SHUTDOWN",
                    reset_required=False,
                )
        finally:
            if runtime.station is not None:
                runtime.station.close()     # [올인원 2026-09-25] YOLO 워커 종료
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    exit_code = 0
    try:
        run()
    except KeyboardInterrupt:
        pass
    except Exception:  # noqa: BLE001 - Kit 종료 전에 원인을 반드시 출력한다.
        traceback.print_exc()
        exit_code = 1
    finally:
        app.close()

    if exit_code:
        raise SystemExit(exit_code)
