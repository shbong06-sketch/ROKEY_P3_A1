"""Isaac Sim 안에서 Sim Task ROS 노드와 팔레트 이송을 함께 실행한다.

실행 예:
    # Task Manager 명령 대기
    ~/isaacsim/python.sh runtime/standalone_app.py --autoplay

    # robot_motion_standalone.py처럼 TRANSFER를 즉시 검증
    ~/isaacsim/python.sh runtime/standalone_app.py --demo
"""

import argparse
import json
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
    / "Collected_smartfarm_v008"
    / "Collected_smartfarm_v008.usd"
)

PHYSICS_DT = 1.0 / 60.0
RENDER_EVERY = 3
SETTLE_STEPS = 120

RIG_PATH = "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS"
ROBOT_PATH = f"{RIG_PATH}/chassis_link"
ARM_PATH = f"{RIG_PATH}/m0609_with_fork"
ARM_BASE_PATH = f"{ARM_PATH}/base_link"
LIFT_JOINT_PATH = f"{RIG_PATH}/lift_v3_physics/lift_prismatic_joint"
LIFT_JOINT_NAME = "lift_prismatic_joint"

RACK_FRONT_X = -1.205
BASE_BELOW_SHELF = 0.213
# v008은 팔레트 원점이 밑면보다 25.9 mm 위에 있다.
SHELF_TOP = {
    1: 0.7388,
    2: 1.0388,
    3: 1.3388,
    4: 1.6388,
    5: 1.9388,
}

PALLET_ASSET_NAME = (
    "palette2_palete_tray_romaine_8_hole_physics_1__01"
)
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
    needs_reexec = (
        str(ros_lib) not in current_paths
        and os.environ.get("SMARTFARM_ROS_REEXEC") != "1"
    )
    if needs_reexec:
        os.environ["LD_LIBRARY_PATH"] = ":".join(
            [*current_paths, str(ros_lib)]
        )
        os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
        os.environ["SMARTFARM_ROS_REEXEC"] = "1"

        print(
            f"[ROS2] LD_LIBRARY_PATH에 {ros_lib}를 추가하고 다시 실행합니다.",
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
    BaseWatcher,
    EE_FRAME,
    RobotMotion,
    Task,
    brake_wheels,
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
    lift: LiftController
    transfer: PalletTransferController
    pallets: dict


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
                    child.GetName() for child in parent.GetChildren()
                )
                print(
                    f"[시작] {parent_path} 하위 prim: {children}",
                    flush=True,
                )

    if missing:
        raise RuntimeError(
            "Scene에서 필수 prim을 찾지 못했습니다: " + ", ".join(missing)
        )


def create_simulation_runtime(scene_path):
    stage = open_scene(scene_path)
    print("[시작] USD Scene 로딩이 완료되었습니다.", flush=True)

    pallet_paths = tuple(unit.task.pallet_path for unit in TRANSFER_UNITS)
    require_prims(
        stage,
        (
            ROBOT_PATH,
            ARM_BASE_PATH,
            EE_PATH,
            LIFT_JOINT_PATH,
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
    world.reset()
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

    return SimulationRuntime(
        world=world,
        lift=lift,
        transfer=transfer,
        pallets=pallets,
    )


def initialize_scene(runtime, transfer_operation, step_world):
    """Stop 후 Play를 포함해 Scene과 제어기를 초기 상태로 맞춘다."""

    transfer_operation.reset()
    runtime.world.reset()

    print(f"[장면] 안정화를 위해 {SETTLE_STEPS} physics step을 진행합니다.")
    for _ in range(SETTLE_STEPS):
        step_world()

    runtime.lift.calibrate()


def start_operation(command, transfer_operation, node):
    if command.operation != "TRANSFER":
        node.fail(
            reason="INVALID_COMMAND",
            phase="COMMAND_DISPATCH",
            reset_required=False,
        )
        return

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


def update_operation(node, transfer_operation):
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


def run():
    print(f"[시작] Scene을 불러옵니다: {args.scene.resolve()}", flush=True)
    runtime = create_simulation_runtime(args.scene.resolve())
    print("[시작] Scene과 제어기 구성이 완료되었습니다.", flush=True)
    transfer_operation = TransferOperation(
        runtime.transfer,
        runtime.pallets,
        TRANSFER_UNITS,
    )

    print("[시작] ROS 2 노드를 초기화합니다.", flush=True)
    rclpy.init()
    node = SimTaskNode(
        supported_operations={"TRANSFER"},
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
                    "TRANSFER physical profile loaded"
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
                        transfer_operation,
                    )
                except RuntimeError as error:
                    node.get_logger().error(str(error))
                    fail_operation(
                        node,
                        transfer_operation,
                        reason="MOTION_FAILED",
                        phase="EXECUTION",
                    )

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
