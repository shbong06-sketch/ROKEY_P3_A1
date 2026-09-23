"""v013 장면에서 고정 좌표 솎아내기 Pick 단위 동작을 검증한다.

실행 예:
    ~/isaacsim/python.sh cull_standalone.py --headless
    ~/isaacsim/python.sh cull_standalone.py --autoplay

장면 생명주기와 객체 등록만 이 파일이 담당한다. 실제 동작 순서와 제어는
``cull_motion.CullMotion``을 사용하므로 이후 ``runtime/standalone_app.py``에서도
같은 모듈을 불러올 수 있다.
"""

import argparse
from math import sqrt
import sys
import traceback
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
M0609_DIR = PROJECT_DIR.parent / "M0609"

SCENE_PATH = (
    PROJECT_DIR
    / "scenes"
    / "Collected_smartfarm_v013"
    / "Collected_smartfarm_v013.usd"
)

ROBOT_PATH = "/World/SmartFarm/Placed/M0609/Asset"
BASE_PATH = f"{ROBOT_PATH}/base_link"
EE_PATH = f"{ROBOT_PATH}/link_6"
GRIPPER_ROOT_PATH = f"{ROBOT_PATH}/onrobot_rg2ft"
LEFT_FINGER_PATH = f"{GRIPPER_ROOT_PATH}/left_inner_finger"
RIGHT_FINGER_PATH = f"{GRIPPER_ROOT_PATH}/right_inner_finger"
TARGET_PATH = (
    "/World/SmartFarm/Placed/Pallet_Inspect/root_001/Romaine_03"
)
PALLET_BODY_PATH = "/World/SmartFarm/Placed/Pallet_Inspect/Cube_011_001"
ROMAINE_PATHS = tuple(
    f"/World/SmartFarm/Placed/Pallet_Inspect/root_001/Romaine_{index:02d}"
    for index in range(1, 7)
)

URDF_PATH = M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf"
RMPFLOW_DIR = M0609_DIR / "rmpflow"
RMPFLOW_DESCRIPTION_PATH = RMPFLOW_DIR / "m0609_description.yaml"
RMPFLOW_CONFIG_PATH = RMPFLOW_DIR / "m0609_rmpflow_common.yaml"

# v013의 Romaine_03 AABB 중심에서 RG2 손가락 패드 파지 여유 30 mm를
# 위로 더한 M0609 base_link 좌표다. AABB 중심 Z는 0.1157 m이다.
# 이후 비전 통합에서는 이 상수 대신 카메라→베이스 변환 결과를 start_pick에 넣는다.
FIXED_PICK_POSITION_BASE = (0.0009, 0.4192, 0.1457)

ARM_JOINTS = tuple(f"joint_{index}" for index in range(1, 7))
READY_JOINTS_DEG = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
GRIPPER_JOINTS = ("finger_joint", "right_inner_knuckle_joint")
GRIPPER_OPEN_POSITION = 0.0
GRIPPER_CLOSE_POSITION = 1.18

ARM_DRIVE_STIFFNESS = 1.0e8
ARM_DRIVE_DAMPING = 1.0e4
ARM_DRIVE_MAX_FORCE = 1.0e8
GRIPPER_DRIVE_STIFFNESS = 1.0e5
GRIPPER_DRIVE_DAMPING = 1.0e3
GRIPPER_DRIVE_MAX_FORCE = 1.0e4

PHYSICS_DT = 1.0 / 60.0
SETTLE_STEPS = 120
LOG_INTERVAL_STEPS = 60


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--autoplay",
        action="store_true",
        help="GUI에서도 Play 버튼을 기다리지 않고 바로 단일 동작 시작",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=6000,
        help="단일 동작 최대 physics step",
    )
    parser.add_argument(
        "--diagnose-settle",
        action="store_true",
        help="로봇을 움직이지 않고 Pallet/Romaine 초기 안정성만 측정",
    )
    parser.add_argument(
        "--diagnose-steps",
        type=int,
        default=300,
        help="초기 안정성 진단 physics step 수",
    )
    return parser.parse_known_args()


args, kit_args = parse_args()
if args.max_steps <= 0:
    raise ValueError("--max-steps는 0보다 커야 합니다.")
if args.diagnose_steps <= 0:
    raise ValueError("--diagnose-steps는 0보다 커야 합니다.")

from isaacsim import SimulationApp


app = SimulationApp(
    {
        "headless": args.headless,
        "extra_args": kit_args,
    }
)

import numpy as np
import omni.usd
from pxr import UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.core.utils.xforms import get_world_pose
from isaacsim.robot.manipulators.grippers import ParallelGripper

if str(RMPFLOW_DIR) not in sys.path:
    sys.path.insert(0, str(RMPFLOW_DIR))

from m0609_rmpflow_controller import RMPFlowController
from cull_motion import CullMotion, CullPickConfig


def open_scene():
    """v013을 열고 단독 시험에 필요 없는 Action Graph를 세션에서 끈다."""

    for path in (
        SCENE_PATH,
        URDF_PATH,
        RMPFLOW_DESCRIPTION_PATH,
        RMPFLOW_CONFIG_PATH,
    ):
        if not path.is_file():
            raise FileNotFoundError(path)

    omni.usd.get_context().open_stage(str(SCENE_PATH))
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        app.update()

    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())

    required = (
        ROBOT_PATH,
        BASE_PATH,
        EE_PATH,
        GRIPPER_ROOT_PATH,
        LEFT_FINGER_PATH,
        RIGHT_FINGER_PATH,
        TARGET_PATH,
    )
    for path in required:
        if not stage.GetPrimAtPath(path).IsValid():
            raise RuntimeError(f"필수 Prim이 없습니다: {path}")

    target = stage.GetPrimAtPath(TARGET_PATH)
    if not target.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError(f"Pick 대상이 rigid body가 아닙니다: {TARGET_PATH}")

    disabled_graphs = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() == "OmniGraph":
            prim.SetActive(False)
            disabled_graphs += 1

    print(f"[장면] {SCENE_PATH}")
    print(f"[장면] 단독 시험용 Action Graph 비활성화: {disabled_graphs}개")
    print(f"[로봇] {ROBOT_PATH}")
    print(f"[대상] {TARGET_PATH}")
    return stage


def configure_drives(stage):
    """팔과 RG2가 목표를 추종할 수 있도록 세션 레이어에서 Drive를 설정한다."""

    for name in ARM_JOINTS:
        prim = stage.GetPrimAtPath(f"{ROBOT_PATH}/joints/{name}")
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            raise RuntimeError(f"팔 angular drive가 없습니다: {prim.GetPath()}")
        drive.GetStiffnessAttr().Set(ARM_DRIVE_STIFFNESS)
        drive.GetDampingAttr().Set(ARM_DRIVE_DAMPING)
        drive.GetMaxForceAttr().Set(ARM_DRIVE_MAX_FORCE)

    finger_path = f"{GRIPPER_ROOT_PATH}/joints/finger_joint"
    finger_drive = UsdPhysics.DriveAPI.Get(
        stage.GetPrimAtPath(finger_path),
        "angular",
    )
    if not finger_drive:
        raise RuntimeError(f"그리퍼 angular drive가 없습니다: {finger_path}")
    finger_drive.GetStiffnessAttr().Set(GRIPPER_DRIVE_STIFFNESS)
    finger_drive.GetDampingAttr().Set(GRIPPER_DRIVE_DAMPING)
    finger_drive.GetMaxForceAttr().Set(GRIPPER_DRIVE_MAX_FORCE)


def initialize_gripper(gripper, robot, world):
    gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    gripper.set_default_state(
        np.array([GRIPPER_OPEN_POSITION] * len(GRIPPER_JOINTS))
    )


def set_ready_pose(robot):
    """팔은 위에서 접근 가능한 고정 자세, 그리퍼는 열린 상태로 초기화한다."""

    positions = np.zeros(robot.num_dof)
    for name, degrees in zip(ARM_JOINTS, READY_JOINTS_DEG):
        positions[robot.get_dof_index(name)] = np.deg2rad(degrees)

    robot.set_joints_default_state(
        positions=positions,
        velocities=np.zeros(robot.num_dof),
    )
    robot.set_joint_positions(positions)
    robot.set_joint_velocities(np.zeros(robot.num_dof))


def run_settle_diagnostic(world):
    """로봇 명령 없이 Pallet과 Romaine의 초기 물리 안정성을 측정한다."""

    paths = (PALLET_BODY_PATH, *ROMAINE_PATHS)

    def snapshot():
        return {
            path: tuple(np.asarray(value, dtype=float) for value in get_world_pose(path))
            for path in paths
        }

    initial = snapshot()
    print(
        f"[진단] 무개입 안정성 측정 시작: {args.diagnose_steps} steps "
        f"({args.diagnose_steps * PHYSICS_DT:.2f} s)"
    )

    for step in range(1, args.diagnose_steps + 1):
        world.step(render=not args.headless)
        if step % LOG_INTERVAL_STEPS != 0 and step != args.diagnose_steps:
            continue
        current = snapshot()
        movements = []
        for path in paths:
            delta = current[path][0] - initial[path][0]
            movements.append(
                f"{path.rsplit('/', 1)[-1]}="
                f"{np.linalg.norm(delta) * 1000.0:.1f}mm"
                f"(dz={delta[2] * 1000.0:+.1f})"
            )
        print(f"[진단:{step}] " + " ".join(movements))

    final = snapshot()
    print("[진단:최종]")
    for path in paths:
        start_position, start_quaternion = initial[path]
        final_position, final_quaternion = final[path]
        delta = final_position - start_position
        dot = float(
            np.clip(
                abs(np.dot(start_quaternion, final_quaternion)),
                0.0,
                1.0,
            )
        )
        angle_deg = float(np.rad2deg(2.0 * np.arccos(dot)))
        print(
            f"[진단:결과] {path.rsplit('/', 1)[-1]} "
            f"xyz_mm={np.round(delta * 1000.0, 2).tolist()} "
            f"distance_mm={np.linalg.norm(delta) * 1000.0:.2f} "
            f"angle_deg={angle_deg:.3f}"
        )


def print_pick_pose_summary(initial_poses):
    """Pick 종료 시 Pallet과 Romaine 전체의 pose 변화를 출력한다."""

    print("[PICK:물리요약]")
    for path, (start_position, start_quaternion) in initial_poses.items():
        final_position, final_quaternion = (
            np.asarray(value, dtype=float) for value in get_world_pose(path)
        )
        delta = final_position - start_position
        dot = float(
            np.clip(
                abs(np.dot(start_quaternion, final_quaternion)),
                0.0,
                1.0,
            )
        )
        angle_deg = float(np.rad2deg(2.0 * np.arccos(dot)))
        print(
            f"[PICK:결과] {path.rsplit('/', 1)[-1]} "
            f"xyz_mm={np.round(delta * 1000.0, 2).tolist()} "
            f"distance_mm={np.linalg.norm(delta) * 1000.0:.2f} "
            f"angle_deg={angle_deg:.3f}"
        )


def create_runtime(stage):
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=PHYSICS_DT,
        rendering_dt=PHYSICS_DT,
        physics_prim_path="/physicsScene",
    )

    gripper = ParallelGripper(
        end_effector_prim_path=EE_PATH,
        joint_prim_names=list(GRIPPER_JOINTS),
        joint_opened_positions=np.array(
            [GRIPPER_OPEN_POSITION] * len(GRIPPER_JOINTS)
        ),
        joint_closed_positions=np.array(
            [GRIPPER_CLOSE_POSITION] * len(GRIPPER_JOINTS)
        ),
        action_deltas=None,
    )
    robot = world.scene.add(
        SingleArticulation(
            prim_path=ROBOT_PATH,
            name="cull_m0609",
        )
    )
    target = world.scene.add(
        SingleRigidPrim(
            prim_path=TARGET_PATH,
            name="cull_target_romaine_03",
        )
    )

    world.reset()
    robot.initialize(physics_sim_view=world.physics_sim_view)
    initialize_gripper(gripper, robot, world)
    set_ready_pose(robot)

    controller = RMPFlowController(
        name="cull_m0609_rmpflow",
        robot_articulation=robot,
        physics_dt=PHYSICS_DT,
        urdf_path=str(URDF_PATH),
        robot_description_path=str(RMPFLOW_DESCRIPTION_PATH),
        rmpflow_config_path=str(RMPFLOW_CONFIG_PATH),
        end_effector_frame_name="link_6",
    )

    motion = CullMotion(
        robot=robot,
        gripper=gripper,
        arm_controller=controller,
        get_end_effector_world_pose=lambda: get_world_pose(EE_PATH),
        get_base_world_pose=lambda: get_world_pose(BASE_PATH),
        get_target_world_pose=target.get_world_pose,
        config=CullPickConfig(
            approach_clearance=0.18,
            lift_clearance=0.22,
            # 아래보기 자세를 유지하면서 손가락 간격 축을 90도 돌려
            # Romaine의 짧은 폭(Y 약 56 mm)을 잡는다.
            tool_orientation_base=(0.0, 1.0 / sqrt(2.0), -1.0 / sqrt(2.0), 0.0),
            min_target_rise=0.05,
        ),
    )
    return world, robot, gripper, target, controller, motion


def main():
    stage = open_scene()
    configure_drives(stage)
    world, robot, gripper, target, controller, motion = create_runtime(stage)

    print(f"[계획] 고정 Pick 좌표(base, m): {FIXED_PICK_POSITION_BASE}")
    print("[계획] OPEN → APPROACH → DESCEND → GRASP → LIFT → HOLD")

    if args.headless or args.autoplay:
        world.play()
    else:
        world.pause()
        print("[대기] Isaac Sim에서 Play를 누르면 단일 Pick 시험을 시작합니다.")

    if args.diagnose_settle:
        if not world.is_playing():
            world.play()
        run_settle_diagnostic(world)
        return

    started = False
    pick_initial_poses = None
    pick_summary_printed = False
    step_count = 0
    previous_playing = world.is_playing()

    while app.is_running():
        world.step(render=not args.headless)
        is_playing = world.is_playing()

        if is_playing and not previous_playing:
            world.reset()
            robot.initialize(physics_sim_view=world.physics_sim_view)
            initialize_gripper(gripper, robot, world)
            set_ready_pose(robot)
            controller.reset()
            motion.cancel()
            started = False
            step_count = 0

        if not is_playing:
            previous_playing = is_playing
            continue

        if not started:
            print(f"[안정화] {SETTLE_STEPS} physics step")
            for _ in range(SETTLE_STEPS):
                world.step(render=not args.headless)
            initial_position, _ = target.get_world_pose()
            print(f"[대상] 시작 world={np.round(initial_position, 4).tolist()}")
            pick_initial_poses = {
                path: tuple(
                    np.asarray(value, dtype=float) for value in get_world_pose(path)
                )
                for path in (PALLET_BODY_PATH, *ROMAINE_PATHS)
            }
            motion.start_pick(FIXED_PICK_POSITION_BASE)
            started = True

        try:
            motion.update()
        except Exception:
            if pick_initial_poses is not None and not pick_summary_printed:
                print_pick_pose_summary(pick_initial_poses)
                pick_summary_printed = True
            raise
        step_count += 1

        if step_count % LOG_INTERVAL_STEPS == 0:
            target_position, _ = target.get_world_pose()
            left_finger, _ = get_world_pose(LEFT_FINGER_PATH)
            right_finger, _ = get_world_pose(RIGHT_FINGER_PATH)
            finger = robot.get_joint_positions()[robot.get_dof_index("finger_joint")]
            print(
                f"[상태] step={step_count} stage={motion.current_stage} "
                f"target={np.round(target_position, 4).tolist()} "
                f"finger={finger:+.4f} "
                f"left={np.round(left_finger, 4).tolist()} "
                f"right={np.round(right_finger, 4).tolist()}"
            )

        if motion.is_done:
            if pick_initial_poses is not None and not pick_summary_printed:
                print_pick_pose_summary(pick_initial_poses)
                pick_summary_printed = True
            print(
                "CULL_STANDALONE_PASS "
                f"rise_mm={motion.final_target_rise * 1000.0:.1f}"
            )
            if args.headless:
                break
            world.pause()

        if step_count >= args.max_steps:
            raise RuntimeError(
                f"단일 동작 시간 초과: {args.max_steps} steps, "
                f"stage={motion.current_stage}"
            )

        previous_playing = is_playing


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        raise
    finally:
        app.close()
