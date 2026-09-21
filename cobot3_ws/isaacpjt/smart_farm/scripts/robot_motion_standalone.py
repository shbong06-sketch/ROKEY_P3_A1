"""
리프트로 높이 맞추기 → 팔레트 집기 → 인양 → 인출 → 놓기 Standalone 실행기

수정할 곳
  SCENE_PATH : 어느 월드 USD 를 열지
  RIG_PATH   : 월드 안에서 리그(카터+리프트+팔)가 놓인 자리
  TASKS      : 어느 팔레트를 어느 층으로 옮길지 (순서대로 실행)

Play  : 시작 / 일시정지한 위치에서 재개
Stop  : 다음 Play에서 처음부터 재시작
"""

import os

from isaacsim import SimulationApp

# HEADLESS=1: 자동 시작하고 마무리 하강까지 끝나면 종료합니다. 실패 시 예외로 종료합니다.
# 그냥 실행하면 GUI에서 Play를 기다립니다.
#   GUI      : ~/isaacsim/python.sh robot_motion_standalone.py
#   헤드리스 : HEADLESS=1 ~/isaacsim/python.sh robot_motion_standalone.py
app = SimulationApp({"headless": os.environ.get("HEADLESS") == "1"})

from pathlib import Path

import omni.usd

from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim, SingleXFormPrim
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

from lift import LiftController, check_fork_clear_of_rack
from pallet_transfer import PalletTransferController, TransferState
from robot_motion import (
    BaseWatcher,
    EE_FRAME,
    RobotMotion,
    Task,
    brake_wheels,
    tine_tip_position,
)


# ── 파일·로봇 경로 ───────────────────────────────────────
SCENE_PATH = Path(
    Path(__file__).resolve().parent.parent
        / "scenes"
        / "Collected_smartfarm_v004"
        / "Collected_smartfarm_v004.usd"
)

M0609_DIR = Path(__file__).resolve().parent.parent.parent / "M0609"
URDF_PATH = M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf"
DESCRIPTION_PATH = M0609_DIR / "descriptor/m0609_description.yaml"

# 리그(카터 + 리프트 + 팔) 안의 prim 경로.
#   ROBOT_PATH    : 아티큘레이션 루트. 관절을 읽고 쓰는 창구입니다.
#   ARM_BASE_PATH : 팔이 실제로 서 있는 자리. IK 의 기준점입니다.
#                   카터가 움직이거나 리프트가 오르면 이 자리가 따라 움직입니다.
RIG_PATH = "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS"
ROBOT_PATH = f"{RIG_PATH}/chassis_link"
ARM_PATH = f"{RIG_PATH}/m0609_with_fork"
ARM_BASE_PATH = f"{ARM_PATH}/base_link"
EE_PATH = f"{ARM_PATH}/{EE_FRAME}"

# 리프트
LIFT_JOINT_PATH = f"{RIG_PATH}/lift_v3_physics/lift_prismatic_joint"
LIFT_JOINT_NAME = "lift_prismatic_joint"

RACK_FRONT_X = -1.205          # 선반판 앞면. 이보다 안쪽(작은 x)은 랙 내부입니다
BASE_BELOW_SHELF = 0.213       # 집을 선반 윗면보다 팔 베이스를 이만큼 아래에 둡니다
SETTLE_STEPS = 120             # Play 후 리그가 내려앉기를 기다리는 물리 스텝 수
                               # (안정되기 전에 리프트 기준을 잡으면 10 cm 넘게 틀립니다)


# ── 작업 목록 ────────────────────────────────────────────
# 위에서부터 순서대로 실행합니다. 한 작업이 끝나면 다음 작업의 계획을 새로 만듭니다.
#   pallet_path           : 집을 팔레트 prim
#   destination_shelf_top : 놓을 선반 윗면 높이. None 이면 집은 자리와 같은 층
#     0.713 = 1단, 1.013 = 2단, 1.313 = 3단, 1.613 = 4단, 1.913 = 5단
#
# 작업 전에 집을 선반 높이에 맞춰 리프트를 움직입니다.
# 팔로 집고 놓는 동안에는 그 높이를 유지합니다.
# 닿지 않는 작업을 적으면 계획 단계에서 이유를 말하고 멈춥니다.
SHELF_TOP = {1: 0.713, 2: 1.013, 3: 1.313, 4: 1.613, 5: 1.913}

# 팔레트 prim 은 '강체 그 자체'를 가리켜야 합니다.
# 이 월드에서는 Pallet_N 은 빈 Xform 이고 그 안의 Asset 이 강체입니다.
# 바깥 Xform 을 적으면 강체가 둘로 겹쳐 물리 결과가 흔들립니다.
TASKS = [
    Task("/World/SmartFarm/Placed/Pallet_2/Asset", SHELF_TOP[2]),               # 3단 → 2단
    Task("/World/SmartFarm/Placed/Pallet_3/Asset", SHELF_TOP[3]),               # 4단 → 3단
    Task("/World/SmartFarm/Placed/Pallet_1/Asset", None, pick_only=True),       # 1단 집기만
    # 2단이 비어 있는 상태에서 시작합니다. 위 칸부터 한 칸씩 내려 채운 뒤,
    # 맨 아래 팔레트를 집어 든 채로 멈춥니다 (AMR 이 이동할 차례).
]

# 모든 작업을 마친 뒤 리프트를 이 층의 작업 높이로 내려둡니다.
# 다음 사이클(AMR 이 1단에 팔레트를 놓아주는 것)을 바로 받을 수 있게 합니다.
# None 이면 마지막 작업 높이에 그대로 둡니다.
PARK_SHELF_TOP = None   # 마지막에 팔레트를 든 채 멈추므로 리프트를 내리지 않습니다

# 화면 갱신 주기. 물리는 매 스텝 계산하고, 그림만 이 간격으로 그립니다.
# 물리 결과와 로봇 동작은 전혀 바뀌지 않고 보이는 부드러움만 줄어듭니다.
#   0 이면 아예 안 그림 (헤드리스)
#   1 이면 매 스텝    (예전 방식)
#   3 이면 초당 20번   (눈으로는 차이가 거의 없고 시간은 크게 줄어듦)
RENDER_EVERY = 0 if os.environ.get("HEADLESS") == "1" else 3

PHYSICS_DT = 1.0 / 60.0


def open_scene():
    """파일과 프림을 확인하고, 수정 대상이 세션 레이어인 stage를 반환합니다."""
    if not SCENE_PATH.is_file():
        raise FileNotFoundError(SCENE_PATH)
    for path in (URDF_PATH, DESCRIPTION_PATH):
        if not path.is_file():
            raise FileNotFoundError(path)
    if not TASKS:
        raise ValueError("TASKS 가 비어 있습니다.")

    omni.usd.get_context().open_stage(str(SCENE_PATH))
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        app.update()

    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())

    needed = [ROBOT_PATH, ARM_BASE_PATH, EE_PATH] + [t.pallet_path for t in TASKS]
    for path in needed:
        if not stage.GetPrimAtPath(path).IsValid():
            raise RuntimeError(f"Prim이 없습니다: {path}")

    return stage


def create_world():
    """로봇·베이스·팔레트를 등록하고 Play를 기다리는 World를 만듭니다."""
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=PHYSICS_DT,
        rendering_dt=PHYSICS_DT,
        physics_prim_path="/physicsScene",
    )
    # 아티큘레이션 루트는 카터(chassis_link)이고, 팔은 그 안의 관절 6개입니다.
    robot = world.scene.add(
        SingleManipulator(
            prim_path=ROBOT_PATH,
            name="carter_m0609",
            end_effector_prim_path=EE_PATH,
        )
    )
    # IK 의 기준점. 카터가 움직이거나 리프트가 오르면 이 자리가 따라 움직입니다.
    arm_base = world.scene.add(
        SingleXFormPrim(prim_path=ARM_BASE_PATH, name="arm_base")
    )
    # 작업 목록에 나오는 팔레트를 모두 등록합니다 (같은 팔레트는 한 번만).
    pallets = {}
    for task in TASKS:
        if task.pallet_path not in pallets:
            pallets[task.pallet_path] = world.scene.add(
                SingleRigidPrim(prim_path=task.pallet_path, name=f"pallet{len(pallets)}")
            )

    world.reset()
    world.pause()

    return world, robot, arm_base, pallets


def main():
    stage = open_scene()
    brake_wheels(stage, RIG_PATH)
    world, robot, arm_base, pallets = create_world()

    # 관절 인덱스는 초기화 뒤에야 읽히므로 여기서 만듭니다.
    # 기준 잡기(calibrate)는 아래 루프에서 reset 직후에 합니다.
    lift = LiftController(robot, stage, arm_base, LIFT_JOINT_PATH, LIFT_JOINT_NAME)

    solver = LulaKinematicsSolver(
        robot_description_path=str(DESCRIPTION_PATH),
        urdf_path=str(URDF_PATH),
    )
    motion = RobotMotion(robot, arm_base, solver, stage, ARM_PATH)
    motion.initialize()
    watcher = BaseWatcher()
    transfer = PalletTransferController(
        lift,
        motion,
        arm_base,
        watcher,
        BASE_BELOW_SHELF,
        lambda: check_fork_clear_of_rack(
            tine_tip_position(robot)[0], RACK_FRONT_X
        ),
    )

    step_count = 0

    def step_world():
        """물리를 한 스텝 진행합니다. 화면은 RENDER_EVERY 간격으로만 그립니다."""
        nonlocal step_count
        step_count += 1
        world.step(
            render=RENDER_EVERY > 0 and step_count % RENDER_EVERY == 0
        )

    task_index = 0
    needs_reset = True
    failed = False           # 오류 후에는 Stop → Play 전까지 재개하지 않습니다
    headless = os.environ.get("HEADLESS") == "1"
    parking = None          # None / 'going' / 'done'

    print(f"작업 {len(TASKS)}개. Play: 시작/재개 | Pause: 대기 | Stop: 처음부터 재시작")

    if headless:
        world.play()

    while app.is_running():
        if world.is_stopped():
            needs_reset = True
            world.render()
            continue

        if not world.is_playing():
            world.render()
            continue

        if (failed or transfer.state == TransferState.FAILED) and not needs_reset:
            world.pause()
            continue

        try:
            # Stop 후 Play 에서만 처음부터 다시 시작합니다.
            if needs_reset:
                needs_reset = False
                failed = False
                transfer.cancel()
                if transfer.state == TransferState.FAILED:
                    raise RuntimeError(f"초기화 전 정지 실패: {transfer.error}")
                world.reset()
                task_index = 0
                parking = None

                # 리그가 내려앉기를 기다린 뒤에 리프트 기준을 잡습니다.
                # 건너뛰면 '리프트 값 ↔ 베이스 높이' 관계를 10 cm 넘게 틀리게 잽니다.
                print(f"[장면] 안정될 때까지 {SETTLE_STEPS} 스텝 기다립니다")
                for _ in range(SETTLE_STEPS):
                    step_world()
                lift.calibrate()

            if task_index >= len(TASKS):
                # 작업이 다 끝났습니다. 리프트를 지정한 층 높이로 내려둡니다.
                if PARK_SHELF_TOP is not None and parking is None:
                    # 포크가 랙 안에 있으면 리프트를 움직이면 안 됩니다.
                    check_fork_clear_of_rack(
                        tine_tip_position(robot)[0], RACK_FRONT_X
                    )
                    goal = lift.clamp_height(PARK_SHELF_TOP - BASE_BELOW_SHELF)
                    lift.start_move(goal)
                    parking = 'going'
                    print(f'[마무리] 리프트를 베이스 {goal:.3f} m 로 내립니다')

                if parking == 'going':
                    lift.update(PHYSICS_DT)
                    if lift.is_done:
                        parking = 'done'
                        print('[마무리] 리프트 하강 완료. 대기합니다.')
                elif parking == 'done':
                    lift.hold()

                step_world()
                if headless and (PARK_SHELF_TOP is None or parking == 'done'):
                    return
                continue

            if not transfer.is_running:
                task = TASKS[task_index]
                print(f"\n── 작업 {task_index + 1}/{len(TASKS)} ──")
                transfer.start(task, pallets[task.pallet_path])

            transfer.update(PHYSICS_DT)
            if transfer.state == TransferState.FAILED:
                raise RuntimeError(f"팔레트 이송 실패: {transfer.error}")
            step_world()

            if transfer.state == TransferState.SUCCEEDED:
                task_index += 1
                if task_index >= len(TASKS):
                    print("[완료] 모든 작업을 마쳤습니다.")

        except RuntimeError as error:
            failed = True
            transfer.cancel()
            world.pause()
            print(f"[중단] {error}")
            if transfer.error is not None:
                print(f"[정지 오류] {transfer.error}")
            if headless:
                raise
            print("원인을 확인하세요. Stop → Play로 처음부터 재시험합니다.")


try:
    main()
finally:
    app.close()
