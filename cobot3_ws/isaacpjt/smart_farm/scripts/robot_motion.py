"""Isaac Sim에서 단일 슬롯 PICK을 시험하는 실행 진입점.

SimulationApp, USD 로드, World.reset, 프레임 루프, app.close는 이 파일만 소유한다.
`motion` 패키지는 재사용 가능한 모션 로직만 포함한다.
"""

from pathlib import Path
import sys

from isaacsim import SimulationApp

# 반드시 Isaac 관련 모듈보다 먼저 생성한다.
app = SimulationApp({"headless": False})

import numpy as np
import omni.usd
from pxr import UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.robot.manipulators.manipulators import SingleManipulator


SMART_FARM_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SMART_FARM_ROOT))

from motion import ForkMotion, LulaTool0IK, MotionConfig, Pose, SlotPose


SCENE_PATH = SMART_FARM_ROOT / "scenes/demo_test/demo_test.usd"
ROBOT_PATH = "/World/m0609_with_fork"
PALLET_PATH = "/World/simple_pallet1"
JOINT_NAMES = [f"joint_{index}" for index in range(1, 7)]
PHYSICS_DT = 1.0 / 60.0

# 기존 waypoint 주석의 APPROACH 좌표를 바탕으로 둔 임시 이행값이다.
# 이 값은 측정된 포크 TCP pose가 아니므로, 실제 시험 전 슬롯 캘리브레이션 값으로
# 반드시 교체해야 한다. 방향 quaternion도 실제 포크 삽입 자세를 측정해야 한다.
LEGACY_DEMO_SLOT = SlotPose(
    tcp_at_channel_entrance=Pose(
        position=np.array([-1.000, -0.040, 1.072]),
        # 기존 APPROACH 정기구학과 같은 방향이다. 포크 진행축(local +Z)은
        # world -X, 두 갈래 간격축(local +X)은 world -Y를 향한다.
        orientation=np.array([0.5, 0.5, -0.5, -0.5]),
    ),
    insertion_axis_world=np.array([-1.0, 0.0, 0.0]),
    slot_id="simple_rack",
)


def read_joint_limits(stage):
    """USD의 실제 관절 limit을 radian 배열로 읽는다."""

    lower_limits = []
    upper_limits = []
    for joint_name in JOINT_NAMES:
        joint_path = f"{ROBOT_PATH}/joints/{joint_name}"
        joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(joint_path))
        if not joint:
            raise RuntimeError(f"관절 prim을 찾지 못했습니다: {joint_path}")
        lower_limits.append(joint.GetLowerLimitAttr().Get())
        upper_limits.append(joint.GetUpperLimitAttr().Get())
    return np.deg2rad(lower_limits), np.deg2rad(upper_limits)


def make_tcp_pose_reader(robot, config):
    """`link_6` pose에서 실제 포크 팁 중심 TCP pose를 계산하는 함수.

    현재 fork URDF에서 tool0는 link_6에 고정되어 있다고 가정한다. 따라서
    link_6/world 회전에 TCP local offset을 적용한다. USD의 고정 조인트가
    다르면 MotionConfig.tcp.offset_m 또는 이 reader를 수정해야 한다.
    """

    def read_tcp_pose():
        link6_position, link6_orientation = robot.end_effector.get_world_pose()
        w, x, y, z = link6_orientation / np.linalg.norm(link6_orientation)
        rotation = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )
        tcp_position = np.asarray(link6_position) + rotation @ config.tcp.offset_m
        return tcp_position, np.asarray(link6_orientation)

    return read_tcp_pose


def find_or_create_physics_scene_path(stage) -> str:
    """기존 물리 scene을 사용하고, 없으면 하나 생성한다."""

    scene_paths = [str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    if len(scene_paths) > 1:
        raise RuntimeError(
            "UsdPhysics.Scene prim이 둘 이상입니다. 발견한 경로: "
            f"{scene_paths}"
        )
    if scene_paths:
        return scene_paths[0]

    physics_scene_path = "/World/physicsScene"
    UsdPhysics.Scene.Define(stage, physics_scene_path)
    return physics_scene_path


def main(slot: SlotPose = LEGACY_DEMO_SLOT):
    """선택한 단일 슬롯에서 PICK 상태 기계를 실행한다."""

    if not SCENE_PATH.is_file():
        raise FileNotFoundError(SCENE_PATH)

    omni.usd.get_context().open_stage(str(SCENE_PATH))
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        app.update()

    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    for prim_path in (ROBOT_PATH, PALLET_PATH, f"{ROBOT_PATH}/link_6"):
        if not stage.GetPrimAtPath(prim_path).IsValid():
            raise RuntimeError(f"필수 prim이 없습니다: {prim_path}")

    physics_scene_path = find_or_create_physics_scene_path(stage)
    print(f"물리 scene: {physics_scene_path}")
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=PHYSICS_DT,
        rendering_dt=PHYSICS_DT,
        physics_prim_path=physics_scene_path,
    )
    robot = world.scene.add(
        SingleManipulator(
            prim_path=ROBOT_PATH,
            name="m0609_with_fork",
            end_effector_prim_path=f"{ROBOT_PATH}/link_6",
        )
    )
    pallet = world.scene.add(SingleRigidPrim(prim_path=PALLET_PATH, name="pallet"))

    world.reset()
    world.pause()

    config = MotionConfig()
    joint_indices = np.array([robot.get_dof_index(name) for name in JOINT_NAMES])
    ik_solver = LulaTool0IK(
        robot=robot,
        description_path=SMART_FARM_ROOT.parent / "M0609/descriptor/m0609_description.yaml",
        urdf_path=SMART_FARM_ROOT / "assets/fork_tool/output/m0609_with_fork.urdf",
        tcp_offset_m=config.tcp.offset_m,
    )
    motion = ForkMotion(
        robot=robot,
        pallet=pallet,
        tcp_pose_reader=make_tcp_pose_reader(robot, config),
        ik_solver=ik_solver,
        joint_indices=joint_indices,
        joint_limits_rad=read_joint_limits(stage),
        config=config,
    )

    print(f"Play로 PICK 시작: {slot.slot_id}")
    print(f"TCP 보정: tool0 local {config.tcp.offset_m} m")
    print("Stop → Play는 reset 후 처음부터 다시 시작합니다.")

    needs_reset = True
    while app.is_running():
        if world.is_stopped():
            needs_reset = True
            world.render()
            continue
        if not world.is_playing():
            world.render()
            continue

        if needs_reset:
            world.reset()
            motion.reset()
            motion.start_pick(slot)
            needs_reset = False

        result = motion.update(PHYSICS_DT)
        world.step(render=True)

        if result.state.value in ("FAILED", "SUCCEEDED"):
            message = result.reason or "PICK 물리 판정을 통과했습니다."
            print(f"[{result.state}] {message}")
            world.pause()


try:
    main()
finally:
    app.close()
