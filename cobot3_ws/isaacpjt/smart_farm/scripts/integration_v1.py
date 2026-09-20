"""Integration V1: integrated scene에서 M0609 Pick/Retract를 검증합니다."""

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": False})


from pathlib import Path

import numpy as np
import omni.usd
from pxr import UsdPhysics

from isaacsim.core.api import World

import robot_motion


USD_PATH = (
    Path(__file__).resolve().parent.parent
    / "scenes"
    / "Collected_smartfarm_v003"
    / "Collected_smartfarm_v003.usd"
)

PHYSICS_DT = 1.0 / 60.0

# Scene configuration confirmed from Collected_smartfarm_v003.usd.
WORLD_PRIM_PATH = "/World"
SMART_FARM_PRIM_PATH = "/World/SmartFarm"
PLACED_PRIM_PATH = f"{SMART_FARM_PRIM_PATH}/Placed"

LIFT_RIG_PRIM_PATH = f"{PLACED_PRIM_PATH}/LiftRig"
NOVA_PRIM_PATH = f"{LIFT_RIG_PRIM_PATH}/Asset/nova_carter_ROS"
NOVA_ARTICULATION_ROOT_PATH = f"{NOVA_PRIM_PATH}/chassis_link"

LIFT_PRIM_PATH = f"{NOVA_PRIM_PATH}/lift_v3_physics"
LIFT_JOINT_PATH = f"{LIFT_PRIM_PATH}/lift_prismatic_joint"

M0609_PRIM_PATH = f"{NOVA_PRIM_PATH}/m0609_with_fork"
# The integrated M0609 has no separate articulation root. It belongs to the
# Carter-rooted composite articulation through the lift fixed joints.
M0609_ARTICULATION_ROOT_PATH = NOVA_ARTICULATION_ROOT_PATH
M0609_BASE_LINK_PRIM_PATH = f"{M0609_PRIM_PATH}/base_link"
M0609_LINK6_PRIM_PATH = f"{M0609_PRIM_PATH}/link_6"
M0609_TCP_PRIM_PATH = M0609_LINK6_PRIM_PATH
FORK_PRIM_PATH = f"{M0609_PRIM_PATH}/fork_tool"
FORK_FIXED_JOINT_PATH = f"{M0609_PRIM_PATH}/joints/tool0_to_fork_tool"

RACK_PRIM_PATH = f"{PLACED_PRIM_PATH}/Rack_1"
PALLET_ROOT_PRIM_PATH = f"{PLACED_PRIM_PATH}/Pallet_1"
PALLET_PRIM_PATH = f"{PALLET_ROOT_PRIM_PATH}/Asset"
CONVEYOR_PRIM_PATH = f"{PLACED_PRIM_PATH}/Conveyor"

NOVA_CMD_VEL_GRAPH_PATH = f"{NOVA_PRIM_PATH}/differential_drive"
NOVA_CMD_VEL_SUBSCRIBER_PATH = (
    f"{NOVA_CMD_VEL_GRAPH_PATH}/ros2_subscribe_twist"
)
NOVA_ODOMETRY_GRAPH_PATH = f"{NOVA_PRIM_PATH}/transform_tree_odometry"

CONFIGURED_PRIMS = (
    ("World", WORLD_PRIM_PATH),
    ("SmartFarm", SMART_FARM_PRIM_PATH),
    ("LiftRig", LIFT_RIG_PRIM_PATH),
    ("Nova Carter", NOVA_PRIM_PATH),
    ("Composite articulation root", NOVA_ARTICULATION_ROOT_PATH),
    ("Lift", LIFT_PRIM_PATH),
    ("Lift joint", LIFT_JOINT_PATH),
    ("M0609", M0609_PRIM_PATH),
    ("M0609 base link", M0609_BASE_LINK_PRIM_PATH),
    ("M0609 link_6 / TCP", M0609_TCP_PRIM_PATH),
    ("Fork", FORK_PRIM_PATH),
    ("Fork fixed joint", FORK_FIXED_JOINT_PATH),
    ("Rack", RACK_PRIM_PATH),
    ("Pick pallet root", PALLET_ROOT_PRIM_PATH),
    ("Pick pallet rigid body", PALLET_PRIM_PATH),
    ("Conveyor", CONVEYOR_PRIM_PATH),
    ("Nova cmd_vel graph", NOVA_CMD_VEL_GRAPH_PATH),
    ("Nova Twist subscriber", NOVA_CMD_VEL_SUBSCRIBER_PATH),
    ("Nova odometry graph", NOVA_ODOMETRY_GRAPH_PATH),
)


def print_scene_debug(stage):
    """Print the configured integration prims once without dumping the stage."""
    print("[Integration V1] Configured prims:")
    for label, prim_path in CONFIGURED_PRIMS:
        prim = stage.GetPrimAtPath(prim_path)
        prim_type = prim.GetTypeName() if prim.IsValid() else "MISSING"
        print(f"  - {label}: {prim_path} [{prim_type}]")

    articulation_roots = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    print("[Integration V1] Articulation roots:")
    for prim_path in articulation_roots:
        print(f"  - {prim_path}")

    nova_action_graphs = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.GetTypeName() == "OmniGraph"
        and str(prim.GetPath()).startswith(f"{NOVA_PRIM_PATH}/")
    ]
    print("[Integration V1] Nova Carter Action Graphs:")
    for prim_path in nova_action_graphs:
        print(f"  - {prim_path}")


def print_motion_poses(stage):
    """IK와 Pick 기준이 되는 pose를 초기화 직후 한 번만 출력합니다."""
    for label, prim_path in (
        ("M0609 base", M0609_BASE_LINK_PRIM_PATH),
        ("link_6", M0609_LINK6_PRIM_PATH),
        ("fork", FORK_PRIM_PATH),
        ("pick pallet", PALLET_PRIM_PATH),
    ):
        position, quaternion = robot_motion.prim_world_pose(stage, prim_path)
        print(
            f"[Integration V1] {label}: "
            f"position={np.round(position, 4).tolist()}, "
            f"quaternion(wxyz)={np.round(quaternion, 5).tolist()}"
        )


def main():
    if not USD_PATH.is_file():
        raise FileNotFoundError(f"USD scene not found: {USD_PATH}")

    print(f"[Integration V1] Loading scene: {USD_PATH}")
    context = omni.usd.get_context()
    if not context.open_stage(str(USD_PATH)):
        raise RuntimeError(f"Failed to open USD scene: {USD_PATH}")

    while context.get_stage_loading_status()[2] > 0:
        simulation_app.update()

    for _ in range(10):
        simulation_app.update()

    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("USD stage is not available after loading.")
    stage.SetEditTarget(stage.GetSessionLayer())

    print(f"[Integration V1] Stage ready: {stage.GetRootLayer().realPath}")
    print_scene_debug(stage)

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=PHYSICS_DT,
        rendering_dt=PHYSICS_DT,
        physics_prim_path="/physicsScene",
    )
    solver, robot, pallet, indices, lower_deg, upper_deg = (
        robot_motion.initialize_motion(
            world=world,
            stage=stage,
            articulation_root_path=M0609_ARTICULATION_ROOT_PATH,
            robot_path=M0609_PRIM_PATH,
            end_effector_path=M0609_TCP_PRIM_PATH,
            pallet_path=PALLET_PRIM_PATH,
        )
    )

    for _ in range(10):
        world.step(render=True)

    print_motion_poses(stage)
    base_position, base_quaternion = robot_motion.prim_world_pose(
        stage, M0609_BASE_LINK_PRIM_PATH
    )
    sequence = None
    failed = False
    completion_reported = False
    try:
        sequence = robot_motion.build_pick_sequence(
            solver,
            robot,
            pallet,
            indices,
            lower_deg,
            upper_deg,
            base_position,
            base_quaternion,
        )
        print("[Integration V1] READY → Pick → Retract를 시작합니다.")
    except RuntimeError as error:
        failed = True
        world.pause()
        print(f"[Integration V1] Pick 계획 생성 실패: {error}")
        print("[Integration V1] Scene pose를 확인할 수 있도록 창을 유지합니다.")

    while simulation_app.is_running():
        if world.is_stopped() or not world.is_playing():
            world.render()
            continue
        if failed:
            world.pause()
            continue

        try:
            sequence.update(PHYSICS_DT)
            world.step(render=True)
            if sequence.done and not completion_reported:
                print("[Integration V1] Pick/Retract 완료. 창을 닫으면 종료합니다.")
                completion_reported = True
        except RuntimeError as error:
            failed = True
            world.pause()
            print(f"[Integration V1] Pick/Retract 중단: {error}")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
