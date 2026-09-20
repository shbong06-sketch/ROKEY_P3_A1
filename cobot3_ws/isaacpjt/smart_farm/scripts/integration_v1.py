"""Integration V1: Pick/Retract 뒤 ROS 2 /cmd_vel 주행을 준비합니다."""

import os
import sys
from pathlib import Path


# Isaac Sim 5.1 내장 ROS 2 Jazzy 라이브러리는 프로세스 시작 때 검색되어야 합니다.
isaac_root = os.environ.get("ISAAC_PATH") or os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(sys.executable)), "..", "..", "..")
)
ros_lib = os.path.join(
    isaac_root, "exts", "isaacsim.ros2.bridge", "jazzy", "lib"
)
if (
    os.path.isdir(ros_lib)
    and ros_lib not in os.environ.get("LD_LIBRARY_PATH", "")
    and not os.environ.get("ROS_DISTRO")
    and os.environ.get("SMARTFARM_ROS_REEXEC") != "1"
):
    current_ld_path = os.environ.get("LD_LIBRARY_PATH", "")
    os.environ["LD_LIBRARY_PATH"] = os.pathsep.join(
        path for path in (current_ld_path, ros_lib) if path
    )
    os.environ["SMARTFARM_ROS_REEXEC"] = "1"
    print(f"[ROS2] Bridge library path 추가 후 다시 실행합니다: {ros_lib}")
    os.execv(sys.executable, [sys.executable] + sys.argv)

os.environ.setdefault("ROS_DISTRO", "jazzy")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": False})


import numpy as np
import omni.usd
from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade

from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension

import robot_motion


SCENE_DIR = (
    Path(__file__).resolve().parent.parent
    / "scenes"
    / "Collected_smartfarm_v004"
)
sys.path.insert(0, str(SCENE_DIR))
from rig_mode import RigModeSwitch, apply_rig_stability


enable_extension("isaacsim.ros2.bridge")
simulation_app.update()
print("[Integration V1] ROS 2 Bridge extension enabled: /cmd_vel")


USD_PATH = SCENE_DIR / "Collected_smartfarm_v004.usd"

PHYSICS_DT = 1.0 / 60.0

# Scene configuration confirmed from Collected_smartfarm_v004.usd.
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

GRIP_MATERIAL_PATH = "/World/IntegrationMaterials/ForkPalletGrip"
GRIP_STATIC_FRICTION = 1.5
GRIP_DYNAMIC_FRICTION = 1.2
GRIP_RESTITUTION = 0.0


def apply_grip_friction(stage):
    """Session Layer에서 fork/pallet collider에 고마찰 재질을 적용합니다."""
    stage.DefinePrim("/World/IntegrationMaterials", "Scope")
    material = UsdShade.Material.Define(stage, GRIP_MATERIAL_PATH)
    material_prim = material.GetPrim()

    physics_material = UsdPhysics.MaterialAPI.Apply(material_prim)
    physics_material.CreateStaticFrictionAttr().Set(GRIP_STATIC_FRICTION)
    physics_material.CreateDynamicFrictionAttr().Set(GRIP_DYNAMIC_FRICTION)
    physics_material.CreateRestitutionAttr().Set(GRIP_RESTITUTION)

    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(material_prim)
    physx_material.CreateFrictionCombineModeAttr().Set("max")

    for label, root_path in (
        ("fork", FORK_PRIM_PATH),
        ("pallet", PALLET_PRIM_PATH),
    ):
        root = stage.GetPrimAtPath(root_path)
        if not root.IsValid():
            raise RuntimeError(f"{label} prim이 없습니다: {root_path}")

        # Root binding은 reference/instance 내부 collider에도 상속됩니다.
        UsdShade.MaterialBindingAPI.Apply(root).Bind(
            material,
            UsdShade.Tokens.strongerThanDescendants,
            "physics",
        )

        collider_paths = []
        for prim in Usd.PrimRange(root):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            collider_paths.append(str(prim.GetPath()))
            if prim.IsInstanceProxy():
                continue
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                material,
                UsdShade.Tokens.strongerThanDescendants,
                "physics",
            )

        print(
            f"[Grip friction] {label}: {len(collider_paths)} collider(s), "
            f"static={GRIP_STATIC_FRICTION}, "
            f"dynamic={GRIP_DYNAMIC_FRICTION}, combine=max"
        )
        for collider_path in collider_paths:
            print(f"  - {collider_path}")
        if not collider_paths:
            print(
                f"  [Warning] 직접 탐색된 collider가 없어 root binding만 적용했습니다: "
                f"{root_path}"
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
    poses = {}
    for label, prim_path in (
        ("M0609 base", M0609_BASE_LINK_PRIM_PATH),
        ("link_6", M0609_LINK6_PRIM_PATH),
        ("fork", FORK_PRIM_PATH),
        ("pick pallet", PALLET_PRIM_PATH),
    ):
        position, quaternion = robot_motion.prim_world_pose(stage, prim_path)
        poses[label] = (position, quaternion)
        print(
            f"[Integration V1] {label}: "
            f"position={np.round(position, 4).tolist()}, "
            f"quaternion(wxyz)={np.round(quaternion, 5).tolist()}"
        )

    link_position, link_quaternion = poses["link_6"]
    fork_position, fork_quaternion = poses["fork"]
    link_rotation = robot_motion.quat_to_rot_matrix(link_quaternion)
    fork_in_link_position = link_rotation.T @ (fork_position - link_position)
    fork_in_link_rotation = (
        link_rotation.T @ robot_motion.quat_to_rot_matrix(fork_quaternion)
    )
    print(
        "[Integration V1] fork relative to link_6: "
        f"position={np.round(fork_in_link_position, 5).tolist()}"
    )
    print(
        "[Integration V1] fork rotation relative to link_6=\n"
        f"{np.array2string(fork_in_link_rotation, precision=4, suppress_small=True)}"
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
    apply_grip_friction(stage)

    print(f"[Integration V1] Stage ready: {stage.GetRootLayer().realPath}")
    print_scene_debug(stage)
    apply_rig_stability(stage, NOVA_PRIM_PATH)

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
            initial_joints_deg=robot_motion.TRAVEL_STOW_JOINTS_DEG,
        )
    )

    for _ in range(10):
        world.step(render=True)
    rig_switch = RigModeSwitch(robot)

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
        print("[Integration V1] PRE_PICK → READY → Pick → Retract → Carry Rotate를 시작합니다.")
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
            rig_switch.auto(PHYSICS_DT)
            world.step(render=True)
            if sequence.done and not completion_reported:
                _, fork_quaternion = robot_motion.prim_world_pose(
                    stage, FORK_PRIM_PATH
                )
                fork_direction = robot_motion.quat_to_rot_matrix(
                    fork_quaternion
                )[:, 2]
                print(
                    "[Integration V1] 운반 자세 fork +Z world axis="
                    f"{np.round(fork_direction, 4).tolist()}"
                )
                print(
                    "[Integration V1] 운반 자세 완료. ROS 2 /cmd_vel 입력을 기다립니다."
                )
                completion_reported = True
        except RuntimeError as error:
            failed = True
            rig_switch.work()
            world.pause()
            print(f"[Integration V1] Pick/Carry 중단: {error}")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
