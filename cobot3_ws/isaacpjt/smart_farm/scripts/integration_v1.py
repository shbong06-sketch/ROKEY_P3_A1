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
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

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
CONVEYOR_SEG6_PRIM_NAME = "Seg_6"
CONVEYOR_SEG6_PRIM_PATH = f"{CONVEYOR_PRIM_PATH}/{CONVEYOR_SEG6_PRIM_NAME}"
CONVEYOR_SEG6_REPORTED_XY = np.array([2.328, -5.121], dtype=float)
CONVEYOR_LATERAL_OFFSET_X_M = 0.3
CONVEYOR_STANDOFF_M = 1.5
PLACE_PALLET_QUATERNION = np.array(
    [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)], dtype=float
)
NAVIGATION_START_DISTANCE_M = 0.05
NAVIGATION_ARRIVAL_TOLERANCE_M = 0.15

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
    ("Conveyor Seg_6", CONVEYOR_SEG6_PRIM_PATH),
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




def find_conveyor_seg6(stage):
    """Conveyor 아래에서 seg_6를 찾아 실제 prim path를 반환합니다."""
    conveyor = stage.GetPrimAtPath(CONVEYOR_PRIM_PATH)
    if not conveyor.IsValid():
        raise RuntimeError(f"Conveyor prim이 없습니다: {CONVEYOR_PRIM_PATH}")

    exact = stage.GetPrimAtPath(CONVEYOR_SEG6_PRIM_PATH)
    if exact.IsValid():
        return exact

    try:
        matches = [
            prim
            for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
            if prim.GetName() == CONVEYOR_SEG6_PRIM_NAME
            and str(prim.GetPath()).startswith(f"{CONVEYOR_PRIM_PATH}/")
        ]
    except Exception as error:
        raise RuntimeError(
            f"seg_6 stage traversal 실패: {type(error).__name__}: {error}"
        ) from error
    if not matches:
        raise RuntimeError(
            f"{CONVEYOR_PRIM_PATH} 아래에서 {CONVEYOR_SEG6_PRIM_NAME} prim을 "
            "찾지 못했습니다. Scene의 실제 이름을 확인하세요."
        )

    def xy_error(prim):
        position, _ = robot_motion.prim_world_pose(stage, str(prim.GetPath()))
        return float(np.linalg.norm(position[:2] - CONVEYOR_SEG6_REPORTED_XY))

    seg6 = min(matches, key=xy_error)
    if len(matches) > 1:
        print(
            f"[Integration V1] seg_6 후보 {len(matches)}개 중 "
            f"보고 좌표 {CONVEYOR_SEG6_REPORTED_XY.tolist()}에 가장 가까운 prim을 사용합니다."
        )
    return seg6


def conveyor_place_target(stage):
    """Seg_6 접근측 가장자리 안쪽에 팔레트를 안착할 pose를 만듭니다."""
    print("[Integration V1] Conveyor seg_6 Place pose를 계산합니다.")
    seg6 = find_conveyor_seg6(stage)
    seg6_path = str(seg6.GetPath())
    seg6_position, seg6_quaternion = robot_motion.prim_world_pose(stage, seg6_path)

    try:
        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
            True,
        )
        aligned_box = bbox_cache.ComputeWorldBound(seg6).ComputeAlignedBox()
        bounds_min = np.array(aligned_box.GetMin(), dtype=float)
        bounds_max = np.array(aligned_box.GetMax(), dtype=float)
    except Exception as error:
        raise RuntimeError(
            f"seg_6 world bounding box 계산 실패 ({seg6_path}): "
            f"{type(error).__name__}: {error}"
        ) from error
    if not np.all(np.isfinite(bounds_min)) or not np.all(np.isfinite(bounds_max)):
        raise RuntimeError(f"seg_6 world bounding box를 계산하지 못했습니다: {seg6_path}")

    destination = np.array(
        [
            seg6_position[0],
            bounds_max[1] - robot_motion.PALLET_FRONT,
            bounds_max[2],
        ],
        dtype=float,
    )
    print(f"[Integration V1] Conveyor seg_6 path: {seg6_path}")
    print(
        f"[Integration V1] Conveyor seg_6 pose: "
        f"position={np.round(seg6_position, 4).tolist()}, "
        f"quaternion(wxyz)={np.round(seg6_quaternion, 5).tolist()}"
    )
    print(
        f"[Integration V1] Conveyor seg_6 bounds: "
        f"min={np.round(bounds_min, 4).tolist()}, "
        f"max={np.round(bounds_max, 4).tolist()}"
    )
    print(
        f"[Integration V1] Place pallet origin: "
        f"position={np.round(destination, 4).tolist()}, "
        f"quaternion(wxyz)={np.round(PLACE_PALLET_QUATERNION, 5).tolist()}"
    )
    print(
        "[Integration V1] Place Y calculation: "
        f"Seg_6 max_y={bounds_max[1]:.4f} - "
        f"pallet_half_depth={robot_motion.PALLET_FRONT:.4f} "
        f"= {destination[1]:.4f}"
    )
    return destination


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
        ("LiftRig", LIFT_RIG_PRIM_PATH),
        ("Nova chassis", NOVA_ARTICULATION_ROOT_PATH),
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

    # path_runner_smooth의 waypoint 변환에 바로 사용할 시작 기준값입니다.
    for label in ("LiftRig", "Nova chassis"):
        position, quaternion = poses[label]
        print(
            f"[Navigation reference] {label}: "
            f"world_xy={np.round(position[:2], 4).tolist()}, "
            f"world_yaw_deg={robot_motion.yaw_deg(quaternion):.2f}"
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
    navigation_goal_xy = CONVEYOR_SEG6_REPORTED_XY + np.array(
        [CONVEYOR_LATERAL_OFFSET_X_M, CONVEYOR_STANDOFF_M], dtype=float
    )
    print(
        f"[Integration V1] Navigation arrival target: "
        f"world_xy={np.round(navigation_goal_xy, 4).tolist()}, "
        f"tolerance={NAVIGATION_ARRIVAL_TOLERANCE_M:.2f} m"
    )

    base_position, base_quaternion = robot_motion.prim_world_pose(
        stage, M0609_BASE_LINK_PRIM_PATH
    )
    sequence = None
    phase = "PICK"
    failed = False
    navigation_started = False
    navigation_start_position = None
    arrival_watcher = robot_motion.BaseWatcher()
    place_completion_reported = False
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
            if phase == "PLACE":
                rig_switch.work()
            else:
                rig_switch.auto(PHYSICS_DT)
            world.step(render=True)

            if phase == "PICK" and sequence.done:
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
                phase = "WAIT_NAVIGATION"
                navigation_start_position = np.asarray(
                    robot.get_world_pose()[0], dtype=float
                )
                arrival_watcher.reset()

            elif phase == "WAIT_NAVIGATION":
                chassis_position = np.asarray(robot.get_world_pose()[0], dtype=float)
                moved_from_start = float(
                    np.linalg.norm(chassis_position[:2] - navigation_start_position[:2])
                )
                wheel_speed = rig_switch.commanded_wheel_speed()
                if not navigation_started and (
                    wheel_speed > 1.0e-3
                    or moved_from_start >= NAVIGATION_START_DISTANCE_M
                ):
                    navigation_started = True
                    arrival_watcher.reset()
                    print(
                        "[Integration V1] AMR 주행 시작 감지: "
                        f"start_xy={np.round(navigation_start_position[:2], 4).tolist()}"
                    )

                if navigation_started:
                    arrival_watcher.update(robot, PHYSICS_DT)
                    if arrival_watcher.settled and rig_switch.mode == "work":
                        arrival_error = float(
                            np.linalg.norm(chassis_position[:2] - navigation_goal_xy)
                        )
                        print(
                            "[Integration V1] AMR 정지 확인: "
                            f"actual_xy={np.round(chassis_position[:2], 4).tolist()}, "
                            f"target_xy={np.round(navigation_goal_xy, 4).tolist()}, "
                            f"error={arrival_error:.3f} m"
                        )
                        if arrival_error > NAVIGATION_ARRIVAL_TOLERANCE_M:
                            raise RuntimeError(
                                "AMR이 목표 밖에서 정지했습니다: "
                                f"위치 오차 {arrival_error:.3f} m "
                                f"(허용 {NAVIGATION_ARRIVAL_TOLERANCE_M:.3f} m)"
                            )

                        base_position, base_quaternion = robot_motion.prim_world_pose(
                            stage, M0609_BASE_LINK_PRIM_PATH
                        )
                        _, place_fork_quaternion = robot.end_effector.get_world_pose()
                        print(
                            "[Integration V1] Place에 운반 자세 link_6 quaternion을 "
                            f"유지합니다: {np.round(place_fork_quaternion, 5).tolist()}"
                        )
                        place_position = conveyor_place_target(stage)
                        sequence = robot_motion.build_place_sequence(
                            solver,
                            robot,
                            pallet,
                            indices,
                            lower_deg,
                            upper_deg,
                            base_position,
                            base_quaternion,
                            place_position,
                            PLACE_PALLET_QUATERNION,
                            place_fork_quaternion,
                        )
                        phase = "PLACE"
                        print(
                            "[Integration V1] AMR 도착 → 브레이크 ON → "
                            "seg_6 Place → Fork Out을 시작합니다."
                        )

            elif phase == "PLACE" and sequence.done and not place_completion_reported:
                print("[Integration V1] Pick → Transport → Place 1 cycle 완료.")
                place_completion_reported = True

        except RuntimeError as error:
            failed = True
            rig_switch.work()
            world.pause()
            print(f"[Integration V1] {phase} 중단: {error}")
            print("[Integration V1] 상태를 확인할 수 있도록 창을 유지합니다.")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
