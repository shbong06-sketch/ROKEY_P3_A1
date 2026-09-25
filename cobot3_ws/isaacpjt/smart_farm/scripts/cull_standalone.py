"""Cabbage 시험 장면에서 고정 좌표 솎아내기 Pick 단위 동작을 검증한다.

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
    / "Collected_smartfarm_cull_test"
    / "Collected_smartfarm_cull_test.usd"
)
CONTROL_CUBE_PATH = "/World/CullContactControlCube"
CONTROL_SUPPORT_PATH = "/World/CullContactControlSupport"

ROBOT_PATH = "/World/SmartFarm/Placed/M0609/Asset"
BASE_PATH = f"{ROBOT_PATH}/base_link"
EE_PATH = f"{ROBOT_PATH}/link_6"
GRIPPER_ROOT_PATH = f"{ROBOT_PATH}/onrobot_rg2ft"
LEFT_FINGER_PATH = f"{GRIPPER_ROOT_PATH}/left_inner_finger"
RIGHT_FINGER_PATH = f"{GRIPPER_ROOT_PATH}/right_inner_finger"
CABBAGE_PALLET_PATH = "/World/SmartFarm/Placed/cabbage_pallet_6_inspect"
TARGET_PATH = f"{CABBAGE_PALLET_PATH}/root_001/Cabbage_03"
GRIP_COLLIDER_PATH = f"{TARGET_PATH}/collision_grip"
PALLET_BODY_PATH = f"{CABBAGE_PALLET_PATH}/Cube_011_001"
CABBAGE_PATHS = tuple(
    f"{CABBAGE_PALLET_PATH}/root_001/Cabbage_{index:02d}"
    for index in range(1, 7)
)

URDF_PATH = M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf"
RMPFLOW_DIR = M0609_DIR / "rmpflow"
RMPFLOW_DESCRIPTION_PATH = RMPFLOW_DIR / "m0609_description.yaml"
RMPFLOW_CONFIG_PATH = RMPFLOW_DIR / "m0609_rmpflow_common.yaml"

# Cabbage_03/collision_grip의 안정화 후 base 기준 중심은 약
# (0.2459, 0.5153, 0.1107) m이다. 입력 Z=0.1407에서는 GRASP 시
# 손가락 collider 하단이 grip 상단보다 36.9 mm 높아 접촉이 없다.
# 입력 Z를 40~50 mm 낮추면 접촉은 생기지만 물리 pose가 불안정해진다.
# 원인 분리 전까지 검증된 도달 좌표를 유지한다.
# CullPickConfig.pick_z_offset 50 mm는 이 값에 추가 적용된다.
# 이후 비전 통합에서는 이 상수 대신 카메라→베이스 변환 결과를 start_pick에 넣는다.
FIXED_PICK_POSITION_BASE = (0.2459, 0.5153, 0.0937)

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
    parser.add_argument("--scene", type=Path, default=SCENE_PATH)
    parser.add_argument("--diagnose-contact", action="store_true")
    parser.add_argument("--control-cube", action="store_true",
                        help="세션 레이어의 Cube로 Cabbage 접촉을 대조")
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
        help="로봇을 움직이지 않고 Pallet/Cabbage 초기 안정성만 측정",
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
if args.control_cube and not args.diagnose_contact:
    raise ValueError("--control-cube에는 --diagnose-contact가 필요합니다.")

from isaacsim import SimulationApp


app = SimulationApp(
    {
        "headless": args.headless,
        "extra_args": kit_args,
    }
)

import numpy as np
import omni.usd
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics
from omni.physx import get_physx_simulation_interface

from isaacsim.core.api import World
from isaacsim.core.api.sensors import RigidContactView
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim, SingleXFormPrim
from isaacsim.core.utils.xforms import get_world_pose
from isaacsim.robot.manipulators.grippers import ParallelGripper

if str(RMPFLOW_DIR) not in sys.path:
    sys.path.insert(0, str(RMPFLOW_DIR))

from m0609_rmpflow_controller import RMPFlowController
from cull_motion import CullMotion, CullPickConfig


def open_scene():
    """선택한 장면을 열고 단독 시험에 필요 없는 Action Graph를 세션에서 끈다."""

    for path in (
        args.scene,
        URDF_PATH,
        RMPFLOW_DESCRIPTION_PATH,
        RMPFLOW_CONFIG_PATH,
    ):
        if not path.is_file():
            raise FileNotFoundError(path)

    omni.usd.get_context().open_stage(str(args.scene.resolve()))
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
        GRIP_COLLIDER_PATH,
        PALLET_BODY_PATH,
        *CABBAGE_PATHS,
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

    print(f"[장면] {args.scene.resolve()}")
    print(f"[장면] 단독 시험용 Action Graph 비활성화: {disabled_graphs}개")
    print(f"[로봇] {ROBOT_PATH}")
    print(f"[대상] {TARGET_PATH}")
    return stage


def _is_within(path, root):
    path, root = str(path), str(root)
    return path == root or path.startswith(root + "/")


def _rigid_owner(prim):
    while prim and prim.IsValid() and not prim.IsPseudoRoot():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return str(prim.GetPath())
        prim = prim.GetParent()
    return None


def diagnose_usd_collision(stage):
    """합성 USD만 검사한다. YES는 PhysX contact 발생을 보증하지 않는다."""
    prims = list(stage.Traverse(Usd.TraverseInstanceProxies()))
    groups = [p for p in prims if p.IsA(UsdPhysics.CollisionGroup)]
    group_table = UsdPhysics.CollisionGroup.ComputeCollisionGroupTable(stage)
    roots = (LEFT_FINGER_PATH, RIGHT_FINGER_PATH, TARGET_PATH)
    related = [p for p in prims if any(_is_within(p.GetPath(), r) for r in roots)]
    members = {}
    colliders = {}

    for group in groups:
        print(f"[GROUP RULE] prim={group.GetPath()} "
              f"filteredGroups={list(group.GetFilteredGroupsRel().GetTargets())} "
              f"invertFilteredGroups={group.GetInvertFilteredGroupsAttr().Get()}")

    for prim in related:
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        path = str(prim.GetPath())
        collision = UsdPhysics.CollisionAPI(prim)
        mesh = UsdPhysics.MeshCollisionAPI(prim)
        groups_here = [g for g in groups if g.GetCollidersCollectionAPI()
                       .ComputeMembershipQuery().IsPathIncluded(prim.GetPath())]
        members[path] = groups_here
        print(f"[COLLIDER] prim={path} type={prim.GetTypeName()} "
              f"collisionEnabled={collision.GetCollisionEnabledAttr().Get()} "
              f"owner={_rigid_owner(prim)} "
              f"approximation={mesh.GetApproximationAttr().Get() if mesh else 'primitive'} "
              f"instanceProxy={prim.IsInstanceProxy()} "
              f"physxSchemas={[s for s in prim.GetAppliedSchemas() if s.startswith('Physx')]}")
        print(f"[COLLISION GROUP] prim={path} "
              f"groups={[str(g.GetPath()) for g in groups_here]}")
        if collision.GetCollisionEnabledAttr().Get():
            for root in roots:
                if _is_within(path, root):
                    colliders.setdefault(root, []).append(prim)

    filtered = []
    for prim in prims:
        if prim.HasAPI(UsdPhysics.FilteredPairsAPI):
            for target in UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets():
                filtered.append((str(prim.GetPath()), str(target)))
                print(f"[FILTERED PAIR] source={prim.GetPath()} target={target}")

    articulation = stage.GetPrimAtPath(f"{ROBOT_PATH}/root_joint")
    if articulation:
        print(f"[ARTICULATION] prim={articulation.GetPath()} "
              f"selfCollision={articulation.GetAttribute('physxArticulation:enabledSelfCollisions').Get()}")
    for prim in stage.Traverse():
        attr = prim.GetAttribute("physxScene:invertCollisionGroupFilter")
        if attr:
            print(f"[PHYSX SCENE] prim={prim.GetPath()} "
                  f"invertCollisionGroupFilter={attr.Get()}")

    for root in (LEFT_FINGER_PATH, RIGHT_FINGER_PATH):
        reasons = []
        left, right = colliders.get(root, []), colliders.get(TARGET_PATH, [])
        if not left or not right:
            reasons.append("활성 collider 없음")
        for a in left:
            for b in right:
                a_path, b_path = str(a.GetPath()), str(b.GetPath())
                a_owner, b_owner = _rigid_owner(a), _rigid_owner(b)
                if not a_owner or not b_owner or a_owner == b_owner:
                    reasons.append(f"rigid-body owner 불일치: {a_path} ↔ {b_path}")
                a_scope = (a_path, a_owner, GRIPPER_ROOT_PATH, ROBOT_PATH,
                           f"{ROBOT_PATH}/root_joint")
                b_scope = (b_path, b_owner, TARGET_PATH)
                for source, target in filtered:
                    source_on_a = source == f"{ROBOT_PATH}/root_joint" or _is_within(a_path, source)
                    source_on_b = _is_within(b_path, source)
                    if ((source_on_a and any(_is_within(p, target) for p in b_scope))
                            or (source_on_b and any(_is_within(p, target) for p in a_scope))):
                        reasons.append(f"FilteredPairs: {source} → {target}")
                for ga in members[a_path]:
                    for gb in members[b_path]:
                        if not group_table.IsCollisionEnabled(ga.GetPrim(), gb.GetPrim()):
                            reasons.append(f"CollisionGroup 차단: {ga.GetPath()} ↔ {gb.GetPath()}")
        print(f"[PAIR] {root} ↔ {TARGET_PATH} collision expected: "
              f"{'NO' if reasons else 'YES'}; "
              f"reason: {'; '.join(dict.fromkeys(reasons)) if reasons else '활성 collider, 별도 rigid body, 명시적 filter 없음 (USD 판정)'}")

    for path in (ROBOT_PATH, GRIPPER_ROOT_PATH, LEFT_FINGER_PATH,
                 RIGHT_FINGER_PATH, f"{LEFT_FINGER_PATH}/collisions", TARGET_PATH,
                 GRIP_COLLIDER_PATH):
        prim = stage.GetPrimAtPath(path)
        if prim:
            print(f"[PRIM STACK] prim={path} "
                  f"layers={[spec.layer.identifier for spec in prim.GetPrimStack()]}")


def create_control_cube(stage):
    """별도 대조 실행에서만 Cabbage collider를 끄고 session Cube를 만든다."""
    if stage.GetPrimAtPath(CONTROL_CUBE_PATH).IsValid():
        enabled_colliders = [
            str(prim.GetPath())
            for prim in Usd.PrimRange(stage.GetPrimAtPath(TARGET_PATH))
            if prim.HasAPI(UsdPhysics.CollisionAPI)
            and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        ]
        if enabled_colliders:
            raise RuntimeError(
                "기존 Cube Scene에 Cabbage collider가 활성화되어 있습니다. "
                "이전 Romaine용 overlay 대신 기본 Cabbage Scene에서 "
                "--control-cube를 실행하세요: " + ", ".join(enabled_colliders)
            )
        print(f"[CONTROL CUBE] 별도 Scene에 이미 구성됨: {CONTROL_CUBE_PATH}")
        return
    grip = stage.GetPrimAtPath(GRIP_COLLIDER_PATH)
    center = UsdGeom.XformCache().GetLocalToWorldTransform(grip).Transform(Gf.Vec3d(0, 0, 0))
    for prim in Usd.PrimRange(stage.GetPrimAtPath(TARGET_PATH)):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
    stage.GetPrimAtPath(TARGET_PATH).GetAttribute("physics:kinematicEnabled").Set(True)
    cube = UsdGeom.Cube.Define(stage, CONTROL_CUBE_PATH)
    cube.GetSizeAttr().Set(0.045)
    UsdGeom.Xformable(cube).AddTranslateOp().Set(center)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim()).CreateRigidBodyEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(0.05)
    # 작은 고정 받침이 Cube의 바닥만 지탱한다. 없으면 GRASP 전 낙하한다.
    support = UsdGeom.Cube.Define(stage, CONTROL_SUPPORT_PATH)
    support.GetSizeAttr().Set(1.0)
    UsdGeom.Xformable(support).AddTranslateOp().Set(
        center - Gf.Vec3d(0, 0, 0.0325)
    )
    UsdGeom.Xformable(support).AddScaleOp().Set(Gf.Vec3f(0.02, 0.02, 0.02))
    UsdPhysics.CollisionAPI.Apply(support.GetPrim()).CreateCollisionEnabledAttr(True)
    print(f"[CONTROL CUBE] prim={CONTROL_CUBE_PATH} center={center} "
          "size=0.045m mass=0.05kg; small static support; "
          "Cabbage collider disabled in session")


def make_contact_views(filter_path):
    return {
        side: RigidContactView(
            prim_paths_expr=path, filter_paths_expr=[filter_path],
            name=f"cull_contact_{side}", prepare_contact_sensors=True,
            disable_stablization=False, max_contact_count=32,
        )
        for side, path in (("left", LEFT_FINGER_PATH), ("right", RIGHT_FINGER_PATH))
    }


def log_contact_frame(views, frame, label):
    fields = []
    for side, view in views.items():
        normal_forces, _, _, _, counts, _ = view.get_contact_force_data(dt=PHYSICS_DT)
        count = int(np.asarray(counts).sum())
        force = float(np.abs(np.asarray(normal_forces)[:count]).sum())
        fields.append(f"{side} contact={count > 0} points={count} "
                      f"sum_abs_normal_N={force:.5f}")
    print(f"[GRASP {frame}] target={label} " + " ".join(fields), flush=True)


class RawContactAudit:
    """PhysX contact report의 실제 rigid body와 collider 경로를 집계한다."""

    def __init__(self, stage, motion):
        self.motion = motion
        self.pairs = {}
        for path in (LEFT_FINGER_PATH, RIGHT_FINGER_PATH, TARGET_PATH, CONTROL_CUBE_PATH):
            prim = stage.GetPrimAtPath(path)
            if prim and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
                api.CreateThresholdAttr().Set(0.0)
        self.subscription = get_physx_simulation_interface().subscribe_contact_report_events(
            self._on_contacts
        )

    def _on_contacts(self, headers, contact_data):
        for header in headers:
            if header.num_contact_data == 0:
                continue
            actors = tuple(str(PhysicsSchemaTools.intToSdfPath(value))
                           for value in (header.actor0, header.actor1))
            colliders = tuple(str(PhysicsSchemaTools.intToSdfPath(value))
                              for value in (header.collider0, header.collider1))
            if not any(_is_within(path, finger) for path in (*actors, *colliders)
                       for finger in (LEFT_FINGER_PATH, RIGHT_FINGER_PATH)):
                continue
            key = (actors, colliders)
            contacts = contact_data[
                header.contact_data_offset:header.contact_data_offset + header.num_contact_data
            ]
            if key not in self.pairs:
                self.pairs[key] = {"events": 0, "grasp_events": 0,
                                   "min_separation": float("inf"),
                                   "max_separation": float("-inf"),
                                   "sample_position": None,
                                   "sample_normal": None}
                print(f"[RAW CONTACT FIRST] stage={self.motion.current_stage} "
                      f"actor0={actors[0]} actor1={actors[1]} "
                      f"collider0={colliders[0]} collider1={colliders[1]} "
                      f"position={contacts[0].position} normal={contacts[0].normal} "
                      f"separation={contacts[0].separation}", flush=True)
            pair = self.pairs[key]
            pair["events"] += 1
            for contact in contacts:
                pair["min_separation"] = min(pair["min_separation"], float(contact.separation))
                pair["max_separation"] = max(pair["max_separation"], float(contact.separation))
                pair["sample_position"] = str(contact.position)
                pair["sample_normal"] = str(contact.normal)
            if self.motion.current_stage == "GRASP":
                pair["grasp_events"] += 1

    def print_summary(self):
        for (actors, colliders), counts in sorted(self.pairs.items()):
            print(f"[RAW CONTACT SUMMARY] actor0={actors[0]} actor1={actors[1]} "
                  f"collider0={colliders[0]} collider1={colliders[1]} "
                  f"events={counts['events']} grasp_events={counts['grasp_events']} "
                  f"separation_range_m=({counts['min_separation']:.5f},"
                  f"{counts['max_separation']:.5f}) "
                  f"last_position={counts['sample_position']} "
                  f"last_normal={counts['sample_normal']}",
                  flush=True)


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
    """로봇 명령 없이 Pallet과 Cabbage의 초기 물리 안정성을 측정한다."""

    paths = (PALLET_BODY_PATH, *CABBAGE_PATHS)

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
    """Pick 종료 시 Pallet과 Cabbage 전체의 pose 변화를 출력한다."""

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


def create_runtime(stage, contact_filter_path=None):
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
    target_type = SingleXFormPrim if args.control_cube else SingleRigidPrim
    target = world.scene.add(
        target_type(prim_path=TARGET_PATH, name="cull_target_cabbage_03")
    )
    contact_views = make_contact_views(contact_filter_path) if contact_filter_path else {}

    world.reset()
    robot.initialize(physics_sim_view=world.physics_sim_view)
    for view in contact_views.values():
        view.initialize(physics_sim_view=world.physics_sim_view)
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
            approach_clearance=0.12,
            lift_clearance=0.12,
            # 아래보기 자세를 유지하면서 손가락 간격 축을 90도 돌려
            # 기존 아래보기 자세와 손가락 간격 축 회전을 유지한다.
            tool_orientation_base=(0.0, 1.0 / sqrt(2.0), -1.0 / sqrt(2.0), 0.0),
            min_target_rise=0.05,
        ),
    )
    return world, robot, gripper, target, controller, motion, contact_views


def main():
    stage = open_scene()
    if args.diagnose_contact:
        diagnose_usd_collision(stage)
    if args.control_cube:
        create_control_cube(stage)
    configure_drives(stage)
    contact_filter_path = (CONTROL_CUBE_PATH if args.control_cube else TARGET_PATH) if args.diagnose_contact else None
    world, robot, gripper, target, controller, motion, contact_views = create_runtime(
        stage, contact_filter_path
    )
    raw_audit = RawContactAudit(stage, motion) if args.diagnose_contact else None

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
    grasp_frame = 0
    previous_playing = world.is_playing()

    while app.is_running():
        stage_before_step = motion.current_stage if started else None
        world.step(render=not args.headless)
        is_playing = world.is_playing()

        if is_playing and not previous_playing:
            world.reset()
            robot.initialize(physics_sim_view=world.physics_sim_view)
            for view in contact_views.values():
                view.initialize(physics_sim_view=world.physics_sim_view)
            initialize_gripper(gripper, robot, world)
            set_ready_pose(robot)
            controller.reset()
            motion.cancel()
            started = False
            step_count = 0
            grasp_frame = 0

        if not is_playing:
            previous_playing = is_playing
            continue

        if stage_before_step == "GRASP" and contact_views:
            grasp_frame += 1
            log_contact_frame(contact_views, grasp_frame, contact_filter_path)

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
                for path in (PALLET_BODY_PATH, *CABBAGE_PATHS)
            }
            motion.start_pick(FIXED_PICK_POSITION_BASE)
            started = True

        try:
            motion.update()
        except Exception:
            if pick_initial_poses is not None and not pick_summary_printed:
                print_pick_pose_summary(pick_initial_poses)
                pick_summary_printed = True
            if raw_audit:
                raw_audit.print_summary()
            raise
        step_count += 1

        if args.control_cube and stage_before_step == "GRASP" and motion.current_stage == "LIFT":
            cube_position, _ = get_world_pose(CONTROL_CUBE_PATH)
            print(f"[CONTROL CUBE] GRASP 종료 world={np.round(cube_position, 4).tolist()}", flush=True)
            raw_audit.print_summary()
            break


        if step_count % LOG_INTERVAL_STEPS == 0:
            target_position, _ = target.get_world_pose()
            left_finger, _ = get_world_pose(LEFT_FINGER_PATH)
            right_finger, _ = get_world_pose(RIGHT_FINGER_PATH)
            finger = robot.get_joint_positions()[robot.get_dof_index("finger_joint")]
            tcp_world = motion._tcp_world_position()
            tcp_goal = motion._goal_world if motion._entered else None
            tcp_error_mm = (
                float(np.linalg.norm(tcp_world - tcp_goal) * 1000.0)
                if tcp_goal is not None else None
            )
            arm_positions = robot.get_joint_positions()[
                [robot.get_dof_index(name) for name in ARM_JOINTS]
            ]
            print(
                f"[상태] step={step_count} stage={motion.current_stage} "
                f"target={np.round(target_position, 4).tolist()} "
                f"tcp={np.round(tcp_world, 4).tolist()} "
                f"tcp_goal={np.round(tcp_goal, 4).tolist() if tcp_goal is not None else None} "
                f"tcp_error_mm={round(tcp_error_mm, 1) if tcp_error_mm is not None else None}"
            )
            print(
                f"[관절] step={step_count} "
                f"arm_deg={np.round(np.rad2deg(arm_positions), 2).tolist()} "
                f"finger={finger:+.4f} "
                f"left={np.round(left_finger, 4).tolist()} "
                f"right={np.round(right_finger, 4).tolist()}"
            )
            if args.control_cube:
                cube_position, _ = get_world_pose(CONTROL_CUBE_PATH)
                print(f"[CONTROL CUBE] step={step_count} world={np.round(cube_position, 4).tolist()}")

        if motion.is_done:
            if raw_audit:
                raw_audit.print_summary()
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
