"""좌표 기반 준비 → 도킹 → 삽입 → 인양 → 유지.

좌표는 월드 기준 link_6(손목) 위치이며 m 단위입니다.
초기 준비 위치만 IK로 배치하고 이후에는 물리 관절 제어로 이동합니다.
ponytail: IK 검사는 장애물 회피가 아닙니다. 배치 변경 시 충돌도 확인하세요.
"""

from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": False})

import omni.usd
from pxr import UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.rotations import (
    quat_to_rot_matrix,
    rot_matrix_to_quat,
)
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
)


# ── 파일과 Prim 경로 ──────────────────────────────────────
ISAACPJT = Path(__file__).resolve().parents[2]

SCENE = ISAACPJT / "smart_farm/scenes/rack_pick_test/rack_pick_test.usd"
URDF = ISAACPJT / "M0609/doosan-robot2/urdf/m0609_isaac_sim.urdf"
DESCRIPTION = ISAACPJT / "M0609/descriptor/m0609_description.yaml"

ROBOT_PATH = "/World/m0609"
PALLET_PATH = "/World/RecycledWoodPallet_A08_PR_NVD_01"
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]


# ── 여기에서 동작을 조정하세요 ────────────────────────────
# 이전 도킹 관절값을 URDF로 환산한 손목 좌표입니다.
# 원점은 월드입니다. 포크 끝 좌표가 아닙니다.
DOCK_POSITION = np.array([-1.192103, -0.009390, 0.833957])
READY_DISTANCE = 0.150      # 도킹점보다 15 cm 후퇴한 준비 위치
INSERT_DISTANCE = 0.050     # 도킹점에서 추가로 넣는 거리
LIFT_HEIGHT = 0.080         # 기존 3 cm 대신 8 cm 목표 (도달 검사 필수)
DOCK_SPEED = 0.010          # m/s
INSERT_SPEED = 0.010
LIFT_SPEED = 0.010

# 포크는 월드 -X로 향하고, 두 날은 월드 Y 방향으로 나란히 놓입니다.
# 기존 예시 자세와 같은 뒤집힘 방향을 유지한 정확한 수평 자세입니다.
FORK_ROTATION = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])
PREFLIGHT_SPACING = 0.003   # 경로를 3 mm 간격으로 IK 검사

SETTLE_SECONDS = 1.0
REACH_TIMEOUT = 5.0          # 보간 종료 후 도달을 기다릴 최대 시간

POSITION_TOL = 0.004         # 목표 위치 오차 4 mm
ANGLE_TOL_DEG = 3.0
MAX_TRACKING_ERROR = 0.020   # 명령보다 2 cm 이상 뒤처지면 중단
JOINT_SPEED_DEG_S = 10.0     # 관절 목표 변화 속도 제한
MAX_IK_GAP_DEG = 15.0       # 실제 관절과 IK 목표의 최대 허용 차이
PALLET_MOVE_TOL = 0.015      # 삽입 전에 팔레트가 1.5 cm 밀리면 중단
MIN_PALLET_RISE = 0.015     # 실제 팔레트가 1.5 cm 이상 올라야 인양 확인

DT = 1.0 / 120.0


def angle_error_deg(a, b):
    """두 회전행렬 사이의 각도 차이."""
    cosine = (np.trace(a.T @ b) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))


def main():
    for path in (SCENE, URDF, DESCRIPTION):
        if not path.is_file():
            raise FileNotFoundError(path)

    omni.usd.get_context().open_stage(str(SCENE))
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        app.update()

    stage = omni.usd.get_context().get_stage()

    # 실행 중 변경은 세션 레이어에만 적용합니다.
    # 원본 USD에는 저장하지 않습니다.
    stage.SetEditTarget(stage.GetSessionLayer())

    for path in (ROBOT_PATH, PALLET_PATH, f"{ROBOT_PATH}/link_6"):
        if not stage.GetPrimAtPath(path).IsValid():
            raise RuntimeError(f"Prim 경로를 확인하세요: {path}")

    # 관절값은 사용자 설정 대신 IK로 계산합니다. 한계는 USD에서 읽습니다.
    lower_deg, upper_deg = [], []
    for name in JOINT_NAMES:
        prim = stage.GetPrimAtPath(f"{ROBOT_PATH}/joints/{name}")
        joint = UsdPhysics.RevoluteJoint(prim)
        if not joint:
            raise RuntimeError(f"관절이 없습니다: {name}")
        lower_deg.append(joint.GetLowerLimitAttr().Get())
        upper_deg.append(joint.GetUpperLimitAttr().Get())
    lower = np.deg2rad(lower_deg)
    upper = np.deg2rad(upper_deg)

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=DT,
        rendering_dt=1.0 / 60.0,
        physics_prim_path="/physicsScene",
    )
    robot = world.scene.add(
        SingleManipulator(
            prim_path=ROBOT_PATH,
            name="m0609",
            end_effector_prim_path=f"{ROBOT_PATH}/link_6",
        )
    )
    pallet = world.scene.add(
        SingleRigidPrim(prim_path=PALLET_PATH, name="pallet")
    )

    world.reset()
    indices = np.array([robot.get_dof_index(n) for n in JOINT_NAMES])

    world.pause()

    lula = LulaKinematicsSolver(
        robot_description_path=str(DESCRIPTION),
        urdf_path=str(URDF),
    )
    lula.set_robot_base_pose(*robot.get_world_pose())
    solver = ArticulationKinematicsSolver(robot, lula, "link_6")

    def step():
        """Pause에서는 대기하고, Stop 또는 창 닫기에서는 작업을 종료."""
        while app.is_running():
            if world.is_stopped():
                raise RuntimeError("Stop을 눌렀습니다. 재시험은 프로그램을 다시 실행하세요.")
            if world.is_playing():
                world.step(render=True)
                return
            world.render()
        raise RuntimeError("시뮬레이터 창을 닫았습니다.")

    def wait(seconds):
        for _ in range(int(np.ceil(seconds / DT))):
            step()

    def pallet_position():
        return np.array(pallet.get_world_pose()[0], dtype=float)

    def ee_pose():
        return solver.compute_end_effector_pose()

    def solve_target(position, quaternion, seed):
        """실제 이동과 사전 검사에서 동일한 IK 및 관절 한계를 사용합니다."""
        q, success = lula.compute_inverse_kinematics(
            frame_name="link_6",
            target_position=position,
            target_orientation=quaternion,
            warm_start=seed,
            position_tolerance=0.001,
            orientation_tolerance=np.deg2rad(0.5),
        )
        if not success or not np.all(np.isfinite(q)):
            raise RuntimeError(f"IK 실패: 손목 목표 {np.round(position, 4)}")
        q = np.array(q, dtype=float)
        # ±360° 표현 차이를 실제 회전으로 명령하지 않습니다.
        for i in range(len(q)):
            equivalents = q[i] + 2 * np.pi * np.arange(-2, 3)
            valid = equivalents[(equivalents >= lower[i]) & (equivalents <= upper[i])]
            if not len(valid):
                raise RuntimeError(f"{JOINT_NAMES[i]} IK 목표가 관절 한계 밖입니다.")
            q[i] = valid[np.argmin(np.abs(valid - seed[i]))]
        return q

    def apply_target(position, orientation):
        current = robot.get_joint_positions()[indices]
        q = solve_target(position, orientation, current)
        ids = indices

        gap_deg = np.max(np.abs(np.rad2deg(q - current)))
        if gap_deg > MAX_IK_GAP_DEG:
            raise RuntimeError(
                f"IK 목표와 실제 관절 차이가 {gap_deg:.1f}°입니다. "
                "도달 범위·충돌·자세를 확인하세요."
            )

        # 관절별 이동 비율을 유지하며 한 스텝의 명령 변화량을 제한합니다.
        max_delta = np.deg2rad(JOINT_SPEED_DEG_S) * DT
        delta = q - current
        largest_delta = float(np.max(np.abs(delta)))

        ratio = min(1.0, max_delta / max(largest_delta, 1e-12))
        limited_q = current + ratio * delta

        robot.apply_action(
            ArticulationAction(
                joint_positions=limited_q,
                joint_indices=ids,
            )
        )

    def move(name, goal, quaternion, speed, guard=None):
        """도킹 때의 포크 방향을 유지하면서 위치만 보간합니다."""
        start, _ = ee_pose()
        target_rotation = quat_to_rot_matrix(quaternion)
        distance = np.linalg.norm(goal - start)
        duration = max(distance / speed, 0.5)
        count = int(np.ceil(duration / DT))
        print(f"[{name}] 목표 {np.round(goal, 3)}, 약 {duration:.1f}초")

        for i in range(1, count + 1):
            alpha = i / count
            position = start + alpha * (goal - start)
            apply_target(position, quaternion)
            step()
            if guard:
                guard()
            actual, actual_rotation = ee_pose()
            if angle_error_deg(actual_rotation, target_rotation) > ANGLE_TOL_DEG:
                raise RuntimeError(f"{name}: 포크 방향 유지에 실패했습니다.")
            if np.linalg.norm(actual - position) > MAX_TRACKING_ERROR:
                raise RuntimeError(f"{name}: 움직임이 막히거나 목표를 따라가지 못합니다.")

        # 보간 시간이 끝났다고 성공 처리하지 않습니다.
        for _ in range(int(REACH_TIMEOUT / DT)):
            actual, actual_rotation = ee_pose()
            reached = (
                np.linalg.norm(actual - goal) <= POSITION_TOL
                and angle_error_deg(actual_rotation, target_rotation)
                <= ANGLE_TOL_DEG
            )
            if reached:
                return
            apply_target(goal, quaternion)
            step()
            if guard:
                guard()

        raise RuntimeError(f"{name}: 제한시간 안에 목표에 도달하지 못했습니다.")

    try:
        print("전체 좌표 경로를 검사합니다. 검사 종료 전에는 Play를 누르지 마세요.")
        if list(lula.get_joint_names()) != JOINT_NAMES:
            raise RuntimeError("Lula와 USD의 관절 순서를 확인하세요.")
        direction = FORK_ROTATION[:, 2]
        dock_quaternion = rot_matrix_to_quat(FORK_ROTATION)
        ready = DOCK_POSITION - direction * READY_DISTANCE
        inserted = DOCK_POSITION + direction * INSERT_DISTANCE
        lifted = inserted + np.array([0.0, 0.0, LIFT_HEIGHT])
        waypoints = [
            ("준비", ready), ("도킹", DOCK_POSITION),
            ("삽입", inserted), ("인양", lifted),
        ]
        if any(not np.all(np.isfinite(v)) for _, v in waypoints):
            raise RuntimeError("목표 좌표가 유효하지 않습니다.")
        if min(READY_DISTANCE, INSERT_DISTANCE, LIFT_HEIGHT,
               DOCK_SPEED, INSERT_SPEED, LIFT_SPEED, PREFLIGHT_SPACING) <= 0:
            raise RuntimeError("거리·높이·속도·검사 간격은 양수여야 합니다.")

        # 이동 전에 전체 경로의 IK와 연속성을 검사합니다. 관절은 움직이지 않습니다.
        seed = robot.get_joint_positions()[indices].copy()
        seed = solve_target(ready, dock_quaternion, seed)
        initial_q = seed.copy()
        previous = ready
        for name, goal in waypoints[1:]:
            count = max(1, int(np.ceil(np.linalg.norm(goal - previous) / PREFLIGHT_SPACING)))
            for alpha in np.linspace(0.0, 1.0, count + 1)[1:]:
                target = previous + alpha * (goal - previous)
                try:
                    q = solve_target(target, dock_quaternion, seed)
                except RuntimeError as error:
                    raise RuntimeError(
                        f"사전 검사 [{name}]: {error}. "
                        "아직 작업을 시작하지 않았습니다. 로봇 베이스 높이 또는 작업 높이를 조정하세요."
                    ) from error
                if np.max(np.abs(np.rad2deg(q - seed))) > MAX_IK_GAP_DEG:
                    raise RuntimeError(f"사전 검사 [{name}]: 관절 해가 불연속입니다.")
                seed = q
            previous = goal
            print(f"[사전 IK 통과] {name}: {np.round(goal, 4)}")

        # 준비 좌표로 최초 배치합니다. 도킹/삽입/인양은 순간이동하지 않습니다.
        robot.set_joint_positions(initial_q, joint_indices=indices)
        robot.set_joint_velocities(np.zeros(6), joint_indices=indices)
        robot.apply_action(ArticulationAction(joint_positions=initial_q, joint_indices=indices))
        print("준비 손목 좌표:", np.round(ready, 4))
        print("준비 자세의 IK 관절값:", np.round(np.rad2deg(initial_q), 2))
        print("사전 검사가 끝났습니다. Play를 누르세요.")
        wait(SETTLE_SECONDS)

        # URDF 계산과 실제 USD 링크 위치가 맞는지 먼저 검사합니다.
        fk_position, fk_rotation = ee_pose()
        usd_position, usd_quaternion = robot.end_effector.get_world_pose()
        if (
            np.linalg.norm(fk_position - usd_position) > 0.01
            or angle_error_deg(
                fk_rotation, quat_to_rot_matrix(usd_quaternion)
            ) > 5.0
        ):
            raise RuntimeError("URDF와 USD의 link_6 자세가 다릅니다.")

        pallet_start = pallet_position()

        def check_not_pushed():
            moved = np.linalg.norm(pallet_position() - pallet_start)
            if moved > PALLET_MOVE_TOL:
                raise RuntimeError(
                    f"팔레트가 {moved * 1000:.1f} mm 움직였습니다. "
                    "구멍 정렬과 충돌 형상을 확인하세요."
                )

        if np.linalg.norm(fk_position - ready) > POSITION_TOL:
            raise RuntimeError("준비 위치를 유지하지 못했습니다. 접촉·Drive를 확인하세요.")
        if angle_error_deg(fk_rotation, FORK_ROTATION) > ANGLE_TOL_DEG:
            raise RuntimeError("준비 자세의 포크가 수평을 유지하지 못합니다.")
        move("도킹", DOCK_POSITION, dock_quaternion, DOCK_SPEED, check_not_pushed)
        move("삽입", inserted, dock_quaternion, INSERT_SPEED, check_not_pushed)

        before_lift = pallet_position()
        move("인양", lifted, dock_quaternion, LIFT_SPEED)
        wait(SETTLE_SECONDS)

        after_lift = pallet_position()
        rise = after_lift[2] - before_lift[2]
        drift = np.linalg.norm(after_lift[:2] - before_lift[:2])
        if rise < MIN_PALLET_RISE or drift > PALLET_MOVE_TOL:
            raise RuntimeError(
                f"인양 확인 실패: 상승 {rise * 1000:.1f} mm, "
                f"수평 이동 {drift * 1000:.1f} mm"
            )

        print(f"[인양 확인] 팔레트가 {rise * 1000:.1f} mm 올라왔습니다.")
        print("그대로 유지합니다. 내려놓기와 운반은 포함하지 않았습니다.")

        while app.is_running():
            step()
            if pallet_position()[2] < after_lift[2] - 0.01:
                raise RuntimeError("유지 중 팔레트가 1 cm 이상 내려갔습니다.")

    except RuntimeError as error:
        print(f"[중단] {error}")
        if app.is_running():
            world.pause()
            print("일시정지했습니다. 화면 확인 후 창을 닫고 값을 조정하세요.")
            while app.is_running():
                world.pause()
                app.update()


try:
    main()
finally:
    app.close()