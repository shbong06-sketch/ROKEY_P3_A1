"""Lula IK <-> TCP 오프셋 브릿지 (robot_sample_v3용, control/mm_control.py와 함께 사용).

배경 (이미지 2, 3 내용 그대로):
  - Lula가 실제로 풀어주는 목표점은 M0617 link_6(플랜지) 원점이다. 손가락 끝(TCP)이 아니다.
  - IK는 URDF만 본다 -> 그리퍼(l_out_joint 등)는 IK가 전혀 모른다. 팔은 IK로, 그리퍼는
    mm_control.MobileManipulatorControl.set_gripper_deg()로 따로 제어한다 (v3 mm_control.py 그대로 사용).

이 파일이 하는 일:
  1) "손가락 끝을 어디에 놓고 싶다(TARGET_TCP) + 어떤 자세로 접근하고 싶다(roll/pitch)"를 받아서
  2) TCP 오프셋(FINGER_PAD_TIP_Z)만큼 역보정한 link_6 목표 pose를 만들고
  3) LulaKinematicsSolver로 그 link_6 pose에 대한 IK를 풀어서
  4) 결과 관절각을 mm_control.py의 set_arm_joints_deg()에 넘긴다
     (v3에서 이미 고쳐둔 실제 두산 Default 관절 리미트 클램프/속도 램프를 그대로 재사용).

*** 아직 채워야 하는 것 ***
  ROBOT_URDF_PATH는 이제 패키지 안의 lula/dsr_description2/urdf/m0617.urdf를 자동으로 가리킨다(v3.1).
  ROBOT_DESCRIPTION_YAML만 아직 없다 -- Isaac Sim에서 저 URDF를 Lula Robot Description(XRDF) Editor에
  넣어 생성한 뒤 lula/config/m0617_robot_description.yaml로 저장할 것. "반지름 0" 문제가 났던 게 이
  파일이니, lula/README.md의 진단(“base” 링크가 유력 원인)부터 확인. 이 파일이 없으면
  LulaKinematicsSolver 생성 자체가 실패한다.

*** "조인트 파라미터"는 서로 다른 3개 레이어다 -- 헷갈리면 안 됨 ***
  A) 현재 관절 각도값 (q1~q6) -- USD Articulation을 직접 움직임.
     mm_control.set_arm_joints_deg() / apply_action()이 이 레이어.
  B) IK가 풀 때 지키는 조인트 제한 -- Lula 쪽(URDF/robot_description.yaml)에 들어있는 값.
     이 값은 실제 dsr_description2 URDF 기준 J1/J2/J4/J5/J6 = ±360(무제한), J3 = ±165(하드웨어 최대)다.
     mm_control이 쓰는 관절 리미트(J2=±95, J3=±145, J5=±135 -- v3에서 적용한 두산 소프트웨어 Default값)
     보다 훨씬 넓다. 즉 Lula는 이 좁은 범위를 전혀 모른 채 IK를 풀 수 있다 -> Lula가 "성공"으로 반환한
     각도를 mm_control에 그냥 넘기면, mm_control이 자기 리미트로 몰래 clamp해버려서 실제 도달 자세가
     Lula가 계산한 자세와 달라질 수 있다. 아래 M0617IKBridge가 이걸 자동으로 걸러낸다
     (mm_ctl을 넘기면 clamp 전에 범위를 확인하고, 벗어나면 조용히 자르는 대신 실패로 처리).
  C) 실제 물리 드라이브 특성(stiffness/damping/friction) -- USD PhysxJoint Drive 설정.
     assets/m0617_rebuilt.usd의 drive:angular:physics:stiffness 등이 이 레이어. Lula/mm_control 어느
     쪽도 이 값을 안 건드린다. 여기는 지금 손댈 필요 없음(관절이 실제로 못 움직이거나 떨릴 때만 확인).
"""
import os

import numpy as np

from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats, quats_to_rot_matrices
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver

# mm_control.py는 SingleArticulation으로 Isaac Sim 5.1에서 실제 검증됨(v2 README 참고)이라 그대로 쓰지만,
# 5.0.0 공식 튜토리얼 코드는 같은 자리에 SingleArticulation이 아니라 Articulation을 씀. 둘 중 하나가
# 이 환경에 없을 가능성에 대비해 폴백을 둔다 (둘 다 같은 isaacsim.core.prims 모듈, 단일 로봇 용도로는
# 동일하게 동작).
try:
    from isaacsim.core.prims import SingleArticulation as _ArticulationCls
except ImportError:
    from isaacsim.core.prims import Articulation as _ArticulationCls

# ----------------------------------------------------------------------
# *** 채워야 하는 경로 ***
# URDF는 v3.1부터 패키지 안에 들어있음 (lula/dsr_description2/urdf/m0617.urdf) -- 기본값으로 자동 계산.
# robot_description.yaml은 아직 없음: Isaac Sim에서 위 URDF를 Lula XRDF Editor로 열어 생성한 뒤
# lula/config/ 밑에 저장하고 아래 경로를 채울 것 ("반지름 0" 문제, lula/README.md 참고).
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # control/ 의 한 단계 위
ROBOT_URDF_PATH = os.path.join(_PKG_ROOT, "lula", "dsr_description2", "urdf", "m0617.urdf")
ROBOT_DESCRIPTION_YAML = os.path.join(_PKG_ROOT, "lula", "config", "m0617_robot_description.yaml")
# ----------------------------------------------------------------------

# M0617 flange(link_6) 기준 좌표계 이름 -- "tool0"이 아니라 실제 usd 프림 이름(link_6)을 써야
# Lula가 인식한다. v3 mobile_manipulator.usda / m0617_fixed_base.usda 둘 다 이 이름을 그대로 씀.
END_EFFECTOR_FRAME = "link_6"

# link_6(=rg6/base_link, m0617_to_rg6 연결 조인트가 identity라 두 좌표계가 같음) 에서
# RG6 hand_tcp까지의 로컬 +Z 오프셋 [m].
# 실측: g_main_joint localPos0.z(0.04953) + hand_tcp_joint_joint localPos0.z(0.21) = 0.25953
# (v3 패키지 robots/rg6_gripper.usd에서 직접 읽은 값. 강의 자료의 0.19671은 예시 로봇 기준이라 다름 -- 본인 로봇은 아래 값 사용)
FINGER_PAD_TIP_Z = 0.25953

# 접근 자세 예시값 (사용자가 준 값 그대로 기본값으로 둠 -- 태스크에 맞게 바꿔 쓰면 됨)
APPROACH_ROLL_DEG = 180.0
APPROACH_PITCH_DEG = 90.0
APPROACH_YAW_DEG = 0.0

# 목표 TCP(손가락 끝) 월드 좌표 예시
TARGET_TCP = np.array([0.55, 0.10, 0.30])


def tcp_target_to_flange_target(target_tcp_pos, roll_deg, pitch_deg, yaw_deg=0.0,
                                 tip_z=FINGER_PAD_TIP_Z):
    """TCP(손가락 끝) 목표 pose -> link_6(플랜지) 목표 pose.

    TCP는 flange 로컬 +Z로 tip_z만큼 떨어져 있으므로,
    flange_position = tcp_position - R(orientation) @ [0, 0, tip_z]
    (이미지 2가 설명하는 그 보정을 그대로 구현한 것)
    """
    rpy_rad = np.radians([roll_deg, pitch_deg, yaw_deg])
    orientation_quat = euler_angles_to_quats(rpy_rad)          # (w, x, y, z)
    rot = quats_to_rot_matrices(orientation_quat)               # 3x3
    offset_world = rot @ np.array([0.0, 0.0, tip_z])
    flange_position = np.asarray(target_tcp_pos, dtype=float) - offset_world
    return flange_position, orientation_quat


class M0617IKBridge:
    """LulaKinematicsSolver를 M0617 articulation(mm_control의 self.arm)에 연결.

    mm_control.MobileManipulatorControl과 나란히 쓴다 -- 팔 목표각을 이 클래스가 계산해서
    ctl.set_arm_joints_deg()에 넘겨주면, v3에서 고친 관절 리미트/속도 램프가 그대로 적용된다.

    mm_ctl을 넘기면(권장) Lula가 계산한 각도가 mm_control의 실제 안전 리미트(±95/±145/±135 등)
    밖으로 나갈 때 조용히 clamp되게 두지 않고 명시적으로 실패 처리한다 -- 위 "레이어 B" 문제.
    """

    def __init__(self, arm_prim_path: str, mm_ctl=None):
        self._kinematics_solver = LulaKinematicsSolver(
            robot_description_path=ROBOT_DESCRIPTION_YAML,
            urdf_path=ROBOT_URDF_PATH,
        )
        print("[ik_bridge] valid frame names:", self._kinematics_solver.get_all_frame_names())
        if END_EFFECTOR_FRAME not in self._kinematics_solver.get_all_frame_names():
            raise RuntimeError(
                f"'{END_EFFECTOR_FRAME}' 프레임이 robot_description.yaml에 없음. "
                f"위에 출력된 valid frame 목록에서 M0617 flange에 해당하는 이름을 찾아 "
                f"END_EFFECTOR_FRAME을 그 이름으로 바꿀 것."
            )
        self._arm = _ArticulationCls(arm_prim_path, name="ik_bridge_arm")
        self._arm.initialize()
        self._art_kin_solver = ArticulationKinematicsSolver(
            self._arm, self._kinematics_solver, END_EFFECTOR_FRAME
        )
        self._mm_ctl = mm_ctl  # mm_control.MobileManipulatorControl 인스턴스 (선택)
        self._refresh_base_pose()

    def _refresh_base_pose(self):
        """M0617이 AMR(MiR100) 위에 고정된 A안에서는 AMR이 움직이면 팔의 world pose도
        바뀐다. IK를 풀기 전에 매번 현재 base pose를 Lula에 알려줘야 한다 -- __init__에서
        한 번만 하면 AMR이 조금이라도 이동한 뒤 모든 IK 결과가 틀어진다 (B안 고정 베이스에서는
        base pose가 안 바뀌니 매번 호출해도 비용만 약간 더 들 뿐 결과에 영향 없음).
        """
        arm_base_pos, arm_base_quat = self._arm.get_world_pose()
        self._kinematics_solver.set_robot_base_pose(arm_base_pos, arm_base_quat)

    def solve_flange_ik_deg(self, flange_pos, flange_quat):
        """flange(link_6) 목표 pose에 대한 IK를 풀어 joint_1..joint_6 각도[deg]를 반환.
        실패(IK 미수렴 또는 mm_ctl 리미트 위반)하면 None을 반환한다.
        """
        self._refresh_base_pose()  # AMR이 움직였을 수 있으니 IK 풀기 직전에 항상 갱신
        action, success = self._art_kin_solver.compute_inverse_kinematics(flange_pos, flange_quat)
        if not success:
            print("[ik_bridge] WARNING: IK did not converge for", flange_pos)
            return None
        # action.joint_positions는 self._arm.dof_names 순서(PhysX DOF 순서) -- mm_control이
        # 기대하는 joint_1..joint_6 순서로 다시 뽑아준다.
        names = self._arm.dof_names
        order = [f"joint_{i}_joint" for i in range(1, 7)]
        idx = [names.index(n) for n in order]
        q_rad = np.asarray(action.joint_positions)[idx]
        q_deg = np.degrees(q_rad)

        # "레이어 B" 검사: Lula는 URDF 기준 넓은 리미트(J2/J4/J5/J6 사실상 무제한, J3 ±165)로 풀기
        # 때문에, mm_ctl이 있으면 그 실제 안전 리미트(±95/±145/±135 등)를 벗어나는지 먼저 확인한다.
        # mm_control.set_arm_joints_deg()가 알아서 clamp해주니 생략해도 "돌아는" 가지만, 그러면
        # Lula가 계산한 자세와 실제 도달 자세가 조용히 달라진다 -- 그래서 여기서 명시적으로 막는다.
        if self._mm_ctl is not None:
            lo, hi = self._mm_ctl._arm_lo, self._mm_ctl._arm_hi
            over = (q_deg < lo) | (q_deg > hi)
            if over.any():
                bad = [(f"J{i+1}", round(float(q_deg[i]), 1), round(float(lo[i]), 1), round(float(hi[i]), 1))
                       for i in range(6) if over[i]]
                print(f"[ik_bridge] WARNING: IK solution violates mm_control safety limits (J, solved, lo, hi): {bad}. "
                      f"clamp하지 않고 실패 처리함 -- 목표 pose를 로봇이 안전 범위 안에서는 도달할 수 없다는 뜻.")
                return None
        return q_deg

    def solve_tcp_ik_deg(self, target_tcp_pos, roll_deg, pitch_deg, yaw_deg=0.0,
                          tip_z=FINGER_PAD_TIP_Z):
        """TCP(손가락 끝) 목표로 바로 호출하는 헬퍼. 내부에서 flange 목표로 역변환 후 IK."""
        flange_pos, flange_quat = tcp_target_to_flange_target(
            target_tcp_pos, roll_deg, pitch_deg, yaw_deg, tip_z
        )
        return self.solve_flange_ik_deg(flange_pos, flange_quat)


# ------------------------------------------------------------------ 사용 예시
# (run_control.py / script_editor_example.py 안에서 world.reset() 이후,
#  ctl.initialize()까지 끝낸 다음 -- mm_ctl._arm_lo/_arm_hi가 initialize()에서 채워지므로 순서 중요)
#
# from ik_bridge import M0617IKBridge, TARGET_TCP, APPROACH_ROLL_DEG, APPROACH_PITCH_DEG
#
# ctl.initialize()   # mm_control.py 쪽, 항상 먼저
# ik = M0617IKBridge(arm_prim_path=root + "/m0617", mm_ctl=ctl)   # root는 run_control.py의 그 root
# q_deg = ik.solve_tcp_ik_deg(TARGET_TCP, APPROACH_ROLL_DEG, APPROACH_PITCH_DEG)
# if q_deg is not None:
#     ctl.set_arm_joints_deg(q_deg)   # 여기서부터는 mm_control.py의 기존 램프가 그대로 처리
#     # (리미트 자체는 위에서 이미 통과했으니 이 clamp는 사실상 no-op)
# else:
#     print("IK 실패 -- TARGET_TCP가 도달 불가능하거나, 안전 리미트 밖 자세가 필요함")
