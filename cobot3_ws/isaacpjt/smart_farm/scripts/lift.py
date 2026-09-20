"""
리프트(마스트) 축 제어

지게차와 같은 규칙으로 움직입니다.
  1. 한 번에 하나만 움직인다   : 승강 중에는 팔을 멈추고, 팔 작업 중에는 승강하지 않는다
  2. 실었으면 천천히           : 팔레트를 든 상태에서는 속도를 낮춘다
  3. 급가감속 금지             : 목표를 한 번에 주지 않고, 속도 제한을 걸어 조금씩 옮긴다

쓰는 법
    lift = LiftController(robot, stage, arm_base, LIFT_JOINT_PATH, LIFT_JOINT_NAME)
    # 물리가 안정된 뒤에 (Play 직후에 부르면 엉뚱한 값을 잽니다)
    lift.calibrate()
    lift.start_move(base_height=1.05, loaded=False)
    # 이후 물리 스텝마다 한 번씩 호출합니다.
    lift.update(PHYSICS_DT)
    if lift.is_done:
        print("리프트 이동 완료")

이 파일은 경로를 상수로 갖고 있지 않습니다. 월드마다 prim 경로가 다르기 때문에
쓰는 쪽에서 넘겨 줍니다. (경로를 박아 두면 월드가 바뀔 때 조용히 깨집니다)
"""

import numpy as np
from pxr import UsdPhysics

from isaacsim.core.utils.types import ArticulationAction


# ── 동작 설정 (여기를 조정합니다) ────────────────────────
LIFT_SPEED_EMPTY = 0.05        # m/s, 빈 포크일 때
LIFT_SPEED_LOADED = 0.02       # m/s, 팔레트를 실었을 때 (더 느리게)

# 도달 판정 여유. 위치 드라이브는 중력을 이기느라 목표보다 조금 아래에 섭니다.
#   처짐 = 필요한 힘 / 강성.  조인트 위 질량 약 33 kg → 약 320 N
#   강성 1e5 N/m 이면 3.2 mm, 1e6 이면 0.3 mm 처집니다.
# 에셋 강성이 1e6 으로 올라가면 이 값을 0.003 으로 되돌려도 됩니다.
LIFT_REACHED_TOL = 0.008       # m
LIFT_SETTLE_SECONDS = 0.5      # 도달 후 이만큼 멈춰 있어야 '정지'로 봅니다
LIFT_TIMEOUT_SECONDS = 15.0    # 이 시간 안에 도달하지 못하면 중단
LIFT_TRACKING_LIMIT = 0.05     # m,  명령과 실제가 이만큼 벌어지면 중단 (끼임·간섭)

LIFT_LIMIT_MARGIN = 0.005      # m,  기계 한계에서 이만큼 안쪽까지만 씁니다

# 승강하기 전에 포크가 랙에서 이만큼은 나와 있어야 합니다.
FORK_CLEAR_MARGIN = 0.050      # m


class LiftError(RuntimeError):
    """리프트 때문에 작업을 멈춰야 할 때"""


class LiftController:
    """
    리프트 한 축을 제어합니다.

    '리프트 값'과 '팔 베이스 높이'는 다른 값입니다.
      리프트 값   : 조인트 위치 (0 ~ 행정 끝)
      베이스 높이 : 팔이 실제로 서 있는 월드 z  ← 계획이 쓰는 값
    둘의 차이(offset)는 에셋마다 다르므로 재서 씁니다.
    """

    def __init__(self, robot, stage, arm_base, joint_path, joint_name):
        self._robot = robot
        self._arm_base = arm_base                 # 팔 베이스(base_link) prim 래퍼
        self._index = robot.get_dof_index(joint_name)
        self._lower, self._upper = read_travel_limits(stage, joint_path)

        self._offset = None                       # 베이스 높이 = 리프트 값 + offset
        self._command = None                      # 지금 내보내고 있는 목표
        self._goal = None                         # 최종 목표
        self._speed = LIFT_SPEED_EMPTY
        self._elapsed = 0.0
        self._settled_seconds = 0.0
        self._done = True

    # ── 읽기 ────────────────────────────────────────
    def joint_position(self):
        """리프트 조인트의 현재 값 (m)"""
        return float(self._robot.get_joint_positions()[self._index])

    def base_height(self):
        """팔 베이스의 실제 월드 높이 (m). 이 값이 계획의 기준입니다."""
        return float(self._arm_base.get_world_pose()[0][2])

    @property
    def is_done(self):
        return self._done

    def calibrate(self):
        """
        '리프트 값 ↔ 베이스 높이' 관계를 잽니다.

        반드시 물리가 안정된 뒤에 부르세요. Play 직후에 부르면 리그가 아직
        내려앉는 중이라 10 cm 넘게 틀리게 잽니다.
        """
        self._offset = self.base_height() - self.joint_position()
        low, high = self.reachable_height_range()
        print(
            f"[리프트] 기준 잡기: 조인트 {self.joint_position():.3f} m = "
            f"베이스 높이 {self.base_height():.3f} m (차이 {self._offset:+.3f})"
        )
        print(f"[리프트] 만들 수 있는 베이스 높이 {low:.3f} ~ {high:.3f} m")
        return self._offset

    def height_to_joint(self, base_height):
        """원하는 베이스 높이를 리프트 조인트 값으로 바꿉니다."""
        if self._offset is None:
            raise LiftError("calibrate() 를 먼저 부르세요.")
        return base_height - self._offset

    def reachable_height_range(self):
        """이 리프트로 만들 수 있는 베이스 높이 범위 (m)"""
        if self._offset is None:
            raise LiftError("calibrate() 를 먼저 부르세요.")
        return (self._lower + LIFT_LIMIT_MARGIN + self._offset,
                self._upper - LIFT_LIMIT_MARGIN + self._offset)

    def clamp_height(self, base_height):
        """요청 높이를 행정 안으로 잘라서 돌려줍니다. 잘랐으면 알려 줍니다."""
        low, high = self.reachable_height_range()
        clamped = float(np.clip(base_height, low, high))
        if abs(clamped - base_height) > 1e-6:
            print(
                f"[리프트] 요청 {base_height:.3f} m 는 행정 밖 → {clamped:.3f} m 로 맞춤 "
                f"(가능 {low:.3f} ~ {high:.3f})"
            )
        return clamped

    # ── 명령 ────────────────────────────────────────
    def start_move(self, base_height, loaded=False):
        """
        베이스를 이 높이로 옮깁니다. 실제 이동은 update() 가 합니다.

        loaded : 팔레트를 실은 상태인가 (속도를 낮춥니다)
        """
        # 리프트가 움직였으면 관계가 조금 달라집니다. 매번 다시 잽니다.
        self._offset = self.base_height() - self.joint_position()

        goal = self.height_to_joint(base_height)
        low, high = self._lower + LIFT_LIMIT_MARGIN, self._upper - LIFT_LIMIT_MARGIN
        if not low <= goal <= high:
            reach_low, reach_high = self.reachable_height_range()
            raise LiftError(
                f"리프트 행정 밖입니다: 요청 높이 {base_height:.3f} m "
                f"(가능 {reach_low:.3f} ~ {reach_high:.3f} m). "
                "리프트를 더 긴 것으로 바꾸거나 작업 층을 조정하세요."
            )

        self._goal = goal
        self._command = self.joint_position()
        self._speed = LIFT_SPEED_LOADED if loaded else LIFT_SPEED_EMPTY
        self._elapsed = 0.0
        self._settled_seconds = 0.0
        self._done = False
        print(
            f"[리프트] {self.base_height():.3f} → {base_height:.3f} m "
            f"({'적재' if loaded else '공차'}, {self._speed} m/s)"
        )

    def update(self, dt):
        """
        물리 스텝마다 부릅니다. 목표까지 조금씩 옮기고, 도달·정지를 확인합니다.

        도착해서 멈추면 self.is_done 이 True 가 됩니다.
        """
        if self._done:
            return

        self._elapsed += dt

        # 1) 명령을 속도 제한만큼만 옮긴다 (급가속 방지)
        step = self._speed * dt
        difference = self._goal - self._command
        self._command += float(np.clip(difference, -step, step))
        self._apply(self._command)

        # 2) 명령과 실제가 크게 벌어지면 무언가에 걸린 것
        error = abs(self._command - self.joint_position())
        if error > LIFT_TRACKING_LIMIT:
            raise LiftError(
                f"리프트가 명령을 따라오지 못합니다 (차이 {error * 1000:.0f} mm). "
                "끼임이나 간섭을 확인하세요."
            )

        # 3) 목표에 닿고, 그 상태로 잠시 멈춰 있어야 '정지'
        gap = abs(self._goal - self.joint_position())
        if gap <= LIFT_REACHED_TOL:
            self._settled_seconds += dt
            if self._settled_seconds >= LIFT_SETTLE_SECONDS:
                self._done = True
                # 목표와 실제의 절대 차이를 출력합니다. 위·아래 방향은 구분하지 않습니다.
                print(
                    f"[리프트] 도착. 베이스 높이 {self.base_height():.3f} m "
                    f"(목표와의 차이 {gap * 1000:.1f} mm)"
                )
                return
        else:
            self._settled_seconds = 0.0

        if self._elapsed > LIFT_TIMEOUT_SECONDS:
            goal = self._goal
            current = self.joint_position()
            self.stop()
            raise LiftError(
                f"리프트 시간 초과: 목표 {goal:.3f} m, 현재 {current:.3f} m "
                f"(차이 {gap * 1000:.0f} mm). 드라이브 강성이 모자라면 여기서 멈춥니다."
            )

    def stop(self):
        """현재 조인트 위치를 새 명령으로 고정하고 이동을 종료합니다."""
        current = self.joint_position()
        self._command = current
        self._goal = current
        self._elapsed = 0.0
        self._settled_seconds = 0.0
        self._done = True
        self._apply(current)

    def hold(self):
        """지금 위치를 유지합니다. 팔이 움직이는 동안 매 스텝 부릅니다."""
        self._apply(self.joint_position() if self._command is None else self._command)

    def _apply(self, joint_target):
        self._robot.apply_action(
            ArticulationAction(
                joint_positions=np.array([joint_target]),
                joint_indices=np.array([self._index]),
            )
        )


# ── USD 에서 읽는 값 ─────────────────────────────────────
def read_travel_limits(stage, joint_path):
    """리프트 행정(최소·최대)을 USD 에서 읽습니다. 에셋이 바뀌어도 따라갑니다."""
    prim = stage.GetPrimAtPath(joint_path)
    if not prim.IsValid():
        raise LiftError(f"리프트 조인트를 찾지 못했습니다: {joint_path}")
    joint = UsdPhysics.PrismaticJoint(prim)
    lower = float(joint.GetLowerLimitAttr().Get())
    upper = float(joint.GetUpperLimitAttr().Get())
    axis = joint.GetAxisAttr().Get()
    if axis != "Z":
        raise LiftError(f"리프트 축이 Z 가 아닙니다: {axis}")
    print(f"[리프트] 행정 {lower:.3f} ~ {upper:.3f} m (길이 {upper - lower:.3f})")
    return lower, upper


# ── 움직이기 전에 확인할 것 ──────────────────────────────
def check_fork_clear_of_rack(tine_tip_x, rack_front_x):
    """
    리프트를 올려도 되는 상태인지 봅니다.

    포크가 랙 안에 있으면 승강 중 선반을 들이받습니다.
    손목이 아니라 '갈래 끝'으로 판정해야 합니다. 갈래가 손목보다 0.294 m 앞에
    있어서, 손목 기준으로 보면 랙 안에 있는데도 통과해 버립니다.
    """
    if tine_tip_x < rack_front_x + FORK_CLEAR_MARGIN:
        raise LiftError(
            f"포크가 아직 랙 안에 있습니다 (갈래 끝 x={tine_tip_x:.3f}, "
            f"랙 앞 {rack_front_x:.3f}, 필요 여유 {FORK_CLEAR_MARGIN}). "
            "먼저 빼낸 뒤 승강하세요."
        )


def check_base_level(base_quaternion, limit_deg=2.0):
    """
    팔 베이스가 기울지 않았는지 봅니다.

    카터가 기울면 포크도 같이 기울어 팔레트가 미끄러집니다.
    limit_deg 2.0 은 시작값입니다. 실제 흔들림을 재 보고 조정하세요.
    """
    w, x, y, z = (float(v) for v in base_quaternion)
    # z축이 얼마나 누웠는지 (roll·pitch 합)
    tilt = np.degrees(np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0)))
    if tilt > limit_deg:
        raise LiftError(f"베이스가 {tilt:.1f}° 기울었습니다 (허용 {limit_deg}°).")
    return tilt
