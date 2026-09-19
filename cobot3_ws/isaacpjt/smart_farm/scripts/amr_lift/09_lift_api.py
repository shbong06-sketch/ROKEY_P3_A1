"""
리프트 제어 API (스탠드얼론 스크립트용) + 자체 검증.

    ~/isaacsim/python.sh 09_lift_api.py

제안받은 절차를 그대로 따르되, 실제 값과 몇 가지 교정을 반영했다.

  확인된 실제 값
    아티큘레이션 루트 : <rig>/Nova_Carter/chassis_link   (예시의 /World/lift 아님)
    리프트 조인트 이름 : lift_prismatic_joint            (예시의 lift_prismatic 아님)
    모듈              : isaacsim.core.prims.SingleArticulation  (5.1.0 에서 정상 동작)

  교정한 것
    1. `world.scene.add(SingleArticulation(...))` 뒤 `world.reset()` 을 하면 그 안에서
       이미 초기화된다. `initialize()` 를 또 부르는 것은 무해하지만 필수는 아니다.
       (GUI Script Editor 에서 쓸 때는 Play 상태여야 한다는 점이 핵심)
    2. 위치 명령만 주면 **계단 입력**이라 드라이브가 최대 속도로 튀어나간다.
       사용자가 속도를 정할 수 있어야 하므로 목표를 초당 speed 만큼 끌어주는
       램프를 넣었다 (`step_toward`). 05_lift_panel.py 와 같은 방식.
    3. 소프트웨어 clamp 는 USD 리미트가 아니라 **리그가 계산해 둔 안전 범위**
       (lift:safeLower / lift:safeUpper)를 우선 쓴다. 이 값이 캐리지가 마스트
       폴리곤에 닿기 직전까지를 뜻한다.
    4. `get_dof_limits()` 는 **SingleArticulation 에 없다** (ArticulationView 의 메서드).
       실제로 호출해 보면 AttributeError 가 난다. 5.1.0 에서는
       `robot.dof_properties["lower"/"upper"]` 를 써야 한다.
"""

import numpy as np

from isaacsim import SimulationApp

app = SimulationApp({"headless": False})

import omni.usd                                    # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics, UsdLux   # noqa: E402

from isaacsim.core.api import World                        # noqa: E402
from isaacsim.core.api.objects import GroundPlane          # noqa: E402
from isaacsim.core.prims import SingleArticulation         # noqa: E402
from isaacsim.core.utils.types import ArticulationAction   # noqa: E402


RIG_USD = (
    "/home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/assets/amr_lift_rig/amr_lift_rig.usd"
)
RIG_PATH = "/World/amr_lift_rig"
LIFT_JOINT = "lift_prismatic_joint"
PHYSICS_DT = 1.0 / 120.0
LIFT_SPEED = 0.08          # m/s


def build_stage():
    """리그를 참조로 얹은 빈 장면. (이미 있는 장면을 쓰려면 open_stage 로 바꾸면 된다)"""
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world_prim = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world_prim.GetPrim())
    UsdLux.DistantLight.Define(stage, "/World/light").CreateIntensityAttr(1500.0)
    UsdGeom.Xform.Define(stage, RIG_PATH).GetPrim().GetReferences().AddReference(RIG_USD)
    for _ in range(15):
        app.update()
    return stage


def find_lift_joint(stage):
    for prim in stage.Traverse():
        if prim.GetName() == LIFT_JOINT and prim.GetTypeName() == "PhysicsPrismaticJoint":
            return prim
    raise RuntimeError("%s 를 못 찾았습니다." % LIFT_JOINT)


class Lift:
    """리프트 한 축만 다루는 얇은 래퍼."""

    def __init__(self, robot, joint_prim, speed=LIFT_SPEED):
        self.robot = robot
        self.joint = joint_prim
        self.index = robot.get_dof_index(LIFT_JOINT)     # 숫자 하드코딩 금지
        self.speed = speed

        # 1) USD 리미트
        pj = UsdPhysics.PrismaticJoint(joint_prim)
        lo_usd = float(pj.GetLowerLimitAttr().Get() or 0.0)
        hi_usd = float(pj.GetUpperLimitAttr().Get() or 0.0)
        # 2) 물리 쪽 리미트.
        #    주의: SingleArticulation 에는 get_dof_limits() 가 **없다**(ArticulationView 쪽 메서드).
        #    5.1.0 에서는 `dof_properties` 구조화 배열의 lower/upper 를 읽는다.
        props = robot.dof_properties
        lo_phys = float(props["lower"][self.index])
        hi_phys = float(props["upper"][self.index])
        # 3) 리그가 계산해 둔 안전 범위 (폴리곤에 닿기 직전)
        safe_lo = self._attr("lift:safeLower", lo_usd)
        safe_hi = self._attr("lift:safeUpper", hi_usd)

        self.lower = max(lo_usd, lo_phys, safe_lo)
        self.upper = min(hi_usd, hi_phys, safe_hi)
        self.command = float(robot.get_joint_positions()[self.index])
        self.goal = self.command

        print("리프트 DOF 인덱스 : %d" % self.index)
        print("  USD 리미트   : %.4f ~ %.4f" % (lo_usd, hi_usd))
        print("  물리 리미트  : %.4f ~ %.4f" % (lo_phys, hi_phys))
        print("  안전 범위    : %.4f ~ %.4f" % (safe_lo, safe_hi))
        print("  적용 범위    : %.4f ~ %.4f  (속도 %.3f m/s)"
              % (self.lower, self.upper, self.speed))

    def _attr(self, name, fallback):
        a = self.joint.GetAttribute(name)
        return float(a.Get()) if a and a.Get() is not None else float(fallback)

    def set_goal(self, target_m):
        """목표 높이 지정. 범위 밖이면 잘라낸다."""
        self.goal = float(np.clip(target_m, self.lower, self.upper))
        return self.goal

    def step_toward(self, dt=PHYSICS_DT):
        """물리 한 스텝 분량만큼 목표를 끌어당겨 명령한다 (= 속도 제어)."""
        delta = self.goal - self.command
        if abs(delta) > 1e-6:
            move = min(abs(delta), self.speed * dt)
            self.command += move if delta > 0 else -move
        self.robot.apply_action(
            ArticulationAction(
                joint_positions=np.array([self.command]),
                joint_indices=np.array([self.index]),
            )
        )

    def actual(self):
        return float(self.robot.get_joint_positions()[self.index])

    def arrived(self, tol=0.005):
        return abs(self.actual() - self.goal) < tol


def main():
    stage = build_stage()
    joint_prim = find_lift_joint(stage)

    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT)
    GroundPlane(prim_path="/World/GroundPlane", size=20.0)
    robot = world.scene.add(
        SingleArticulation(prim_path=RIG_PATH + "/Nova_Carter/chassis_link", name="amr_rig")
    )
    world.reset()          # 여기서 물리 컨텍스트가 살아나고 아티큘레이션이 바인딩된다
    robot.initialize()     # 이미 초기화돼 있지만, 순서를 분명히 하려고 한 번 더 부른다

    print("DOF %d개: %s" % (robot.num_dof, list(robot.dof_names)))
    lift = Lift(robot, joint_prim)
    arm_idx = np.array([robot.get_dof_index("joint_%d" % i) for i in range(1, 7)])

    def run(seconds):
        for _ in range(int(seconds / PHYSICS_DT)):
            lift.step_toward()
            world.step(render=True)

    def report(tag):
        print("%-12s 목표 %.4f / 실제 %.4f / 오차 %.4f  %s"
              % (tag, lift.goal, lift.actual(), abs(lift.goal - lift.actual()),
                 "OK" if lift.arrived() else "MISMATCH"))

    run(1.0)
    report("시작")

    lift.set_goal(lift.upper)
    run((lift.upper - lift.command) / lift.speed + 1.5)
    report("맨 위")

    # 팔은 리프트와 무관하게 따로 움직인다
    robot.apply_action(
        ArticulationAction(
            joint_positions=np.deg2rad(np.array([0.0, -30.0, 60.0, 0.0, 30.0, 0.0])),
            joint_indices=arm_idx,
        )
    )
    run(2.5)
    print("팔 각도(deg): %s"
          % np.round(np.rad2deg(robot.get_joint_positions()[arm_idx]), 1))
    report("팔 동작 중")

    lift.set_goal(lift.lower)
    run((lift.upper - lift.lower) / lift.speed + 1.5)
    report("맨 아래")

    print("\n창을 닫으려면 Ctrl+C. 계속 돌려두고 lift.set_goal(...) 로 조작해도 된다.")
    while app.is_running():
        lift.step_toward()
        world.step(render=True)


main()
app.close()
