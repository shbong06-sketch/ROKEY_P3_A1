# 리프트 사용법 정리

조작 방법은 4가지다. 목적에 맞는 것 하나만 쓰면 된다.
아래 값·동작은 전부 실제로 돌려서 확인한 것이다.

| 방법 | 언제 | 파일 |
|---|---|---|
| A. 패널(창) | 손으로 조작 | `11_control_panel.py` |
| B. Script Editor 한 줄 | 잠깐 테스트 | 아래 B |
| C. 스탠드얼론 스크립트 | 자동 시퀀스 | `09_lift_api.py` |
| D. test.py 안에 넣기 | 픽앤플레이스와 섞기 | 아래 D |

공통 이름·경로

```
리프트 조인트 이름 : lift_prismatic_joint          (PhysicsPrismaticJoint)
아티큘레이션 루트  : <리그>/Nova_Carter/chassis_link
가동 범위          : 0.0000 ~ 0.2788 m   (마스트 폴리곤까지 0.2838, 여유 5 mm)
USD 안에 저장된 값 : lift:safeLower / lift:safeUpper / lift:defaultSpeed
                     physxJoint:maxJointVelocity = 0.5 m/s (물리 쪽 상한)
```

---

## A. 패널로 조작 (사람이 직접)

1. 고쳐진 파일을 연다 → **Play**
2. Window > Script Editor 에 `11_control_panel.py` 내용을 붙여넣고 **Ctrl+Enter**

창에서 쓰는 것

```
[Lift] min/max + Apply   가동 범위 (안전 범위를 넘기면 자동으로 잘림)
       speed m/s         승강 속도
       height m          목표 높이 슬라이더
       Down/Up           50 mm 씩
       Bottom/Top/Hold   맨 아래 / 맨 위 / 지금 자리에서 정지
```

---

## B. Script Editor 에서 코드로 (패널을 띄운 상태)

패널을 한 번 실행하면 `rig` 객체가 살아 있다. 그 다음부터는 이렇게 쓴다.

```python
rig.lift_speed(0.05)        # 승강 속도 0.05 m/s
rig.lift_to(0.20)           # 0.20 m 로 이동 (범위 밖이면 잘라냄)
rig.lift_range(0.0, 0.25)   # 가동 범위 자체를 바꿈
rig.lift_stop()             # 지금 높이에서 정지

rig.arm_deg([0, -30, 60, 0, 30, 0])   # 팔 6축 (도)
rig.arm_strengthen()                   # 팔 드라이브 강화 (뻗어도 안 처짐)
rig.drive(0.3, 0.0)                    # 카터 전진 0.3 m/s
rig.stop()                             # 바퀴·리프트 정지
```

패널 없이 USD 만으로 쓰려면 (Play 전에도 먹는다)

```python
import omni.usd
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()
joint = None
for prim in stage.Traverse():
    if prim.GetName() == "lift_prismatic_joint":
        joint = prim
        break

drive = UsdPhysics.DriveAPI.Get(joint, "linear")
drive.GetTargetPositionAttr().Set(0.20)   # m — 이 값만 바꾸면 리프트가 움직인다
drive.GetTargetVelocityAttr().Set(0.0)    # 위치 제어에서는 0
```

> 이렇게 한 번에 목표를 주면 **최대 속도(0.5 m/s)로 튀어 올라간다.**
> 속도를 정하고 싶으면 A/B 의 `lift_to()` 처럼 목표를 조금씩 끌어 줘야 한다.

---

## C. 스탠드얼론 스크립트 (`~/isaacsim/python.sh`)

`09_lift_api.py` 가 그대로 예제다. 핵심만 옮기면:

```python
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction

world = World(stage_units_in_meters=1.0, physics_dt=1/120.0)
robot = world.scene.add(SingleArticulation(
    prim_path="/World/amr_lift_rig/Nova_Carter/chassis_link", name="rig"))
world.reset()                      # 여기서 물리가 붙는다. 그 전에 쓰면 안 된다.

lift_idx = robot.get_dof_index("lift_prismatic_joint")

# 리미트는 dof_properties 로 읽는다 (SingleArticulation 에 get_dof_limits 는 없다)
props = robot.dof_properties
lower = float(props["lower"][lift_idx])
upper = float(props["upper"][lift_idx])

command, goal, speed = 0.0, 0.20, 0.08      # m, m/s
for _ in range(1200):
    delta = goal - command
    if abs(delta) > 1e-6:                    # 속도만큼만 목표를 끌어 준다
        step = min(abs(delta), speed / 120.0)
        command += step if delta > 0 else -step
    robot.apply_action(ArticulationAction(
        joint_positions=np.array([command]),
        joint_indices=np.array([lift_idx]),
    ))
    world.step(render=True)

print("실제 높이", robot.get_joint_positions()[lift_idx])
```

---

## D. `test.py` 안에 리프트 넣기

이미 `robot.get_dof_index(...)` 로 인덱스를 찾고 있으므로 경로만 바꾸면 된다.

```python
ROBOT_PATH = "/World/amr_lift_rig/Nova_Carter/chassis_link"   # 아티큘레이션 루트

lift_idx = robot.get_dof_index("lift_prismatic_joint")
arm_idx  = np.array([robot.get_dof_index(n) for n in JOINT_NAMES])   # joint_1~6

def command_lift(height_m):
    robot.apply_action(ArticulationAction(
        joint_positions=np.array([height_m]),
        joint_indices=np.array([lift_idx]),
    ))
```

시퀀스에 넣을 자리: RETRACT(포크 빼기) 다음, DESCEND(놓을 높이) 앞.
팔 명령과 리프트 명령은 `joint_indices` 가 다르므로 **서로 간섭하지 않는다.**

---

## 단위 — 여기서 제일 많이 틀린다

| 대상 | USD 드라이브 속성 | ArticulationAction |
|---|---|---|
| 리프트(프리즈매틱) | `targetPosition` = **m** | `joint_positions` = **m** |
| 팔(레볼루트) | `targetPosition` = **도(deg)** | `joint_positions` = **라디안(rad)** |
| 바퀴 | `targetVelocity` = rad/s → **런타임에 안 먹음** | `joint_velocities` = **rad/s (이걸 써야 함)** |

바퀴는 실측으로 확인했다: USD 로 2.14 rad/s 를 써도 실제 0.03 rad/s,
`ArticulationAction` 으로 주면 2.13 rad/s. 리프트·팔의 `targetPosition` 은 USD 로도 정상 반영된다.

---

## 확인된 성능

```
정지 상태 정밀도 : 목표 0.2788 → 실제 0.27855  (오차 0.3 mm)
속도 제어        : 0.08 m/s 램프로 0.2788 m 까지 3.8 초 (이론 3.5 초)
물리 상한        : maxJointVelocity 0.5 m/s 가 실제로 걸린다 (한 번에 명령 시 약 0.48 m/s)
주행 중 승강     : 2 m 주행하면서 0.2 m 올려도 캐리지·팔 상대위치 변화 1 mm 이하
팔 동작 중 유지  : 팔 6축을 움직여도 리프트 높이 변화 0.0000
```

## 잘 안 될 때

| 증상 | 원인 | 확인 |
|---|---|---|
| 리프트가 안 움직임 | Play 안 누름 / 조인트가 `jointEnabled=0` | `01_diagnose.py` |
| Play 하자마자 튐 | 조인트 프레임 어긋남(스냅) | `01_diagnose.py` 의 "프레임 어긋남" |
| 명령보다 덜 올라감 | 목표가 리미트 밖 → 잘림 | 리미트 0 ~ 0.2788 확인 |
| 하중 걸면 처짐 | Drive type 이 `force` | `acceleration` 이어야 함 |
| 팔이 리프트와 같이 안 올라감 | 캐리지↔팔 FixedJoint 꺼짐/`excludeFromArticulation=1` | DOF 가 14 인지 확인 (7 이면 분리된 것) |
| 창 글자가 `??` | Isaac 기본 폰트에 한글 없음 | `11_control_panel.py` 의 `UI_FONT` 참고 |
