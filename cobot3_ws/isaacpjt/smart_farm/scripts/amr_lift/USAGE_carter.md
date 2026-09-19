# 카터(Nova Carter) 주행 제어 정리

파일: `12_carter_control.py` — 주행만 들어 있다. 리프트·팔과 독립이다
(같은 아티큘레이션이지만 바퀴 DOF 인덱스만 건드리므로 서로 간섭하지 않는다).

## 카터 제원 (에셋에서 실측)

```
구동륜 반지름  0.14 m      (wheel_left 원점 z = 0.14)
좌우 간격      0.4132 m    (wheel_left y = +0.2066 / wheel_right y = -0.2066)
바퀴 Drive     stiffness 0, damping 1e6   = 속도 제어 전용
바퀴 DOF 이름  joint_wheel_left, joint_wheel_right   (인덱스 1, 2)
캐스터 4축     자유 회전. 명령하지 않는다
```

## 쓰는 법

### A. GUI (Script Editor)

파일 열고 **Play** → `12_carter_control.py` 붙여넣고 Ctrl+Enter → 작은 창이 뜬다.

```python
carter.drive(0.3, 0.0)     # 전진 0.3 m/s
carter.drive(0.3, 0.4)     # 전진하면서 좌회전
carter.turn(0.6)           # 제자리 좌회전 0.6 rad/s (음수면 우회전)
carter.back(0.3)
carter.stop()

carter.pose()              # (x, y, yaw[rad])
carter.measured()          # 바퀴에서 되돌린 (전진 m/s, 회전 rad/s)
carter.wheel_velocities()  # 실제 바퀴 각속도 [left, right] rad/s
```

### B. 정확한 거리·각도 (폐루프, 권장)

명령만 주고 시간으로 재면 **회전이 30% 부족하다**(아래 실측 참고).
목표를 주고 스스로 멈추게 하면 슬립과 무관하게 맞는다.

```python
carter.turn_by(90, w=0.6)     # 왼쪽으로 90도 돌고 스스로 정지
carter.move_by(1.0, v=0.4)    # 1 m 가고 스스로 정지
carter.goal_done()            # 도착했는지 확인 (True 면 끝)
```

### C. 스탠드얼론 스크립트

```python
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation

world = World(stage_units_in_meters=1.0, physics_dt=1/120.0)
robot = world.scene.add(SingleArticulation(
    prim_path="/Nova_Carter/chassis_link", name="carter"))
world.reset()

carter = CarterDriver(articulation=robot, stage=stage)   # 12_carter_control.py 의 클래스
carter.move_by(1.0, v=0.4)
while not carter.goal_done():
    carter._apply()          # 속도 명령 유지 + 목표 도달 검사
    world.step(render=True)
```

GUI 모드(인자 없이 생성)에서는 물리 스텝 콜백이 `_apply()` 를 자동으로 부른다.
스탠드얼론에서는 위처럼 매 스텝 직접 불러야 한다.

## 반드시 알아야 할 것

**USD 의 `targetVelocity` 는 Play 중에 안 먹는다.**

```python
# 이렇게 하면 안 돈다 (실측: 명령 2.14 rad/s → 실제 0.03 rad/s)
UsdPhysics.DriveAPI.Get(wheel_joint, "angular").GetTargetVelocityAttr().Set(2.14)

# 이렇게 해야 돈다 (실측: 2.13 rad/s)
robot.apply_action(ArticulationAction(
    joint_velocities=np.array([2.14, 2.14]),
    joint_indices=np.array([1, 2])))
```

리프트·팔의 `targetPosition` 은 USD 로도 반영되는 것과 다르다. 바퀴만 예외다.

## 실측 성능

```
직진   명령 0.30 m/s x 3 s = 0.90 m  →  실제 0.882 m (98%)
후진   명령 0.30 m/s x 2 s = 0.60 m  →  실제 0.585 m (98%)
곡선   명령 (v 0.3, w 0.4)           →  바퀴에서 잰 (0.294, 0.553)

제자리 회전 (열린 루프)
       명령 0.6 rad/s x 3 s = 103.1도 →  실제 72.6도 (70%), 중심 0.083 m 밀림
       바퀴는 명령대로 도는데 차체가 덜 돈다 = 캐스터 저항 + 타이어 슬립

폐루프
       turn_by(90)   →  실제 89.3도 (오차 0.7도)
       move_by(1.0)  →  실제 0.996 m (오차 4 mm)
```

→ **각도가 중요하면 `turn_by()` 를 쓸 것.** 열린 루프로 각도를 맞추려면
명령을 `w / 0.70` 으로 키워야 하는데, 바닥 마찰·적재 하중이 바뀌면 이 비율도 바뀐다.

## 주행 중 리프트·팔

같이 명령해도 서로 간섭하지 않는다 (실측).

```
2 m 주행 + 리프트 0.2 m 상승 동시 → 섀시 기준 캐리지 1.2 mm, link_6 0.5 mm 변화
팔 6축 동작 중에도 리프트 높이 변화 0.0000
```

단, 팔을 세운 상태의 무게중심이 약 1.9 m 로 높다. 급가속·급회전은 아직 확인하지 않았다.
`MAX_LINEAR = 1.5`, `MAX_ANGULAR = 2.0` 상한이 걸려 있으니 필요하면 더 낮춰서 쓴다.

## 잘 안 될 때

| 증상 | 원인 | 확인 |
|---|---|---|
| 안 움직임 | Play 안 누름 | `carter.pose()` 가 None 이면 Play 전 |
| 안 움직임 | USD targetVelocity 로 명령함 | 위 "반드시 알아야 할 것" |
| Play 하자마자 제멋대로 감 | 바퀴 Drive 에 targetVelocity 가 남아 있음 | `10_fix_lift_test.py` 가 0 으로 정리 |
| Stop 후 다시 Play 하면 무반응 | 아티큘레이션 핸들이 무효 | 모듈이 타임라인 이벤트로 자동 재바인딩 |
| 회전 각도가 부족 | 캐스터 저항·슬립 (정상) | `turn_by()` 사용 |
