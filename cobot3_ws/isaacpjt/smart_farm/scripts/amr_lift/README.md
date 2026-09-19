# AMR + 리프트 + M0609 리그 (Isaac Sim 5.1)

목표 두 가지

1. 리프트가 **물리 엔진으로** 위아래로 움직이고, 노바 카터 위에 고정된 채로 있으며,
   M0609 를 태우고, 사용자가 높이를 조작할 수 있고, 가동 범위가 콜리전을 벗어나지 않는다.
2. 움직이는 카터 위에서 리프트가 흔들리지 않고, **다른 PC 에서 파일 하나를 불러오면**
   (리프트 + M0609 + 카터) 묶음이 그대로 동작한다.

---

## 1. 이전 스크립트(assemble_amr_lift.py)가 안 됐던 이유

`demo_test _0918.usd` 를 직접 읽어서 확인한 것들이다.

| # | 문제 | 실제 값 |
|---|------|---------|
| 1 | **경로가 틀렸다** | 리프트는 `/World/...` 가 아니라 `/Group/lift_v3_physics_1___1__02` 이고, 카터는 `/Group/Nova_Carter` 다. 스크립트 상수는 `/World/lift_v3_physics_1___1_02` (밑줄 개수도 다름) → 첫 줄에서 RuntimeError |
| 2 | **Drive 목표가 가동 범위 밖** | 리미트는 `0 ~ 0.309 m` 인데 `targetPosition = -53.4`, `targetVelocity = 13.1`. 리미트와 Drive 가 계속 싸운다 (떨거나 터짐) |
| 3 | **캐리지 위치가 조인트 프레임과 어긋남** | 장면에서 `lift_moveparts_1` 에 `translate z = -0.2973`, 그 밑 Mesh 에 `+0.5653` 이 덧칠돼 있었다. 조인트 `localPos0=localPos1=(0,0,0)` 이므로 Play 순간 0.297 m 스냅 |
| 4 | **마스트가 kinematic** | `lift_holder` 가 `kinematicEnabled=True`. 물리로 안 움직이고 아티큘레이션에도 못 들어간다 |
| 5 | **팔이 월드에 못박혀 있음** | `m0609_with_fork/root_joint` 는 body0 가 빈 FixedJoint(= 월드 고정) + ArticulationRootAPI |
| 6 | **리프트 에셋 안의 낡은 조인트** | `lift_v3_physics.usdc` 안에 `M0609_FixedJoint` 가 남아 있고 body1 이 존재하지 않는 `/World/m0609/base_link` 를 가리킨다 → "disjointed body transforms" 경고 |
| 7 | **이식성** | 카터는 `https://...amazonaws.com/...` 참조, 리프트는 `../lift_v3_physics(1) (1).usdc` 라는 공백·괄호 섞인 **절대 경로** 참조. 다른 PC 에서 그대로 깨진다 |

그리고 설계 자체의 문제가 하나 더 있다.
`lift_holder` 를 `chassis_link` **밑으로 reparent** 하는 방식은,
RigidBody 를 지우고 콜리전만 남기는 처리를 하더라도 아티큘레이션 구성이 애매해진다.
계층을 옮기지 않고 **조인트로만 묶는 쪽**이 훨씬 안전하다.

---

## 2. 채택한 구조 — 아티큘레이션 하나로 합친다

```
/amr_lift_rig                     defaultPrim. 이것만 옮기면 전체가 따라간다
  Nova_Carter                     ref -> nova_carter.usd
    chassis_link                  ArticulationRootAPI  = 리그 전체의 뿌리
  lift                            ref -> lift_v3_physics.usdc
    lift_holder                   마스트(고정부)
    lift_moveparts_1              캐리지(승강부)
      M0609_Mount                 팔이 붙는 자리 (local z = 0.827889)
  m0609_with_fork                 ref -> m0609_with_fork.usd
    root_joint                    active = false  (월드 고정 해제)
  Joints
    chassis_to_lift_holder        FixedJoint   섀시 <-> 마스트
    mover_to_m0609                FixedJoint   캐리지 <-> 팔 base_link
```

PhysX 는 **조인트로 이어진 바디를 같은 아티큘레이션에 넣는다.**
그래서 결과가 이렇게 된다 (실측 DOF 14개):

```
joint_caster_base, joint_wheel_left, joint_wheel_right,
joint_swing_left, joint_swing_right,
lift_prismatic_joint,                 <- 리프트
joint_caster_left, joint_caster_right,
joint_1 ... joint_6                   <- M0609
```

이 구조를 고른 이유

* 아티큘레이션이 둘로 갈리면 그 사이를 잇는 조인트는 최대좌표 구속이 되어
  **물렁해진다**. 28 kg 짜리 팔이 캐리지 위에서 출렁인다.
  하나로 합치면 리프트가 "붙어 있는 척"이 아니라 진짜 관절이 된다.
* RigidBody 를 RigidBody 밑으로 reparent 하지 않는다 (PhysX 금지).
  계층은 그대로, 조인트로만 묶는다.
* 조인트 local frame 을 **지금 놓인 위치에서 역산**하므로 Play 해도 스냅이 없다.

주의: 카터 에셋은 `enabledSelfCollisions = 0` 이다.
합쳐진 뒤에는 팔·포크도 같은 아티큘레이션이라 **카터/리프트와 서로 충돌하지 않는다.**
안정성에는 유리하지만, 팔이 마스트를 통과해도 막아 주지 않는다는 뜻이기도 하다.

---

## 3. 순서대로 할 일

| 단계 | 하는 일 | 스크립트 | 확인 방법 |
|------|---------|----------|-----------|
| 0 | 현재 장면 진단 (읽기만) | `01_diagnose.py` | 아티큘레이션 루트 개수, 범위 밖 Drive, 프레임 어긋남이 표로 나온다 |
| 1 | 리그 USD 생성 | `02_build_rig.py` | `assets/amr_lift_rig/amr_lift_rig.usd` 가 생긴다. 열려 있는 장면은 건드리지 않는다 |
| 2 | 리그 단독 확인 | 그 파일을 열고 Play | 리프트가 제자리에 서 있고, 팔이 캐리지 위에 선다 |
| 3 | 수동 조작 | `05_lift_panel.py` (범위·속도 조절 포함) | 슬라이더로 0 ~ 0.279 m 이동 |
| 4 | 장면에 얹기 | `04_add_rig_to_scene.py` | demo_test 에 참조로 들어간다. 확인 후 예전 `/Group`, `/World/m0609_with_fork` 삭제 |
| 5 | 포터블 패키징 | File > Collect As | 아래 5번 참고 |
| 6 | `test.py` 연결 | 아래 6번 참고 | 경로 세 줄 수정 |

---

## 4. 가동 범위 — 폴리곤에 닿기 직전까지

바운딩 박스로 재면 **틀린다.** 마스트 맨 위 가로보(z 1.167~1.197)를 놓쳐서
캐리지가 보를 25 mm 뚫고 올라간다.

그래서 `02_build_rig.py` 는 **수직 레이캐스트 스윕**으로 잰다.
캐리지 밑면 격자(12 mm) 칼럼마다 수직선을 쏘아, 그 선 위에서 캐리지 바로 위·아래에
있는 마스트 면까지의 거리를 재고 그 최솟값을 쓴다.

```
캐리지가 점유한 칼럼 1021 개
마스트 면이 캐리지 z 구간을 지나는 칼럼 0 개   (지금 서로 파고들지 않음)
폴리곤에 닿기까지 : 위 0.2840 m / 아래 0.1636 m
여유 5 mm 를 뺀 안전 범위 : -0.1586 ~ +0.2790 m
기본 가동 범위(조인트 리미트) : 0.0 ~ 0.2790 m
```

이 값은 USD 안에 같이 저장된다 (조작 패널이 읽는다).

```
lift:safeLower = -0.15858
lift:safeUpper =  0.27896
lift:defaultSpeed = 0.08
physxJoint:maxJointVelocity = 0.5     # 물리 쪽 속도 상한 (실측 확인됨)
```

### M0609 장착면

팔 `base_link` 의 플랜지 밑면은 자기 좌표계 z = 0.0000 이고 (반경 0.10 m 안 2050 점),
캐리지 상판 윗면 = `M0609_Mount` = 0.88307 (월드).
리그에서 실측한 결과

```
팔 플랜지 최저 z 0.88307 - 상판 0.88307 = 간극 -0.00000 m   (딱 붙음, 파고들지 않음)
상판 사각형 안에서 상판보다 낮은 팔 점 : 0 개
```

단, 팔 밑면에 붙은 **케이블 메시**(반경 0.356 m 까지 뻗음)는 상판보다 45 mm 아래로
내려간다. 상판 **바깥**이라 뚫고 들어가지는 않고 옆으로 늘어뜨려진 상태다.
거슬리면 `m0609_with_fork` 의 base_link 비주얼에서 그 부분만 숨기면 된다.

## 5. 다른 PC 에서 쓰기

리그 파일이 참조하는 것은 세 개다.

| 대상 | 경로 형태 | 다른 PC 에서 |
|------|-----------|--------------|
| Nova Carter | `https://...` (NVIDIA 공식 에셋) | 인터넷만 되면 자동으로 받는다 |
| 리프트 | 상대 경로 `../../scenes/demo_test/lift_v3_physics.usdc` | smart_farm 폴더째 옮기면 된다 |
| M0609 | 상대 경로 `../../scenes/demo_test/assets/robots/...` | 〃 |

**완전 오프라인 / 폴더 하나로 넘기려면**: 리그 파일을 연 뒤
`File > Collect As...` 로 한 폴더에 모은다. 카터의 웹 참조까지 전부 로컬로 복사된다.
받는 쪽은 그 폴더의 `amr_lift_rig.usd` 하나만 참조하면 된다.

장면에 넣을 때는 **복사하지 말고 참조**로 넣는다 (`04_add_rig_to_scene.py`).
그래야 리그를 고치면 모든 장면에 반영된다.

---

## 6. 기존 `test.py` 연결

아티큘레이션이 하나로 합쳐졌으므로 **경로만** 바뀐다.
`robot.get_dof_index("joint_1")` 로 인덱스를 찾는 기존 코드는 그대로 동작한다.

```python
RIG = "/World/amr_lift_rig"
ROBOT_PATH = f"{RIG}/Nova_Carter/chassis_link"      # 아티큘레이션 루트
EE_FRAME   = "m0609_with_fork/link_6"               # end_effector_prim_path 용
# setup_arm_drives 의 조인트 경로
ARM_JOINTS = f"{RIG}/m0609_with_fork/joints"
```

리프트를 동작 시퀀스에 넣을 때는 두 가지 방법이 있다.

```python
# (a) USD Drive 목표를 직접 쓴다 — 03_lift_control.py 의 lift_to() 와 같은 방식
lift.set_goal(0.25)

# (b) 아티큘레이션 액션으로 다른 관절과 함께 보낸다
lift_idx = robot.get_dof_index("lift_prismatic_joint")
robot.apply_action(ArticulationAction(joint_positions=[0.25], joint_indices=[lift_idx]))
```

---

## 7. 검증 결과 (헤드리스 실측, 2026-09-18)

`physics_dt = 1/120`, 바닥 평면만 있는 빈 장면에 리그를 참조로 올려서 측정했다.

```
아티큘레이션 루트 1개 / kinematic 바디 0개 / 모든 조인트 프레임 어긋남 0.0000 m
DOF 14개 : 바퀴·캐스터 8 + lift_prismatic_joint + joint_1~6

리프트 0.279 m 상승 명령 → 캐리지 +0.2788, link_6 +0.2788, 섀시 +0.0001
리프트 0 m 하강 명령     → 시작 위치로 복귀 (오차 0.0000)

바퀴 3 rad/s 로 약 2 m 직진 (5초)
  섀시 기준 캐리지 상대위치 변화 : 0.0000 m
  섀시 기준 link_6 상대위치 변화 : 0.002 m 이하 (팔 관절 Drive 의 탄성)
  섀시 roll / pitch               : 0.1도 이하 (넘어지지 않음)

주행 중 리프트 0.2 m 명령 → 실제 0.1998 m
```

속도 측정

```
A) maxJointVelocity 0.5 m/s 만 걸고 한 번에 명령
   0.1초마다 0.048 m 씩 = 약 0.48 m/s 로 올라감 → 물리 상한이 실제로 걸린다

B) 패널 방식(목표를 0.08 m/s 로 램프)
   0.5초마다 0.040 m, 0.279 m 도달까지 3.81 초 (이론 3.49 초)
   등속 구간 추종 오차 8 mm → 목표 속도 피드포워드를 넣어 줄였다
   멈춘 뒤 정지 오차 0.2 mm
```

### 튜닝하면서 알게 된 것

* `Drive type = "force"` + stiffness 1e5 → 하중 때문에 **3 mm 처짐**.
* stiffness 를 1e6 으로 올리면 → **솔버가 터져서 로봇이 뒤집힌다** (pitch -88도).
* `Drive type = "acceleration"` + stiffness 1e5 → **오차 0.2 mm, 안정**. 이걸 채택했다.
* 원본 에셋 질량(마스트 4.5 / 캐리지 1.5 kg)은 28 kg 짜리 팔에 비해 가벼워
  질량비 때문에 솔버가 불안정해진다. 10 / 5 kg 으로 올렸다.
* 가동 범위를 바운딩 박스로 계산하면 위쪽 가로보를 놓친다 (0.309 vs 실제 0.284).

### 남은 것

* 팔이 선 자세일 때 무게중심이 약 1.9 m 로 높다. 급가속·급회전이 들어가면
  기울 수 있다. 실제 주행 시나리오로 한 번 확인이 필요하다.
* 아티큘레이션 self-collision 이 꺼져 있어(카터 에셋 설정) 캐리지와 마스트는
  **물리적으로 서로 막아 주지 않는다.** 뚫고 나가지 않게 하는 것은 조인트 리미트다.
  그래서 리미트 계산을 레이캐스트로 정확히 하는 것이 중요하다.
* 리프트 배치값(`LIFT_TRANSLATE`, `LIFT_ORIENT_WXYZ`)은 `demo_test` 장면에서
  눈으로 맞춘 값이다. 바꾸려면 상수를 고치고 `02_build_rig.py` 를 다시 돌린다.
