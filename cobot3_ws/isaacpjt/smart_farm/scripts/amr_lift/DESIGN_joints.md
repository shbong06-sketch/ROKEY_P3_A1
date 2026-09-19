# 불러온 모델들의 물리·조인트 현황과, 그에 맞춘 모션·고정장치 설계

세 에셋의 USD / URDF 를 직접 읽어서 정리한 것. 추측 없이 파일에 적힌 값만 적었다.

---

## 1. Nova Carter — `nova_carter.usd` (variants: Physics_Base / All_Sensors / No_Internals)

| 항목 | 값 |
|---|---|
| 아티큘레이션 루트 | **`chassis_link`** (prim 이 아니라 링크에 붙어 있음) |
| chassis_link | RigidBody, mass **41.68 kg**, COM (-0.175, 0, 0.22), `visual` 에 convexHull 콜라이더 |
| self-collision | `physxArticulation:enabledSelfCollisions = 0` |
| 외피 상단 | z = **0.4799** (발자국 안 최고점), 그 위로 XT_32 라이다가 z 0.5543 까지 |

조인트 (전부 Revolute)

| 조인트 | body0 → body1 | Drive |
|---|---|---|
| `joint_wheel_left/right` | chassis_link → wheel_left/right | k=0, c=1e6 → **속도 제어용** |
| `joint_caster_base` | chassis_link → caster_frame_base | k=100, c=10 |
| `joint_swing_left/right` | caster_frame_base → caster_swivel_* | k=0, c=1e-5 (거의 자유) |
| `joint_caster_left/right` | caster_swivel_* → caster_wheel_* | Drive 없음 (자유 회전) |

→ 바퀴는 **속도 드라이브**로 굴리게 되어 있다. 위치 제어로 쓰면 안 된다.

---

## 2. M0609 + fork — `m0609_with_fork.usd` / 원본 `m0609_isaac_sim.urdf`

| 항목 | 값 |
|---|---|
| 아티큘레이션 루트 | **`root_joint`** (FixedJoint, body0 = 없음 = **월드에 못박기**) |
| 링크 질량 | 3.18 / 5.02 / 8.04 / 3.60 / 3.57 / 2.83 / 1.16 kg + fork 0.45 = **27.85 kg** |
| 공구 부착 | `tool0_to_fork_tool` FixedJoint (link_6 → fork_tool) |
| 플랜지 밑면 | base_link 로컬 z = **0.0000** (반경 0.10 m 안 2050 점) → 원점이 곧 장착면 |

조인트 (전부 Revolute, 로컬 축 Z)

| 조인트 | USD 리미트 | URDF effort / velocity | URDF dynamics (k / c) | USD Drive (k / c) |
|---|---|---|---|---|
| joint_1 | ±360.0° | 9600 / 2.618 | 266.7 / 26.7 | 40.73 / 0.0163 |
| joint_2 | ±360.0° | 9600 / 2.618 | 266.7 / 26.7 | 102.3 / 0.0409 |
| joint_3 | **±150.0°** | 5400 / 3.1416 | 150 / 15 | 1160.8 / 0.4643 |
| joint_4 | ±360.0° | 2700 / 3.927 | 750 / 7.5 | 773.5 / 0.3094 |
| joint_5 | ±360.0° | 2700 / 3.927 | 750 / 7.5 | 45.96 / 0.0184 |
| joint_6 | ±360.0° | 2700 / 3.927 | 750 / 7.5 | 31.48 / 0.0126 |

**주의**: URDF 의 dynamics 값과 USD Drive 값이 서로 대응되지 않는다(임포터가 다시 계산한 값).
USD 기본 게인은 자세 유지에 약해서 팔을 뻗으면 처진다.
`test.py` 는 실행할 때마다 `setup_arm_drives()` 로 k=1e8, c=1e4 로 덮어쓴다 —
그 방식을 그대로 유지하는 것을 권한다 (`07_fix_colliders.py` 의 `ARM_DRIVE_STIFFNESS` 로도 가능).

---

## 3. 리프트 — `lift_v3_physics.usdc`

| 항목 | 원본 값 | 리그에서 바꾼 값 | 이유 |
|---|---|---|---|
| `lift_prismatic_joint` | holder → mover, **축 Z**, 리미트 0 ~ 0.309 | 0 ~ **0.2785** | 0.309 는 마스트 맨 위 가로보를 25 mm 뚫는다 (레이캐스트 실측 0.2835 − 여유 5 mm) |
| Drive | force, k=1e5, c=1e4, maxF=1e4 | **acceleration**, 같은 게인 | force 는 하중 때문에 3 mm 처짐. k 를 1e6 으로 올리면 솔버가 터져 로봇이 뒤집힘 |
| `lift_holder` | RigidBody **kinematic = True**, 4.5 kg | kinematic **False**, 10 kg | kinematic 바디는 아티큘레이션에 못 들어간다 |
| `lift_moveparts_1` | RigidBody dynamic, 1.5 kg | 5 kg | 28 kg 짜리 팔 대비 질량비가 커서 솔버가 불안정 |
| `M0609_FixedJoint` | mover → `/World/m0609/base_link` (없는 경로) | **active = false** | "disjointed body transforms" 경고의 원인 |
| `M0609_Mount` | 로컬 z 0.827889 | (참고만) | 이건 **브래킷 꼭대기**다. 실제 상판은 z **0.75275** → 여기 붙여야 75 mm 뜨지 않는다 |

---

## 4. 콜라이더 점검 결과

| 대상 | 상태 | 조치 |
|---|---|---|
| `base_link/collisions/.../Scene` | **convexHull**, 퍼짐 **0.431 m** — 메시에 케이블이 포함돼 있어 훌이 케이블 끝까지 감싼다 = 보이지 않는 유령 충돌체 | **convexDecomposition 으로 수정** (원본 레이어 `m0609_with_fork_physics.usd` 의 `/colliders/base_link/MF0609_0_0/Scene` 에 저장 → 리그 폴더 안이라 다른 PC 로 따라감) |
| link_1 ~ link_6 콜라이더 | convexHull, 퍼짐 0.09 ~ 0.247 m — 형상에 맞음 | 그대로 |
| `fork_tool` | Cube(box) 콜라이더 3개 (mount_plate / left_tine / right_tine) | 그대로. 얇은 포크 날은 box 가 정확하고 빠르다 |
| 리프트 마스트·캐리지 | 원본 convexHull → 속 빈 마스트가 꽉 찬 상자가 된다 | **convexDecomposition** (리그에서 이미 적용) |
| 카터 `chassis_link/visual` | convexHull (NVIDIA 기본) | 그대로. 5백만 삼각형을 분해하면 쿠킹·성능이 나빠진다 |
| `combine_test_1` 의 별도 팔 | `visuals` 와 `collisions` 양쪽에 **approximation = "none"** (삼각형 메시) | **움직이는 바디에는 쓸 수 없는 설정.** 그 팔 자체를 끄는 것으로 정리 (`08_fit_scene.py`). 남겨 두려면 07 이 convexHull 로 바꾸고 visuals 쪽 콜리전을 끈다 |

---

## 5. 조인트 구조를 고려한 **모션 설계**

움직임을 넣을 수 있는 자리는 세 곳뿐이고, 역할이 겹치지 않는다.

```
카터 바퀴 (Revolute × 2)   : 속도 드라이브   → 주행
리프트   (Prismatic × 1)   : 위치 드라이브   → 승강      ★ 여기에만 모션을 건다
팔       (Revolute × 6)    : 위치 드라이브   → 작업 (독립)
```

* **승강은 `lift_prismatic_joint` 하나로만 한다.** 축이 Z 하나뿐이라 구조적으로
  위아래로만 움직인다 (다른 5 자유도는 조인트가 구속). 키프레임 애니메이션이 아니라
  드라이브 목표값을 주는 방식이라, 위에 얹힌 28 kg 팔의 무게·관성이 그대로 반영된다.
* **범위**는 조인트 리미트로 강제한다. self-collision 이 꺼져 있어 캐리지와 마스트는
  서로 막아 주지 않으므로, 리미트가 유일한 방어선이다.
  그래서 리미트를 레이캐스트 실측(5 mm 격자)으로 잡는다 — 바운딩 박스로 잡으면 틀린다.
* **속도**는 목표값을 초당 `speed` 만큼만 끌어주는 램프로 만든다(`05_lift_panel.py`).
  추가로 `physxJoint:maxJointVelocity = 0.5 m/s` 로 물리 쪽 상한도 걸어 둔다(실측 확인).
* **팔은 따로 움직인다.** 하나의 아티큘레이션 안에 있지만 DOF 는 독립이라
  `get_dof_index("joint_1")` 로 인덱스를 잡아 기존 `test.py` 방식 그대로 명령하면 된다.
  리프트도 같은 방식으로 `get_dof_index("lift_prismatic_joint")` 로 섞어 보낼 수 있다.

---

## 6. 조인트 구조를 고려한 **고정장치 설계**

```
chassis_link ──FixedJoint── lift_holder ──PrismaticJoint── lift_moveparts_1 ──FixedJoint── base_link
 (카터 루트)      (마스트)                    (승강)              (캐리지)                    (팔)
```

| 결정 | 이유 |
|---|---|
| 마스트를 **reparent 하지 않고 FixedJoint** 로 묶는다 | RigidBody 를 RigidBody 밑으로 옮기는 것은 PhysX 금지. 서로 다른 참조 트리 사이의 reparent 는 Kit 이 내용을 복사해 버려 원본 갱신이 끊긴다 |
| 팔의 `root_joint` 를 **active = false** 로 끈다 | body0 가 비어 있어 월드에 못박는 조인트. 이게 살아 있으면 리프트가 올라가도 팔은 제자리에 남는다 |
| 팔은 캐리지에 **FixedJoint** 로 붙인다 (`mover_to_m0609`) | 부모-자식 계층이 아니라 조인트라서, 캐리지가 물리로 움직이면 팔도 물리로 따라온다 |
| 조인트 local frame 을 **현재 자세에서 역산**한다 | 어긋나 있으면 Play 순간 끌려간다("disjointed body transforms") |
| 결과적으로 **아티큘레이션 1개** (DOF 14) | 아티큘레이션이 둘로 갈리면 그 사이 연결이 물렁해져 28 kg 팔이 출렁인다 |
| 팔 장착 높이는 `M0609_Mount` 가 아니라 **상판 실측값** | Mount 마커는 브래킷 꼭대기라 75 mm 높다 |

---

## 7. `combine_test_1.usd` 에서 실측한 최종 상태 (2026-09-18)

```
DOF 14: caster_base, wheel_L, wheel_R, swing_L, swing_R,
        lift_prismatic_joint, caster_L, caster_R, joint_1 ... joint_6

팔 플랜지 z 0.89772 / 캐리지 상판 z 0.89772  →  간극 +0.00000 m
리프트 0.2785 m 상승 명령 → 캐리지 +0.2782, 팔 base +0.2782, link_6 +0.2782, 섀시 +0.0000
하강 명령 → 시작 위치 복귀 (오차 0.0000)
2.06 m 주행 → 섀시 기준 캐리지 상대변화 1 mm, link_6 상대변화 1.1 mm
```
