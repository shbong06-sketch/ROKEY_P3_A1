# robot_sample_v3_1 — MiR100 + M0617 + RG6 (Isaac Sim 5.0.1 대상, 5.1에서 최초 제작)

v1 대비 추가: **파라미터 제어 스크립트(control/)**, **B안: M0617 고정 베이스 에셋**.
v2 대비 변경: 관절 리미트/솔버 정합성 수정.
v3 대비 변경: Lula IK 브릿지 착수, ParallelGripper 검토 — 아래 "v3 → v3.1 수정 내역" 참고.

**Isaac Sim 5.0.1 관련 안내**: 이 패키지는 원래 Isaac Sim 5.1에서 제작/검증됨. 사용된 API
(`isaacsim.core.api.World`, `isaacsim.core.prims.SingleArticulation`, `isaacsim.core.utils.*`,
`isaacsim.SimulationApp`)는 Isaac Sim 4.5에서 `omni.isaac.*` → `isaacsim.*`로 이름이 바뀐 뒤
6.0에서 Core Experimental API로 대체되기 전까지(즉 4.5~5.x 전체) 이름이 바뀌지 않는 레거시 API이므로
5.0.1에서도 코드 레벨에서는 동일하게 동작할 것으로 확인됨(NVIDIA 공식 마이그레이션 가이드 기준).
다만 5.0.1에서 실제로 Play/실행까지 해본 기록은 아직 없음 — 아래 "5.0.1 스모크 테스트" 절차로 최초 1회 확인 필요.

## 파일
| 파일 | 용도 |
|---|---|
| `scenes/mobile_manipulator_demo.usda` | **A안 데모 씬** (MiR100+M0617+RG6, 240Hz, 바닥/조명) — File > Open |
| `scenes/mobile_manipulator.usda` | A안 로봇 에셋만 — 내 씬에 드래그 |
| `scenes/m0617_fixed_base_demo.usda` | **B안 데모 씬** (M0617+RG6를 높이 0.75m 테이블에 고정, 120Hz) — File > Open |
| `scenes/m0617_fixed_base.usda` | B안 로봇 에셋만 — 원점 = 베이스 바닥면. 테이블/바닥 윗면에 드래그하면 그 자리에 고정됨 |
| `assets/` | 부품 파일(구 `robots/`). **직접 열거나 드래그하지 말 것** (단독으로 넣으면 M0617이 넘어짐) |
| `control/mm_control.py` | 제어 라이브러리 (A/B 공용) |
| `control/run_control.py` | 명령줄 파라미터로 실행하는 스탠드얼론 러너 |
| `control/script_editor_example.py` | GUI Window > Script Editor에 붙여넣어 쓰는 예제 |
| `control/ik_bridge.py` | Lula IK 연결 브릿지(TCP 오프셋 역변환 + IK 결과를 mm_control에 전달) — robot_description.yaml 미확보로 아직 미완성, "v3 → v3.1 수정 내역" 참고 |
| `lula/dsr_description2/` | Lula 전용 M0617 원본 URDF+xacro+메쉬 (doosan-robot2 공식 저장소에서 추출, USD 시뮬레이션 자산과는 별개) |
| `lula/config/` | `robot_description.yaml` 배치 완료(`m0617_robot_description.yaml`) — cspace 조인트명 URDF와 일치 확인됨(`check_yaml_joint_names.py` PASS) |
| `lula/scripts/` | 단계별 검증 스크립트 6종(`test_urdf.py`, `check_yaml_joint_names.py`는 지금 바로 실행 가능, 나머지 4개는 Isaac Sim 필요) — "v3.1 추가 검증" 참고 |
| `lula/README.md` | tool0/반지름0 진단 내용 |

**폴더 구조를 이렇게 나눈 이유**: USD(`scenes/`, `assets/`)는 시뮬레이션 전용, URDF(`lula/`)는 Lula의
kinematic model 전용으로 완전히 분리했다. 같은 M0617이라도 "지금 시뮬레이션에서 몇 도인지"(USD
Articulation)와 "IK가 풀 때 쓰는 관절 체인·리미트"(URDF)는 다른 소스이므로, 파일 위치로도 그 경계를
명확히 해뒀다(자세한 이유는 `control/ik_bridge.py` 모듈 docstring의 "조인트 파라미터 3개 레이어" 참고).

## 제어 방법 3가지
### 1) 명령줄 파라미터 (가장 간단)
```bash
cd ~/Downloads/robot_sample_v3
# A: 90도 회전 -> 1m 직진 -> 팔 자세 -> 그리퍼 닫기
~/isaacsim/python.sh control/run_control.py --robot mobile --turn-deg 90 --distance 1.0 --arm 90,20,70,0,90,0 --gripper close
# B: 팔 자세 + 그리퍼 각도
~/isaacsim/python.sh control/run_control.py --robot fixed --arm -60,20,90,0,70,45 --gripper -10
```
옵션: `--v --w --drive-time`(개루프 속도), `--turn-deg`, `--distance`(폐루프), `--arm j1,..,j6`(deg), `--arm-speed`(deg/s),
`--gripper open|close|각도`, `--headless`, `--keep-open`(GUI 창 유지), `--usd`/`--root`(다른 씬/경로).
실행 끝에 명령값 vs 실측값 REPORT 출력.

### 2) GUI Script Editor
씬을 연(또는 에셋을 드래그한) 상태에서 `control/script_editor_example.py` 내용을 Window > Script Editor에 붙여넣고
맨 위 `SEQUENCE`만 수정 후 Run. 로봇 경로/패키지 경로는 자동 탐색, Play 자동 시작, Stop 시 제어 해제.

### 3) Property 패널에서 직접 값 입력 (Play 중)
| 대상 | 속성 | 단위/주의 |
|---|---|---|
| `m0617/joint_N_joint` | Drive > Target Position | **deg**. 보간 없이 즉시 이동 → 큰 값은 위험 |
| `rg6/l_out_joint` | Drive > Target Position | deg, -30(닫힘)~+30(열림). **나머지 5개 rg6 조인트는 입력해도 효과 없음**(mimic 종속) |
| `mir100/left(right)_wheel_joint_joint` | Drive > Target **Velocity** | **deg/s**, 바퀴별 개별 입력. Target Position은 효과 없음 |
Play 중 입력한 값은 Stop 후에도 스테이지에 남음 → 저장하면 다음 Play부터 그 값으로 움직임.

### 파이썬 API 요약 (`mm_control.MobileManipulatorControl(root, has_base)`)
`initialize()`(Play 후) · `step(dt)`(매 물리 스텝) · `set_base_velocity(v, w)` · `turn_by_deg(deg)` · `drive_distance(m)` · `base_done()` · `base_pose()` ·
`set_arm_joints_deg([6])` · `arm_joints_deg()` · `arm_done()` · `set_gripper_deg(deg)` · `open_gripper()` · `close_gripper()` · `gripper_deg()`
- 팔 목표는 자동 램프(기본 45deg/s, M0617 공식 한계 100/100/150/225/225/225 deg/s로 제한) + 관절 한계 클램프
- 입력값 범위 초과 시 잘라내고 경고(`ctl.warnings`)

## 검증 결과 (v2, Isaac Sim 5.1, 이 PC, PhysX 경고 0건 — v3의 관절 리미트/솔버 변경 이후 재검증 안 됨, 아래 "v2 → v3 수정 내역" 참고)
| 항목 | 명령 | 실측 |
|---|---|---|
| A 직진(개루프) | v=0.3m/s × 3s | 0.869m (97%) |
| A 회전(개루프) | w=0.5rad/s × 3s | 62.8° / 이상값 85.9° (**73%**) |
| A 회전(폐루프 `turn_by_deg`) | 90° / -45° | **90.2° / -45.1°** |
| A 직진(폐루프 `drive_distance`) | 1.0m / -0.5m | **1.019m / -0.496m** |
| A 팔 | 90,20,70,0,90,0 | 오차 0.04° |
| B 팔 | 45,30,60,0,60,0 / -60,20,90,0,70,45 | 오차 0.03° / 0.01° |
| 그리퍼 | close / open / 10 / -10 | -30.00 / 30.00 / 10.00 / -10.00° |
| 범위 초과 입력 | v=3, J3=200°, gripper=50° | 1.0m/s, **165°**(v2 당시 J3 리미트. v3에서 J3 리미트가 145°로 좁아졌으므로 재검증 시 145°로 clamp되는 게 맞음), 30°로 제한 + 경고, 크래시 없음 |
| B를 새 씬 테이블(2,-1,0.8)에 30° 회전해 드롭 | Play 45s | 베이스 이동 0mm, 팔/그리퍼 정상 |
| Script Editor 예제 | A 데모 / B 드롭 씬 | 시퀀스 전부 정상 수행 |
| Property 패널 값 입력 | joint_1=45 / l_out=-30 / 바퀴 229deg/s | 44.1° / 65mm로 닫힘 / 0.732m 이동 |

## 알려진 제한
- **개루프 회전은 부정확**(바퀴 슬립·캐스터 저항, 명령 대비 50~85%) → 정확한 이동은 `turn_by_deg`/`drive_distance` 사용
- 제자리 회전 시 로봇 중심이 약 0.1m 이동함(캐스터 영향)
- **좌표(x,y,z) 기반 팔 제어(IK)**: control/ik_bridge.py로 Lula IK 연결 작업 중. 과거 막혔던 원인 두 가지 특정됨 — (1) end effector 이름을 `tool0`로 넣었는데 본인 로봇엔 그런 링크가 없음(`link_6`이 맞음, ik_bridge.py에 반영 완료), (2) robot_description.yaml의 특정 링크 collision sphere 반지름이 0 — URDF는 확보해서 `lula/`에 포함시킴, yaml만 아직 XRDF Editor에서 직접 생성 필요(`lula/README.md` 참고)
- 팔 목표 자세가 바닥/테이블/MiR과 충돌하면 강한 드라이브 때문에 로봇이 넘어지거나 튕길 수 있음 (충돌 회피 없음)
- 장착 오프셋(MiR 상판 0.3942m, 베이스 바닥 0.0452m)은 형상 기준값 — CAD 확정 시 교체
- MiR 라이다 시각 메쉬 위치 부정확(원본 URDF 필요)
- RG6 l_out_joint 실사용 범위는 ±30°(mm_control.py가 항상 이 범위로 clip). 조인트 자체의 PhysX 하드 리미트는 -42.97°~+34.99°로 더 넓지만, mimic follower 조인트(r_out/l_tip/r_tip/l_passive/r_passive)는 ±30~35°까지만 따라올 수 있음 — Property 패널로 l_out에 -30° 미만 값을 직접 입력하면 기어비가 안 맞아 걸리거나 떨릴 수 있으니 API(`set_gripper_deg`/`open_gripper`/`close_gripper`) 사용을 권장

## v0 → v1 수정 내역
v1 README 내용 유지: 장착 위치 정렬, excludeFromArticulation, MiR 관절 잠금 해제(±inf), MiR self-collision off,
라이다 콜라이더 비활성, 캐스터 마찰 0, RG6 self-collision off, 에셋/씬 분리, 데모 240Hz.

## v2 → v3 수정 내역 (Isaac Sim 5.0.1 호환성 점검 후 반영)
1. **M0617 관절 리미트를 두산 공식 "Default"(공장 출하 안전 범위) 값으로 수정.**
   기존 v2는 joint_1/2/4/5/6이 사실상 무제한(±360.0008°)이었고 joint_3만 ±165°(기구 최대치)였음.
   mm_control.py의 안전 클램프(`_arm_lo`/`_arm_hi`)가 이 값을 그대로 읽어 쓰므로, 사실상 joint_3 말고는
   클램프가 작동하지 않는 상태였음. 아래 표의 "Default" 값으로 `robots/m0617_rebuilt.usd`의
   `physics:lowerLimit`/`upperLimit`을 수정함(코드는 리미트를 런타임에 usd에서 읽으므로 mm_control.py는 수정 불필요).

   | 조인트 | 기구 최대(Min/Max) | **v3에 적용한 값(Default)** | v2 상태 |
   |---|---|---|---|
   | J1 | ±360° | **±360°** (변경 없음, 연속회전) | ±360.0008° |
   | J2 | ±360° | **±95°** | ±360.0008° (사실상 무제한 — 수정됨) |
   | J3 | ±165° | **±145°** | ±165.0004° (기구 최대치로 되어 있었음 — 수정됨) |
   | J4 | ±360° | **±360°** (변경 없음, 연속회전) | ±360.0008° |
   | J5 | ±360° | **±135°** | ±360.0008° (사실상 무제한 — 수정됨) |
   | J6 | ±360° | **±360°** (변경 없음, 연속회전) | ±360.0008° |

   출처: [Doosan Robotics 공식 매뉴얼 — Upper/Lower Threshold Range and Default Value of Safety Parameters, M0617](https://manual.doosanrobotics.com/en/user-manual/3.6.0/1-m-h-series/upper-lower-threshold-range-and-default-value-of-s).
   조인트 속도 리미트(J1..J6 = 100/100/150/225/225/225 deg/s)는 같은 표 기준으로 이미 v2부터 정확했음 — 변경 없음.
   실제 로봇을 Default보다 넓은 범위로 운용하도록 재설정했다면(DRCF 세이프티 설정에서 조정 가능) 이 값을 실물에 맞춰 넓혀야 함.

2. **MiR100 / RG6 articulation의 solver iteration을 M0617과 맞춤.**
   v2는 M0617만 `physxArticulation:solverPositionIterationCount = 32`가 별도로 걸려 있었고, MiR100/RG6는 자기 파일
   안에 PhysxArticulationAPI가 있었지만 solverPositionIterationCount는 PhysX 기본값(4)을 그대로 썼음. 세 개가
   `excludeFromArticulation` Fixed Joint로만 연결된 별개 articulation이라, 수렴 품질 차이가 접합부(그리퍼가
   물체를 쥘 때 등)에서 떨림/부정확으로 나타날 수 있음. `mobile_manipulator.usda`, `m0617_fixed_base.usda`에서
   mir100/rg6 wrapper에 `PhysxArticulationAPI` + `solverPositionIterationCount = 16`(velocity=1)을 추가함.
   32와 16 중 어느 쪽이 맞는지는 실측(특히 그립 안정성) 기준으로 재조정 필요 — 우선 M0617의 절반으로 설정.

3. **B안(`m0617_fixed_base_demo.usda`) GroundPlane/Table에 A안과 동일한 방식으로 마찰 물리 재질을 바인딩.**
   v2는 A안 GroundPlane에만 마찰 재질(정지 0.8/동 0.7)이 바인딩돼 있었고 B안 GroundPlane·Table은 PhysX 기본
   마찰값을 썼음. 지금 시퀀스(팔+그리퍼만 동작)에는 영향 없지만, 테이블 위에 물체를 놓는 시나리오로 확장할 때
   재현성 문제가 될 수 있어 미리 맞춤(Table은 마찰 정지 0.7/동 0.6로 약간 낮게 별도 설정 — 필요시 조정).

4. **`m0617_fixed_base.usda`의 `world_to_base` 조인트에 설명 주석 추가.**
   이 조인트만 다른 두 연결 조인트와 달리 `physics:excludeFromArticulation`이 없는데, 이는 버그가 아니라
   world 앵커 조인트를 articulation에 포함시켜야 진짜 "고정 베이스"가 되기 때문에 의도된 것. 나중에 실수로
   `excludeFromArticulation=1`을 추가하면 베이스 고정이 풀려버리므로, 이를 usd 파일의 doc 주석으로 직접 남김.

5. **5.0.1 호환성은 문서 레벨로 확인, 런타임 테스트는 아직 미실시.**
   NVIDIA 공식 마이그레이션 가이드 기준으로 `isaacsim.core.api`/`isaacsim.core.prims`/`isaacsim.core.utils`/
   `isaacsim.SimulationApp`는 4.5에서 이름이 바뀐 뒤 6.0에서 Core Experimental API로 대체되기 전까지 그대로
   유지되는 API라 5.0.1에서도 코드가 그대로 동작할 가능성이 높음. 다만 이건 문서 대조로 확인한 것이고, 실제
   Isaac Sim 5.0.1 인스턴스에서 열어서 확인한 적은 없음 — 아래 스모크 테스트를 사용자 환경에서 1회 실행 권장.

## Isaac Sim 5.0.1 스모크 테스트 (최초 1회 직접 확인 필요)
```bash
cd ~/Downloads/robot_sample_v3
# 1) B안(더 가볍고 빠름)부터: 팔+그리퍼만 움직여서 리미트/솔버 변경이 문제없는지 확인
~/isaacsim/python.sh control/run_control.py --robot fixed --arm 90,-90,120,0,90,0 --gripper close --headless
# -> REPORT에 arm target=[90, -90, ...] reached=[...] 이 나오는지, J2=-90(±95 안쪽)이 그대로 들어가는지 확인
# -> J2에 -95 초과값(예: -100)을 넣어서 -95로 clamp + 경고가 뜨는지도 한 번 확인
~/isaacsim/python.sh control/run_control.py --robot fixed --arm 0,-100,0,0,0,0 --headless
# 2) A안(MiR100 포함, 무거움): 기본 시퀀스 통과 확인
~/isaacsim/python.sh control/run_control.py --robot mobile --turn-deg 90 --distance 1.0 --arm 90,20,70,0,90,0 --gripper close --headless
```
세 번의 실행 모두 PhysX 경고/크래시 없이 REPORT가 끝까지 출력되면 5.0.1 호환 확인 완료로 간주.
그립 안정성(솔버 iteration 변경 효과)은 GUI로 열어서 그리퍼로 작은 물체를 쥐어보는 것으로 육안 확인 권장(자동화된 검증 스크립트는 아직 없음).

## v3 → v3.1 수정 내역 (Lula IK 연결 착수, ParallelGripper 검토)

1. **`control/ik_bridge.py` 추가.** TCP(손가락 끝) 목표 → link_6(플랜지) 목표로 오프셋 역변환 후
   `LulaKinematicsSolver`로 IK를 풀어 `mm_control.py`의 `set_arm_joints_deg()`에 넘기는 브릿지.
   `end_effector_name`을 기존에 막혔던 `tool0`(존재하지 않는 링크) 대신 실제 프림 이름인 `link_6`로
   지정해 해당 문제를 해결. `FINGER_PAD_TIP_Z`는 본인 RG6 실측값(0.25953m, `g_main_joint`+`hand_tcp_joint_joint`
   오프셋 합)으로 설정.
   **아직 안 풀린 것**: `robot_description.yaml`의 특정 링크 collision sphere 반지름이 0인 문제는 코드로
   고칠 수 없음 — XRDF Editor에서 반지름을 양수로 수정해야 진행 가능. URDF 자체는 이후 확보해서
   `lula/dsr_description2/`에 포함시킴(아래 3번 항목); yaml만 아직 미생성 상태.

   **추가 수정**: M0617이 AMR(MiR100) 위에 고정된 A안에서, 처음엔 `set_robot_base_pose()`를
   `__init__`에서 한 번만 호출해서 AMR이 이동한 뒤에는 IK 결과가 다 틀어지는 문제가 있었음.
   IK를 풀 때마다(`solve_flange_ik_deg` 호출마다) 현재 팔의 world pose를 다시 읽어 Lula에
   갱신하도록 고침(`_refresh_base_pose()`). B안(고정 베이스)에서는 어차피 base pose가 안
   바뀌므로 매번 호출해도 결과에 영향 없음.

   **URDF ↔ USD 조인트 정합성 검증 완료.** joint_1~6의 origin(xyz)과 rotation을 URDF(`<origin rpy=.. xyz=..>`)와
   USD(`physics:localPos0`/`localRot0`)를 좌표별로 직접 대조 — 6개 조인트 전부 소수점 단위까지 일치함을 확인.
   즉 "Lula(URDF 기준)는 맞게 계산했는데 USD 로봇이 엉뚱한 방향으로 움직인다"는 종류의 문제는 이 자산에서는
   구조적으로 발생하지 않음(조인트 축이 다르거나 origin이 어긋난 게 없다는 뜻).

   **IK 리미트(레이어 B) vs 시뮬레이션 클램프 불일치 문제 발견 및 처리.** Lula는 URDF 기준 조인트 리미트로
   IK를 푸는데, dsr_description2 URDF 자체는 J2/J4/J5/J6이 사실상 무제한(±360°)이고 J3만 ±165°(하드웨어
   최대치)임 — v3에서 mm_control이 쓰도록 좁혀놓은 소프트웨어 Default 리미트(J2=±95, J3=±145, J5=±135)를
   Lula는 전혀 모른 채 IK를 풀 수 있음. 이 상태로 Lula의 IK 결과를 그냥 `set_arm_joints_deg()`에 넘기면
   mm_control이 조용히 clamp해버려서, **IK가 계산한 자세와 로봇이 실제로 도달하는 자세가 달라짐**(TCP가
   목표 지점에 정확히 안 닿을 수 있다는 뜻). `M0617IKBridge`에 `mm_ctl` 파라미터를 추가해서, IK 결과가
   mm_control의 실제 안전 리미트를 벗어나면 clamp 대신 명시적으로 실패(`None`) 처리하도록 고침 — 목표
   지점이 안전 범위 밖에서만 도달 가능하다는 걸 조용히 숨기지 않고 바로 알 수 있게.

2. **Isaac Sim 내장 `ParallelGripper` 클래스를 RG6 제어에 쓸 수 있는지 검토 — 채택 보류, 코드 변경 없음.**
   `ParallelGripper`는 `joint_prim_names`/`joint_opened_positions`/`joint_closed_positions`를 등록해두고
   `forward()`로 열기/닫기를 호출하는 구조이고, `action_deltas=None`이면 절대 위치 명령, 지정하면 상대 명령이 된다.
   대조해보면:

   | 항목 | 강의 예시(finger_joint) | 본인 RG6(l_out_joint) |
   |---|---|---|
   | 단위/범위 | 0.0(완전 열림) ~ 약 1.18(완전 닫힘) | -30°(닫힘) ~ +30°(열림), 실측 하드리미트 -42.97°~+34.99° |
   | 방향 | 값이 커질수록 닫힘 | 값이 커질수록 열림 (강의 예시와 반대) |
   | 실사용값 | 닫힘 0.8 | 닫힘 -30°, 열림 +30° (`mm_control.GRIPPER_RANGE_DEG`) |
   | 한계 초과 시 | 한계에서 멈춤 | 동일 (mm_control이 clip + 경고) |
   | 물체에 닿으면 | 목표 도달 못하고 중간에 멈춤 | 동일하게 동작할 것으로 예상(PD 드라이브 구조상) — 실측 확인 안 됨 |

   결론: 강의 자료의 `joint_opened_positions`/`joint_closed_positions` 숫자(0.0/1.18)를 그대로 갖다 쓰면
   RG6에서는 완전히 틀린 값이 된다(방향도 반대, 단위도 라디안 기준으로 환산 필요: 열림 +30°=+0.5236rad,
   닫힘 -30°=-0.5236rad). `mm_control.py`의 `set_gripper_deg()`가 이미 이 값들로 안전 클램프와 경고까지
   구현돼 있어서, 지금 시점엔 `ParallelGripper`로 갈아탈 실익이 없다고 판단 — 코드 변경 없이 검토만 기록.
   나중에 `ParallelGripper`로 통일하고 싶다면 `joint_prim_names=["l_out_joint"]`,
   `joint_opened_positions=[0.5236]`, `joint_closed_positions=[-0.5236]`(라디안)로 등록하면 될 것.

## v3.1 추가 검증 (외부 적대적 검토 반영) — 현재 상태를 A/B/C/D로 정리

| 단계 | 내용 | 상태 |
|---|---|---|
| A. M0617 USD가 Isaac Sim 5.0.1에서 안정적으로 움직이는가 | Articulation 구조·solver·리미트는 정적 검증 완료. **실제 Play 안정성은 미확인**(이 환경에서 Isaac Sim 실행 불가) | 🟡 `lula/scripts/test_usd_articulation.py`로 직접 실행 필요 |
| B. M0617 URDF가 Lula에서 FK/IK 되는가 | URDF 체인/axis/limit/mesh경로는 검증 완료(`test_urdf.py` 통과). **robot_description.yaml이 없어 Lula 자체를 아직 못 돌림** | 🔴 yaml부터 생성 |
| C. USD와 URDF의 joint/link/frame이 동일한가 | joint_1~6의 **parent/child, axis, origin(위치+회전) 전부 좌표 단위로 대조해 일치 확인** (아래 표) | 🟢 완료 |
| D. Lula q를 USD DOF에 넣었을 때 동일 자세가 나오는가 | 코드(`ik_bridge.py`)는 준비됐지만 **실측 비교는 미실시** | 🔴 `test_usd_lula_mapping.py`로 직접 실행 필요 |

**Isaac Sim 5.0 공식 API/구조 검증 완료 — 5.0.1 런타임 검증은 별개(아직 안 됨).** `docs.isaacsim.omniverse.nvidia.com/5.0.0/manipulators/manipulators_lula_kinematics.html`의
`Lula_Kinematics_python/scenario.py` 전문을 `ik_bridge.py`와 비교함. 이건 "5.0.0 문서 = 5.0.1 설치본 동작 보장"이
아니라 "5.0 계열 공식 API·코드 구조와 일치"라는 정적 검증이라는 점을 분명히 함 — 실제 5.0.1 인스턴스에서
돌려본 적은 여전히 없음. 일치 확인된 것: import 경로(`isaacsim.robot_motion.motion_generation`), `set_robot_base_pose()`를
매 프레임 다시 호출하는 패턴(우리 `_refresh_base_pose()`와 동일), `compute_inverse_kinematics()` 실패 시
`apply_action` 안 하고 경고만 내는 패턴, `get_all_frame_names()`/`ArticulationKinematicsSolver(...)` 시그니처.
새로 알게 된 것: **`ArticulationKinematicsSolver.compute_inverse_kinematics()`의 warm start는 현재 살아있는
Articulation의 관절 상태**라고 공식 문서에 명시됨 — 예전에 "joint_2/5 default를 안전범위 안에 둬야 한다"고 한
우려는 이 실행 경로(우리가 쓰는 것)에는 해당 안 됨. 다만 yaml의 `default_q` 자체가 무의미한 건 아니고(로봇의
"기본 자세"라는 의미는 유지), "매 IK 호출의 실시간 시드"라는 의미는 아니라는 것.
`SingleArticulation` vs `Articulation` 관련해서는 — 한 문서 페이지(5.0.0 Lula Kinematics 튜토리얼)만 보고
"불일치 발견"이라 한 게 과했음. 이후 Articulation Controller/Articulation Joint Sensors 등 5.0~5.1 계열
공식 문서 여러 곳에서 `SingleArticulation`이 반복적으로 쓰이는 걸 확인했고, deprecated 표시는 6.0.0부터임 —
즉 5.0.1에서 `SingleArticulation`은 실제 위험이 아니라 정상적으로 지원되는 패턴. `ik_bridge.py`/새 테스트
스크립트에 넣어둔 `Articulation` 폴백은 "혹시 몰라 넣은 방어 코드"로 성격을 낮추되, 해가 없으므로 코드는 유지함.

**지금 상태를 가장 정직하게 요약하면:**

| 항목 | 상태 |
|---|---|
| 정적 API 검증(5.0 공식 문서와 코드 구조 대조) | ✅ 완료 |
| USD/URDF joint mapping(parent/child, axis, origin) 검증 | ✅ 완료 |
| 독립 테스트 구조(URDF/USD/Lula 분리) | ✅ 완료 |
| clean-package 경로 검증(절대경로 잔재 없음, 새 위치에서 재현) | ✅ 완료 |
| robot_description.yaml 생성 | ✅ 완료 (`lula/config/m0617_robot_description.yaml`) |
| yaml `cspace:` 조인트명 ↔ URDF 이름 검증 | ✅ 완료 — `check_yaml_joint_names.py` PASS (`joint_1`~`joint_6`, URDF와 완전 일치) |
| Isaac Sim 5.0.1 실제 Play/실행 | ⏳ 미검증 (사용자 환경에서만 가능) |

"거의 다 됐다"는 맞지만, yaml 생성 후 5.0.1에서 실제로 한 번 끝까지 돌려보기 전까지는 "완전 검증됐다"고 부르지 않는 게 정확함.

**질량/관성(mass/inertia) 정적 확인 완료 — 정상 패턴, 버그 아님.** `assets/m0617_rebuilt.usd`의 7개 링크
전부 `physics:mass`가 양수로 명시돼 있음(4.12~11.55kg, 실제 그럴듯한 값). `physics:diagonalInertia=(0,0,0)`,
`physics:centerOfMass=(-inf,-inf,-inf)`로 보여서 "이상값"처럼 보였지만, USD Physics 공식 스키마 문서에
이 값들이 "미지정 시 PhysX가 collision mesh로부터 자동 계산"하는 sentinel 기본값이라고 명시돼 있어서
정상 패턴임을 확인함. 다만 자동계산 정확도는 collision mesh 품질에 달려있으므로, 실제 Play 시 여전히
관절이 튀면 Isaac Sim의 Mass Properties 디버그 뷰로 자동계산된 실제 값을 확인해볼 것 — 정적 분석만으로는
"완전히 결백"까지는 확인 불가능한 영역.

**robot_description.yaml 실제 반입 완료.** 두 버전을 받음 — 하나(`m0617.yaml`)는 예측했던 그대로
`cspace: [joint_1_joint, ...]`(USD DOF 이름)로 돼 있어 `check_yaml_joint_names.py`가 MISMATCH를
정확히 잡아냄. 다른 하나(`m0617_lula.yaml`)는 `cspace: [joint_1, ...]`로 이미 URDF와 일치하게
수정된 버전이었음 — diff로 대조해보니 이 6줄(조인트명) 외에는 완전히 동일한 파일. PASS인
`m0617_lula.yaml`을 `lula/config/m0617_robot_description.yaml`로 배치함(`ik_bridge.py`의
`ROBOT_DESCRIPTION_YAML` 기본값과 정확히 일치 확인). `collision_spheres:`/`cspace_to_urdf_rules:`는
비어있는데 이건 예상된 상태(순수 IK/FK 목적에는 문제 없음), `default_q`는 전부 0 근처라 J2/J5
안전범위 안. **다만 이건 텍스트 레벨 검증까지고, Lula가 실제로 이 yaml+URDF를 로드해서 FK/IK를
계산하는 건 여전히 Isaac Sim에서 `test_lula_fk.py`/`test_lula_ik.py`/`test_usd_lula_mapping.py`를
직접 돌려야 확인됨(이 환경엔 Isaac Sim이 없어서 대신할 수 없음).**

C 검증 상세 (joint_1~6 전부):

| 조인트 | type | axis(URDF=USD) | parent→child(URDF=USD) | origin(위치+회전, URDF=USD) |
|---|---|---|---|---|
| joint_1~6 | revolute=revolute | (0,0,1)=Z | base_link→link_1 ... link_5→link_6 | 6개 전부 소수점 단위 일치 |

이번에 추가로 확인한 것(이전엔 origin만 봤고 axis/parent·child는 안 봤었음 — 외부 검토가 짚은 진짜 구멍):
`<axis xyz="0 0 1">` vs USD `physics:axis`, `<parent>`/`<child>` vs USD `physics:body0`/`physics:body1` 6개 전부 대조 완료, 전부 일치.

**mesh 경로 resolve 검증**: `package://dsr_description2/...`로 참조되는 파일 60건(m0617/m0617.blue/m0617.white URDF 전부 합쳐서) 실제 존재 여부 확인 — 처음엔 `m0617.blue.urdf`가 쓰는 `meshes/m0617_blue/` 10개가 빠져있어서 FAIL이었음(패키징할 때 collision/white만 챙기고 blue를 빠뜨림) → 원본에서 마저 복사해서 60건 전부 resolve 확인.

**`lula/scripts/` 5개 검증 스크립트 추가** (외부 검토의 단계적 테스트 제안 반영):
- `test_urdf.py` — **Isaac Sim 불필요, 지금 이 상태로 실행해서 PASS 확인 완료.** 체인 연결성/axis/리미트/mesh resolve.
- `test_usd_articulation.py` — Lula 없이 USD Articulation만 검증(DOF 순서, 방치 시 드리프트, 목표 추종). "로봇이 튀는 문제"는 Lula 이전에 여기서 먼저 잡을 것.
- `test_lula_fk.py` — Lula FK 단독 (yaml 필요, USD 불필요).
- `test_lula_ik.py` — Lula IK round-trip 단독 (yaml 필요, USD 불필요).
- `test_usd_lula_mapping.py` — **가장 중요.** 같은 q를 USD와 Lula 양쪽에 넣어서 link_6 world position을 직접 비교(5mm 기준 PASS/FAIL). 이게 통과해야 `ik_bridge.py`를 실전에 써도 됨.
4개(urdf 제외)는 전부 Isaac Sim 안에서 실행해야 하는 스크립트라 여기서는 문법 검증까지만 했음 — 순서대로(위 표 순서) 실행해서 각 단계 PASS 여부를 직접 확인할 것.

3. **폴더 구조 재편 — USD(시뮬레이션)와 URDF(Lula kinematic model)를 물리적으로 분리.**
   기존엔 `robots/`(부품 usd) + 루트에 4개 usda가 뒤섞여 있었음. 아래처럼 바꿈:
   - `robots/` → `assets/` (이름만 변경, 내용 동일)
   - 루트의 usda 4개 → `scenes/` 밑으로 이동
   - `lula/dsr_description2/`(URDF+xacro+메쉬), `lula/config/`(robot_description.yaml 저장 위치),
     `lula/README.md`(tool0/반지름0 진단) 신설

   이동에 따라 실제로 깨질 수 있는 지점을 전부 고치고 재검증함:
   - `scenes/mobile_manipulator.usda`, `scenes/m0617_fixed_base.usda`의 `@./robots/...@` 참조를
     `@../assets/...@`로 수정 (씬 안 에셋 참조가 실제로 풀리는지 `Usd.Stage.Open`으로 재검증 완료 —
     mir100/m0617/rg6 전부 정상 로드, solverPositionIterationCount·조인트 리미트 값도 그대로 유지 확인)
   - `control/run_control.py`가 여는 기본 데모 경로를 `scenes/mobile_manipulator_demo.usda` 등으로 수정
   - `control/script_editor_example.py`의 `_find_package_dir()`가 이제 `scenes/` 밑의 usda를 찾으므로,
     패키지 루트를 한 단계 위로 잡도록 수정 (안 고치면 `control/` 폴더를 못 찾아서 import 실패했을 것)
   - `control/ik_bridge.py`의 `ROBOT_URDF_PATH`가 이제 `lula/dsr_description2/urdf/m0617.urdf`를
     `__file__` 기준 상대경로로 자동 계산하도록 변경(더 이상 직접 채울 필요 없음). `ROBOT_DESCRIPTION_YAML`도
     기본값이 `lula/config/m0617_robot_description.yaml`을 가리키도록 변경 — yaml만 그 자리에 만들어 넣으면 됨.
