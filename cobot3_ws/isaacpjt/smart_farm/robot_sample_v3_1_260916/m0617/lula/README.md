# M0617 URDF 패키지 (Lula robot_description.yaml 생성용)

패키지 안 위치: `robot_sample_v3_1/lula/dsr_description2/` (control/ik_bridge.py의
`ROBOT_URDF_PATH`가 이 폴더 안 `urdf/m0617.urdf`를 자동으로 가리킴).

출처: https://github.com/DoosanRobotics/doosan-robot2 (humble 브랜치)의
`dsr_description2/{urdf,xacro,meshes}` 중 M0617 관련 파일만 추림
(전체 저장소는 다른 로봇 모델까지 포함해 251MB라 M0617분만 7.8MB로 축소).

## 이 폴더 그대로 쓰는 법
Lula Robot Description(XRDF) Editor / `LulaKinematicsSolver(urdf_path=...)`에는
`dsr_description2/urdf/m0617.urdf`를 지정하면 됩니다. urdf 안의 mesh 참조가
`package://dsr_description2/meshes/...` 형태이므로, 이 폴더 구조
(`.../dsr_description2/urdf/`, `.../dsr_description2/meshes/`)를 그대로 유지한 채
ROS 패키지 검색 경로(`package://` 해석 경로)에 잡히게 두거나, XRDF Editor에
이 폴더 자체를 열어야 메쉬 경로가 깨지지 않습니다.

**`xacro/`는 참고용일 뿐 실제 파이프라인에서 안 쓰입니다.** `urdf/m0617.urdf`가 xacro를 이미
빌드해놓은 최종 결과물이라, `ik_bridge.py`도 XRDF Editor도 전부 이 urdf 파일만 직접 읽습니다.
xacro는 "원본 소스가 뭐였는지" 추적용으로만 같이 넣어둔 것 — 수정할 일 있으면 xacro를 고치고
다시 빌드해서 urdf를 갱신해야지, urdf만 따로 고치면 다음에 xacro로 재빌드할 때 그 수정이 날아갑니다.

## Environment / Robot
- Isaac Sim 5.0.1 대상 (5.1에서 제작, API 호환성은 메인 README 참고)
- 로봇: 두산 M0617 (6DOF 팔), RG6 그리퍼는 이 URDF에 없음 — IK는 팔만, 그리퍼는 `mm_control.py`가 별도 제어

## Pipeline
```
assets/m0617_rebuilt.usd  ──► Isaac Sim Articulation (시뮬레이션 실행 대상)
dsr_description2/urdf/m0617.urdf + config/m0617_robot_description.yaml
                           ──► LulaKinematicsSolver (FK/IK 계산 전용)

control/ik_bridge.py 가 둘을 잇는 지점:
  TARGET_TCP ──(오프셋 역변환)──► link_6 목표 pose
             ──(LulaKinematicsSolver.compute_inverse_kinematics)──► q1..q6
             ──(mm_ctl 안전 리미트 통과 확인)──► ctl.set_arm_joints_deg() ──► Articulation
```

## Test (순서대로, `scripts/` 참고)
1. `test_urdf.py` — URDF 단독, Isaac Sim 불필요, 지금 바로 실행 가능 (PASS 확인됨)
2. `test_usd_articulation.py` — USD 단독 (Lula 없이 안정성/추종 확인)
3. **yaml 생성 직후, Lula를 실제로 돌려보기 전에 `check_yaml_joint_names.py --yaml lula/config/m0617_robot_description.yaml` 먼저 실행.**
   Isaac Sim 불필요, 순수 텍스트 검사. Lula의 robot_description.yaml 포맷은 `cspace:` 리스트가 URDF의
   `<joint name="...">`와 문자 그대로 일치해야 한다고 파일 포맷 자체에 명시돼 있음(NVIDIA 공식 예시로 확인).
   Robot Description Editor는 Stage에 올라온 USD Articulation을 보고 조인트 목록을 만드는데, 우리 USD DOF
   이름(`joint_1_joint`)이 URDF(`joint_1`)와 접미사가 달라서, Editor가 USD 쪽 이름을 그대로 써버리면
   `cspace: [joint_1_joint, ...]`가 되어 Lula가 URDF에서 못 찾고 로드 자체가 실패할 수 있음. 이 스크립트가
   그 불일치를 감지하고, 단순 접미사 차이면 `--fix`로 그 자리에서 고쳐줌(원본은 `.bak`로 백업).
4. `test_lula_fk.py` / `test_lula_ik.py` — Lula 단독 (USD 없이 FK/IK). 3번 PASS 이후에 의미 있음.
5. `test_usd_lula_mapping.py` — 최종, 둘을 실제로 비교

## Known Issues
- `config/m0617_robot_description.yaml` 배치 완료, cspace 이름 검증 PASS — 실제 Lula 로드/FK/IK는 Isaac Sim에서 확인 필요(아래 "완료됨" 참고)
- `tool0`, 반지름 0 문제는 원인 특정 완료(아래 1, 2번)
- yaml 생성 시 `cspace:` 조인트명이 URDF(`joint_1`)가 아니라 USD DOF명(`joint_1_joint`)으로 나올 가능성 —
  yaml 만들면 `check_yaml_joint_names.py`로 제일 먼저 확인할 것 (아래 4번)

## 확인된 사실 (원본 파일을 직접 읽어서 확인함)

1. **"tool0" 문제 원인 확정.** `m0617.urdf` 맨 끝을 보면:
   ```xml
   <!-- <link name="tool0"/>
   <joint name="joint_6-tool0" type="fixed">
     <origin rpy="3.1415926535 -1.570796327 0" xyz="0 0 0"/>
     <parent link="link_6"/>
     <child link="tool0"/>
   </joint> -->
   ```
   `tool0` 링크/조인트가 **통째로 주석 처리**돼 있음 -- 즉 공식 URDF에 `tool0`는 실제로 존재하지
   않는다. `end_effector_name="tool0"`로 넣으면 100% 실패하는 게 맞았음. `link_6`을 쓰는 게 정답
   (ik_bridge.py에 이미 반영됨). 필요하면 이 주석을 해제해서 `tool0`를 살려 써도 되는데, 그러면
   `origin rpy="180°, -90°, 0"` 회전이 끼기 때문에 오히려 좌표 변환이 하나 더 늘어남 -- 굳이 살릴
   이유는 없어 보임.

2. **"반지름 0" 문제의 유력 원인 특정.** 6축 링크(`link_1`~`link_6`, `base_link`)는 전부
   `<collision>` mesh가 있는데, 유일하게 `<link name="base"/>` 하나만 collision은커녕 visual도
   inertial도 없는 순수 참조 프레임이다(`base_link` 위 z=0.450 지점, 아마 컨트롤러가 쓰는 "base"
   좌표계 기준점). Lula XRDF Editor가 이 링크에 자동으로 충돌 구를 씌우려다 반지름 0으로 떨어졌을
   가능성이 가장 크다. **XRDF Editor에서 `base` 링크를 collision sphere 목록에서 제외하거나,
   수동으로 반지름을 양수(예: 0.02m)로 지정**하면 해결될 것으로 보임. 나머지 6개 링크는 전부
   정상적인 mesh collision이 있어서 문제 없을 것.

3. **하드웨어(URDF) 조인트 리미트는 v2 USD의 원래 값과 정확히 일치.** joint_1/2/4/5/6은 URDF에도
   `lower="-6.2832" upper="6.2832"`(=±360°, 기구적으로 진짜 무제한)로 돼 있고, joint_3만
   `lower="-2.8798" upper="2.8798"`(=±165.0°)로 제한돼 있음 -- v3에서 적용한 ±95°/±145°/±135°
   (J2/J3/J5)는 URDF의 하드웨어 리미트가 아니라 **두산 세이프티 매뉴얼의 "Default"(공장 출하
   소프트웨어 안전값)** 라는 뜻. 둘 다 근거가 있는 값이지만 레이어가 다르다는 걸 구분해서 알아두면
   됨 -- 시뮬레이션 안전 클램프 목적으로는 Default 값이 맞는 선택.

4. **yaml의 `cspace:` 조인트명은 USD가 아니라 URDF 이름이어야 한다는 게 Lula 포맷 자체 사양.**
   NVIDIA 공식 robot_description.yaml 예시(Franka)를 직접 확인함 — 파일 안 주석에 "각 cspace 좌표는
   cspace_urdf_bridge에 별도 지정된 경우가 아니면 URDF에 그 이름의 entry가 있다고 가정한다"고 명시돼
   있고, 실제 예시도 `cspace: [panda_joint1, ...]`처럼 URDF 조인트명을 그대로 씀. Robot Description
   Editor는 Stage의 USD Articulation을 보고 조인트 목록을 만드는데, 우리 USD DOF명(`joint_1_joint`)은
   URDF(`joint_1`)와 접미사가 다르므로, Editor가 USD 쪽 이름을 그대로 cspace에 쓰면 이후
   `LulaKinematicsSolver(robot_description_path=yaml, urdf_path=urdf)`가 URDF에서 그 이름을 못 찾아
   로드부터 실패할 수 있음. `cspace_to_urdf_rules`는 "cspace에 없는 URDF 조인트를 고정값으로 처리"하는
   용도라 이 이름 불일치를 대신 해결해주지 않음 -- 즉 자동으로 풀리는 문제가 아님. `check_yaml_joint_names.py`로
   yaml 생성 직후 바로 확인하고, 단순 접미사 차이면 `--fix`로 고칠 것(가짜 yaml로 감지/자동수정/정상케이스
   3가지 다 테스트해서 스크립트 자체는 동작 확인됨).

## 완료됨 -- robot_description.yaml 반입
`config/m0617_robot_description.yaml`이 실제로 배치돼 있음. XRDF Editor로 생성된 원본은
`cspace: [joint_1_joint, ...]`(USD DOF명)이라 위 4번 문제 그대로 재현됐었고, 이후 `cspace: [joint_1, ...]`로
수정된 버전을 받아 `check_yaml_joint_names.py`로 URDF와 완전히 일치함을 확인 후 이 경로에 배치함.
`collision_spheres:`/`cspace_to_urdf_rules:`는 비어있는데(아래 문단 참고) 예상된 상태, `default_q`는
전부 0 근처라 J2/J5 안전범위 안. **단, 이건 텍스트 레벨 검증까지 -- Lula가 실제로 이 yaml+URDF를 로드해서
FK/IK를 계산하는지는 Isaac Sim에서 `test_lula_fk.py`/`test_lula_ik.py`/`test_usd_lula_mapping.py`를
직접 돌려야 확인됨(이 환경엔 Isaac Sim이 없어서 대신할 수 없음).**

**collision sphere는 현재 목표(Lula FK/IK)에는 필수 입력이 아님, 단 "완전히 불필요"도 아님.**
LulaKinematicsSolver 자체는 world collision avoidance를 하지 않으므로, 지금 단계(팔이 목표
pose에 도달하는 관절각을 계산하는 것)에는 collision sphere 없이 export해도 된다. 다만 이후
RMPflow 같은 collision-aware motion generation을 붙일 계획이면 그때는 별도로 collision
representation을 구성해야 한다 — "무조건 불필요"가 아니라 "지금 단계의 목적에는 핵심 아님"이 정확한 표현.
