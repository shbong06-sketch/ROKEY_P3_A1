# 올인원(수확 → Nav2 → 컨베이어 → 비전 검사 → 솎아내기) 디버깅 · 수정 내역 (2026-09-25)

브랜치 `feature/cabbage-place-fix` · Isaac Sim 5.1 · Windows 11 + RTX 3060 12GB + WSL2 Ubuntu 24.04 (ROS 2 Jazzy Nav2)

> **저장소 정리 (2026-09-25 저녁):** 브랜치에는 최종 올인원 코드만 남겼다 —
> `runtime/standalone_app.py`, `scripts/{conveyor, conveyor_rollers, cull_motion, inspection_cull_station}.py`,
> `tools/cabbage/make_cabbage_scene.py`. (YOLO 워커는 inspection_cull_station.py 에 합침 — 같은 파일을 `--yolo-worker` 로 띄운다,
> 06+run_in_isaac 은 make_cabbage_scene.py 로 합침.) 양배추 씬이 scenes/Collected_smartfarm_v013 에 있으면 올인원이 기본으로 연다. 양배추 에셋은 공유 zip 으로 전달.
> 아래 표에 나오는 시험·녹화·Windows/WSL 도구(01~05, 07~13, aio_wrapper, run_*.ps1, wsl/*)는 커밋 `ee1043d` 에 있다.
테스트 횟수는 이 PC 의 실행 기록(`D:\smartfarm-sim\out\*` 폴더·로그) 기준이다. 오전 에셋 제작 단계는 스크립트 기록 기준의 대략값.

---

## 0. 코드가 연결되는 방법 (전체 그림)

```
run_allinone_live.ps1 (Windows, 한 번에 실행)
 ├─ WSL: wsl/start_nav2_stack.sh
 │    ├─ wsl/ros_tcp_relay.py (서버)  ← DDS 가 WSL 경계를 못 넘어서 TCP 로 토픽 중계
 │    ├─ nav2.launch (팀 smart_farm_navigation) + RViz2
 │    └─ navigation_node + feeder_dock (standoff_m 0.92 파라미터로 기동)
 ├─ Windows: run_isaac_for_nav2.ps1
 │    ├─ ros_tcp_relay.py (클라이언트)
 │    └─ isaacsim\python.bat aio_wrapper.py runtime/standalone_app.py --scene <양배추 씬> --autoplay
 │         aio_wrapper: 포기·트레이 감시(monitor.json) + 공정 카메라 녹화(captures/) + 뷰 저장 트리거
 └─ WSL: wsl/run_flow.py → /sim_task/command (PICK_HARVEST → NAVIGATION FEEDER_DOCK → PLACE_INSPECT)

standalone_app.py (팀 올인원, [올인원 2026-09-25] 표시가 이번 변경)
 create_simulation_runtime()
   ├─ conveyor.install(stage, 팔레트들, vision_x=-0.69, auto_resume=None)   ← world.reset() 전
   ├─ inspection_cull_station.install(stage, world, M0609_DIR)             ← world.reset() 전 (이송 프레임 생성)
   ├─ world.reset()
   └─ conveyor.attach() · station.attach(conveyor)
 run() 루프
   ├─ SimTaskNode 명령 → robot_motion (PICK_HARVEST / PLACE_INSPECT, 팀 코드)
   │     PLACE 시작: conveyor.hold_stem(True) · 끝: hold_stem(False)
   └─ step_world(): world.step → conveyor.update(dt) → station.update(dt)

inspection_cull_station.VisionCullStation.update()  (상태 기계)
   IDLE → PUSH_IN → MOVE_INSPECT → CAPTURE → CULL ⇄ HOME → RECHECK_MOVE → RECHECK → PUSH_OUT → IDLE
   ├─ 이송 프레임: build_transfer_frame() 이 만든 kinematic 틀을 _step_moves() 로 움직임
   ├─ 팔: cull_motion.CullMotion (팀 파일 무수정) + m0609_rmpflow_controller.RMPFlowController
   ├─ 비전: replicator rgb (손목 RealSense) → inspection_cull_station.py --yolo-worker (같은 파일을 하위 프로세스로, best.pt)
   └─ 끝: conveyor.inspection_done() → 컨베이어가 트레이를 다시 반송
```

씬 준비는 별도: `tools/cabbage/make_cabbage_scene.py` (옛 이름 06_make_cabbage_scene.py) 가 원본 v013 씬에서 `<씬>_cabbage.usd` 를 만든다 (원본 그대로).

---

## 1. 디버깅 목록 (파트별)

표 열: **문제 / 원인 / 원인 찾기까지 테스트 횟수 / 해결 / 사용한 코드 / 코드 연결**

### 1-1. 양배추 에셋

| # | 문제 | 원인 | 테스트 | 해결 | 코드 | 연결 |
|---|---|---|---|---|---|---|
| A1 | 만든 .usd 를 Isaac 이 못 읽음 / pxr import 불가 | Isaac 5.1 USD(24.05) ≠ pip usd-core(26.08) 이진 포맷 | 약 3 | 계산은 pip 파이썬, 교환은 .usda, 이진 저장은 Isaac 파이썬(`run_in_isaac.py`) | `02_build_cabbage_pallet.py`, `03_finalize_isaac.py`, `run_in_isaac.py` | 02 → .usda → 03(Isaac) → .usd |
| A2 | RG2 가 양배추를 못 감쌈 | 첫 모델 폭 112 mm, RG2 완전 개방 패드 간격 99.6 mm(실측) | 2 (측정 1 + 재모델 1) | 머리 배율 0.86 → 0.60 (폭 85~90 mm, 밴드 78 mm) | `01_model_cabbage_blender.py` HEAD_SCALE | 01 → 02 |
| A3 | 프로토타입이 class 가 아니라 def | CreateClassPrim + Define 조합 | 1 | `_prototypes` Scope + SetSpecifier(Class) | `02` | 인스턴스 참조 |
| A4 | Asset Validator 경고 | UsdUVTexture 출력 타입, indexed primvar | 2 | Float3 출력, st indexed | `02`, `03` | 03 검사 |
| A5 | RG2 로 잡으면 손가락이 포기를 관통 | 팀 그리퍼 힘 1e4 N·m(손끝 수백 kN) + 접촉 여유 0 | 약 15 (05 변형 시험) | 포기 contactOffset 4 mm, 위치 반복 64, 그리퍼 최대 힘 8 N·m → 6칸 6/6 | `02` HEAD_CONTACT_OFFSET/HEAD_POS_ITERS, `05`, 스테이션 GRIPPER_DRIVE | 에셋 물성 + 스테이션 install() |
| A6 | (확인) 색 변화가 비전에 통하는가 | — | 3 (romaine / 단색 / 자연색) | 원래 두 톤 모습 + 노랑·갈색은 평균색만: YOLO 95.7 % | `02` CONDITIONS, `08_yolo_color_check.py` | variant `condition` |

### 1-2. 씬 교체 · 배치

| # | 문제 | 원인 | 테스트 | 해결 | 코드 | 연결 |
|---|---|---|---|---|---|---|
| S1 | 양배추로 바꾸면 로메인과 겹쳐 폭발 | 서브레이어 편집은 루트 레이어의 prepend 참조보다 약함 → 두 참조가 합쳐짐 | 2 | 루트 레이어 복사본에서 참조 교체 | `06_make_cabbage_scene.py` | 원본 씬 → `_cabbage.usd` |
| S2 | Play 누르면 양배추가 튀어나옴 | 컨베이어 옆가이드가 벨트 위 시험 트레이(Pallet_Inspect*)를 관통한 채 생성 | 1 | 처음엔 벨트 밖으로 옮김 → 최종: 시험 트레이 삭제 | `06` DELETE, `standalone_app` CONVEYOR_PARK(없으면 건너뜀) | 06 → 씬 |
| S3 | 씬 열면 옛 트레이가 몇 초 보였다 사라짐 | 위 트레이를 실행 시 옮기기 전까지 보임 | 1 | 삭제(Pallet_Inspect/_01/_02/_03, Lettuce_1~3) | `06` | 씬 |
| S4 | SortBox 에 버린 포기가 뚜껑 위에 얹힘 | SortBox 가 속이 찬 큐브 | 1 | 같은 크기·색의 윗면 열린 통 SortBin_1/2 | `06` scene_fix | 스테이션이 SortBin 경로 사용 |
| S5 | 통 바닥 아래로 포기가 빠짐 | 바닥 두께 2 cm, 먼저 버린 포기 위로 떨어질 때 뚫림 | 1 | 바닥 5 cm | `06` | 씬 |
| S6 | 비전룸 벽 통로가 "뻐큐" 모양 | 통로 윗부분 메시(BackWall_03/04_Head, 16점)에 가운데 홈 | 3 (렌더 확인) | 같은 자리 평판(충돌 포함)으로 교체 | `06` | 씬 |
| S7 | 씬 재생성 실패 (파일 잠김) | Isaac GUI 가 씬을 열고 있음 | 2 | 재생성 전 Isaac 종료 (`run_allinone_live.ps1` 도 시작 시 정리) | — | — |
| S8 | 06 이 오류 없이 끝남 | 삭제 API 이름 오류(`RemoveNameChild`), Kit 종료 때 stderr 가 사라짐 | 3 | `del parent.nameChildren[...]` + 예외를 `<스크립트>.error.txt` 로 | `06`, `run_in_isaac.py` | — |

### 1-3. PLACE_INSPECT (포크 로봇이 컨베이어 줄기에 내려놓기)

| # | 문제 | 원인 | 테스트 | 해결 | 코드 | 연결 |
|---|---|---|---|---|---|---|
| P1 | 로메인 원본에서도 PLACE 실패 (IK 110° 점프) | ① 놓는 방향이 TurnTable +x 기준이라 도킹 방향과 90° 어긋남 ② TurnTable 쿼터니언 비정규화(0.507) ③ 높이 25.9 mm 낮음 ④ 포크판이 롤러 사이에 걸려 EXIT 에서 팔레트가 따라 올라옴 | 5 (placefix ×4 + romaine) + 스냅샷 분석 | `turntable_place_pose()`: 놓을 자리→팔 베이스 방향, 정규화, 벨트 윗면+25.9 mm, 포크판 35 mm 밖 | `standalone_app.py` | robot_motion PLACE_STAGES 목표 |
| P2 | 놓는 동안 컨베이어가 팔레트를 끌고 감 | 벨트는 계속 돎 → "포크 인출 중 팔레트 따라옴" 검사 걸림 | 1 | 인터록 `hold_stem(True/False)` | `conveyor.py`, `standalone_app.py` | PLACE 시작·끝 |
| P3 | DESCEND_5 관절 20.7° (한계 20°) 실패 | 도킹 거리 0.85 m 에서 팔 베이스→놓을 자리 0.72 m | 2 | feeder_dock standoff_m 0.92 (파라미터) | `wsl/start_nav2_stack.sh` | navigation_node 기동 인자 |

### 1-4. Nav2 (Windows 한 대 + WSL)

| # | 문제 | 원인 | 테스트 | 해결 | 코드 | 연결 |
|---|---|---|---|---|---|---|
| N1 | headless 올인원에서 odom/clock 없음 | headless 에서 ROS OmniGraph 가 돌지 않음 | 2 | GUI 모드 | `run_isaac_for_nav2.ps1` | — |
| N2 | Windows↔WSL 토픽이 안 옴 | mirrored 모드에서 늦게 뜬 노드 데이터 유실(DDS), 방화벽 | 약 6 (프로파일 시도 포함) | NAT 모드 + TCP 중계(`ros_tcp_relay.py`), 도메인 101/102 분리 | `wsl/ros_tcp_relay.py`, `.wslconfig` | 양쪽 중계기 |
| N3 | v011 씬에 /clock 없음 | 씬에 clock 그래프 없음 | 1 | v013 사용 | — | — |
| N4 | Nav2 bringup 중단 | 이전 WSL 프로세스 잔존 | 2 | 실행 전 `wsl --terminate` | `run_allinone_live.ps1` | — |

### 1-5. 컨베이어

| # | 문제 | 원인 | 테스트 | 해결 | 코드 | 연결 |
|---|---|---|---|---|---|---|
| C1 | 로봇이 든 트레이가 벨트 위에서 돌며 가이드에 끼임 | 운반 중 팔이 트레이를 돌리므로 처음부터 요 고정 불가 | 1 | `carried_paths`: 벨트에 올라온 순간 요 고정 | `conveyor.py` | install() |
| C2 | 트레이가 교차점 출구(x -1.424)에서 멈춤 (CONVEYOR_FAILED) | 교차점 프레임 부품 `SM_ConveyorBelt_A49_01` 충돌면이 롤러 윗면보다 2 mm 높아, 휠이 내려가면 트레이 뒤 받침이 걸침 | **14** (올인원 2 + 단독 12: 푸셔·벽·통·카메라·리프트 끄기 → 원본 씬 비교 → 접촉 보고 → 마찰 0 → 충돌 끔) | 그 프레임 부품 충돌 끔 (트레이는 롤러·휠이 받침) | `06` scene_fix, `09 --contacts` | 씬 |

### 1-6. 비전 검사 · 솎아내기 스테이션 (새로 만든 부분)

| # | 문제 | 원인 | 테스트 | 해결 | 코드 | 연결 |
|---|---|---|---|---|---|---|
| V1 | 올인원에서 검사·솎아내기가 빠져 있음 | 컨베이어는 10 초 뒤 자동 배출만 있었음 | — | 스테이션 신규 + `inspection_done()` 으로 재가동 | `inspection_cull_station.py`, `standalone_app.py` | step_world |
| V2 | 비전 로봇 팔이 트레이에 닿지 않음 | 줄(y -6.73)이 base 에서 0.89 m, 위에서 집기+접근 18 cm 는 0.6 m 까지(트레이 중심 0.48 m) | 1 (IK 격자 14 조합) | 받침대 +0.146 m + 트레이를 로봇 쪽으로 옮김(y -7.00) | `11_vision_arm_reach_check.py`, `06`, 스테이션 | — |
| V3 | 옮긴 뒤 포기가 칸에서 빠짐 | ① kinematic 트레이를 USD 로 밀면 순간이동 ② 멈춘 뒤 USD 자세가 갱신 안 됨 ③ 먼 줄 포기가 보이지 않는 남쪽 가이드와 겹쳐 밀려남 | 7 (station_test1~7) | 검사 중 남쪽 가이드 충돌 끔 + (최종) 이송 프레임이 물리로 밀기 | 스테이션 `_set_guide`, `build_transfer_frame` | — |
| V4 | 이송이 순간이동처럼 보임 / 뒤 푸셔가 롤러를 뚫음 | 텐서로 옮기는 방식은 화면 갱신이 안 됨 / 롤러 아래에서 올라오는 판 | 3 (push1, push2, frame_test1) | 트레이를 감싸는 사각 이송 프레임(PlateN 주황·PlateS 빨강), 롤러 위·레일 안쪽에서만 움직임 | `build_transfer_frame`, `_step_moves` | PUSH_IN / PUSH_OUT |
| V5 | 첫 포기를 옮기다 떨어뜨림 | 팀 계획은 LIFT→PLACE 직선이라 base 위를 지나며 팔이 꺾임 | 1 | base 둘레 중간점 VIA + 포기마다 검사 자세 복귀(HOME) | `_via_point`, `_state_home` | CULL 계획 |
| V6 | 같은 통 두 번째 포기가 굴러 나감 | 같은 자리에 20 cm 위에서 떨어뜨림 | 1 | 긴 변을 따라 자리 바꿈(DROP_SPREAD), 10 cm 위에서 놓음 | 스테이션 상수 | — |
| V7 | 먼 줄 포기 놓침 | 비전 좌표 오차 2~10 mm 가 RG2 여유(한쪽 10.8 mm)에 가까움 | 2 | 집는 좌표 = 트레이 자세 + 칸 배치, 비전 좌표는 대조 기록 | `_next_cull` | — |
| V8 | 검사 사진에 손가락이 가림 | 카메라가 공구축과 같은 방향, 45 mm 옆 | 1 (마운트 측정) | 팔 쪽 위에서 비스듬히 보는 검사 자세 | `_inspect_pose` | — |
| V9 | 녹화가 초당 1장 | 캡처 코드가 15 스텝 감시 조건 뒤에 있음 | 1 | 캡처를 앞으로 | `aio_wrapper.py` | — |

### 1-7. 카터 + M0609 리프트

| # | 문제 | 원인 | 테스트 | 해결 | 코드 | 연결 |
|---|---|---|---|---|---|---|
| L1 | 팔이 리프트 프레임(기둥·윗가로대)을 뚫고 지나감 | 프레임 꼭대기 1.48 m 가 팔 베이스(1.04 m)보다 0.44 m 높음, 같은 articulation 이라 충돌 없이 통과 | 3 (구조 분석 2 + 렌더 1) | 텔레스코픽: 바깥 기둥 0.78 m 로 낮춤 + 리프트 판에 안쪽 기둥 0.63 m | `06` telescopic_mast | 씬 |

---

## 2. 수정사항 목록

### 2-1. 이송 프레임(푸셔) — 신규, 실행 시 생성 (`inspection_cull_station.build_transfer_frame`)

| 부품 | 치수 / 위치 | 물리 |
|---|---|---|
| Frame (루트) | x = 트레이 x, 대기 y -6.73, 대기 높이 0.965 / 내림 0.805 | kinematic 강체, 8 kg |
| PlateN (주황) | 0.616 × 0.02 × 0.06, 트레이 북쪽 면 12 mm 밖 | 충돌 — 로봇 쪽으로 밀기 |
| PlateS (빨강) | 같은 크기, 트레이 남쪽 면 12 mm 밖 | 충돌 — 벨트로 되밀기, 작업 중 멈춤판 |
| ArmW / ArmE | 0.02 × 0.276 × 0.06, 트레이 양 끝 | 충돌 |
| Rod0 / Rod1 | 0.03 × 0.86, PlateN → 북쪽 | 보이기만 |
| Carriage | 0.60 × 0.08 × 0.08, y -6.05, 프레임과 같은 높이 | 보이기만 |
| Column0 / 1 | 0.06 × 1.20, x ±0.33, y -6.05 | 보이기만 |

- 컨베이어를 뚫지 않게: 판 바닥 0.775 > 롤러 윗면 0.769, 판은 레일(y -7.19 / -6.31) 안쪽, 로드 바닥 0.835 > 레일 윗면 0.798, 기둥은 컨베이어 바깥 끝(-6.175) 북쪽, 로드 끝은 북쪽 벽(-5.70) 전.
- 동작: 올린 채 트레이 위로 → 내림 → PlateN 이 y -7.00 까지 밀기 → (검사·솎아내기) → PlateS 가 줄로 되밀기 → 올림. 속도 0.08 m/s (화면에서 보이게).
- 검사 중에만 컨베이어 남쪽 옆가이드 충돌을 끔 (트레이가 가이드 자리를 지나감).

### 2-2. Nav2

| 항목 | 내용 |
|---|---|
| 팀 navigation 코드 | **변경 없음** |
| feeder_dock | `standoff_m` 0.85 → **0.92** (기동 파라미터, 가이드 허용 0.78~0.94) |
| 한 PC 실행 | WSL NAT 모드 + `ros_tcp_relay.py` (Windows 도메인 101, WSL 102), 방화벽 규칙 |
| 스크립트 | `wsl/start_nav2_stack.sh`, `wsl/run_flow.py`, `run_allinone_live.ps1` |
| 씬 | v013 (use_sim_time 용 /clock 있음) |
| 남은 과제 | 바닥 선 따라가기 (경유점 방식 반나절 / FollowPath 1~2일) |

### 2-3. M0609 + 노바 카터 리프트 에셋 (씬 복사본에서만)

| 항목 | 내용 |
|---|---|
| lift_holder 메시 | z 0.95 위 정점 96개를 0.70 m 내림 → 기둥 꼭대기·윗가로대 1.48 → 0.78 m (충돌 모양도 같이 바뀜) |
| 안쪽 기둥 | `lift_moveparts_1/MastInner_0,1` (0.63 m, 리프트 판과 같이 움직임, 보이기만) |
| 그대로 | 로봇 위치, 리프트 행정 0 ~ 0.61 m, 팀 모션 코드 |
| 확인 필요 | lift_holder 질량을 충돌 부피로 계산한다면 질량이 줄었을 수 있음 → 운반 흔들림 확인 권장 |
| 경량화 | 카터 몸체 속 부품 `chassis_link/visual/internal_components`(152만 점, 원래 비표시·충돌체 없음) 비활성. 파일 `SubUSDs/nova_carter_sim_optimized.usd` 에서도 제거한 경량본 193 → 57 MB (`tools/cabbage/12_slim_carter_layer.py`). 화면 도형 390만 점 그대로 |

### 2-5. 씬 경량화 조사 (공유용)

| 대상 | 결과 |
|---|---|
| 보이지 않고 기능도 없는 요소 (씬에 불러오는 것) | 520 점 1개(RealSense 케이스 안쪽) — 사실상 없음 |
| 보이지 않지만 충돌체 (M0609 팔 충돌 메시 등, 37.7만 점) | 물리에 필요 → 유지 |
| 파일 안에 있으나 씬에 안 나오는 도형 | 카터 파일 69 %(내부 부품) → 경량본으로 제거. 나머지 큰 파일(컨베이어 A08/A49/A05, RG2, 바퀴 …)은 100 % 사용 |
| 무게의 실제 원인 | 보이는 도형: 컨베이어 138만 점, 카터+포크 로봇 176만 점, 비전 M0609 64만 점(그중 RG2 앵글 브래킷 22만 점) — 더 줄이려면 보이는 도형을 단순화해야 함 (모양 변화 있음, 별도 판단) |

### 2-4. 양배추 에셋 (이번 최종 vs 색 작업 전)

| | 색 작업 전 | 최종 |
|---|---|---|
| 모양 | A/B/C 3종 | 3종 × green/yellow/brown = 9 프로토타입 |
| 색 전환 | 없음 | 포기마다 variant `condition` |
| 라벨 | class | class + `condition` + `farm:condition` |
| 노랑·갈색 | — | 같은 텍스처에 평균색만 (UsdUVTexture scale) |
| 파일 | pallet_6, empty, A/B/C | + `cabbage_pallet_6_inspect.usd` |
| 같음 | 폭 85~90 mm, 밴드 78 mm, 0.3 kg, 마찰 0.8/0.6, contactOffset 4 mm, 반복 64/4, 충돌체 32/48/32/17 점, 트레이 상자 26 + 쐐기 96 | 같음 |

---

## 3. 모션

### 3-1. 소요시간 (최종 GUI 실행 `out/allinone_final2`, 시뮬레이션 시간 = 실물 동작 시간 기준)

Pallet_01 (G Y B G Y G → 노랑 2 + 갈색 1 제거) 한 장. 녹화(카메라 5대)를 켜서 실제 벽시계로는 약 2~3 배 걸렸다.

| 구간 | 시작 → 끝 (s) | 소요 | 세부 |
|---|---|---|---|
| PICK_HARVEST | 2 → 74 | **72 s** | 포크가 랙 팔레트를 드는 순간 38 s, 이후 리프트 적재 상승(0.02 m/s)·운반 자세 회전 |
| NAVIGATION (Nav2 + 도킹) | 76 → 113 | **37 s** | 랙 앞 → 컨베이어 줄기 앞 도킹 |
| PLACE_INSPECT | 113 → 122 | **9 s** | 팔레트가 줄기 벨트에 놓임 (인출 포함 약 12 s) |
| 컨베이어 반송 | 122 → 145 | **23 s** | 줄기 → 교차점 → 비전룸 정지 |
| 이송 프레임 밀기 | 145 → 151 | **6 s** | 내림 + 0.27 m 밀기 (0.08 m/s) |
| 검사 | 151 → 156 | **5 s** | 검사 자세 이동 + 3 프레임 YOLO |
| 솎아내기 3포기 | 156 → 248 | **92 s** (포기당 28~33 s) | 집기·통 이동·버리기·검사 자세 복귀 |
| 재검사 | 248 → 255 | **7 s** | |
| 되밀기 · 프레임 올림 | 255 → 261 | **6 s** | 이후 배출 |
| **합계** | | **약 4 분 19 초** | 팔레트 1장, 불량 3포기 |

### 3-2. 실물과 비교한 개선점 · 속도 개선 가능성

| 모션 | 실물 대비 필요한 개선 | 더 빠르게 |
|---|---|---|
| PICK_HARVEST (팀 robot_motion) | 리프트 속도 0.05/0.02 m/s 는 실제 리프트보다 느림, 포크 진입 전 정렬 확인 없음 | 적재 상승 0.02→0.05 m/s, 팔 이동 보간 단축 → 약 30~40 % 단축 예상 |
| NAVIGATION (Nav2 + feeder_dock) | 선 따라가기 없음(대각선 주행), 도킹 마지막 저속 구간이 김 | max_vel 상향(Nova Carter 실측 한계 내), 도킹 전 감속 구간 단축 |
| PLACE_INSPECT | 벨트 인터록은 시뮬 신호 — 실물은 PLC 인터록 필요 | DESCEND/EXIT 단계 속도 상향 |
| 컨베이어 반송 | 0.30 m/s (실물 범위) | 0.4~0.5 m/s 가능 (교차점 전환 시 포기 흔들림 6.6 mm 확인 필요) |
| 이송 프레임 | 실물은 공압/전동 액추에이터 — 가이드 레일·센서 필요 | 0.08 → 0.25 m/s (화면용으로 느리게 둔 값) |
| 검사 | 손목 카메라라 검사 자세로 팔이 움직여야 함 | 천장 고정 카메라면 팔 이동 없이 즉시 촬영 (약 5 s 단축) |
| 솎아내기 (포기당) | 그리퍼 힘 8 N·m 는 시뮬 값 — 실제 RG2 파지력(최대 40 N)으로 확인 필요 | TCP 0.24→0.5 m/s, 열기·닫기 1.5·3 s → 0.7·1 s, HOME 복귀 생략(다음 포기로 바로) → 포기당 약 30 s → 12~15 s |
| 재검사 · 되밀기 | — | 재검사 생략 옵션, 프레임 속도 상향 |

### 3-3. 기존(팀) 모션 코드와 달라진 점

| 모션 | 기존 | 이번 |
|---|---|---|
| PICK_HARVEST | 팀 `robot_motion` | **변경 없음** |
| NAVIGATION | 팀 navigation_node / feeder_dock | 코드 변경 없음, standoff 파라미터만 0.92 |
| PLACE_INSPECT | `turntable_place_pose` 가 TurnTable +x 기준, 쿼터니언 비정규화, 높이 낮음 | 놓을 자리→팔 베이스 방향, 정규화, 벨트+25.9 mm, 포크판 35 mm 밖 + 벨트 인터록 |
| 솎아내기 | `cull_motion.CullMotion` 은 고정 좌표 **Pick 만** (OPEN→APPROACH→DESCEND→GRASP→LIFT→HOLD), 그리퍼 1e4 N·m | 같은 `CullMotion` 실행기(파일 무수정)에 **Pick→Place 전체 계획**(팀 `build_cull_plan`) + 중간점 VIA + 포기마다 HOME, 좌표는 트레이 자세+칸(비전 대조), 그리퍼 8 N·m, 버리는 곳 SortBin 1/2 번갈아 |
| 검사 | 팀 inspection 노드(ROS, 별도 PC) | Isaac 안에서 같은 best.pt 를 하위 프로세스로 (ROS 토픽 없이) — 팀 노드로 바꾸려면 `_state_capture` 만 교체 |
| 컨베이어 | 10 s 뒤 자동 배출 | 스테이션 완료 시 `inspection_done()` |
