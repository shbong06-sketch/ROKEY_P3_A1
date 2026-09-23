# 검사(Inspection) 파트 인계 문서

작성일: 2026-09-23 · 대상: 검사 비전 + Isaac Sim 연동 실행 인수자

## 0. 요약

- 검사 파이프라인은 세 덩어리다. **Isaac Sim**(카메라 `/rgb` 발행) → **Inspection Node**(Docker, YOLO, `/inspection/*`) → **Sim Task Executor**(Isaac 안, 픽셀→로봇 base 좌표).
- 기존 코드는 `feature/Inspection-Place` 브랜치(zip 기준)에, 이번 수정은 `feature/inspection-v011-verify` 브랜치(커밋 `9a37ddb` + 후속 커밋 예정)에 있다. **두 브랜치는 아직 합쳐지지 않았다.**
- 이번 수정의 목적: Docker Isaac Sim 5.1.0 에서 검사 조건을 재현해 `/rgb` 를 내보내고, 검출 결과를 base 좌표로 바꿔 보는 **시험용 실행기**를 만드는 것. 본 공정의 Executor(`standalone_app.py`)를 대체하지 않는다.
- 실제 Isaac Sim 에서 끝까지 돌려 본 적은 아직 없다. 수식·형식은 오프라인 검증만 했다(7장).

## 1. 브랜치 · 커밋 현황

| 브랜치 | 상태 | 검사 관련 핵심 내용 |
| --- | --- | --- |
| `feature/Inspection-Place` | 기존 코드 (zip 로 받음) | Inspection Node Docker, 인터페이스, Sim Task Executor(`standalone_app.py`), `camera_to_robot_base.py` |
| `feature/inspection-v011-verify` | `bcdd382` ROI 보정·검증 도구 → `b8c4602` pixel-to-base 브리지 → **`9a37ddb` 이번 수정** | `runtime/vision_functest.py`, `vision_bridge.py`, `run_isaac_docker.sh` |

두 브랜치 차이는 아래로 확인한다 (원격 이름은 `git branch -r` 로 확인).

```bash
git fetch origin
git log --oneline origin/feature/Inspection-Place..origin/feature/inspection-v011-verify
git diff --stat origin/feature/Inspection-Place origin/feature/inspection-v011-verify \
  -- cobot3_ws/src/smart_farm_vision cobot3_ws/isaacpjt/smart_farm/runtime
```

## 2. 기존 코드 목록 (`feature/Inspection-Place`)

경로는 저장소 루트 기준.

| 구성 요소 | 위치 | 역할 | 실행 방식 |
| --- | --- | --- | --- |
| Inspection Node | `cobot3_ws/src/smart_farm_vision/` | YOLO 검사. `/inspection/command` 를 받아 fresh frame 1장 추론, 슬롯 판정 | Docker (`compose.vision.yaml`), 이미지 `smart-farm-vision:jazzy-cu124` |
| 검사 설정 | `smart_farm_vision/config/object_detection.yaml` | `/rgb`, `best.pt`, 슬롯 ROI 6개, class→판정 매핑 | 컨테이너 `/config` 로 read-only 마운트 |
| 모델 | `smart_farm_vision/resource/best.pt` | **git 에 없음** (`*.pt` 제외). 별도 전달 | 컨테이너 `/models` 로 마운트 |
| 인터페이스 | `cobot3_ws/src/smart_farm_interfaces/` | `TaskCommand`, `TaskResult`, `ExecutorStatus`, `CycleStatus`, `StartCycle` | 비전 이미지 빌드 시 함께 빌드 |
| Sim Task Executor | `cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py` | Isaac 안에서 `/sim_task/*` JSON 처리. TRANSFER · PICK_HARVEST · PLACE_INSPECT | 네이티브 `~/isaacsim/python.sh runtime/standalone_app.py --autoplay` |
| ROS 어댑터 | `runtime/sim_task_node.py` | `/sim_task/command` 검증·큐잉, result 캐시, status heartbeat | standalone_app 에서 사용 |
| 좌표 변환 | `runtime/camera_to_robot_base.py` (+ `tests/test_camera_to_robot_base.py`) | detections_2d + Isaac Camera depth → base 좌표 | **아직 standalone_app 에 연결되지 않음** |
| Task Manager | `cobot3_ws/src/smart_farm_manager/` | 공정 상태기계, mock executor, fake/navigation 테스트 launch | ROS 2 launch |
| Navigation | `cobot3_ws/src/smart_farm_navigation/` | Nav2 연동, 스테이션 이동 | ROS 2 launch |
| 설계 문서 | `docs/01~03`, 통합 시험 문서 3종 | 아키텍처, 인터페이스 계약, Task Manager | — |

사용 씬: `standalone_app.py` 는 `scenes/Collected_smartfarm_v011/` (LiftRig: Nova Carter + 리프트 + `m0609_with_fork`) 를 기본으로 쓴다. **씬 USD 는 git 에 없다.**

## 3. 이번 수정 사항 (`feature/inspection-v011-verify`)

위치: `cobot3_ws/isaacpjt/smart_farm/runtime/`

### 3-1. `run_isaac_docker.sh` (신규)

- Isaac Sim 5.1.0 컨테이너(`nvcr.io/nvidia/isaac-sim:5.1.0`)로 `vision_functest.py` 실행.
- `--check`(사전 점검) · `--shell`(디버깅) · 나머지 인자는 functest 로 전달.
- 이미지 안 사용자·HOME 을 조회해 캐시 볼륨 마운트 (첫 실행 셰이더 컴파일 수 분).
- `smart_farm` 을 `/workspace/smart_farm`, `M0609` 를 `/workspace/M0609` 에 read-only 마운트 → 코드의 상대 경로 그대로 유지.
- `--network=host`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` (컨테이너·호스트 사용자 차이로 공유메모리 전송이 조용히 실패하는 문제 회피).

### 3-2. `vision_functest.py` (수정)

| 항목 | 변경 |
| --- | --- |
| 경로 | `FUNCTEST_SCENE`, `M0609_URDF`, `M0609_LULA_DESC` 환경변수로 덮어쓰기. 컨테이너면 `--headless` 강제 |
| ROS 환경 | 번들 lib 를 `LD_LIBRARY_PATH` 맨 앞, `/opt/ros` 제거, `--no-ros-env` 로 끄기 |
| 그래프 점검 | 재생 전 `/rgb` 그래프 노드·RenderProduct 카메라 대상 출력 (`[점검]`) |
| IK | Lula 관절 이름으로 매핑, **드라이브 목표(apply_action) 설정** (팔 관절만), 방위각 후보 ±25° 로 제한 |
| 검증 | settle 후 관절 오차, **FK↔USD 교차검증**, 거리·앙각·방위각·시선오차·roll 학습분포 판정 (`[검증]`), `--strict` |
| 스윕 | 트레이가 강체면 물리 API 로 이동 후 실제 위치 재측정. 숨긴 트레이는 kinematic |
| 브리지 | `--bridge`: `/inspection/detections_2d` → `/inspection/targets_3d` (String JSON, base 좌표). 실패해도 `/rgb` 발행은 계속 |

### 3-3. `vision_bridge.py` (수정)

- 카메라·base 행렬을 외부에서 받을 수 있음 (재생 중 USD 가 물리를 안 따라갈 때 대비).
- 깊이 없으면 시선–수평면 교점으로 근사. 32FC1/16UC1 디코딩.
- 같은 슬롯 중복 검출은 신뢰도 높은 쪽. 해상도 불일치는 예외. `pixel_to_base()` 추가.

### 3-4. 후속 수정 (이 문서와 함께 전달, **아직 커밋 전**)

`feature/Inspection-Place` 코드와 대조하다 발견해 고친 것.

1. `vision_bridge.extract_center` 가 계약 필드 `center_u`/`center_v`, `bbox_x_min~bbox_y_max` 를 못 읽던 문제 → 대응. (`--bridge` 가 모든 검출을 "중심 필드 없음"으로 버렸을 것)
2. 계약의 `header.stamp`, `header.frame_id`, `pallet_id` 를 읽도록 수정, targets_3d 에 pallet_id 포함.
3. `--bridge` 에서 번들 rclpy 경로(`exts/isaacsim.ros2.bridge/jazzy/rclpy`)를 `sys.path` 앞에 추가 — `standalone_app.py` 가 검증한 방식과 동일. 없으면 import 실패 가능성이 높았다.
4. `run_isaac_docker.sh --check` 에 번들 rclpy 폴더 확인 추가.

## 4. 검사 관련 토픽 (계약: `docs/02-interfaces.md`)

| 토픽 | 타입 | 방향 | 비고 |
| --- | --- | --- | --- |
| `/rgb` | `sensor_msgs/Image` | Isaac → Inspection | 1280×720. 씬의 ROS Camera 그래프가 발행 |
| `/inspection/command` | `TaskCommand` | Task Manager → Inspection | `operation=INSPECT`, `task_id`·`command_id`·`pallet_id` 필수 |
| `/inspection/result` | `TaskResult` | Inspection → Task Manager | `defect_slots`, `unknown_slots`. unknown 이 하나라도 있으면 FAILED/UNKNOWN_SLOT |
| `/inspection/status` | `ExecutorStatus` | Inspection → 관찰자 | STARTING / READY / BUSY, 약 1 Hz |
| `/inspection/detections_2d` | `String` JSON | Inspection → Sim Executor | `center_u/v`, `bbox_*`, `slot_id`, `class_name`. result 보다 먼저 발행 |
| `/inspection/debug_image` | `Image` | Inspection → 관찰자 | ROI·bbox·판정 오버레이 |
| `/inspection/targets_3d` | `String` JSON | functest `--bridge` → 관찰자 | **시험용 신규 토픽**, 계약에 없음 |

주의: Inspection Node 는 `continuous_inference: false` 라 **명령을 받아야만** 추론한다. `/rgb` 만 켜서는 detections 가 나오지 않는다.

## 5. 실행 절차 (인계용)

### 5-1. git 밖에서 따로 받아야 하는 것

| 파일 | 놓을 위치 |
| --- | --- |
| `best.pt` (클래스명 정확히 `lettuce_dark_green`, `lettuce_yellow`, `lettuce_brown`) | `cobot3_ws/src/smart_farm_vision/resource/best.pt` |
| 검사 씬 `Collected_smartfarm_v013` (폴더째) | `cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v013/` |
| Lula YAML `m0609_robot_description.yaml` (IK 모드만 필요) | 아무 곳 → `LULA_DIR=` 로 지정 |

### 5-2. 순서

터미널마다 같은 도메인을 쓴다.

```bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

1) 코드 받기

```bash
cd <저장소>
git fetch origin && git switch feature/inspection-v011-verify && git pull
```

2) Isaac 사전 점검 → 실행 (터미널 A)

```bash
cd cobot3_ws/isaacpjt/smart_farm
./runtime/run_isaac_docker.sh --check
./runtime/run_isaac_docker.sh --keep-pose --bridge
```

로그에서 `[준비] 재생 중` 이 나오면 다음 단계. 네이티브 Isaac 을 쓰면 `~/isaacsim/python.sh runtime/vision_functest.py --headless --keep-pose --bridge`.

3) `/rgb` 확인 (터미널 B, 호스트 ROS)

```bash
ros2 topic hz /rgb
```

4) Inspection Node 빌드·실행 (저장소 루트, 터미널 C). **Inspection Node 는 `feature/Inspection-Place` 쪽에 있다** — 현재 브랜치에 `smart_farm_vision` 과 `compose.vision.yaml` 이 있는지 먼저 확인하고, 없으면 두 브랜치 병합 후 진행.

```bash
docker compose -f compose.vision.yaml build vision
docker compose -f compose.vision.yaml up -d vision
docker compose -f compose.vision.yaml exec vision \
  /entrypoint.sh ros2 launch smart_farm_vision object_detection.launch.py \
  config_file:=/config/object_detection.yaml
```

5) 검사 명령 1회 발행 (터미널 D). `command_id` 는 **매번 새 값** (같은 값이면 캐시 결과만 재발행).

```bash
docker compose -f compose.vision.yaml exec vision /entrypoint.sh \
  ros2 topic pub --once -w 1 /inspection/command smart_farm_interfaces/msg/TaskCommand \
  "{task_id: 'TEST-001', command_id: 'TEST-001-CMD-001', operation: 'INSPECT', pallet_id: 'PALLET_001'}"
```

`-w 1` 은 구독자가 연결될 때까지 기다리는 옵션이다. 버전에 따라 없다는 오류가 나면 `--once -w 1` 대신 `-t 3 -r 1` 을 쓴다 (같은 command_id 재수신은 무시되거나 캐시 결과만 재발행되므로 안전).

6) 결과 관찰

```bash
docker compose -f compose.vision.yaml exec vision /entrypoint.sh ros2 topic echo /inspection/result
docker compose -f compose.vision.yaml exec vision /entrypoint.sh ros2 topic echo /inspection/detections_2d
ros2 topic echo /inspection/targets_3d          # functest --bridge 출력
```

7) 종료: 터미널 A `Ctrl+C`, `docker compose -f compose.vision.yaml down`.

## 6. 확인 체크리스트

| # | 확인 | PASS 기준 | 실패 시 |
| --- | --- | --- | --- |
| 1 | `--check` | "점검 통과" | 메시지대로 GPU·경로·마운트 수정 |
| 2 | Isaac 로그 `[점검]` | rgb 토픽 노드 표시 | 씬의 ROS Camera 그래프 확인 |
| 3 | `ros2 topic hz /rgb` | 수 Hz 이상 | list 에만 보이고 hz 0 → DDS 전송 문제 (7장 R1) |
| 4 | Isaac 로그 `[검증]` | 전 항목 PASS | WARN 항목과 값을 기록해 전달 |
| 5 | `/inspection/status` | READY | STARTING 고정 → 모델 경로·클래스명 |
| 6 | `/inspection/result` | SUCCEEDED, unknown 없음 | UNKNOWN_SLOT → ROI·클래스 매핑 확인 |
| 7 | `/inspection/targets_3d` | 슬롯별 `xyz_base` | `[브리지][경고]` 로그 전달 |

## 7. 알려진 위험 · 미결 사항 (우선순위 순)

| ID | 내용 | 영향 | 권장 조치 |
| --- | --- | --- | --- |
| R1 | `compose.vision.yaml` 은 `network_mode: host` 이지만 `ipc: host` 가 없다. Fast DDS 가 같은 호스트로 판단해 공유메모리를 쓰면 **토픽은 보이는데 데이터가 안 오는** 증상이 날 수 있다. | `/rgb` 미수신 → IMAGE_TIMEOUT | compose `environment` 에 `FASTDDS_BUILTIN_TRANSPORTS: "UDPv4"` 추가 검토 (호스트·Isaac 과 통일) |
| R2 | 검사 ROI: `Inspection-Place` 의 설정은 균등 3×2 격자. `v011-verify` 에는 ROI 보정 커밋(`bcdd382`)이 있다. | 병합 시 충돌, 잘못 고르면 슬롯 판정 오류 | 병합 때 보정본 채택, `/inspection/debug_image` 로 확인 |
| R3 | `lettuce_yellow` 판정: YAML 은 `UNKNOWN`, 코드 기본값은 `DEFECT`. 계약상 UNKNOWN 이 하나라도 있으면 검사 전체 FAILED. | 황변 1포기로 공정 중단 | 팀 결정 필요 (DEFECT 로 둘지) |
| R4 | `camera_to_robot_base.py` 는 검출 하나가 예외를 내면 전체 결과를 버리고 `_pending_json` 도 남아 **매 update 마다 같은 예외 반복**. | Executor 루프 오류 반복 | 슬롯 단위 try/except, finally 에서 `_pending_json = None` |
| R5 | 좌표 변환이 두 벌: 본 공정용 `camera_to_robot_base.py`(Isaac Camera API·물리 자세, 권장) vs 시험용 `vision_bridge.py`(USD/FK 행렬). | 결과 불일치 시 혼선 | 본 공정은 `camera_to_robot_base` 로 통일, `vision_bridge` 는 functest 검증 전용 |
| R6 | 씬 불일치: functest 는 v013(고정 M0609 + 손목 RealSense), standalone_app 은 v011(LiftRig). | 검사 조건이 본 공정 씬과 다를 수 있음 | 최종 검사 씬 확정 후 prim 경로 통일 |
| R7 | `best.pt` 클래스명이 `lettuce_dark_green/yellow/brown` 과 다르면 Inspection Node 가 시작 시 실패. | 노드 기동 불가 | 모델 `names` 확인 |
| R8 | functest 판정 기준 중 `look_err`(10°), `roll`(±10°) 은 임시값. FK 가 월드 좌표를 준다는 가정도 미확인. | WARN 오판 가능 | 데이터셋 스크립트 `62_v011_tray_inspect_dataset.py` 로 확정 |
| R9 | `--bridge` 깊이 토픽 없이 쓰면 수평면(z=0.869 m) 근사, base 는 `/M0609/Asset` prim 기준. | 수 cm 오차, cull 기준 프레임과 다를 수 있음 | 깊이 토픽 연결 또는 camera_to_robot_base 사용 |
| R10 | 실제 Isaac Sim 5.1.0 실행 미검증 (Isaac API 이름, 번들 경로, 컨테이너 사용자). | 첫 실행 시 오류 가능 | `--check` → `--keep-pose` 순으로 로그 확보 |

## 8. 인수자에게 받아야 할 것 (첫 실행 후)

1. `--check` 출력 전체
2. `--keep-pose --bridge` 실행 로그 중 `[점검]`, `[검증]`, `[브리지]`, 오류 트레이스백
3. `ros2 topic hz /rgb` 결과
4. `/inspection/result`, `/inspection/detections_2d` 한 건씩
5. `/inspection/debug_image` 스크린샷 1장 (ROI 위치 확인용)
