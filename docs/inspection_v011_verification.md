# Inspection v011 검증

`feature/inspection-v011-verify` 브랜치. 2026-09-23 기준.

비전 노드는 `smart_farm_vision` ROS 2 패키지이고 GPU/CPU 컨테이너 안에서
돈다. 호스트에서 `python inspection_node.py` 로 띄우는 경로는 없다.

---

## 1. 확정된 구성

| 항목 | 값 |
|---|---|
| 씬 | `scenes/Collected_smartfarm_v013/Collected_smartfarm_v013.usd` |
| 트레이 | 6구 `romaine_pallet_6_v005_inspect`, 트레이 1 + 상추 6 |
| 검사 카메라 | M0609 손목 D455 `Camera_OmniVision_OV9782_Color` |
| 카메라 Topic | `/rgb` (`ROS_Camera/RGBPublish`), depth `/depth` |
| 렌더 해상도 | 1280x720 (학습 원본과 동일) |
| 가중치 | `resource/best.pt`, md5 `b66a101648c7195d4a12e9c63ebec427` |
| 클래스 | 0 `lettuce_dark_green` / 1 `lettuce_yellow` / 2 `lettuce_brown` |
| 추론 | `imgsz=640`, `conf=0.5` |
| 슬롯 | `SLOT_01` ~ `SLOT_06` |
| 판정 | green=NORMAL, **yellow=DEFECT**, brown=DEFECT |

### 판정 매핑에 대한 주의

handoff README 2절은 노랑을 `보류 HOLD, CULL 아님 (R-T02)` 으로 둔다.
**2026-09-23 팀 결정으로 노랑도 갈색과 같이 솎아내기로 했다.** 계약에
HOLD 를 담을 필드가 없어 DEFECT 로 합친 것이다. 모델을 만든 쪽에 알려
문서를 맞춰야 한다. 보류를 되살리려면 `hold_slots` 를
`TaskResult.msg` 에 추가하고 `docs/02-interfaces.md` 13절,
task_manager, monitor 를 함께 고쳐야 한다 — 계약 변경이므로 팀 합의가
필요하다.

`config/object_detection.yaml` 과 노드의 `DEFAULT_CLASS_OUTCOMES` 를
같은 값으로 맞춰 두었다. 파라미터 파일 없이 띄워도 판정이 같다.

---

## 2. 실행 환경

### 이 PC (Isaac Sim 이 있는 쪽)

Isaac Sim 5.1, RTX 5080 Laptop, ROS 2 Jazzy, Docker 29.8.1 + NVIDIA
Container Toolkit.

**GPU 추론은 쓸 수 없다.** RTX 5080 은 compute capability 12.0(sm_120)
인데 `Dockerfile` 이 고정한 `torch 2.4.1+cu124` 는 sm_90 까지만 커널을
담고 있다. 빌드 검사와 `torch.cuda.is_available()` 은 둘 다 통과하므로
실제 추론 전까지 경고가 없다.

그래서 CPU 전용 이미지를 따로 만들었다. 공용 `Dockerfile` 과
`compose.vision.yaml` 은 건드리지 않았다.

| 파일 | 내용 |
|---|---|
| `smart_farm_vision/Dockerfile.cpu` | torch 2.4.1+cpu. CUDA 빌드가 섞이면 빌드 실패 |
| `Dockerfile.cpu.dockerignore` | `resource/*.pt` 제외 |
| `compose.vision.cpu.yml` | 단독 실행용. GPU 예약 없음, DDS 설정 포함 |

이미지 4.65 GB (GPU 판 11.7 GB). CPU 추론 **16.3 ms/frame**
(Core Ultra 9 275HX, 640px, 20회 평균). `image_timeout_sec: 3.0` 예산의
0.5% 라 GPU 전환을 서두를 이유가 없다.

근본 해결은 `Dockerfile` 의 torch 를 cu128 계열(2.7 이상)로 올리는
것이다. 공용 파일이라 vision 담당과 합의가 필요하다.

### DDS

호스트는 `~/.bashrc` 에서 `ROS_DOMAIN_ID=102` 와
`FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_whitelist.xml` 을 쓴다.
프로파일은 `useBuiltinTransports=false` 에 인터페이스 화이트리스트가
걸려 있다. 컨테이너가 같은 설정을 쓰지 않으면 토픽 목록에는 보이는데
데이터가 오지 않는다. `compose.vision.cpu.yml` 이 맞춰 준다.

**검증은 격리 도메인에서 한다.** 팀 도메인 102 에 시험용 `/rgb` 와
`/inspection/result` 를 뿌리면 팀원 task_manager 가 오동작할 수 있다.
아래 절차는 모두 `ROS_DOMAIN_ID=77` 을 쓴다.

호스트에서 ROS 를 쓸 때는 먼저 소싱한다. `.bashrc` 에 alias 만 있다.

```bash
ros_set
```

---

## 3. 도구

전부 기존 패키지 구조 안에 있다.

| 도구 | 위치 | 용도 |
|---|---|---|
| `topic_audit` | `smart_farm_vision/topic_audit.py` | 계약 Topic·QoS·카메라 후보 대조 |
| `frame_grab` | `smart_farm_vision/frame_grab.py` | `/rgb` 한 장 저장 (ROI 재측정용) |
| `vision_functest.py` | `isaacpjt/smart_farm/runtime/` | 학습 조건 재현 실행기 |
| `vision_rig.py` | `isaacpjt/smart_farm/runtime/` | 트레이 위 고정 카메라 리그 |
| `conveyor_stop_report.py` | `isaacpjt/smart_farm/tools/` | 컨베이어 정지 산포 파서 |

`topic_audit` 과 `frame_grab` 은 entry point 로 등록돼 있어
`ros2 run smart_farm_vision <이름>` 으로 돈다. 소스를 고쳤으면 이미지를
다시 빌드해야 컨테이너에 반영된다.

---

## 4. 검증 절차

### 단계 0 — 모델과 클래스 계약

가중치는 Git 과 Docker 이미지에서 제외된다. 호스트에 직접 둔다.

```bash
unzip -j ~/Downloads/romaine3_v011_yolo11n_handoff.zip \
  romaine3_v011_yolo11n/weights/best.pt \
  -d ~/ROKEY_P3_A1/cobot3_ws/src/smart_farm_vision/resource/
```

`resource/` 가 컨테이너에 `/models` 로 read-only 마운트된다. 파라미터
기본값이 `/models/best.pt` 이므로 파일명을 맞추면 설정을 안 고쳐도 된다.

```bash
docker compose -f compose.vision.cpu.yml run --rm vision \
  python3 -c "from ultralytics import YOLO; print(YOLO('/models/best.pt').names)"
```

통과 기준 `{0: 'lettuce_dark_green', 1: 'lettuce_yellow', 2: 'lettuce_brown'}`.
`yolo_detector.py` 의 `EXPECTED_CLASS_NAMES` 와 한 글자라도 다르면 노드가
기동 중 `ModelClassMismatchError` 로 죽는다. 여기서 멈춘다.

### 단계 1 — 단일 이미지 추론

ROS 없이 모델 로드와 검출을 확인한다.

```bash
docker compose -f compose.vision.cpu.yml run --rm vision \
  python3 -m smart_farm_vision.yolo_detector \
  --model /models/best.pt --image /models/test.jpg \
  --device cpu --allow-cpu-fallback
```

`resource/test.jpg` 는 **8구 트레이를 찍은 Blender 뷰포트 스크린샷**이다.
학습 데이터가 아니고 현재 6구 구성과도 다르다. 모델이 도는지 보는
용도로만 쓰고, 슬롯 판정 기준으로 삼지 않는다.

### 단계 2 — 씬 기동

```bash
cd ~/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm
env -u FASTRTPS_DEFAULT_PROFILES_FILE ROS_DOMAIN_ID=77 \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ~/isaacsim/python.sh runtime/vision_functest.py --headless --keep-pose
```

`--keep-pose` 는 씬에 저장된 팔 자세를 그대로 쓴다. 빼면 IK 로 학습
분포 안의 자세를 다시 찾는다.

Isaac 의 ROS 그래프는 전부 `OnPlaybackTick` 이다. **타임라인이 멈춰
있으면 한 프레임도 안 나간다.** GUI 로 씬만 열어 두고 토픽을 찾으면
아무것도 없다.

### 단계 3 — Topic 대조

```bash
ROS_DOMAIN_ID=77 docker compose -f compose.vision.cpu.yml \
  exec vision /entrypoint.sh ros2 run smart_farm_vision topic_audit \
  --ros-args -p camera_topic:=/rgb
```

계약 Topic, `/clock` 퍼블리셔 수, 카메라 후보와 QoS,
`/inspection/status` durability, `/inspection/debug_image` 를 본다.

`/clock` 이 없어도 비전 노드는 READY 에 도달한다. READY 조건은 모델
로드와 첫 이미지 수신 두 가지뿐이고, 노드 내부 timeout 과 주기는 전부
`time.monotonic()` 기준이라 시뮬레이션 시간과 무관하다. `/clock` 부재는
다른 노드와의 sim time 정렬 문제이지 READY 차단 요인이 아니다.
v013 씬에는 `ROS_Clock` 그래프가 들어 있어 퍼블리셔가 1개 잡힌다.

### 단계 4 — 노드 기동과 READY

```bash
cd ~/ROKEY_P3_A1
ROS_DOMAIN_ID=77 docker compose -f compose.vision.cpu.yml up -d vision
ROS_DOMAIN_ID=77 docker compose -f compose.vision.cpu.yml exec -d vision \
  /entrypoint.sh ros2 run smart_farm_vision object_detection --ros-args \
  --params-file /config/object_detection.yaml \
  -p device:=cpu -p allow_cpu_fallback:=true -p camera_topic:=/rgb
```

`ros2 launch` 는 `--ros-args` 를 받지 않는다. 파라미터를 덮어쓰려면
`ros2 run` 을 쓴다. launch 를 쓸 때 인자는 `config_file` 이다.

```bash
ROS_DOMAIN_ID=77 docker compose -f compose.vision.cpu.yml \
  exec vision /entrypoint.sh ros2 topic echo /inspection/status
```

`--qos-durability transient_local` 을 붙이지 않는다. status 퍼블리셔는
RELIABLE/VOLATILE 이라 durability 를 올려 구독하면 한 건도 못 받는다
(`docs/02-interfaces.md` 14절).

통과 기준: `state` 가 `STARTING` → `READY`, detail `model and image ready`.

| phase / reason | 원인 | 조치 |
|---|---|---|
| `MODEL_INIT_FAILED` | `/models/best.pt` 없음 또는 클래스 불일치 | 단계 0 |
| `WAITING_IMAGE` 유지 | `camera_topic` 불일치 또는 타임라인 정지 | 단계 2·3 |
| `IMAGE_TIMEOUT` | 이미지 끊김 (기본 3초) | 카메라 그래프 |

### 단계 5 — 판정

```bash
ROS_DOMAIN_ID=77 docker compose -f compose.vision.cpu.yml exec vision \
  /entrypoint.sh ros2 topic pub -w 1 --once /inspection/command \
  smart_farm_interfaces/msg/TaskCommand \
  "{task_id: TASK-1, command_id: TASK-1-CMD-1, operation: INSPECT, pallet_id: PALLET_004}"
```

디버그 영상은 `/inspection/debug_image` 다. `/inspection/annotated` 는
없다. 명령 없이 미리 띄워 두려면 `continuous_inference` 를 켠다.

---

## 5. 검증 결과 (2026-09-23)

| 단계 | 결과 | 수치 |
|---|---|---|
| 0 클래스 계약 | PASS | `{0: dark_green, 1: yellow, 2: brown}` |
| 1 단일 이미지 | PASS | 8검출, conf 0.900~0.930, CPU 16.3 ms/frame |
| 2 씬 기동 | PASS | `/rgb` 1280x720 rgb8 |
| 3 Topic 대조 | PASS | 계약 Topic 4종 OK, `/clock` 1개 |
| 4 READY | PASS | `model and image ready` |
| 5 판정 | PASS | 아래 |

학습 조건(손목 D455, 1280x720, 거리 0.537 m, 시선각 1.0°) 에서:

| 슬롯 | 검출 | 신뢰도 | 판정 |
|---|---|---|---|
| SLOT_01 | dark_green | 0.955 | NORMAL |
| SLOT_02 | dark_green | 0.952 | NORMAL |
| SLOT_03 | yellow | 0.952 | DEFECT |
| SLOT_04 | yellow | 0.952 | DEFECT |
| SLOT_05 | brown | 0.952 | DEFECT |
| SLOT_06 | dark_green | 0.948 | NORMAL |

```
status: SUCCEEDED / phase: INSPECTION_COMPLETE / reason: NONE
defect_slots: [SLOT_03, SLOT_04, SLOT_05]   unknown_slots: []
```

여섯 칸 중 셋이 CULL 대상이다. **CULL 담당에게 알릴 것**: 대상이
갈색만이 아니라 노랑까지 포함으로 늘었다.

---

## 6. best.pt 학습 조건 대조

기준은 `romaine3_v011_yolo11n_handoff.zip` 의 README 와
`scripts/62_v011_tray_inspect_dataset.py` 다.

| 항목 | 학습 조건 | 현재 |
|---|---|---|
| 가중치 | md5 `b66a101…` | 동일 |
| 클래스 순서 | 0/1/2 | 동일 |
| 추론 크기·신뢰도 | `imgsz=640`, `conf=0.5` | 동일 |
| 트레이 | 6구, 구멍에 박힌 상태 | 동일 |
| 카메라 | 손목 RealSense, 0.50~0.72 m, 앙각 50~88° | 0.537 m, 시선각 1.0° |
| 조명 | Panel_0 9000 · ring 32000 | 동일 |
| 해상도 | 1280x720 | 1280x720 (씬 수정함) |

### 씬에 반영한 변경

두 건 모두 루트 레이어 `Collected_smartfarm_v013.usd` 를 고쳤다.

| 변경 | 이전 | 이후 | 백업 |
|---|---|---|---|
| `ROS_Camera/RenderProduct` | 640x640 | 1280x720 | `.bak-0923-1308` |
| M0609 팔 자세 (joint_1~6, 도) | `[0.4, 0.4, 90.6, 0, 89.5, 0]` | `[90.8, 48.9, -13.2, -0.3, 114.2, 90.4]` | `.bak-0923-*-armpose` |

팔 자세는 드라이브 목표와 초기 관절 상태를 함께 썼다. **관절 값은
Play 전까지 링크 xform 에 반영되지 않는다.** 정적 USD 만 읽어서는
카메라 자세를 확인할 수 없고, 재생 뒤에 재야 한다
(`vision_functest.py --keep-pose` 가 찍어 준다).

바꾸기 전 팔은 홈 자세로 수직 아래를 보고 있었다. 컨베이어 정지 위치도
시선각 68.4° 로 화각 밖이었다.

### 학습 조건의 한계 (README 6절)

- 합성 데이터만으로 학습했다 (실사 없음)
- 포기가 트레이에 박혀 선 상태만 학습했다. 벨트에 누운 포기, 트레이 밖
  낱개는 대상이 아니다
- 색은 3가지 고정 코드다. 중간색·얼룩은 학습하지 않았다
- val 수치(mAP50 0.995)는 같은 방식으로 만든 합성 val 기준이다

---

## 7. ROI 캘리브레이션

기존 `slot_rois` 는 화면을 3x2 로 균등 분할한 값이었다. 실제로 통하는지
재려고 **팔은 고정한 채 트레이만** 흔들었다. 실제로도 팔은 검사 자세로
가고 트레이가 컨베이어를 타고 와서 서므로, 흔들리는 쪽은 트레이다.

```bash
~/isaacsim/python.sh runtime/vision_functest.py --headless --sweep
```

dx ±30 mm, dy ±20 mm 의 9자세를 돌며 각 자세를 6초 유지한다. 그 사이
`frame_grab` 으로 한 장씩 받는다.

### 슬롯 중심 산포 (1280x720 기준)

| 슬롯 | u 평균 | u 산포 | v 평균 | v 산포 |
|---|---|---|---|---|
| SLOT_01 | 0.3271 | ±38.6 px | 0.3614 | ±18.7 px |
| SLOT_02 | 0.5003 | ±36.4 px | 0.3620 | ±18.5 px |
| SLOT_03 | 0.6732 | ±38.5 px | 0.3627 | ±18.7 px |
| SLOT_04 | 0.3000 | ±45.5 px | 0.5807 | ±24.2 px |
| SLOT_05 | 0.5000 | ±41.4 px | 0.5819 | ±24.3 px |
| SLOT_06 | 0.6998 | ±45.4 px | 0.5826 | ±24.3 px |

### 균등 격자는 쓸 수 없다

경계까지의 여유에서 산포를 뺀 값이다. 음수는 경계를 넘는다는 뜻이다.

| 슬롯 | 균등 격자 | 캘리브레이션 후 |
|---|---|---|
| SLOT_01 | **-30.6 px** | +63.5 px |
| SLOT_02 | +176.6 px | +82.8 px |
| SLOT_03 | **-30.1 px** | +63.8 px |
| SLOT_04 | **-2.9 px** | +91.2 px |
| SLOT_05 | +171.9 px | +77.9 px |
| SLOT_06 | **-3.0 px** | +90.9 px |

9자세를 실제로 판정시킨 결과.

```
실패: 균등 격자 6/9 · 캘리브레이션 후 0/9
```

균등 격자는 정중앙 근처에서만 통했다. **±30 mm 만 틀어져도 슬롯이
옆으로 넘어가 UNKNOWN_SLOT 으로 실패한다.** 검출 자체는 9장 모두 6/6
으로 멀쩡했다. 깨지는 것은 검출이 아니라 슬롯 배정이다.

### 반영한 값

슬롯 사이 경계는 이웃 중심의 중점으로 잡고, 바깥 경계는 중심에서 안쪽
경계와 같은 거리에 두었다. 좌우·상하 여유가 같아지고 ROI 가 화면
가장자리까지 번지지 않아 트레이 밖 오검출이 슬롯에 잡히지 않는다.
`config/object_detection.yaml`, 백업 `object_detection.yaml.bak-roi`.

```yaml
slot_rois.SLOT_01: [0.220275, 0.252170, 0.186589, 0.219712]
slot_rois.SLOT_02: [0.406864, 0.252170, 0.186476, 0.219712]
slot_rois.SLOT_03: [0.593340, 0.252170, 0.186363, 0.219712]
slot_rois.SLOT_04: [0.220275, 0.471882, 0.186589, 0.219712]
slot_rois.SLOT_05: [0.406864, 0.471882, 0.186476, 0.219712]
slot_rois.SLOT_06: [0.593340, 0.471882, 0.186363, 0.219712]
```

화면 점유는 가로 55.9% · 세로 43.9% 다. 스윕 9장 재평가에서 실패 0/9,
여유는 가로 57~83 px · 세로 54~61 px 로 균형이 잡혔다. 화면 전체를
덮던 이전 판도 실패 0/9 였지만 세로 여유가 사실상 무한대였고 바깥이
가장자리까지 열려 있었다.

**카메라 자세가 바뀌면 다시 재야 한다.** 이 값은 거리 0.537 m,
시선각 1.0°, 트레이 (-0.675, -6.700) 기준이다.

### 컨베이어 정지 산포와의 관계

새 ROI 의 가로 여유는 약 63 px 이고 트레이 30 mm 가 약 38 px 이다.
**대략 ±50 mm 까지 견딘다.** 컨베이어 정지 산포를 재면 이 값과 바로
비교하면 된다.

---

## 8. 컨베이어 정지

`conveyor.py` 는 정지선을 넘으면 벨트를 먼저 멈추고, 트레이가 마찰로
`BRAKE_SPEED`(0.02 m/s) 아래까지 느려진 뒤에 고정한다.
`BRAKE_TIMEOUT`(3초) 안에 못 내려가면 경고를 찍고 고정한다.

예전에는 VISION 구역 진입 즉시 `disable_rigid_body_physics()` 로
고정했다. 그러면 트레이는 사실상 무한 질량이 되는데 위에 얹힌 상추는
동적 강체로 남아 벨트 속도를 그대로 안고 있어서, 구멍 벽에 부딪힌
충격량이 전부 되돌아와 사방으로 튀었다.

로그에 줄이 하나 늘었다.

```
[컨베이어] Pallet_Inspect 카메라 앞 정지 (-0.478, -6.751)
           감속 0.42초 · 관성 이동 +22 mm
```

```bash
python3 cobot3_ws/isaacpjt/smart_farm/tools/conveyor_stop_report.py \
  ~/records/conveyor_stops.log --tray-width-m 0.36 --slot-cols 3
```

**정지 위치가 예전보다 +x 로 밀린다.** 관성 이동만큼 `VISION_X` 를
앞으로 당겨야 한다. 팔 도달거리는 한계에 가깝다 (정지 위치까지
0.893 m, M0609 공칭 900 mm).

`inspecting` 은 고정이 끝난 팔레트만 돌려준다. 감속 중에는 None 이다.
움직이는 트레이에 팔을 뻗지 않게 하기 위한 것이다.

`settle_duration_s: 0.5` 는 컨베이어가 아니라 navigation 파라미터다
(`smart_farm_navigation/config/carter2_dock.yaml`, `path_runner.yaml`).

GUI 로 컨베이어를 돌릴 때는 `scripts/e8_gui_session.py` 를 쓴다.
같은 이름 파일이 `~/Downloads` 에도 있는데 그쪽은 `SCRIPTS` 가
`~/cobot3_ws` 를 가리키므로 쓰지 않는다.

---

## 9. 트레이 180° 뒤집힘

**영상만으로는 구분할 수 없다.** 실측한 결과다.

같은 장면을 트레이 yaw 90°(정상)와 270°(180° 뒤집힘)로 찍어 판정했다.

| yaw | defect_slots | 검출 | status |
|---|---|---|---|
| 90° | `[SLOT_03, SLOT_04, SLOT_05]` | 6/6 | SUCCEEDED |
| 270° | `[SLOT_02, SLOT_03, SLOT_04]` | 6/6 | SUCCEEDED |

`01↔06, 02↔05, 03↔04` 매핑과 정확히 일치한다. **오류도 UNKNOWN 도 없이
조용히 틀린 칸을 솎아낸다.** 검출·신뢰도·슬롯 배정 모두 정상이므로
노드 쪽에서 탐지할 방법이 없다.

6구 트레이는 2회 대칭이고 바닥 팔레트의 포크 포켓도 대칭이다. 화면에
방향을 알려 줄 비대칭 특징이 없다.

### 그래서 방향은 놓는 시점에 정해진다

`conveyor.py` 의 `watch()` 가 트레이에
`PhysxRigidBodyAPI.CreateLockedRotAxisAttr().Set(LOCKED_ROT_Z)` 를 건다.
**주행 중 yaw 가 잠겨 있어 벨트 위에서는 돌 수 없다.** 즉 검사 지점의
방향은 팔이 줄기 벨트에 놓을 때 정해지고 그대로 유지된다.

대응은 셋 중 하나다.

1. **놓을 때 yaw 를 고정한다.** 컨베이어가 이미 유지해 주므로 가장 싸고
   확실하다. `PLACE_INSPECT` 를 구현할 때 트레이 yaw 를 상수로 못박으면
   된다 — 통합 담당에게 넘길 요구사항이다
2. 트레이 에셋 한쪽 끝에 비대칭 마커를 넣는다. 에셋 변경이고 학습
   데이터도 다시 만들어야 할 수 있다
3. 슬롯 번호를 포기하고 위치로만 보고한다. CULL 이 팔 좌표계의 슬롯
   번호를 필요로 하므로 현실적이지 않다

**1번을 권한다.** 지금은 미해결 상태로 두되, 위험이 "틀린 칸을 솎아냄"
이라는 점을 CULL·통합 담당이 알고 있어야 한다.

---

## 10. 그 밖의 미확정

- **R-C04 정지 위치 미합의.** 컨베이어 수치는 현재 배치 기준이다
- **카메라 QoS.** Isaac 퍼블리셔가 RELIABLE/VOLATILE 이다. 노드는
  BEST_EFFORT 로 구독해 호환되지만, `docs/02-interfaces.md` 14절의
  "검사 영상은 BEST_EFFORT" 와 방향이 반대다. 네트워크를 타는 구성에서
  재확인할 것
- **v013 패키지에 `SubUSDs/materials.usd` 가 빠졌다.** XT-32 라이다
  머티리얼이다. 비전과 무관하나 내비게이션 담당에게 알릴 것
- **씬 `PLACE_INSPECT`·`CULL`·`CONVEYOR_OUT` 미구현.**
  `runtime/standalone_app.py` 는 `TRANSFER` 와 `PICK_HARVEST` 만 받는다
  (522행, 538행). task_manager 가 컨베이어와 검사를 부를 수 없다.
  **통합 담당 몫이라 여기서는 건드리지 않았다.** 9절의 yaw 고정
  요구사항도 이 구현에 같이 들어가야 한다

### 해결한 것 (2026-09-23)

- `smart_farm_monitor/topics.py` 의 `SLOTS` 가 8칸(`range(1, 9)`)이라
  대시보드가 SLOT_07·SLOT_08 을 그렸다. 거기엔 상태가 오지 않는다.
  `range(1, 7)` 로 고쳤다
- 바깥 네 칸 ROI 가 화면 가장자리까지 넓어 트레이 밖 오검출에
  취약했다. 7절의 좁힌 값으로 바꿨다

---

## 11. 재현 시 걸리는 함정

- **컨테이너에서 편집 중인 소스로 테스트할 때** entrypoint 가
  `/vision_ws/install/setup.bash` 를 소싱하며 `PYTHONPATH` 앞자리를
  차지한다. 호스트 소스를 앞에 두지 않으면 이미지에 구워진 옛 소스로
  돈다

  ```bash
  docker compose -f compose.vision.cpu.yml run -T --rm \
    -v $PWD/cobot3_ws/src/smart_farm_vision:/host_src:ro vision \
    bash -c 'export PYTHONPATH=/host_src:$PYTHONPATH; python3 -m pytest /host_src/test -q'
  ```

- **Isaac 의 `app.close()` 가 트레이스백보다 먼저 프로세스를 끝낸다.**
  예외를 직접 찍지 않으면 로그에 아무것도 안 남는다
- **`isaacsim/python.sh` 에 pxr 이 바로 안 잡힌다.** USD 만 읽으려면
  `PYTHONPATH`/`LD_LIBRARY_PATH` 를
  `extscache/omni.usd.libs-*` 로 지정한다
- **물리 객체를 로봇 안으로 옮기지 말 것.** 트레이를 그리퍼 위치로
  텔레포트했다가 리셋 순간 물리가 터져 팔레트가 튕겨 나갔다. 카메라를
  옮기거나 새로 만드는 쪽이 안전하다

---

## 12. 기록 양식

| 단계 | 실행일 | 결과 | 수치 / 진단 코드 | 비고 |
|---|---|---|---|---|
| 0 클래스 계약 | 2026-09-23 | PASS | 3클래스 순서 일치 | |
| 1 단일 이미지 | 2026-09-23 | PASS | CPU 16.3 ms/frame | |
| 2 씬 기동 | 2026-09-23 | PASS | `/rgb` 1280x720 | |
| 3 Topic 대조 | 2026-09-23 | PASS | 계약 4종 OK | |
| 4 READY | 2026-09-23 | PASS | model and image ready | |
| 5 판정 | 2026-09-23 | PASS | SUCCEEDED, defect 3칸 | |
| 컨베이어 정지 산포 | | | | 감속 정책 변경 후 미측정 |
