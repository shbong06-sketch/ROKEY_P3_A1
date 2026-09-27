# smart_farm_vision

스마트팜 검사를 위한 ROS 2 Jazzy/NVIDIA GPU Docker 환경과 YOLO 기반 Inspection Executor다.
이미지 빌드 시 `smart_farm_interfaces`와 `smart_farm_vision`을 함께 빌드한다.

## 현재 상태

- ROS 2 Jazzy, CUDA PyTorch, OpenCV, Ultralytics 실행 환경
- YOLO 모델 단일 로드와 GPU 추론 worker
- ROS Image 구독, encoding별 BGR 변환, camera timeout 감지
- `/inspection/command` 기반 fresh frame 검사
- `/inspection/result`, `/inspection/status`, `/inspection/detections_2d` 발행
- `SLOT_01~SLOT_06` ROI와 YAML 기반 NORMAL/DEFECT/UNKNOWN 판정
- 원본 RGB header·해상도·pixel 좌표를 유지한 2D detection
- ROI, bbox, class, confidence, 슬롯 판정을 포함한 debug image
- ROS 및 `/vision_ws/install` 자동 소싱
- 모델과 설정의 read-only runtime volume 연결

Task Manager는 `TaskResult`의 검사 결과만 관리한다. 동적 Pick에 필요한 Depth와
로봇 좌표 계산은 `/inspection/detections_2d`를 받는 Isaac Sim 내부 Executor의 책임이다.

## 실행 구조

- 호스트: Isaac Sim 및 ROS 2 카메라 토픽 실행
- 컨테이너: ROS 2 Jazzy, OpenCV, Ultralytics, CUDA PyTorch 실행
- 통신: `network_mode: host`를 통한 ROS 2 DDS 통신
- 모델: 패키지의 `resource/`를 컨테이너 `/models`에 read-only 마운트
- 설정: 패키지의 `config/`를 컨테이너 `/config`에 read-only 마운트
- 빌드 결과: 이미지 내부 `/vision_ws/install`

컨테이너 시작 시 `/opt/ros/jazzy/setup.bash`와
`/vision_ws/install/setup.bash`를 순서대로 자동 소싱한다.

## 사전 요구사항

실행 PC에 다음 항목이 필요하다.

- NVIDIA GPU 및 호환 드라이버
- Docker Engine
- Docker Compose plugin (`docker compose`)
- NVIDIA Container Toolkit
- Docker Hub, Ubuntu/ROS apt 저장소, PyPI 및 PyTorch wheel 저장소에 대한 네트워크 접근

호스트에서 먼저 확인한다.

```bash
docker --version
docker compose version
nvidia-smi
```

NVIDIA Container Toolkit 설치 후 Docker에서 GPU가 보이는지도 확인한다.

```bash
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

## 모델과 설정 준비

모델 파일만 별도로 준비한다.

```bash
mkdir -p cobot3_ws/src/smart_farm_vision/resource
# best.pt를 아래 경로에 복사
# cobot3_ws/src/smart_farm_vision/resource/best.pt
```

`config/object_detection.yaml`은 Git으로 관리하며 ROI와 class 판정을 포함한다.
컨테이너에서는 모델을 `/models/best.pt`, 설정을
`/config/object_detection.yaml`로 읽는다.

`*.pt` 파일은 Git과 Docker 이미지에서 제외된다. 모델 파일을 커밋하지 않는다.

## 이미지 빌드

모든 명령은 저장소 루트에서 실행한다.

```bash
cd /path/to/ROKEY_P3_A1
docker compose -f compose.vision.yaml config --quiet
docker compose -f compose.vision.yaml build vision
```

이미지 빌드 과정은 다음 두 패키지만 빌드한다.

```text
smart_farm_interfaces
smart_farm_vision
```

`smart_farm_vision/package.xml`의 의존성에 따라
`smart_farm_interfaces`가 먼저 빌드된다.

## 실행

호스트의 다른 ROS 2 노드와 같은 Domain ID를 지정하고 컨테이너를 시작한다.

```bash
export ROS_DOMAIN_ID=0
docker compose -f compose.vision.yaml up -d vision
docker compose -f compose.vision.yaml ps
```

Compose 기본 명령은 개발·검증을 위해 `sleep infinity`를 유지한다.
Inspection Executor는 entrypoint를 통해 실행한다.

```bash
docker compose -f compose.vision.yaml exec vision \
  /entrypoint.sh ros2 launch smart_farm_vision object_detection.launch.py \
  config_file:=/config/object_detection.yaml
```

주요 Topic을 별도 터미널에서 확인한다.

```bash
docker compose -f compose.vision.yaml exec vision \
  /entrypoint.sh ros2 topic echo /inspection/status
docker compose -f compose.vision.yaml exec vision \
  /entrypoint.sh ros2 topic echo /inspection/result
docker compose -f compose.vision.yaml exec vision \
  /entrypoint.sh ros2 topic echo /inspection/detections_2d
docker compose -f compose.vision.yaml exec vision \
  /entrypoint.sh ros2 topic info /inspection/debug_image --verbose
```

로그와 종료 명령:

```bash
docker compose -f compose.vision.yaml logs -f vision
docker compose -f compose.vision.yaml down
```

## 빌드 결과 검증

`docker compose run`은 서비스 entrypoint를 거치므로 별도의 `source` 명령 없이
ROS와 workspace 환경이 적용된다.

```bash
docker compose -f compose.vision.yaml run --rm vision \
  ros2 pkg prefix smart_farm_interfaces

docker compose -f compose.vision.yaml run --rm vision \
  ros2 pkg prefix smart_farm_vision

docker compose -f compose.vision.yaml run --rm vision \
  ros2 interface show smart_farm_interfaces/msg/TaskCommand

docker compose -f compose.vision.yaml run --rm vision \
  python3 -c 'from smart_farm_interfaces.msg import TaskCommand, TaskResult, ExecutorStatus; print(TaskCommand()); print(TaskResult()); print(ExecutorStatus())'
```

GPU와 Vision Python 패키지를 확인한다.

```bash
docker compose -f compose.vision.yaml run --rm vision nvidia-smi

docker compose -f compose.vision.yaml run --rm vision \
  python3 -c 'import torch, cv2, ultralytics; print("torch:", torch.__version__); print("CUDA wheel:", torch.version.cuda); print("GPU:", torch.cuda.is_available()); assert torch.cuda.is_available()'
```

실행 중인 컨테이너에서 검사할 때는 entrypoint를 명시적으로 재사용한다.
`docker compose exec`가 시작하는 새 프로세스는 기존 entrypoint에서 소싱한 셸 환경을
직접 상속하지 않기 때문이다.

```bash
docker compose -f compose.vision.yaml exec vision \
  /entrypoint.sh ros2 pkg prefix smart_farm_vision
```

## 다른 PC에 배포

1. 저장소의 `feature/Inspection` 브랜치를 받는다.
2. NVIDIA 드라이버, Docker, Compose plugin, NVIDIA Container Toolkit을 설치한다.
3. `resource/best.pt`를 별도로 전달받아 배치하고 Git의 `config/object_detection.yaml`을 환경에 맞게 검토한다.
4. Isaac Sim/Task Manager와 같은 `ROS_DOMAIN_ID`를 지정한다.
5. 이미지를 빌드하고 서비스를 실행한다.
6. 위 smoke test로 GPU, 패키지, 사용자 정의 메시지를 확인한다.

```bash
git switch feature/Inspection
export ROS_DOMAIN_ID=0
docker compose -f compose.vision.yaml build vision
docker compose -f compose.vision.yaml up -d vision
```

서로 다른 PC의 ROS 2 노드와 통신할 때는 같은 LAN과 `ROS_DOMAIN_ID`를 사용하고,
호스트 방화벽이 DDS discovery 및 data traffic을 차단하지 않는지 확인한다.

## 재빌드가 필요한 변경

다음 항목이 변경되면 이미지를 다시 빌드해야 한다.

- `smart_farm_interfaces/msg/*.msg`
- `smart_farm_interfaces/srv/*.srv`
- `smart_farm_interfaces/package.xml`
- `smart_farm_interfaces/CMakeLists.txt`
- `smart_farm_vision/package.xml`
- `smart_farm_vision/setup.py`
- `smart_farm_vision` Python 소스
- `smart_farm_vision/Dockerfile`

```bash
docker compose -f compose.vision.yaml build vision
docker compose -f compose.vision.yaml up -d --force-recreate vision
```

모델과 설정 디렉터리는 runtime volume이므로 파일 내용만 바뀐 경우 이미지 재빌드는
필요하지 않다. 다만 노드가 파일을 시작 시 한 번만 읽는다면 컨테이너를 재시작한다.

```bash
docker compose -f compose.vision.yaml restart vision
```

소스나 `/vision_ws/install`을 runtime volume으로 마운트하지 않는다. 이미지에 빌드된
workspace 결과가 가려져 패키지와 인터페이스 버전이 불일치할 수 있다.

## 문제 해결

### `ModuleNotFoundError: No module named 'rclpy'`

설치 누락이 아니라 `docker compose exec` 프로세스에서 ROS 환경을 소싱하지 않은 경우가
많다. entrypoint를 통해 다시 실행한다.

```bash
docker compose -f compose.vision.yaml exec vision \
  /entrypoint.sh python3 -c 'import rclpy; print("rclpy OK")'
```

### `torch.cuda.is_available()`가 `False`

호스트의 `nvidia-smi`와 Docker GPU 접근부터 확인한다.

```bash
nvidia-smi
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
docker compose -f compose.vision.yaml run --rm vision nvidia-smi
```

### 인터페이스 변경이 컨테이너에 반영되지 않음

인터페이스는 이미지 빌드 시 생성된다. Compose 서비스 재시작만으로는 갱신되지 않는다.

```bash
docker compose -f compose.vision.yaml build vision
docker compose -f compose.vision.yaml up -d --force-recreate vision
```

### 모델 또는 설정 파일이 보이지 않음

호스트 파일과 read-only mount를 확인한다.

```bash
ls -l cobot3_ws/src/smart_farm_vision/resource
ls -l cobot3_ws/src/smart_farm_vision/config
docker compose -f compose.vision.yaml exec vision ls -l /models /config
```
