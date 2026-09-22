# smart_farm_vision

스마트팜 검사 노드를 위한 ROS 2 Jazzy/NVIDIA GPU Docker 실행 환경이다.
현재 이미지는 `smart_farm_interfaces`와 `smart_farm_vision`을 빌드하고 실행 환경만 유지한다.
YOLO 추론 노드와 카메라 구독 코드는 아직 포함하지 않는다.

## 현재 상태

Vision Docker 실행 환경 구축과 다음 검증을 완료했다.

- Docker Compose 문법 검사 및 이미지 빌드
- NVIDIA GPU 접근과 CUDA PyTorch 동작
- OpenCV, Ultralytics, `rclpy`, `sensor_msgs`, `cv_bridge` import
- `smart_farm_interfaces` 이후 `smart_farm_vision` 빌드
- 두 ROS 패키지 조회와 `TaskCommand` 인터페이스 확인
- `smart_farm_interfaces.msg` Python import
- ROS 및 `/vision_ws/install` 자동 소싱
- 모델과 설정의 read-only runtime volume 연결

현재 완료 범위는 Vision 노드를 개발하고 실행할 컨테이너 기반 환경까지다.
다음 단계는 `/inspection/command`, `/inspection/result`, `/inspection/status` 계약을
구현하는 Inspection Node이며, 카메라 구독과 YOLO 추론은 그 이후에 추가한다.

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

저장소 루트에서 필요한 디렉터리를 만든다.

```bash
mkdir -p cobot3_ws/src/smart_farm_vision/resource
mkdir -p cobot3_ws/src/smart_farm_vision/config
```

모델과 설정 파일을 다음 위치에 배치한다.

```text
cobot3_ws/src/smart_farm_vision/resource/best.pt
cobot3_ws/src/smart_farm_vision/config/inspection.yaml
```

컨테이너에서는 다음 경로로 접근한다.

```text
/models/best.pt
/config/inspection.yaml
```

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

호스트의 다른 ROS 2 노드와 같은 Domain ID를 지정한다.

```bash
export ROS_DOMAIN_ID=0
docker compose -f compose.vision.yaml up -d vision
docker compose -f compose.vision.yaml ps
```

현재 기본 명령은 `sleep infinity`다. 이는 노드 구현 전까지 컨테이너를
개발 및 검증 환경으로 유지하기 위한 설정이다.

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
3. `resource/best.pt`와 `config/inspection.yaml`을 별도로 전달받아 배치한다.
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
