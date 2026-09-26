# TRANSFER → PLACE_INSPECT 실기 확인 (Ubuntu)

이 절차는 `DEMO_HARVEST_01` 생산 시나리오를 그대로 실행한다. `PLACE_INSPECT`의 terminal result 뒤에 Task Manager가 `INSPECT`를 자동 발행한다. 네 단계만 따로 성공 처리하거나 사이클을 `COMPLETE`로 바꾸지 않는다. 구간 판정은 아래 명령·결과 기록에서 한다.

## 준비

각 ROS 터미널에서 같은 `ROS_DOMAIN_ID`와 DDS 설정을 사용한다. 두 PC를 쓰면 상대 PC가 같은 DDS 도메인에서 `/clock`, `/tf`, `/chassis/odom`, 라이다, `/rgb`를 볼 수 있어야 한다. Isaac 터미널에는 시스템 ROS를 source하지 않는다. Isaac Sim은 `runtime/standalone_app.py`가 자체 Jazzy 번들을 설정한다.

```bash
cd /path/to/ROKEY_P3_A1/cobot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to smart_farm_manager smart_farm_navigation smart_farm_vision --symlink-install
source install/setup.bash
```

`scenes/Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd`와 그 참조 자산을 준비한다. 이 장면은 Git에 없다. Inspection 모델은 `src/smart_farm_vision/resource/best.pt`에 둔다. 비전 노드는 `compose.vision.yaml`의 별도 Docker 컨테이너에서 실행한다. 이미지에 `torch`, `ultralytics`, `cv_bridge`, OpenCV가 포함되며, NVIDIA GPU·드라이버·Container Toolkit이 필요하다. 호스트에서 `ros2 run smart_farm_vision object_detection`을 실행하면 호스트 Python에 이 의존성이 없는 경우 `ModuleNotFoundError: No module named 'torch'`로 중단된다. Docker를 켜는 것만으로 호스트 Python 환경이 바뀌지는 않는다. 올인원 스테이션의 YOLO worker를 함께 사용할 때는 그 worker용 Python에도 별도로 `ultralytics`가 필요하다.

## 별도 프로세스 기동 순서

아래 명령은 각각 별도 터미널에서 실행한다. ROS 터미널마다 `source /opt/ros/jazzy/setup.bash`와 `source /path/to/ROKEY_P3_A1/cobot3_ws/install/setup.bash`를 먼저 실행한다. `ROS_DOMAIN_ID`는 모두 같은 값으로 설정한다.

1. **Isaac Sim / Sim Executor** — 시스템 ROS를 source하지 않은 터미널:

   ```bash
   cd /path/to/ROKEY_P3_A1
   /path/to/isaacsim/python.sh cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py --autoplay --scene cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd
   ```

   원격 화면이 필요하면 `isaac_python` 셸 함수 대신 같은 `python.sh`로 `--livestream`을 준다. `PUBLIC_IP`에는 스트리밍 클라이언트에서 접속할 서버 주소를 넣는다. 이 옵션은 일반 headless Isaac을 초기화한 다음 WebRTC 확장을 켜고 화면 렌더링을 유지한다.

   ```bash
   PUBLIC_IP="서버_IP"
   /path/to/isaacsim/python.sh cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py \
     --livestream --autoplay \
     "--/app/livestream/publicEndpointAddress=${PUBLIC_IP}" \
     --/app/livestream/port=49100
   ```

   예전 `isaac_python` 함수가 `sitecustomize.py`로 Streaming 경험을 초기화 시작부터 강제했을 때 `omni.graph.image.core` 초기화 중 충돌이 관찰되었다. 위의 직접 실행 명령은 그 경로를 사용하지 않는다.

2. **Nav2** — `ros2 launch smart_farm_navigation nav2.launch.py scan_mode:=auto use_rviz:=false`
3. **Navigation executor** — `ros2 launch smart_farm_navigation navigation_node.launch.py`
4. **Inspection executor** — 저장소 루트에서 Docker Compose로 실행한다. 호스트와 컨테이너의 `ROS_DOMAIN_ID`를 같게 한다. Compose의 `environment.ROS_DOMAIN_ID`는 저장소 루트의 **호스트별 `.env`** 파일에서 읽는다. VM에는 `ROS_DOMAIN_ID=101`을 설정했고, RTX 5080 노트북에는 그 노트북의 다른 ROS 터미널과 같은 값을 `.env`에 적는다. `.env`는 Git에서 제외한다. 현재 `tmdgusqhd` 계정은 `docker` 그룹에 없어 아래처럼 `sudo`가 필요하다. Docker 소켓 권한이 있는 계정은 `sudo`를 생략할 수 있다. 첫 빌드에는 패키지 저장소 접근이 필요하다.

   ```bash
   cd /path/to/ROKEY_P3_A1
   cat .env  # ROS_DOMAIN_ID가 다른 ROS 터미널과 같은지 확인
   test -f cobot3_ws/src/smart_farm_vision/resource/best.pt
   sudo docker compose -f compose.vision.yaml build vision  # 이미지가 없을 때만
   sudo docker compose -f compose.vision.yaml up -d vision
   sudo docker compose -f compose.vision.yaml exec vision \
     /entrypoint.sh ros2 launch smart_farm_vision object_detection.launch.py \
     config_file:=/config/object_detection.yaml
   ```

   마지막 명령은 노드를 실행한 채 로그를 출력하므로 터미널을 유지한다. `up -d`만 하면 Compose의 기본 명령 `sleep infinity`만 실행된다. 호스트의 `resource/best.pt`와 설정 파일은 각각 컨테이너의 `/models/best.pt`, `/config/object_detection.yaml`로 읽기 전용 연결된다. GPU·의존성 확인이 필요하면 다른 터미널에서 다음을 실행한다.

   `down -v` 후 다시 `up -d`할 때도 저장소 루트의 `.env`를 유지한다. `.env` 파일은 컨테이너 볼륨이 아니므로 `down -v`가 삭제하지 않는다. 실제 재생성 사례에서 호스트는 도메인 101, 컨테이너는 Compose 기본값 0으로 시작되어 `/rgb`를 받지 못했다. Compose는 이제 `.env`나 셸 환경에 `ROS_DOMAIN_ID`가 없으면 오류를 내도록 설정했다. 재생성 후 다음 값이 다른 ROS 터미널과 같은지 확인한다.

   ```bash
   sudo docker compose -f compose.vision.yaml exec vision /entrypoint.sh printenv ROS_DOMAIN_ID FASTDDS_BUILTIN_TRANSPORTS
   ```

   Compose의 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`는 컨테이너의 Fast DDS 공유 메모리 전송을 사용하지 않게 한다. 실제로 호스트는 `/rgb`의 `rgb8` 이미지를 받았지만 기본 설정의 컨테이너는 받지 못했고, 같은 컨테이너에서 이 환경변수를 준 구독자는 이미지를 받았다. Compose 설정을 바꾼 뒤에는 `up -d --force-recreate vision`으로 컨테이너를 재생성하고 위 `exec ... ros2 launch ...`를 다시 실행한다. 재생성하면 기존 Inspection 노드가 종료된다. 재생성 후 `/rgb` `rgb8` 수신과 `/inspection/status`의 `READY`, `phase=IDLE`, `detail=model and image ready`까지 확인했다. 이후 물리 동작과 검사 결과는 아직 확인하지 않았다.

   ```bash
   cd /path/to/ROKEY_P3_A1
   sudo docker compose -f compose.vision.yaml exec vision \
     /entrypoint.sh python3 -c 'import torch, ultralytics, cv2; print(torch.__version__, torch.cuda.is_available())'
   ```

   `up -d`가 `could not select device driver "nvidia" with capabilities: [[gpu]]`로 실패하면 Inspection 노드 실행을 시도하기 전에 **Docker를 실행하는 호스트의 NVIDIA Container Toolkit**을 설치·설정한다. `service "vision" is not running`은 그 실패의 후속 메시지다. 이 절차는 L4 GPU가 있는 VM과 RTX 5080 GPU가 있는 Ubuntu 노트북에 각각 적용한다. `nvidia-smi`가 GPU를 표시해도 Docker GPU runtime은 별도로 필요하다. 현재 확인한 VM에는 `nvidia-container-toolkit` 패키지와 `nvidia-ctk`가 없다. 노트북의 설치 상태는 아직 확인하지 않았다. [NVIDIA 공식 설치 절차](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)에 따라 **Toolkit이 없는 호스트에서만** 다음을 실행한다. Docker 재시작은 실행 중인 다른 컨테이너에도 영향을 줄 수 있으므로 먼저 `sudo docker ps`를 확인한다.

   ```bash
   sudo apt-get update
   sudo apt-get install -y --no-install-recommends ca-certificates curl gnupg2
   curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
     sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
   curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
     sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
     sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
   sudo apt-get update
   sudo apt-get install -y nvidia-container-toolkit
   sudo nvidia-ctk runtime configure --runtime=docker
   sudo systemctl restart docker
   sudo docker info | grep 'Runtimes:'
   ```

   출력에 `nvidia`가 보이면 위 `up -d vision`과 `exec vision ...`을 다시 실행한다. `ROS_DOMAIN_ID`를 설정한 터미널에서 실행한다. 컨테이너가 시작된 뒤에는 `torch.cuda.is_available()`뿐 아니라 **해당 GPU에서 실제 CUDA 연산**도 확인한다.

   ```bash
   sudo docker compose -f compose.vision.yaml exec vision /entrypoint.sh python3 -c \
     'import torch; print(torch.__version__, torch.version.cuda, torch.cuda.get_device_name(0), torch.cuda.get_device_capability(0)); print((torch.ones(1, device="cuda") + 1).item())'
   ```

   **RTX 5080 주의:** 현재 비전 [Dockerfile](src/smart_farm_vision/Dockerfile)은 `torch==2.4.1`의 CUDA 12.4 wheel로 고정되어 있다. RTX 5080의 `sm_120`을 지원하지 않을 수 있으므로 Toolkit 설치만으로 검사 추론 성공을 보장하지 않는다. 실제 CUDA 연산이 `no kernel image is available` 등으로 실패하면 RTX 5080을 지원하는 [PyTorch 공식 CUDA 빌드](https://pytorch.org/get-started/locally/)로 이미지를 갱신한 뒤 재빌드해야 한다. 이 변경은 별도 검증 전까지 완료된 것으로 취급하지 않는다. L4 VM에서도 같은 CUDA 연산 검사로 이미지 동작을 확인한다.

5. **Task Manager** — `ros2 launch smart_farm_manager task_manager.launch.py`

`run_flow.py`, PowerShell 도구, mock executor는 이 실기 절차에 사용하지 않는다. PREFLIGHT는 Sim Task, Navigation, Inspection의 최근 `READY` heartbeat를 모두 요구한다. Inspection은 모델 로드와 `/rgb`의 유효한 이미지 수신 후 `READY`가 된다. 시작 전에 다음을 확인한다.

실제 Task Manager 로그에서 `sim_task:received=True,state=READY`, `navigation:received=True,state=READY`, `inspection:received=False,state=<never>`와 `PREFLIGHT failed: reason=INSPECTION_NOT_READY`가 관찰되었다. 이는 `vision` 서비스가 GPU runtime 오류로 시작하지 못한 상태와 일치한다. 이 사이클은 네 단계 실행 시험에 진입하지 않았으므로 성공으로 기록하지 않는다. 비전 컨테이너와 Inspection 노드가 실행되고 `/inspection/status`에 `READY`가 나온 뒤 새 사이클을 시작한다.

```bash
ros2 topic echo --once /sim_task/status
ros2 topic echo --once /navigation/status
ros2 topic echo --once /inspection/status
ros2 topic hz /rgb
ros2 action list | grep navigate_to_pose
```

## 실제 명령·terminal result 기록

사이클 시작 **전**에 별도 ROS 터미널에서 기록한다. 녹화 종료는 Ctrl-C다.

```bash
ros2 bag record -o /tmp/transfer_to_place \
  /cycle/status /sim_task/command /sim_task/result /sim_task/status \
  /navigation/command /navigation/result /navigation/status \
  /inspection/command /inspection/result /inspection/status \
  /feeder_dock/status /feeder_dock/result
```

다른 ROS 터미널에서 시작한다.

```bash
ros2 service call /start_cycle smart_farm_interfaces/srv/StartCycle "{scenario_id: DEMO_HARVEST_01}"
```

실시간 확인은 `ros2 topic echo /cycle/status`, `ros2 topic echo /sim_task/command`, `ros2 topic echo /sim_task/result`, `ros2 topic echo /navigation/command`, `ros2 topic echo /navigation/result`를 각각 별도 터미널에서 실행한다. 재생이 필요하면 다른 DDS 도메인에서 `ros2 bag play /tmp/transfer_to_place` 후 같은 echo 명령을 사용한다. 실제 성공은 `task_id`가 전 단계에서 같고, 각 명령과 terminal result의 `command_id`·`operation`이 일치할 때만 인정한다.

| 순서 | 기대 명령 | terminal result와 다음 상태 |
|---|---|---|
| 1 | `/sim_task/command` String/JSON `TRANSFER`, `CMD-001` | `/sim_task/result` `SUCCEEDED`, `completed_units`에 `PALLET_002:RACK_L3:RACK_L2`, `PALLET_003:RACK_L4:RACK_L3`; 상태 `PICK_HARVEST` |
| 2 | `/sim_task/command` String/JSON `PICK_HARVEST`, `CMD-002`, `pallet_id=PALLET_001` | `/sim_task/result` `SUCCEEDED`, `safe_to_navigate=true`; 상태 `NAVIGATION` |
| 3 | `/navigation/command` TaskCommand `NAVIGATION`, `CMD-003`, `destination=FEEDER_DOCK` | `/navigation/result` TaskResult `SUCCEEDED`, `reached_station=FEEDER_DOCK`; 상태 `PLACE_INSPECT` |
| 4 | `/sim_task/command` String/JSON `PLACE_INSPECT`, `CMD-004`, `pallet_id=PALLET_001` | `/sim_task/result` `SUCCEEDED`; 상태 `INSPECT` |

전체 `command_id`는 `/start_cycle` 응답의 `task_id` 뒤에 `-CMD-001`처럼 붙는다. Place 결과 후 `/inspection/command` `INSPECT`, `CMD-005`가 나오는 것은 현 생산 시나리오의 예상 동작이다. 이 결과로 네 단계의 성공을 대체하지 않는다. 현재 올인원 스테이션도 Place 후 컨베이어·검사·솎아내기를 자동 진행하므로 이후 구간의 정상 통합을 이 시험에서 주장하지 않는다.

## 실패 시 로그 확인 순서

1. `/cycle/status`가 `PREFLIGHT`에서 끝나면 Task Manager의 `PREFLIGHT failed` 로그와 세 `/.../status`의 `state`, `detail`, heartbeat 갱신을 본다. Inspection `WAITING_IMAGE`면 호스트와 컨테이너 각각에서 `/rgb` 수신을 비교한다. 호스트만 받으면 Compose의 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 설정과 컨테이너 재생성 여부를 확인한다. 모델 로드 오류면 비전 노드를 실행한 Docker 터미널을 본다. `torch` import 오류가 호스트에서 나오면 호스트 `ros2 run` 대신 위 Docker 명령으로 노드를 시작했는지 확인한다. Docker 소켓 접근이 거부되면 현재 계정의 `docker` 그룹 권한 또는 `sudo` 사용 여부를 확인한다.
2. Sim 명령이 없으면 Task Manager의 `Command published`와 `/sim_task/command` 녹화를 본다. Sim 결과가 실패하면 Isaac 터미널의 `Sim command queued`, `Sim result published`, `MOTION_FAILED`, 장면 초기화 오류와 `/sim_task/status.phase`를 본다.
3. `NAVIGATION`이 실패하면 Navigation 터미널의 `goToPose`, Nav2 액션 서버·목표 거부 로그, `/feeder_dock/status`·`/feeder_dock/result`의 `run_id`를 본다. `/clock`, `/tf`, `/chassis/odom`, `/scan`도 확인한다.
4. Place 실패 또는 timeout이면 Isaac의 `WAIT_BASE_SETTLED`, `ARM_PLACE`, TurnTable 목표·포크 인출 로그를 본다. Task Manager의 `Command timeout`과 `/cycle/status.reason`을 함께 확인한다.

`[대기] 팔 베이스 정지를 기다리는 중`은 `BaseWatcher.update()`가 정지 여부와 관계없이 3초마다 출력하는 안내다. Isaac은 Place 명령이 없어도 이 감시기를 매 물리 스텝 갱신하므로, 이 줄만으로 정지 대기 상태라고 판정하지 않는다. 실제 대기라면 `/sim_task/status`의 JSON `phase`가 `TRANSFER_UNIT_XX/WAIT_BASE_SETTLE`, `PICK_HARVEST/WAIT_BASE_SETTLE`, 또는 `PLACE_INSPECT/WAIT_BASE_SETTLED`로 유지된다. 감시 기준은 연속 물리 스텝 간 팔 베이스 world 위치 이동 0.002 m 이하가 0.5초 누적되는 것이다. Place 대기는 시뮬레이션 시간 15초가 지나면 실패하고, Transfer/Pick의 `WAIT_BASE_SETTLE`에는 제어기 내부 timeout이 없다. 대기가 실제로 지속되면 `phase`, `detail`, terminal result와 함께 `/cmd_vel`, `/chassis/odom`, 리프트·팔 베이스 흔들림을 확인한다.

이 절차의 통과 판정은 네 개의 실제 terminal result까지다. 이 저장소에서 정적·단위 검증은 물리 장면 실행이나 도킹 성공을 대신하지 않는다.
