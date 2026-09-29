# guidance2_27차 — `NAVIGATION` 단계 `NAV_FAILED` 원인 확정 (2026-09-29 갱신, 고피1 + 고피2 2대 구성)

이 문서만 위에서 아래로 따라 하면 됨. 이전 차수를 열 필요 없음.

**목적**: 통합 중에 나온 `NAV_FAILED` 가 (가) 라이다 프레임 간격 때문인지, (나) 두 PC 사이 네트워크 때문인지, (다) 빌드·경로 때문인지를 **한 번의 실행으로 가름.**

---

## 0. 기기 배치와 읽는 법

| 기기 | 호스트 | 이 문서에서 맡는 일 | 터미널 수 |
|---|---|---|---|
| **고피1** | `IsaacSim03` / 10.10.0.2 | **Isaac Sim 만** 돌림 | 3개 |
| **고피2** | 10.10.0.1 | **Nav2 · navigation_node · Task Manager · 결과 구독** | 9개 |

- 각 절 제목에 **어느 PC 의 몇 번째 터미널**인지 적혀 있음. 그대로 따라가면 됨.
- **`ROS_DOMAIN_ID` 는 두 PC 모두 `101`** 로 맞춤. 고피2 의 평소 값이 102 라도 이번에는 101 을 씀. 한 대라도 다르면 서로를 못 봄.
- 두 PC 모두 `/home/rokey/.ros/fastdds_whitelist.xml` 안에 `127.0.0.1` 과 `10.10.0.1~4` 가 있어야 함.
- 명령은 전부 절대경로임. 어느 디렉터리에서 실행해도 됨.

### 0-1. `NAV_FAILED` 는 서로 다른 네 실패의 공통 이름임

`navigation_node.py` 가 이 값을 내는 지점은 넷이며, **결과의 `phase` 값과 직전 로그 한 줄로 구분됨.**

| # | `phase` | 그때 찍히는 로그 | 실제 뜻 | 실패까지 |
|---|---|---|---|---|
| ① | `LAUNCH` | `navigate_to_pose 액션 서버가 없습니다.` | Nav2 미기동·두 PC 가 서로를 못 봄 | 약 5초 |
| ② | `DRIVING` | `Nav2 가 목표를 거부했습니다.` | Nav2 가 목표 자체를 거부 | 1초 안쪽 |
| ③ | `DRIVING` | 전용 문구 없음(`goToPose …` 만) | 주행 실패(ABORTED) 또는 선점당해 취소(CANCELED) | 즉시~수십 초 |
| ④ | `DOCKING` | `도킹 결과 FAILED: face_dist=… yaw_err=… lat=…` | 주행은 성공, 정밀 도킹이 실패 | 수십 초 |

**"nav_failed 가 났다" 만으로는 원인을 정할 수 없음.** 8절 터미널이 `phase` 를 자동으로 파일에 남김.

### 0-2. 지금까지 확인된 것 (읽기만)

- **스캔모드는 범인이 아님.** `nav2.launch.py:188` 의 `scan_mode` 기본값이 `auto` 라, `scan_mode:=auto` 로 준 것과 아무것도 안 준 "디폴트" 가 **같은 설정**임. 고피1·고피3 모두 `auto` 는 `cloud` 를 골랐음.
- **전처리는 범인이 아님.** 고피3 계측에서 `cloud_self_filter` → `pointcloud_to_laserscan` 이 더하는 지연은 평균 0.001초, 최대 0.05초였음. 느린 것은 **점군이 만들어지는 주기 자체**임.
- **라이다와 `/clock` 은 렌더 루프에 묶여 있음.** 렌더를 끄면 라이다 publisher 가 0개가 되고 `/clock` 이 한 건도 발행되지 않음. 그래서 렌더가 느려지면 둘이 같이 느려짐.
- **고피1 은 고피3 보다 라이다가 느리고 흔들림** (2026-09-29 12:09·12:10 실측: 0.2~3.0 Hz, 고피3 은 3.2~3.4 Hz 일정). 11절 판정표가 그 비교임.

---

## 1. 고피1 · 터미널 1 — 빌드 경로 확인 (제일 먼저. 여기가 어긋나면 나머지가 무의미함)

2026-09-29 12:04 로그에서 고피1 에 install 트리가 **둘** 있는 것이 확인됐음. 실제로 실행된 것은 `cobot3_ws` 가 빠진 저장소 루트 쪽이었음. 어느 쪽 코드가 도는지부터 확정함.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log

ls -ld /home/rokey/ROKEY_P3_A1/install /home/rokey/ROKEY_P3_A1/cobot3_ws/install
ros2 pkg prefix smart_farm_navigation
grep -n "source_timeout" $(ros2 pkg prefix smart_farm_navigation)/share/smart_farm_navigation/config/nav2_params.yaml
```

**판정**

- `ros2 pkg prefix` 가 **`/home/rokey/ROKEY_P3_A1/cobot3_ws/install/smart_farm_navigation`** 으로 나와야 정상임.
- **`/home/rokey/ROKEY_P3_A1/install/...` 로 나오면 옛 트리를 보고 있는 것임.** 아래로 정리한 뒤 이 절을 다시 실행함.

```bash
mv /home/rokey/ROKEY_P3_A1/install /home/rokey/ROKEY_P3_A1/install_OLD_20260929
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install 2>&1 | tail -5
```

- `grep` 결과가 `source_timeout: 1.0` 이면 최신 판임. 다른 값이면 옛 코드로 돌고 있었다는 뜻이므로 위 정리 후 다시 빌드함.

---

## 2. 고피1 · 터미널 2 — Isaac Sim 실행

**이 터미널만은 시스템 ROS 를 `source` 하지 않음.** Isaac 이 자체 ROS 2 Jazzy 를 씀. 그래서 환경 줄이 3줄임.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log
cd /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm

/home/rokey/isaacsim/python.sh runtime/standalone_app.py --autoplay \
  2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/isaac_$(date +%Y%m%d_%H%M).txt
```

아래 두 줄이 나오면 다음 절로 감. **이 줄이 나오기 전에는 고피2 를 시작하지 않음.**

```text
[READY] Collected_smartfarm_v014_room_core_cabbage scene ready; ...
[대기] /sim_task/command의 String/JSON 명령을 기다립니다.
```

> **중요**: Isaac 을 다시 띄우면 시뮬 시각이 0 으로 돌아감. 그때는 **고피2 의 5·6·7·9절을 전부 다시 실행**해야 함.

---

## 3. 고피1 · 터미널 3 — 라이다 "발행 측" 주기

Isaac 이 만들어 내는 주기를 **고피1 에서** 잼. 뒤에 고피2 가 받는 주기(10-1절)와 비교하면 느려지는 곳이 Isaac 인지 네트워크인지 갈림. 2절이 `[대기]` 를 찍은 뒤에 실행함.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log

timeout 60 ros2 topic hz /front_3d_lidar/lidar_points \
  2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/hz_gopi1_lidar_$(date +%Y%m%d_%H%M).txt
```

60초 뒤 저절로 끝남. `average rate:` 값을 11-2 표와 대조함. **이 값은 벽시계 기준임.**

---

## 4. 고피2 · 터미널 1 — 빌드와 노드 중복 확인

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
colcon build --symlink-install 2>&1 | tail -5
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash

ros2 node list | sort \
  | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/nodelist_gopi2_$(date +%Y%m%d_%H%M).txt
```

**판정**: `/sim_task_executor` 하나만 보여야 함(고피1 의 Isaac). 그 밖의 노드가 보이면 **지난 실행이 살아 있는 것**이므로 아래로 정리함.

```bash
pkill -f "install/smart_farm_navigation/lib"
pkill -f "install/smart_farm_manager/lib"
pkill -f "nav2_"
pkill -f "pointcloud_to_laserscan"
sleep 3
ros2 node list | sort
```

> 런치 창을 Ctrl-C 로 닫아도 **자식 노드는 살아남음.** 위처럼 실행파일 경로로 지워야 완전히 정리됨. 09-28 `NAV_FAILED` 의 원인이 이것이었음.

---

## 5. 고피2 · 터미널 2 — Nav2

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log

ros2 launch smart_farm_navigation nav2.launch.py scan_mode:=cloud use_rviz:=false \
  2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/nav2_$(date +%Y%m%d_%H%M).txt
```

`scan_mode:=cloud` 를 **명시**함. 지금까지 한 번도 명시해 본 적이 없고, `auto` 가 6초 동안 2D 라이다를 듣는 판정 자체를 건너뛰기 위함임.

이 창에 5초마다 아래 줄이 계속 올라옴. **이것이 이번 실측의 핵심 수치임.**

```text
[cloud_self_filter-1] ... : 3.2 Hz, 41276 points/scan, merged 2 msgs, 0 self points removed/scan
```

---

## 6. 고피2 · 터미널 3 — navigation_node

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log

ros2 launch smart_farm_navigation navigation_node.launch.py \
  2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/navnode_$(date +%Y%m%d_%H%M).txt
```

`navigation_node ready; destinations: [...]` 가 나오면 됨. **고피1 에서는 이것을 띄우지 않음.** 두 대에서 띄우면 목표 선점으로 `NAV_FAILED` 가 남.

---

## 7. 고피2 · 터미널 4 — Inspection (둘 중 하나만)

`PREFLIGHT` 를 통과하려면 sim_task · navigation · inspection 셋이 모두 READY 여야 함. 이번 실측은 주행이 목적이라 **가짜 노드로 충분함.**

### 7-A. 가짜 노드 (권함. 도커 필요 없음)

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log

ros2 run smart_farm_manager mock_executor --ros-args \
  -r __node:=mock_inspection_executor -p executor:=inspection \
  2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/mockinsp_$(date +%Y%m%d_%H%M).txt
```

### 7-B. 진짜 비전 컨테이너 (선택. 검사 결과까지 보고 싶을 때만)

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log
cd /home/rokey/ROKEY_P3_A1

sudo docker compose -f compose.vision.yaml up -d vision
sudo docker compose -f compose.vision.yaml exec vision \
  /entrypoint.sh ros2 launch smart_farm_vision object_detection.launch.py \
  2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/vision_$(date +%Y%m%d_%H%M).txt
```

저장소 루트 `.env` 의 `ROS_DOMAIN_ID` 도 101 이어야 함.

---

## 8. 고피2 · 터미널 5 — 결과 구독 (**사이클 시작보다 반드시 먼저**)

명령보다 늦게 띄우면 결과를 놓침. `phase` 가 여기 찍히고, 그것이 0-1 절의 ①~④ 를 가름.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log

ros2 topic echo /navigation/result \
  2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/navresult_$(date +%Y%m%d_%H%M).txt
```

---

## 9. 고피2 · 터미널 6 — `collision_monitor` 상태 구독 (이번 판의 핵심 증거)

`/scan` 이 늦어 카터가 멈춰 세워지는지를 직접 봄. `collision_monitor` 는 `cmd_vel_smoothed` → `cmd_vel` 사이에 있어 여기서 막히면 카터가 그대로 섬.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log

ros2 topic echo /collision_monitor_state \
  2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/colmon_$(date +%Y%m%d_%H%M).txt
```

`action_type: 0` 이면 통과임. **주행 중에 `1`(STOP) 이나 `2`(SLOWDOWN) 가 뜨면 그것이 `NAV_FAILED` 의 직접 원인임.**

---

## 10. 고피2 · 터미널 7 — Task Manager

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log

ros2 launch smart_farm_manager task_manager.launch.py \
  2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/taskmgr_$(date +%Y%m%d_%H%M).txt
```

아래 세 줄이 **모두** 나온 뒤에 11절로 감.

```text
Executor status changed: executor=sim_task, state=READY, ...
Executor status changed: executor=navigation, state=READY, ...
Executor status changed: executor=inspection, state=READY, ...
```

> `PREFLIGHT failed: NAV_NOT_READY` 가 뜨면 **`/clock` 이 멈췄다는 신호임**(`navigation_node` 의 상태 타이머가 시뮬 시각으로 돌기 때문). 고피1 의 Isaac 터미널이 살아 있는지부터 봄.

---

## 11. 고피2 · 터미널 8 — 사이클 시작

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log

ros2 service call /start_cycle smart_farm_interfaces/srv/StartCycle "{scenario_id: 'DEMO_HARVEST_01'}" \
  2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/startcycle_$(date +%Y%m%d_%H%M).txt
```

`accepted=True, task_id='TASK-...'` 가 나오면 시작된 것임.

**이후 진행과 대략의 소요 시간** (고피3 기준. `NAVIGATION` 까지 약 9분)

| 순서 | 단계 | 누적 |
|---|---|---|
| 1 | TRANSFER | 0 ~ 5분 |
| 2 | PICK_HARVEST | 5 ~ 8분 |
| 3 | **NAVIGATION** ← 보려는 구간 | 8 ~ 10분 |
| 4 | PLACE_INSPECT 이후 | 10분 ~ |

`NAVIGATION` 이 지나가면 목적을 달성한 것임. 뒤 단계는 끝까지 둬도 되고 중간에 멈춰도 됨.

### 11-1. 고피2 · 터미널 9 — 라이다 "수신 측" 주기 (선택이지만 권함)

고피2 터미널 7 에 `Command published: ... operation=NAVIGATION` 이 뜬 **직후** 실행함. 3절(고피1 발행 측)과 짝임.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log

timeout 60 ros2 topic hz /front_3d_lidar/lidar_points \
  2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/hz_gopi2_lidar_$(date +%Y%m%d_%H%M).txt
```

---

## 12. 판정표 — 기준값과 대조

### 12-1. 라이다 주기 (고피2 터미널 2 의 `cloud_self_filter` 줄, 시뮬 시각 기준)

| 보이는 것 | 뜻 |
|---|---|
| `3.2~3.4 Hz ... merged 2 msgs` | 고피3 정상값과 같음. **라이다는 범인이 아님** |
| `1.4~2.8 Hz`, `merged 1 msgs` 섞임 | 2026-09-29 고피1 에서 나온 값. `source_timeout` 1.0초에 걸리기 시작하는 구간 |
| `0.2~0.6 Hz` 또는 `no PointCloud2 received in the last 5 s` | **확정.** `/scan` 이 끊겨 카터가 서고 목표가 실패함 |

### 12-2. 네트워크가 범인인지 (3절 ↔ 11-1절 비교, 둘 다 벽시계 기준)

| 고피1 발행 측 | 고피2 수신 측 | 판정 |
|---|---|---|
| 약 1.0 Hz | 약 1.0 Hz | 네트워크 아님. **Isaac 렌더 주기가 원인** |
| 약 1.0 Hz | 뚜렷하게 낮음 | **네트워크가 원인.** 점군이 495 kB/장, 초당 약 0.5 MB 임 |
| 둘 다 1.0 Hz 보다 크게 낮음 | 〃 | Isaac 쪽 렌더 부하. 고피1 Isaac 설정을 고피3 과 비교해야 함 |

### 12-3. `NAV_FAILED` 가 났을 때 볼 순서

1. 고피2 터미널 5(`navresult_*.txt`)의 **`phase`** → 0-1 절 표의 ①~④ 중 무엇인지 확정됨.
2. 고피2 터미널 6(`colmon_*.txt`)에 **`action_type: 1`** 이 있는지 → 있으면 `/scan` 지연이 카터를 세운 것이 증명됨.
3. 고피2 터미널 2 의 그 시각 `cloud_self_filter` Hz → 12-1 표와 대조.

---

## 13. 끝내기와 결과 수집

### 13-1. 끄는 순서 (고피2 먼저, 고피1 나중)

고피2 의 터미널 8·7·6·5·4·3·2 를 Ctrl-C 로 끈 뒤, **고피2 터미널 1 에서** 아래로 자식 노드까지 정리함.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash

pkill -f "install/smart_farm_navigation/lib"
pkill -f "install/smart_farm_manager/lib"
pkill -f "nav2_"
pkill -f "pointcloud_to_laserscan"
sleep 3
ros2 node list | sort
```

그 다음 고피1 터미널 2 의 Isaac 을 Ctrl-C 로 끔. **Isaac 창의 Stop 버튼은 누르지 않음**(팀 `lift.py` 의 `stop()` 이 죽음).

### 13-2. 결과 올리기 (두 PC 각각에서 실행)

`results/` 는 `.gitignore` 에 걸려 있으므로 **`-f` 가 반드시 필요함.**

```bash
cd /home/rokey/ROKEY_P3_A1
git add -f cobot3_ws/src/smart_farm_navigation/results/log/
git commit -m "test(navigation): 27차 고피1+고피2 2대 실측"
git push
```

bag 은 고피2 의 `nav2.launch.py` 가 `record:=true`(기본값)로 `/home/rokey/.ros/smart_farm_navigation/bags/nav2_<날짜>/` 에 자동으로 남기므로 따로 할 것이 없음.

---

## 14. 파일명 - 역할 - 요약

### 14-1. 이 실측이 만드는 파일 (전부 `/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/`)

| 파일 | 만든 곳 | 무엇을 판정하나 |
|---|---|---|
| `isaac_*.txt` | 고피1 터미널 2 | Isaac 실행 인자·장면 준비·오류 |
| `hz_gopi1_lidar_*.txt` | 고피1 터미널 3 | 라이다 **발행 측** 주기 |
| `nodelist_gopi2_*.txt` | 고피2 터미널 1 | 노드 중복 여부 |
| `nav2_*.txt` | 고피2 터미널 2 | **`cloud_self_filter` Hz**, `scan_mode`, AMCL 초기 자세 |
| `navnode_*.txt` | 고피2 터미널 3 | `goToPose`, 목표 거부·도킹 결과 |
| `mockinsp_*.txt` | 고피2 터미널 4 | inspection READY |
| `navresult_*.txt` | 고피2 터미널 5 | **`phase`** (①~④ 판별) |
| `colmon_*.txt` | 고피2 터미널 6 | **`action_type`** (카터가 세워졌는지) |
| `taskmgr_*.txt` | 고피2 터미널 7 | 단계 전환·`PREFLIGHT`·타임아웃 |
| `startcycle_*.txt` | 고피2 터미널 8 | `task_id` |
| `hz_gopi2_lidar_*.txt` | 고피2 터미널 9 | 라이다 **수신 측** 주기 |

### 14-2. 이 실측이 들여다보는 코드 (이번 판에서 고친 것은 없음)

| 파일 | 역할 | 이번 판에서의 의미 |
|---|---|---|
| `smart_farm_navigation/navigation_node.py` | 주행 실행기 | `NAV_FAILED` 발행 4곳(309·324·381·417줄) |
| `smart_farm_navigation/cloud_self_filter.py` | 점군 자기몸통 제거·0.25초 합치기 | 5초마다 Hz 를 스스로 찍음(74~80줄) |
| `launch/nav2.launch.py` | Nav2 런치 | `scan_mode` 기본값 `auto`(188줄), 선택 결과 출력(156줄) |
| `launch/navigation_node.launch.py` | 주행 실행기 런치 | **한 대에서만** 띄워야 함 |
| `config/nav2_params.yaml` | Nav2 설정 | `collision_monitor.source_timeout: 1.0`(271줄). Nav2 기본값은 5.0 |
| `smart_farm_manager/scenario.py` | 시나리오 | 단계별 제한시간(TRANSFER 400초 등) |
