# guidance2_27차 — `NAVIGATION` 단계 `NAV_FAILED` 원인 가르기 (2026-09-29, 교육장 고피1·고피2)

이 문서만 위에서 아래로 따라 하면 됨. 이전 차수를 열 필요 없음.

**목적은 고치는 것이 아니라 가르는 것임.** `NAV_FAILED` 는 서로 다른 네 가지 실패가 같은 이름으로 나오는 값이라, 어느 것인지부터 확정해야 함. 2절까지가 그 판별이고, 3절이 그 결과에 따른 재시도임.

---

## 0. 먼저 알아 둘 것 (읽기만, 명령 없음)

### 0-1. `NAV_FAILED` 는 네 군데에서 나옴

`navigation_node.py` 가 이 값을 내는 지점은 넷이며, **결과의 `phase` 값과 그 직전 로그 한 줄로 구분됨.**

| # | `phase` | 그때 찍히는 로그 | 실제 뜻 |
|---|---|---|---|
| ① | `LAUNCH` | `navigate_to_pose 액션 서버가 없습니다.` | Nav2 가 안 떠 있거나 두 PC 가 서로를 못 봄. 명령 후 **5초** 만에 실패 |
| ② | `DRIVING` | `Nav2 가 목표를 거부했습니다.` | Nav2 가 목표 자체를 거부. 명령 후 **1초 안쪽** |
| ③ | `DRIVING` | (전용 문구 없음. 직전에 `goToPose …` 만 있음) | 목표는 받았는데 결과가 `SUCCEEDED` 가 아님 = **주행 실패(ABORTED) 또는 선점당해 취소(CANCELED)** |
| ④ | `DOCKING` | `도킹 결과 FAILED: face_dist=… yaw_err=… lat=…` | 주행은 됐고 **정밀 도킹**이 실패 |

**따라서 보고할 때 "nav_failed 났다" 만으로는 원인을 못 정함.** ①~④ 중 무엇인지, 그리고 **명령 발행 후 몇 초 만에 실패했는지**가 핵심임.

### 0-2. 스캔모드를 바꿔 본 시도는 사실상 같은 설정이었음

`nav2.launch.py` 의 `scan_mode` 기본값이 **`auto`** 임(`launch/nav2.launch.py:188`). 즉 **`scan_mode:=auto` 로 준 것과 아무것도 안 준 "디폴트" 는 완전히 같은 설정임.** 세 번의 시도가 모두 같은 조건이었으므로, 스캔모드는 아직 한 번도 바꿔 본 적이 없는 상태임.

`auto` 가 하는 일은 이것임(`launch/nav2.launch.py:62~80`).

- 런치 시작 때 `/front_2d_lidar/scan` 을 **6초 동안** 들어 봄.
- 한 장이라도 오면 → `scan2d` (2D 라이다 + `scan_sanitizer`)
- 안 오면 → `cloud` (3D 점군 → `cloud_self_filter` → `pointcloud_to_laserscan`)

지금까지 우리 실측에서 2D 라이다는 채널만 있고 발행이 안 됐으므로 `auto` 는 늘 `cloud` 를 골랐음. **그런데 고피1·고피2 두 대를 DDS 로 묶은 지금은 사정이 다를 수 있음.** 2D 라이다가 발행되면 `auto` 가 `scan2d` 를 고르는데, 이 경로는 검증량이 적어 AMCL 이 틀어지면 위 ③ 이 남. 반대로 두 PC 사이 디스커버리가 6초 안에 안 끝나면 `cloud` 를 고르고도 점군이 늦게 와서 초반에 흔들릴 수 있음.

**어느 쪽을 골랐는지는 런치 로그 한 줄에 그대로 찍힘**(2절에서 확인).

---

## 1. 터미널 1 — 중복 노드 확인 (고피1·고피2 **양쪽에서 각각** 실행)

지난 09-28 실측에서 같은 `NAV_FAILED` 가 났을 때의 원인이 **`navigation_node` 가 두 개 떠 있어 두 번째 목표가 첫 번째를 선점한 것**이었음. 두 PC 를 같은 도메인으로 묶으면 **PC 마다 하나씩 = 두 개**가 되기 쉬우므로 이것부터 봄.

> 아래 `101` 은 **Isaac 을 띄운 고피의 도메인 값**임. 고피1이면 101, 고피2면 102. **양쪽 PC 와 비전 컨테이너가 모두 같은 값**이어야 함.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log

ros2 node list | sort | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/nodelist_$(hostname)_$(date +%Y%m%d_%H%M).txt
echo "--- /navigation/command 구독자 ---"
ros2 topic info /navigation/command --verbose | grep -c "Node name" 
echo "--- navigate_to_pose 액션 서버 ---"
ros2 action info /navigate_to_pose
```

**판정**

- `ros2 node list` 에 `/navigation_node` 가 **정확히 1개**여야 함. 2개면 그것이 원인임 → 2-1 로.
- `/navigation/command` 구독자 수가 **1** 이어야 함. 2 이상이면 같은 원인임.
- `ros2 action info /navigate_to_pose` 에 서버가 **1개** 보여야 함. 0개면 위 ① 이고, 2개면 Nav2 가 두 벌 떠 있는 것임.

### 1-1. 2개 이상이면 (양쪽 PC 에서 각각 실행)

```bash
pkill -f navigation_node
pkill -f nav2.launch
pkill -f 'ros2 bag record'
sleep 2
ros2 node list | grep -c navigation_node   # 0 이 나와야 함
```

그 뒤 **Nav2 와 `navigation_node` 는 오직 한 대(Isaac 을 안 띄운 쪽)에서만** 다시 띄움.

---

## 2. 터미널 2 — Nav2 를 다시 띄우고 `scan_mode` 선택 결과를 눈으로 확인

**Isaac 을 띄우지 않은 쪽 고피**에서 실행함. Isaac 은 이미 떠 있는 상태여야 함(`/clock` 이 나와야 `auto` 판정이 정상적으로 돎).

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log

ros2 launch smart_farm_navigation nav2.launch.py scan_mode:=auto use_rviz:=false \
  2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/nav2_$(date +%Y%m%d_%H%M).txt
```

뜨자마자 아래 한 줄이 반드시 찍힘. **이 줄을 그대로 알려 줄 것.**

```
[nav2.launch] scan_mode auto -> cloud; AMCL initial pose (…, …, … deg) …
```

- `auto -> cloud` 면 지금까지와 같은 경로임 → 3절로.
- `auto -> scan2d` 면 **이번 실패의 유력 원인임**(이 경로는 검증량이 적음) → 3-1 로.

---

## 3. 터미널 3 — 스캔 경로를 `cloud` 로 못 박고 재시도 (선택이지만 권함)

2절이 `auto -> scan2d` 로 나왔거나, 어느 쪽인지 확실히 고정하고 싶을 때 씀. **`cloud` 는 지금까지 12단계 완주를 낸 경로임.**

터미널 2 를 Ctrl-C 로 끄고, 같은 터미널에서 아래를 실행함.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log

ros2 launch smart_farm_navigation nav2.launch.py scan_mode:=cloud use_rviz:=false \
  2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/nav2_cloud_$(date +%Y%m%d_%H%M).txt
```

### 3-1. 실행 순서를 지킬 것

**Isaac 을 다시 띄웠으면 Nav2 도 반드시 다시 띄움.** 시뮬레이션 시각이 0 으로 돌아가 TF·센서 시각이 어긋나고, 그 상태로 목표를 주면 Nav2 가 목표를 실패로 끝내 위 ③ 이 남. 순서는 이것임.

1. 고피(Isaac 쪽): `standalone_app.py --autoplay …`
2. 고피(Nav2 쪽): `nav2.launch.py scan_mode:=cloud use_rviz:=false`
3. 고피(Nav2 쪽): `navigation_node.launch.py`  ← **딱 한 대에서만**
4. 비전 컨테이너
5. Task Manager: `task_manager.launch.py`

---

## 4. 터미널 4 — 실패했을 때 남길 것 (이게 있어야 원인을 확정할 수 있음)

사이클을 시작하기 **전에** 먼저 띄워 두고, 실패한 뒤 Ctrl-C 로 끔.

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

그리고 **`navigation_node` 터미널의 출력도 파일로 남김.** 3절 5단계에서 `navigation_node` 를 띄울 때 아래처럼 `tee` 를 붙임.

```bash
ros2 launch smart_farm_navigation navigation_node.launch.py \
  2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/navnode_$(date +%Y%m%d_%H%M).txt
```

**넘겨줄 것 3개**

1. `navnode_*.txt` — 0-1 표의 ①~④ 를 가르는 로그가 여기 있음
2. `nav2_*.txt` 또는 `nav2_cloud_*.txt` — `scan_mode auto -> ?` 줄과 AMCL 초기 자세
3. `nodelist_*.txt` (양쪽 PC) — 중복 노드 여부

bag 은 `nav2.launch.py` 가 `record:=true`(기본값)로 `~/.ros/smart_farm_navigation/bags/nav2_<날짜>/` 에 자동으로 남기므로 따로 할 것이 없음.

---

## 5. 파일명 - 역할 - 요약

| 파일 | 역할 | 요약 |
|---|---|---|
| `cobot3_ws/src/smart_farm_navigation/smart_farm_navigation/navigation_node.py` | 주행 실행기 | `NAV_FAILED` 를 내는 네 지점(309·324·381·417줄). 이번 판에서 **고치지 않았음** |
| `cobot3_ws/src/smart_farm_navigation/launch/nav2.launch.py` | Nav2 런치 | `scan_mode` 기본값 `auto`(188줄), `auto` 판정 로직(62~80줄), 선택 결과를 로그로 출력(156줄) |
| `cobot3_ws/src/smart_farm_navigation/launch/navigation_node.launch.py` | 주행 실행기 런치 | **한 대에서만** 띄워야 함. 중복이 09-28 `NAV_FAILED` 의 원인이었음 |
| `results/log/navnode_*.txt` | 이번 판 산출물 | `phase` 와 실패까지 걸린 시간 |
| `results/log/nav2_*.txt` | 이번 판 산출물 | `scan_mode` 선택 결과, AMCL 초기 자세 |
| `results/log/nodelist_*.txt` | 이번 판 산출물 | PC 별 노드 목록(중복 확인) |

---

## 6. 고피3 실측으로 잡은 기준값 (2026-09-29 02:14~02:42, 에이전트 대행)

팀장님 진단(3D 라이다 생성 ~ 점군 전처리 구간 지연)에 맞춰 그 구간만 따로 계측했음. **팀장님 브랜치(`fix/navigation_fail_error`) 코드 그대로**, 고피3 한 대, `scan_mode auto -> cloud` 조건임.

### 6-1. 정상일 때의 값 (이 값에서 벗어나면 그 구간이 범인임)

| 항목 | 기준값 | 비고 |
|---|---|---|
| 실시간 배율 | **0.32** | 벽시계 40초에 시뮬 13초 |
| `/front_3d_lidar/lidar_points` | **시뮬 3.2~3.4 Hz / 벽시계 1.0 Hz** | 41,270점, 495 kB/장 |
| `cloud_self_filter` 자체 로그 | **`3.2~3.4 Hz, 41,27x points/scan, merged 2 msgs`** | 5초마다 자동 출력 |
| `/scan` | 시뮬 3.1 Hz | |
| 전처리가 더하는 지연 | **평균 0.001초, 최대 0.05초** | stamp 대비 `/clock` |
| `/scan` 최대 공백 | 벽시계 1.85~2.22초 = **시뮬 0.59~0.71초** | |

**핵심**: 전처리(`cloud_self_filter` → `pointcloud_to_laserscan`)가 더하는 지연은 **1 ms 수준으로 병목이 아님.** 지연의 실체는 **라이다 프레임이 만들어지는 주기 그 자체**임.

### 6-2. 왜 그런가 — 라이다와 `/clock` 은 렌더 루프에 묶여 있음

`--headless` 로 띄우면(`standalone_app.py:1514`, `render=(not args.headless ...)`) 다음이 동시에 일어남을 실측으로 확인했음.

- `/front_3d_lidar/lidar_points` 의 **publisher 가 0개**가 됨 (RTX 라이다는 렌더 결과물임)
- `/clock` 이 토픽만 있고 **한 건도 발행되지 않음**
- 그 결과 `navigation_node` 의 상태 타이머(시뮬 시각 기준)가 영영 안 돌아 Task Manager 가 **`PREFLIGHT failed: NAV_NOT_READY`** 를 냄

즉 **렌더가 느려지면 라이다 주기와 `/clock` 이 같이 느려짐.** Nav2 는 전 노드가 `use_sim_time: True` 라 `/clock` 이 끊기면 제어 주기·TF 보간·센서 유효성 판정이 한꺼번에 흔들림. 이것이 `NAV_FAILED` 로 이어지는 경로임.

### 6-3. 가장 의심스러운 파라미터 — `collision_monitor.source_timeout`

`config/nav2_params.yaml:271` 의 `source_timeout: 1.0` (시뮬초)임. **Nav2 기본값은 5.0 이라 우리 설정이 5배 빡빡함.** `collision_monitor` 는 `cmd_vel_smoothed` → `cmd_vel` 사이에 있어 여기서 막히면 카터가 그대로 섬.

고피3 실측의 `/scan` 최대 공백이 시뮬 0.71초이므로 **여유가 0.3초뿐임.** 고피1·고피2 는 Isaac 과 Nav2 가 다른 PC라 495 kB 짜리 점군이 매번 네트워크를 건너오므로(초당 약 0.5 MB) 공백이 더 벌어지기 쉬움.

### 6-4. 고피1·고피2 에서 볼 것 (터미널 추가 없이 지금 로그로 확인됨)

Nav2 를 띄운 터미널에 `cloud_self_filter` 가 5초마다 찍는 줄이 이미 나오고 있음. 그 줄만 보면 됨.

| 보이는 것 | 뜻 |
|---|---|
| `3.2~3.4 Hz ... merged 2 msgs` | 고피3 정상값과 같음. 이 구간은 범인이 아님 |
| `1.x Hz ... merged 1 msgs` | 렌더가 느려진 상태. `source_timeout` 1.0초에 걸리기 시작함 |
| `0.x Hz` 또는 `no PointCloud2 received in the last 5 s` | **확정적임.** `/scan` 이 끊겨 `collision_monitor` 가 카터를 세우고 Nav2 가 목표를 실패시킴 |

`PREFLIGHT failed: NAV_NOT_READY` 가 뜬다면 그것은 **`/clock` 이 멈췄다는 신호**임(6-2 참조). 스캔모드와 무관함.

### 6-5. 아직 손대지 않은 조치 후보 (승인 필요)

아래는 전부 ADR_nav2 §2.6 의 "keep" 기준선이라 보고만 하고 고치지 않았음.

1. `collision_monitor.source_timeout` **1.0 → 3.0** (`config/nav2_params.yaml:271`). 라이다 주기가 느린 환경에서 카터가 멈춰 서는 것을 막음. 안전 여유는 줄지만 시뮬에서는 타당함. **가장 작은 수정이고 효과가 직접적임.**
2. `cloud_self_filter.accumulate_s` **0.25 → 0.5** (`launch/nav2.launch.py`). `merged 1 msgs` 가 이어질 때 유효함.
3. Isaac 창 크기·렌더 해상도를 줄여 프레임 주기를 당김. 고피3 에서 비전 스테이션 on/off 는 라이다 주기에 **영향이 없었음**(3.15 vs 3.28 Hz)이라 카메라를 끄는 것은 효과가 없음.

### 6-6. 이번 실측에서 같이 확인된 것

- **비전 스테이션은 라이다 주기와 무관함.** `--no-vision-station` 켠 판 3.15 Hz, 끈 판 3.28 Hz 로 차이 없음.
- **이미 죽은 Isaac 이 물려 있던 가상 디스플레이를 재사용하면 Isaac 이 기동 직후 segfault 로 죽음**(3회 연속). 디스플레이를 새로 띄우면 같은 인자로 정상 기동함(2회 연속). 코드 문제가 아님.
- `ros2 node list` 중복은 **런치 래퍼만 죽였을 때** 생김. 자식 노드는 살아남으므로 실행파일 경로로 지워야 함(1-1 절).
