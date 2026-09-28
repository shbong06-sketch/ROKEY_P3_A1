# guidance3_27차 — 관제 웹·DB 띄우기와 전 구간 기록

- 작성 2026-09-28 · **2026-09-28 `guidance/past_monitor_guidance/` 로 이동함**(사용자 지시: 사용자가 당장 해야 할 일 목록이 아니면 `past` 로 보내고 그곳에서 열람함. 이 판부터 실측·녹화는 에이전트가 대행함)
- 기기 **고피3**(`gc-isaacsim-lwh`, GCP VM) 한 대 기준
- 이 문서만 위에서 아래로 따라가면 됨. 이전 차수를 열어볼 필요 없음
- `guidance2_26차.md` 는 그대로 살아 있음. 그쪽은 **Isaac·Nav2 주행·도킹 절차**, 이 문서는 **관제 웹·DB** 임. 두 문서는 4장에서 만남

---

## 0. 이번 판에서 알아 둘 것 (읽기만)

### 0-1. 무엇이 새로 생겼나

새 ROS 2 패키지 **`smart_farm_monitor`** 임. 두 프로세스로 나뉨.

| 프로세스 | 하는 일 |
|---|---|
| `recorder` (rclpy 노드) | 공정 토픽을 받아 SQLite 파일 하나(`farm.db`)에 적음. 웹이 넣은 시작 요청을 읽어 `/start_cycle` 서비스를 대신 호출함 |
| `web` (FastAPI) | 같은 파일을 **읽기 전용**으로 조회해 화면 5개와 API 를 제공함. **ROS 를 전혀 쓰지 않음** |

나누는 이유는 한 프로세스에 "계속 듣는 일"과 "물어보면 답하는 일"을 같이 시키면 스레드·asyncio 가 필요해지고, 그건 ADR 에서 쓰지 않기로 한 것이기 때문임.

### 0-2. 기록되는 토픽

`/cycle/status`, `/sim_task/{command,result,status}`, `/sim_task/inspection_data_status`, `/navigation/{command,result,status}`, `/inspection/{command,result,status}`, `/inspection/detections_2d`, `/feeder_dock/result`(latched), `/amcl_pose`.

### 0-3. 시각을 두 가지로 적음

모든 행에 `*_wall`(컴퓨터 시계)과 `*_sim`(`/clock`)을 함께 적음. Isaac 실시간 배율이 0.3 전후라 **벽시계 경과 ≠ 공정 소요시간**임. 사이클 시간 분석은 `*_sim` 으로만 함.

`/clock` 이 아직 없으면 `*_sim` 은 **0 이 아니라 빈 값**으로 남음.

### 0-4. 2026-09-28 에 고친 것 (이 판에서 반드시 다시 빌드해야 하는 이유)

`use_sim_time: true` 인데 Isaac 이 아직 안 떠서 `/clock` 이 없으면, **웹의 시작 버튼을 눌러도 응답이 오지 않았음.** 우편함을 읽는 타이머가 노드 시계(시뮬레이션 시각)로 돌아서 시각이 0 에 멈춰 아예 뛰지 않았기 때문임.

우편함 타이머를 **벽시계(`ClockType.STEADY_TIME`)** 로 바꿨음. 공정 시간을 재는 것이 아니라 "웹이 뭔가 넣었는지" 보는 것이므로 벽시계가 맞음. 회귀 시험 `test_web_command_drains_without_clock` 으로 고정했음.

### 0-5. 검증 상태

| 항목 | 상태 |
|---|---|
| `colcon build` (monitor, navigation) | **통과** (고피3) |
| 시험 39건 (`store` 17 + `recorder` 7 + `web` 15) | **통과** (고피3, Isaac 없이) |
| 화면 5개 · SSE · 지도 API · 기록 배선 | **실기동 확인** (고피3, 가짜 토픽) |
| 4장 전 구간(Isaac + Nav2 + 검사 + Task Manager + 관제) | **미검증.** 팀 문서와 26차 절차를 합친 것임 |

---

## 1. 터미널 1 — 처음 한 번만 (설치와 빌드)

의존은 **apt** 로 넣음. 우분투 24.04 는 `pip install` 이 막혀 있음(PEP 668). 고피1·고피2·내피도 같은 명령임.

```bash
sudo apt-get install -y python3-fastapi python3-uvicorn python3-httpx
```

빌드.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
cd /home/rokey/ROKEY_P3_A1/cobot3_ws && colcon build --packages-select smart_farm_interfaces smart_farm_manager smart_farm_navigation smart_farm_monitor 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_monitor/results/log/build_monitor_$(date +%Y%m%d_%H%M).txt
```

기대: `4 packages finished`, 실패 0.

시험도 한 번 돌림. **도메인 77 로 격리**해서 다른 노드에 끼지 않게 함.

```bash
export ROS_DOMAIN_ID=77
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
cd /home/rokey/ROKEY_P3_A1/cobot3_ws && python3 -m pytest src/smart_farm_monitor/test -q 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_monitor/results/log/montest_$(date +%Y%m%d_%H%M).txt
```

기대: `39 passed`.

---

## 2. 터미널 2 — 관제만 먼저 띄워 화면 확인 (Isaac 없이 됨)

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_monitor monitor.launch.py 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_monitor/results/log/monitor_$(date +%Y%m%d_%H%M).txt
```

기대 로그:

```
[monitor_recorder]: monitor_recorder ready: db=/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_monitor/data/farm.db
관제 웹: http://0.0.0.0:8080  (DB: .../data/farm.db)
INFO:     Uvicorn running on http://0.0.0.0:8080
```

브라우저에서 엶.

| 상황 | 주소 |
|---|---|
| 고피3 화면이 없으니 **내 노트북 브라우저**에서 | `http://<고피3 외부 IP>:8080` |
| 고피3 안에서 확인만 | `curl -s http://127.0.0.1:8080/api/state` |

선택 인자.

| 인자 | 뜻 |
|---|---|
| `web:=false` | 기록만 하고 웹은 띄우지 않음 |
| `port:=9090` | 웹 포트를 바꿈 (기본 8080) |

**처음 열면 표가 전부 비어 있음.** 아직 공정을 돌린 적이 없기 때문임. 화면 틀·지도·KPI 칸은 다 떠야 정상임.

지도가 보이는지로 판단함. 지도만 확인하려면:

```bash
curl -s http://127.0.0.1:8080/api/map | python3 -m json.tool | head -12
```

기대: `"available": true`, `"resolution": 0.05`, `"origin_x": -4.525`, `"origin_y": -10.025`, `"width_px": 285`, `"height_px": 460`, `stations` 5개.

---

## 3. 시작 버튼을 눌렀을 때 나오는 말 (읽기만)

버튼은 **웹이 직접 Isaac 을 움직이지 않음.** `web_command` 표에 줄 하나를 넣고, `recorder` 가 0.5 초마다 그것을 읽어 `/start_cycle` 을 대신 호출함.

| 버튼에 나오는 말 | 뜻 | 할 일 |
|---|---|---|
| **수락됨 — 진행 중** | Task Manager 가 사이클을 시작함 | 화면을 봄 |
| **거절: SERVICE_NOT_AVAILABLE** | `/start_cycle` 을 제공하는 **Task Manager 가 안 떠 있음**. 2장처럼 관제만 띄운 상태면 정상적으로 이렇게 나옴 | 4장으로 전 구간을 띄움 |
| **거절: BUSY** | 이미 사이클이 돌고 있음 | 끝나기를 기다림 |
| **거절: INVALID_SCENARIO** | 시나리오 이름이 다름 | 허용값은 `DEMO_HARVEST_01` 뿐임 |
| **응답 없음 — recorder 노드 확인** | 우편함을 읽는 노드가 안 돌고 있음 | 2장 터미널에 `monitor_recorder ready` 가 있는지 봄. 없으면 다시 띄움 |
| **응답 없음 — Task Manager 확인** | 서비스는 있는데 응답이 없음 | Task Manager 터미널의 오류를 봄 |

> 2026-09-28 에 **`거절: SENT`** 가 나온 적이 있음. 그건 위 0-4 의 결함이었고 지금은 고쳤음. 1장으로 다시 빌드하면 같은 상황에서 **`거절: SERVICE_NOT_AVAILABLE`** 로 바르게 나옴.

터미널에서 직접 확인하려면:

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 service list | grep start_cycle
```

아무것도 안 나오면 Task Manager 가 없는 것임.

---

## 4. 전 구간 (Isaac + Nav2 + 주행 + 검사 + Task Manager + 관제)

**고피3 에서 이 순서 전체를 통과시킨 기록은 아직 없음.** 팀 문서 `docs/TASK_MANAGED_INSPECTION_CHECK.md` 의 기동 순서와 `guidance2_26차.md` 의 고피3 명령을 합친 것임. 처음 돌릴 때는 5장의 관찰 지점을 함께 봄.

터미널은 여섯임. **순서를 지킴.**

| 터미널 | 무엇 | 다음으로 넘어가는 신호 |
|---|---|---|
| 1 | Isaac | `[대기] /sim_task/command …` |
| 2 | Nav2 | `Managed nodes are active`, `[feeder_dock]: [IDLE] waiting` |
| 3 | 주행 노드 | `navigation_node ready; destinations: …` |
| 4 | 검사 노드(Docker) | 모델 적재 후 `READY` |
| 5 | **관제** | `monitor_recorder ready`, `Uvicorn running` |
| 6 | Task Manager | `Task Manager ready: scenario=DEMO_HARVEST_01` |

### 4-1. 터미널 1 — Isaac

**이 터미널에서는 워크스페이스를 `source` 하지 않음**(팀 앱이 Isaac 번들 ROS 라이브러리를 먼저 써야 함). **`--headless` 를 쓰지 않음.** 화면이 없으니 가상 디스플레이로 감쌈.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1
xvfb-run -a -s "-screen 0 1920x1080x24" /home/nitrouriah92/isaacsim/python.sh /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py --autoplay 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/isaac_$(date +%Y%m%d_%H%M).txt
```

- 26차와 다른 점 하나: **`--no-vision-station` 을 붙이지 않음.** 전 구간은 검사 공정까지 돌리므로 비전 스테이션이 있어야 함
- Isaac 경로의 `$HOME` 만은 다른 기기와 다름(`/home/nitrouriah92`). 위 경로를 그대로 씀
- 씬 로딩까지 **3~5 분** 걸림. `[READY]` 다음 `[대기]` 가 나오면 됨
- 첫 줄 씬 이름에 `_cabbage` 가 없으면 v014 폴더를 못 찾은 것임. `--scene /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd` 를 붙여 다시 띄움
- **Isaac 의 Stop 버튼을 누르지 않음.** 끝낼 때는 그 판에서 `Ctrl+C`
- **`pkill -f` 를 쓰지 않음.** 명령줄 전체를 보므로 다른 셸까지 죽임. `ps -ef | grep isaacsim | grep -v grep` 으로 PID 를 찾아 `kill <PID>`
- **Isaac 을 다시 띄우면 2·3·5·6 번 터미널도 다시 띄움.** 시뮬 시각이 0 으로 돌아가 TF·센서 시각이 어긋남

### 4-2. 터미널 2 — Nav2

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation nav2.launch.py scan_mode:=auto use_rviz:=false record:=true 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/nav2_$(date +%Y%m%d_%H%M).txt
```

기대: `scan_mode auto -> cloud` → `AMCL initial pose (-0.421, 1.006, 90.0deg)` → `Managed nodes are active` → `[feeder_dock]: [IDLE] waiting`.

`use_rviz:=false` 는 이 VM 에 화면이 없기 때문임. 초기 위치는 launch 가 AMCL 에 직접 넣으므로 `2D Pose Estimate` 클릭이 필요 없음. `record:=true` 는 판정 근거임.

### 4-3. 터미널 3 — 주행 노드

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation navigation_node.launch.py 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/navnode_$(date +%Y%m%d_%H%M).txt
```

기대: `navigation_node ready; destinations: ['FEEDER_DOCK', 'RACK_DOCK', 'CORRIDOR_EXIT']`.

### 4-4. 터미널 4 — 검사 노드 (Docker · 팀 구성)

컨테이너가 처음이면 먼저 올림.

```bash
cd /home/rokey/ROKEY_P3_A1 && sudo docker compose -f compose.vision.yaml up -d --force-recreate vision
```

그 다음 이 터미널을 유지함.

```bash
cd /home/rokey/ROKEY_P3_A1 && sudo docker compose -f compose.vision.yaml exec vision /entrypoint.sh ros2 launch smart_farm_vision object_detection.launch.py config_file:=/config/object_detection.yaml 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/vision_$(date +%Y%m%d_%H%M).txt
```

검사 노드는 **모델을 적재하고 `/rgb` 영상을 실제로 받은 뒤에야 `READY`** 가 됨. 이게 안 되면 Task Manager 의 PREFLIGHT 가 `INSPECTION_NOT_READY` 로 떨어짐.

> 컨테이너의 `ROS_DOMAIN_ID` 가 고피3 와 같아야 함. 다르면 토픽이 서로 안 보임.

### 4-5. 터미널 5 — 관제 (2장과 같은 명령)

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_monitor monitor.launch.py 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_monitor/results/log/monitor_$(date +%Y%m%d_%H%M).txt
```

**Task Manager 보다 먼저 띄움.** 그래야 첫 명령부터 빠짐없이 기록됨.

### 4-6. 터미널 6 — Task Manager

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_manager task_manager.launch.py 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/log/taskmgr_$(date +%Y%m%d_%H%M).txt
```

기대: `Task Manager ready: scenario=DEMO_HARVEST_01`.

### 4-7. 시작

**웹 대시보드의 `DEMO_HARVEST_01 시작` 버튼을 한 번 누름.** `수락됨 — 진행 중` 이 나와야 함.

터미널로 하고 싶으면 아래도 같음(둘 중 하나만 함).

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 service call /start_cycle smart_farm_interfaces/srv/StartCycle "{scenario_id: DEMO_HARVEST_01}"
```

> **한 실행에 한 번만 호출함.** 실패한 뒤 같은 장면에서 다시 호출하지 않음(팀 문서 규칙).

---

## 5. 화면에서 볼 것

| 화면 | 주소 | 보는 것 |
|---|---|---|
| 대시보드 | `/` | 지표 6개, 현재 단계, 실행 노드 램프 4개, **공정 12단계 체크리스트**, 지도 위 로봇, 최근 도킹, 이상 배너 |
| 사이클 이력 | `/history` | 사이클 목록, 단계별 소요시간 막대, **operation 별 sim 소요시간 통계**, 정상·이상 라벨 버튼 |
| 검사 결과 | `/inspection` | 슬롯 6칸(불량 빨강·미판정 노랑), 판정 기록, 검출 좌표 |
| 도킹 품질 | `/dock` | 성공률·평균 재시도, 횡 오차 분포(허용 ±0.06 점선) |
| 트레이 추적 | `/pallet` | 팔레트별 현재 논리 위치와 이동 이력 |

### 처음 돌릴 때 특히 볼 것

1. **실행 노드 램프 4개가 다 초록(READY)인가.** 하나라도 회색이면 PREFLIGHT 가 통과하지 않음. `sim_task_data` 는 검사 데이터 저장 상태이므로 검사 공정 전에는 회색이 정상임
2. **지도에 로봇 점이 찍히는가.** 안 찍히면 `/amcl_pose` 가 없는 것이므로 Nav2 를 봄
3. **12단계 체크리스트가 순서대로 ✓ 로 바뀌는가**
4. **이상 배너에 "늦게 도착한 결과" 가 뜨는가.** 뜨면 그 단계의 `timeout_sec` 이 짧다는 뜻임. 이력 화면의 통계와 함께 적어 둠
5. **`/history` 의 operation 별 sim 소요시간.** 이 값이 팀 `scenario.py` 의 `timeout_sec` 을 `/clock` 기준으로 다시 잡을 근거임

---

## 6. 안 될 때

| 증상 | 원인과 조치 |
|---|---|
| 웹이 안 열림 | 포트가 이미 쓰이고 있음. `ss -ltnp \| grep 8080` 으로 PID 를 찾아 `kill <PID>`. 또는 `port:=9090` 으로 띄움. **`pkill -f` 를 쓰지 않음** |
| 화면은 열리는데 표가 전부 비어 있음 | 공정을 돌린 적이 없거나 도메인이 다름. `ros2 topic hz /cycle/status` 로 토픽이 오는지 봄 |
| 시작 버튼이 `SERVICE_NOT_AVAILABLE` | Task Manager 가 없음. 4-6 을 띄움 |
| 시작 버튼이 `응답 없음 — recorder 노드 확인` | 2장 터미널에 `monitor_recorder ready` 가 있는지 봄. 없으면 다시 띄움 |
| 소요시간이 전부 빈 값 | `/clock` 이 없음. Isaac 이 떠 있는지, `use_sim_time` 이 `true` 인지 봄 |
| 도킹 표가 비어 있는데 도킹은 했음 | `/feeder_dock/result` 는 latched(`RELIABLE`+`TRANSIENT_LOCAL`)임. 직접 발행해 시험할 때는 `ros2 topic pub --qos-durability transient_local --qos-reliability reliable` 를 붙여야 함 |
| 램프가 `sim_task` 만 초록 | 검사 노드(4-4)나 주행 노드(4-3)가 없음 |
| 검사 램프가 계속 회색 | 모델 적재나 `/rgb` 수신이 안 됨. 4-4 터미널 로그를 봄 |
| 사이클을 다시 돌리고 싶음 | Isaac Stop→Play 만으로는 Task Manager 가 초기화되지 않음. **4-1 을 다시 띄우고 2·3·5·6 도 다시 띄움** |

---

## 7. 기록 남기기

### 7-1. DB 파일 위치

```
/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_monitor/data/farm.db
```

**git 에 올라가지 않음**(`.gitignore`). 다른 기기로 옮길 때는 `scp` 로 이 파일 하나만 복사하면 됨.

표로 바로 보고 싶으면:

```bash
sqlite3 /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_monitor/data/farm.db \
  "SELECT operation, status, round(duration_sim,1) AS sim_s, late_result FROM step ORDER BY id;"
```

`sqlite3` 명령이 없으면 `sudo apt-get install -y sqlite3`.

### 7-2. 화면 스냅샷 (Isaac 을 돌렸으면 의무)

Isaac 을 실행한 회차는 **반드시 미디어를 남김.** 폴더 이름에 무엇을 했는지와 시각을 넣음.

```bash
mkdir -p /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_monitor/results/log_media/monitor_full_cycle_$(date +%Y%m%d_%H%M)
```

여기에 웹 화면 캡처와 Isaac·RViz2 영상을 넣음. 이 폴더는 git 제외이므로 **무엇을 찍었는지는 보고로 남김**. 화면 3분할 녹화가 필요하면 같은 폴더의 `트러블슈팅_Isaac_VM_녹화환경_20260926.md` 와 `scripts/record_rig.sh` 를 씀.

### 7-3. 터미널 로그

이 문서의 모든 명령은 `tee` 로 `results/log/` 에 남김. 그 폴더는 git 추적 대상임.

---

## 8. 파일과 역할

| 파일 | 역할 |
|---|---|
| `cobot3_ws/src/smart_farm_monitor/smart_farm_monitor/schema.sql` | DB 표 10개 정의 |
| `…/smart_farm_monitor/store.py` | SQLite 읽기·쓰기. ROS 의존 없음 |
| `…/smart_farm_monitor/recorder_node.py` | 토픽 → DB 기록, 우편함 → `/start_cycle` 호출 |
| `…/smart_farm_monitor/web_app.py` | 화면 5개와 조회 API, SSE 실시간 갱신 |
| `…/web/index.html` | 대시보드(지표·단계·지도·배너·시작 버튼) |
| `…/web/history.html` | 사이클 이력·간트·소요시간 통계·라벨링 |
| `…/web/inspection.html` | 슬롯별 검사 결과 |
| `…/web/dock.html` | 도킹 품질과 횡 오차 분포 |
| `…/web/pallet.html` | 트레이 위치 추적 |
| `…/config/monitor.yaml` | `use_sim_time`, `db_path`, 폴링 주기 |
| `…/launch/monitor.launch.py` | 기록+웹 동시 기동 (`web:=false`, `port:=`) |
| `…/data/farm.db` | 기록 원본. git 제외 |
| `…/test/` | 시험 39건. ROS·Isaac 없이 돌아감 |
| `smart_farm_monitor/results/log/` | 관제 쪽 `tee` 로그 (git 추적) |
| `smart_farm_monitor/results/log_media/` | 관제 화면 캡처·녹화 (git 제외, `*.md` 만 추적) |
| `smart_farm_navigation/results/log/` | Isaac·Nav2·주행 쪽 `tee` 로그 (git 추적) |
| `docs/04-monitoring-web-db.md` | 관제 설계의 단일 출처 |
| `guidance2_26차.md` | Isaac·Nav2 주행·도킹 절차 (이 문서 4장이 참조) |
