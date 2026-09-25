# guidance2_26차 — 주행·도킹·Place 전 구간 (고피에서 지시, 내피에서 Nav2)

- 작성일: 2026-09-25, 브랜치 `feature/Inspection-Place-nav2`. 25차와 절차는 같고 **도킹 거리만 0.85 → 0.90 m 로 바뀐 판**이다. 25차는 `guidance/past/` 로 옮겼다. **아직 실측하지 않았다. 검증은 내피 모의뿐이다(ADR 2.7: 모의 통과 ≠ 실측 통과).**
- **PC 분담**: Nav2 쪽(Nav2·RViz2·정밀 도킹·주행 노드)만 내피(`lwh19180`)에서 돌리고, 나머지(Isaac, 명령 발행, 나중의 Task Manager)는 모두 고피(`IsaacSim03`)에서 돌린다.
- 그래서 **고피에도 워크스페이스를 한 번 빌드해야 한다.** 명령에 쓰는 `smart_farm_interfaces` 메시지가 거기 있어야 하기 때문이다. 18차 실측이 막힌 지점이 이것이다.
- `ROS_DOMAIN_ID` 는 두 PC 모두 **101**. 비전 컨테이너를 띄운다면 그것도 101 이다.
- 장면 `Collected_smartfarm_v011.usd`, 지도는 패키지에 설치된 `Collected_smartfarm_v011.yaml` 을 자동으로 쓴다.
- 아래 순서대로 위에서 아래로 실행한다. 블록마다 환경 줄이 들어 있으므로 그대로 복사해 붙이면 된다.

## 0. 이번 판에서 알아 둘 것 (읽기만)

| 항목 | 내용 |
|---|---|
| 고피 빌드 | 명령을 보내는 터미널에는 `smart_farm_interfaces` 가 필요하다. 고피에서 1회 빌드하면 그 뒤로는 `source` 만 하면 된다 |
| Isaac 터미널 주의 | **Isaac 을 띄우는 터미널에서는 워크스페이스를 `source` 하지 않는다.** 팀 앱이 Isaac 번들 ROS 라이브러리를 먼저 쓰도록 되어 있어 시스템 ROS 와 섞이면 죽는다. 빌드와 명령 발행은 Isaac 과 다른 터미널에서 한다 |
| 주행 실행 방식 | `navigation_node` 가 NavigateToPose 액션을 직접 보내고, 도착하면 명령 식별자를 실어 정밀 도킹을 시작시킨다. 같은 식별자로 돌아온 결과만 인정한다 |
| 도킹 시작 | 자동 시작은 꺼져 있다. 명령으로만 시작한다. RViz2 클릭으로 손 시험할 때만 `dock_auto:=true` 로 띄운다 |
| 도킹 거리 | 라이다가 검출하는 면(world y −3.645)에서 **0.90 m**. 팔 밑동에서 place 대상까지가 0.96 m 가 되며, 랙 pick 에서 실제로 성공한 자세 0.932 m 와 팀 상한 1.05 m 사이다. 25차의 0.85 m 는 환산식이 0.05 m 틀려 실제로는 0.91 m 였고 하한 0.89 m 와의 여유가 도킹 공차 0.03 m 보다 작았다 |
| 제한 시간 | NAVIGATION 단계 400 s, 주행 노드 600 s, 도킹 240 s |
| 멈춤 감지 | 위치와 방향을 함께 본다. 제자리 회전은 위치가 거의 안 변하므로 방향 변화도 이동으로 인정한다. 단계가 바뀌면 기준점을 새로 잡는다 |
| 도킹 시작 | `SETTLE` 단계가 생겼다. Nav2 `/cmd_vel` 이 2 s 조용하고 차체 각속도가 0.03 rad/s 아래여야 정렬을 시작한다(ADR 2.2). 로그에 `[SETTLE]` → `[ALIGN_TO_GOAL]` 순서로 보인다 |
| 도킹 조향 | 차체 각속도가 명령을 1~3 s 늦게 따라오는 것을 전제로 짰다. 후진 0.10 m/s, 조향 상한 0.12 rad/s 라 화면상 느리게 보인다. 정상이다 |
| 실측 전 회귀 | **내피에서 `sim_test/dock_regression.py` 를 먼저 돌린다(4-1절).** Isaac 없이 돈다. 실측 중에는 절대 돌리지 않는다(같은 도메인에서 Nav2 가 두 벌 뜬다) |
| 팀 앱 | `standalone_app.py` main loop 에서 운반 중 팔레트 감시 예외를 잡는다. Isaac 이 종료되는 대신 오류 한 줄이 남고 계속 돈다. 자세한 것은 `docs/standalone_app_changes.md` 2-1 |
| 발행 명령 | 모두 `--once --max-wait-time-secs 15`. 15 초 안에 구독자를 못 찾으면 무한 대기 대신 오류로 끝난다 |
| Place 직전 검증 | 팀 앱이 차체 정지를 확인한 뒤 브레이크를 걸고, 실제 정지 위치와 place 대상까지의 거리를 로그로 남긴다. 판정은 하지 않는다 |

### 24차 실측에서 막힌 곳 (읽기만)

Nav2 주행 성공, 23차의 멈춤 오판도 사라짐. 그 뒤 두 가지가 겹쳤다.

| 시점(bag) | 기록 |
|---|---|
| 291.4 | `ALIGN_TO_GOAL` 시작, 방향 오차 30도 |
| 295.1 | `REVERSE` 시작, 방향 오차 0.1도 — **그러나 차체는 아직 0.24 rad/s 로 돌고 있었다** |
| 296~297 | 관성으로 15도 더 돌아감. 후진하며 도킹 선에서 옆으로 0.16 m 벗어남 |
| 304.5 | `CHECK` 도착 방향 **+21.2도** → `BACKOFF` |
| 310.7 | 물러남 완료, 다시 정렬 |
| 313.7 | Isaac 종료: `RuntimeError: HOLD: 운반 중 팔레트가 30.1 mm 미끄러졌습니다` (팀 감시 예외가 main loop 에서 안 잡힘) |

원인 두 가지와 조치.

| 원인 | 내용 | 조치 |
|---|---|---|
| 차체 각속도 응답이 느리다 | 0.35 rad/s 를 명령해도 3 s 걸려 0.26, 끊어도 초당 0.14 씩만 줌(Isaac DifferentialController 가속 제한). 22차엔 제자리 회전이 명령의 12% 만 나왔다 | 정렬 종료에 "실제 각속도 < 0.03" 조건. 모든 조향은 관성으로 더 돌 각도와 측정 지연을 뺀 오차로 계산. 시작 전 `SETTLE` |
| 점을 겨누는 후진 조향 | 남은 거리가 짧을 때 횡 오차를 방향으로 갚다가 도착 방향이 틀어진다 | 면 법선(도킹 선)을 따라가되 횡 보정용 방향 이탈을 10도로 제한, 마지막 0.4 m 는 직각만. 정렬은 G 가 아니라 더 먼 면 가운데를 겨눔. 횡 0.06 m 넘으면 0.9 m 물러나 재시도(최대 2회) |
| 팀 감시 예외로 앱 종료 | 운반 중 팔레트 30 mm 이동 → 예외 → Isaac 종료 | main loop 에서 잡아 한 번 기록하고 계속. PLACE 때 같은 검사가 `MOTION_FAILED` 로 보고 |

진단의 근거(모의가 아니라 실측 기록):

기록된 `/scan` 을 도킹 노드와 같은 검출기에 다시 넣어 보면, **센서는 멀쩡했고 제어가 잘못 판단했다**는 것이 바로 보인다.

| bag 시각 | 면까지 | 방향 오차 | 횡 | G 방위 | 실제 cmd_w | odom 각속도 |
|---|---|---|---|---|---|---|
| 295.1 `[REVERSE]` 선언 | 1.99 | +0.1° | −0.01 | +0.3° | +0.116 | **+0.227** ← 아직 돌고 있다 |
| 296.1 | 1.99 | −8.2° | +0.34 | −11.2° | −0.165 | +0.144 |
| 297~302 | 1.9→1.2 | −15°→+9° | +0.60→0.02 | **−17~−24° 5 초 지속** | **−0.200 고정** | −0.10~−0.14 |
| 303.5 | 1.01 | **+20.7°** | −0.28 | −5.9° | −0.036 | −0.045 |

같은 것을 다시 보려면(내피, Isaac 불필요):

```bash
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
cd /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation
python3 sim_test/replay_bag_dock.py ~/.ros/smart_farm_navigation/bags/nav2_20260923_2154 --from 295 --to 306
```

검증(내피 모의, 실측 아님):

| 시험 | 내용 | 결과 |
|---|---|---|
| 오프라인 격자 | 시작 자세 45종 × 차체 5종 = 225 케이스 | 방향 3°·거리 ±0.05·충돌 없음·횡 0.06 m **225/225**, 최대 횡 0.059 m |
| 이전 이득 비교 | 같은 격자를 `--old` 로 | 횡 기준 206/225, 최대 0.127 m |
| ROS 회귀 10 | 현장과 같은 launch + 합성 로봇(같은 차체 모델) | **10/10 통과** (`results/regression_20260924_sim.txt`) |

ROS 회귀 10 시나리오 결과(내피 모의):

| 시나리오 | 조건 | 결과 |
|---|---|---|
| normal | 접근 자세, 실측형 차체 | 면 0.879 m, 방향 −0.03°, 좌우 +0.003 m |
| yaw_pos_20 | 도착 방향 +20° | 면 0.879 m, 방향 +0.47°, 좌우 −0.006 m |
| yaw_neg_30 | 도착 방향 −30° (24차 시작각) | 면 0.879 m, 방향 −0.38°, 좌우 −0.032 m |
| lat_left | 접근점 0.2 m 왼쪽 | 면 0.867 m, 방향 −0.42°, 좌우 −0.046 m (재시도 1회) |
| lat_right | 접근점 0.2 m 오른쪽 | 면 0.878 m, 방향 +0.30°, 좌우 +0.021 m (재시도 1회) |
| far | 접근점 0.4 m 뒤 | 면 0.878 m, 방향 −0.02°, 좌우 +0.003 m |
| weak_turn | 제자리 회전이 명령의 12% 만 나오는 차체(22차) | 면 0.879 m, 방향 +0.48°, 좌우 +0.004 m |
| ideal | 명령을 즉시 따르는 차체 | 면 0.880 m, 방향 +0.15°, 좌우 −0.013 m |
| jam | 후진 중 구조물에 끼임 | `STALLED_REVERSE` (제때 멈춤) |
| no_face | 면이 안 보이는 곳에서 시작 | `FACE_NOT_FOUND` |

남은 한계: 횡 오차는 재시도 2회로 0.06 m 안에 넣는 구조라 도킹이 최대 sim 75 s(벽시계 4 분) 걸릴 수 있다. 팔 쪽 횡 허용치를 알려 주면 `lat_tol_m`·`max_retry` 로 줄인다.

### 지난 판에서 이미 고친 것 (읽기만)

| 판 | 증상 | 조치 |
|---|---|---|
| 23차 | 후진 0.6 s 만에 `STALLED_REVERSE` | 멈춤 감지가 방향 변화도 이동으로 인정, 단계마다 기준점 재설정 |
| 22차 | 후진 1 m 중 방향 25도 틀어짐 → 컨베이어에 걸림 | 도킹 지점 제자리 회전 금지, 0.6 m 물러나 재진입(최대 2회) |
| 21차 | 도킹이 TurnTable 에 너무 가까움 | 도킹 거리 0.75 → 0.85 m |
| 25차 | 그 0.85 의 환산식이 면 위치를 0.05 m 잘못 잡았음 | 실측 로그로 다시 환산해 **0.90 m** (팔 밑동~대상 0.96 m) |
| 21차 | Place 지시 후 결과가 안 보임 | 결과 구독 터미널을 명령보다 먼저 띄운다(7절) |
| 19차 | Place 직후 Isaac 종료 | 없는 함수·미import 호출 제거. 바뀐 파일 전체 정적 검사 |
| 19차 | 오류가 로그에 안 남음 | `PYTHONUNBUFFERED=1`. `stdbuf` 는 별칭이라 쓸 수 없다 |
| 18차 | 명령에서 메시지 형식 오류 | 고피에도 워크스페이스를 1회 빌드(1절) |

## 0-1. 지금은 실행할 수 없다 — 고피3 임시 기간 (2026-09-25 ~ 약 3일)

**아래 1~9절은 교육장의 고피1·고피2 와 내피가 있을 때의 절차다.** 지금은 교육장 밖이라 그 세 대를 모두 쓸 수 없고, 임시 GCP VM 한 대(고피3)만 있다. 교육장에 돌아가면 1절부터 그대로 실행하면 된다.

고피3 에서 되는 것과 안 되는 것:

| 항목 | 가능 여부 |
|---|---|
| Isaac Sim 실측 | 장면이 들어왔으므로 기술적으로는 가능. 다만 **실행 전에 사용자 승인을 받는다**(유의미한 트러블슈팅은 사용자가 직접 실측·녹화한다) |
| Nav2 · RViz2 | 가능. 2026-09-25 에 `ros-jazzy-navigation2` 등을 설치했다 |
| 워크스페이스 빌드 | 가능 |
| 오프라인 격자 모의 (`dock_sim.py`) | 가능 |
| ROS 회귀 시험 (`dock_regression.py`) | 가능 |

고피3 에서 회귀를 돌릴 때는 아래 블록을 쓴다. 경로는 심볼릭 링크로 맞춰 두었으므로 **다른 기기와 같은 대문자 경로를 그대로 쓴다.** 화이트리스트 줄만 없다.

```bash
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
colcon build --symlink-install --packages-select smart_farm_interfaces smart_farm_navigation
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
python3 src/smart_farm_navigation/sim_test/dock_regression.py 2>&1 | tee src/smart_farm_navigation/results/regression_$(date +%Y%m%d_%H%M).txt
```

오프라인 격자만 빠르게 보려면(2 분):

```bash
source /opt/ros/jazzy/setup.bash
cd /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation
python3 sim_test/dock_sim.py
```

---

## 1. 고피 터미널 1 — 워크스페이스 빌드 (처음 한 번만)

Isaac 을 띄우지 않은 새 터미널에서 실행한다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
colcon build --packages-select smart_farm_interfaces smart_farm_manager 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/build_gopi_$(date +%Y%m%d_%H%M).txt
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 interface show smart_farm_interfaces/msg/TaskCommand
```

기대: `Summary: 2 packages finished` 뒤에 `string task_id` 로 시작하는 필드 목록이 나온다. 이 터미널은 8절에서 다시 쓴다.

**빌드는 한 번이면 되지만 `source` 는 새 터미널을 열 때마다 해야 한다.** 그래서 8절의 모든 명령 블록에 환경 줄과 `source` 가 들어 있다. 블록을 통째로 붙여 쓴다.

- `colcon: command not found` 가 나오면 고피에 colcon 이 없는 것이다. 그때는 5절을 내피에서 실행하고 그 사실을 보고한다.
- 빌드 산출물(`build/`, `install/`, `log/`)은 git 에 올라가지 않는다.

## 2. 고피 터미널 2 — Isaac Sim + 팀 통합 앱

**이 터미널에서는 워크스페이스를 `source` 하지 않는다.** 튜터 지시대로 환경을 준비한 뒤:

```bash
export ROS_DOMAIN_ID=101
export PYTHONUNBUFFERED=1
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py --autoplay 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/isaac_$(date +%Y%m%d_%H%M).txt
```

기대: `[라이다] 3D 라이다 fullScan=True (1개 helper)` → `[READY] Collected_smartfarm_v011 scene ready; TRANSFER/PICK_HARVEST/PLACE_INSPECT physical profiles loaded` → `[대기] /sim_task/command의 String/JSON 명령을 기다립니다`.

- `PYTHONUNBUFFERED=1` 은 출력을 한 줄씩 바로 기록하기 위한 것이다. 이것이 없으면 갑자기 종료됐을 때 마지막 출력이 사라져 원인을 볼 수 없다. 환경 변수라서 `isaac_python` 을 그대로 쓸 수 있다.
- 그래도 마지막 출력이 사라지면 `isaac_python` 뒤에 `-u` 를 붙여 본다. 예: `isaac_python -u /home/rokey/...standalone_app.py --autoplay`.
- Isaac 의 Stop 버튼은 누르지 않는다(팀 앱이 종료 처리에서 죽는다). 끝낼 때는 Ctrl+C 를 쓴다.
- Isaac 을 다시 실행하면 5·6 절의 내피 터미널도 모두 다시 실행한다. 시뮬레이션 시각이 0 으로 돌아가기 때문이다.

## 3. 고피 터미널 3 — GPU 사용량 감시 (실측 내내 켜 둔다)

Isaac 을 띄운 뒤 다른 터미널에서 실행한다. 종료 원인이 메모리 부족인지 가려내기 위한 것이다.

```bash
nvidia-smi --query-gpu=timestamp,name,utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv -l 2 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/gpu_$(date +%Y%m%d_%H%M).csv
```

2 초마다 한 줄씩 쌓인다. 실측이 끝날 때까지 켜 두고 Ctrl+C 로 끝낸다.

- `memory.used` 가 `memory.total` 에 가깝게 붙으면 메모리 부족이다. 그때는 이 파일과 Isaac 로그를 함께 보고한다.
- 어느 프로세스가 얼마나 쓰는지 보려면 다른 터미널에서 아래를 쓴다.

```bash
nvidia-smi --query-compute-apps=timestamp,pid,process_name,used_memory --format=csv -l 5 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/gpuapps_$(date +%Y%m%d_%H%M).csv
```

- `nvidia-smi: command not found` 가 나오면 그 PC 에 NVIDIA 드라이버 도구가 없는 것이다. 그 사실만 알려 주면 된다.

## 4. 내피 터미널 1 — 빌드와 연결 점검

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
colcon build --packages-select smart_farm_interfaces smart_farm_navigation smart_farm_manager
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 run smart_farm_navigation nav2_link_check 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/link_$(date +%Y%m%d_%H%M).txt
```

기대: `/clock`·`/tf`·`/chassis/odom` 이 15 Hz 이상, `/front_3d_lidar/lidar_points` 가 3 Hz 이상, `[6] self returns inside the rig box: N of M points/scan`, 마지막 줄 `RESULT OK - nav2.launch.py scan_mode:=cloud`.

- 전부 0 Hz 면 두 PC 의 도메인이 다르거나 화이트리스트에 상대 IP 가 없다.
- `N` 과 `highest z` 를 적어 둔다. 실측 기준값은 35~66점, 0.55 m 다.

## 4-1. 내피 터미널 1 — 실측 전 회귀 시험 (선택이지만 강력 권장, 20~30 분)

Isaac 없이 내피 혼자 돈다. 현장과 같은 `nav2.launch.py` 를 띄우고 합성 로봇으로 도킹 10 시나리오를 돌려 통과/실패 표를 낸다. **Isaac 이나 실측용 Nav2 가 떠 있는 동안에는 돌리지 않는다.** 끝나면 표를 보고, 실패가 있으면 실측을 미룬다.

```bash
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
python3 src/smart_farm_navigation/sim_test/dock_regression.py 2>&1 | tee src/smart_farm_navigation/results/regression_$(date +%Y%m%d_%H%M).txt
```

도메인 77·LOCALHOST 는 실측용(101)과 섞이지 않게 일부러 다르게 둔 것이다. 이득만 바꿨을 때는 더 빠른 오프라인 모의(2 분)로 먼저 본다.

```bash
source /opt/ros/jazzy/setup.bash
cd /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation
python3 sim_test/dock_sim.py --fail-only
```

## 5. 내피 터미널 2 — Nav2 · RViz2 · 정밀 도킹 · 기록

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation nav2.launch.py record:=true 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/nav2_$(date +%Y%m%d_%H%M).txt
```

기대: `rosbag -> ~/.ros/smart_farm_navigation/bags/nav2_…` → `scan_mode auto -> cloud` → `AMCL initial pose (-0.421, 1.006, 90.0deg)` → `Managed nodes are active` → `[feeder_dock]: [IDLE] waiting`.

- `record:=true` 는 이번 실측에 필요하다. 팔 자세 허용 범위를 여기서 나온 기록으로 정한다.
- RViz2 에서 카터가 Rack_1 과 Rack_4 사이 통로에 있고 스캔 점이 양쪽 랙 면에 붙어 있으면 그대로 둔다. `2D Pose Estimate` 는 찍지 않는다.

## 6. 내피 터미널 3 — 주행 노드

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 launch smart_farm_navigation navigation_node.launch.py 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/navnode_$(date +%Y%m%d_%H%M).txt
```

기대: `navigation_node ready; destinations: ['FEEDER_DOCK', 'RACK_DOCK', 'CORRIDOR_EXIT']`.

## 7. 고피 터미널 4·5 — 결과 구독 (명령을 보내기 전에 먼저 띄운다)

결과 토픽은 보관되지 않으므로, 명령을 보낸 뒤에 구독하면 이미 지나간 결과를 못 본다. 두 터미널을 먼저 띄워 실측이 끝날 때까지 켜 둔다.

고피 터미널 4 (Isaac 작업 결과):

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic echo /sim_task/result std_msgs/msg/String 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/simresult_$(date +%Y%m%d_%H%M).txt
```

고피 터미널 5 (주행 결과):

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic echo /navigation/result smart_farm_interfaces/msg/TaskResult 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/navresult_$(date +%Y%m%d_%H%M).txt
```

두 터미널 모두 아무것도 찍히지 않은 채 대기하는 것이 정상이다. 명령을 보내면 그때 결과가 나타난다.

## 8. 고피 터미널 1 — 명령 발행 (1절 터미널을 그대로 쓴다)

먼저 워크스페이스를 읽고 메시지 타입이 보이는지 확인한다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
hostname
ros2 interface show smart_farm_interfaces/msg/TaskCommand
```

`IsaacSim03` 과 `string task_id` 로 시작하는 필드 목록이 함께 나와야 한다. `No such file or directory` 나 `Unknown package` 가 나오면 1절 빌드가 끝나지 않은 것이므로 1절을 다시 실행한다.

이어서 팔레트를 집는다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String '{data: "{\"task_id\": \"TASK-20260923-001\", \"command_id\": \"TASK-20260923-001-CMD-001\", \"operation\": \"PICK_HARVEST\", \"recipe_id\": \"HARVEST_RACK_L1\", \"pallet_id\": \"PALLET_001\", \"source\": \"RACK_L1\", \"destination\": \"CARRY\"}"}' 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/cmd_$(date +%Y%m%d)_gopi.txt
```

**고피 터미널 4** 에 `"status": "SUCCEEDED"` 와 `"safe_to_navigate": true` 가 나오면 주행을 지시한다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub --once --max-wait-time-secs 15 /navigation/command smart_farm_interfaces/msg/TaskCommand "{task_id: 'TASK-20260923-001', command_id: 'TASK-20260923-001-CMD-002', operation: 'NAVIGATION', destination: 'FEEDER_DOCK'}" 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/cmd_$(date +%Y%m%d)_gopi.txt
```

6절 내피 터미널 3 의 기대 출력은 다음과 같다.

```
goToPose FEEDER_APPROACH: -2.19, -1.55
정밀 도킹 시작 요청: run_id=TASK-20260923-001-CMD-002
도킹 결과 SUCCEEDED: face_dist=0.7x yaw_err=-0.x lat=0.0x
result SUCCEEDED/NONE for TASK-20260923-001-CMD-002 (phase ARRIVED)
```

**고피 터미널 5** 에 `status: SUCCEEDED`, `reason: NONE`, `reached_station: FEEDER_DOCK` 이 나와야 한다.

마지막으로 팔레트를 내려놓는다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String '{data: "{\"task_id\": \"TASK-20260923-001\", \"command_id\": \"TASK-20260923-001-CMD-003\", \"operation\": \"PLACE_INSPECT\", \"recipe_id\": \"PLACE_AT_INSPECTION\", \"pallet_id\": \"PALLET_001\", \"source\": \"CARRY\", \"destination\": \"INSPECT_STATION\"}"}' 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/cmd_$(date +%Y%m%d)_gopi.txt
```

이때 2절 고피 터미널(Isaac)에 `[Place] 차체 정지 확인` 과 `[도킹] 카터 본체 world (…), place 대상까지 x … y … 직선 … m` 이 찍힌다. **이 줄을 그대로 옮겨 적어 주면 팔 자세 허용 범위를 정할 수 있다.**

## 9. 문제가 생겼을 때

| 증상 | 조치 |
|---|---|
| `install/setup.bash: No such file or directory` (고피) | 1절 빌드를 하지 않았다. 1절을 먼저 실행한다 |
| `Unknown package 'smart_farm_interfaces'` 또는 `The passed message type is invalid` | 그 터미널에서 `source .../install/setup.bash` 를 하지 않았다. 8절 블록을 줄 일부만 붙이지 말고 통째로 붙인다. 그래도 안 되면 1절 빌드부터 다시 한다 |
| 발행 명령이 15 초 뒤 오류로 끝남 | 구독자를 못 찾았다. `/navigation/command` 면 6절 터미널이 떠 있는지, `/sim_task/command` 면 2절 Isaac 이 `[대기]` 상태인지 확인한다. 두 PC 의 도메인이 같은지도 본다 |
| `stdbuf: failed to run command 'isaac_python'` | `stdbuf` 처럼 외부 명령을 앞에 붙이면 셸 별칭이 풀리지 않는다. 2절 블록대로 `isaac_python` 을 첫 낱말로 두고 `PYTHONUNBUFFERED=1` 만 쓴다 |
| Place 지시 직후 Isaac 종료 | 19차와 같은 증상이면 2절 로그 끝에 traceback 이 남는다(이번 판부터 유실되지 않는다). 그 몇 줄과 `gpu_*.csv` 를 함께 보고한다 |
| Isaac 터미널에서 rclpy 오류로 죽음 | 그 터미널에서 워크스페이스를 `source` 했을 가능성이 크다. 새 터미널에서 워크스페이스 없이 2절만 실행한다 |
| `navigate_to_pose 액션 서버가 없습니다` | 5절 Nav2 가 아직 안 떴다. `Managed nodes are active` 를 본 뒤 명령을 보낸다 |
| 접근 지점에 멈춘 뒤 도킹이 시작되지 않음 | 5절 터미널에 `FEEDER_APPROACH 부근에 서 있으나 자동 시작이 꺼져 있습니다` 경고가 뜬다. 8절 주행 명령으로 시작한다 |
| Place 명령 뒤 Isaac 에 `[Place] 차체 정지를 확인합니다` 만 반복 | 차체가 계속 미세하게 움직이고 있다. 15 초 뒤 이유를 담아 실패한다. 주행 결과를 받은 뒤 2~3 초 기다렸다 다시 보낸다 |
| 명령을 보냈는데 결과가 안 보임 | 7절 결과 터미널을 명령보다 **먼저** 띄웠는지 확인한다. 나중에 띄우면 이미 지나간 결과를 못 본다 |
| 도킹 결과 `STALLED_REVERSE` 또는 `STALLED_CREEP` | 카터가 무언가에 걸렸다. 라이다에 안 보이는 낮은 구조물이거나 팔이 든 팔레트가 걸린 것이다. Isaac 화면에서 어디가 닿았는지 확인하고 bag 이름과 함께 알려 준다 |
| 도킹 결과 `YAW_OFF_..DEG` | 두 번 다시 맞췄는데도 방향이 3도 안으로 안 들어왔다. 접근 지점의 위치가 많이 틀어졌을 수 있다. `navresult_*` 와 bag 을 함께 알려 준다 |
| 도킹 중 `BACKOFF` 가 보임 | 정상이다. 도착 방향이 틀어져 물러나 다시 맞추는 중이다 |
| 팔이 닿지 않음(역기구학 실패) | 도킹 거리를 조정한다. 5절을 `ros2 launch smart_farm_navigation nav2.launch.py record:=true dock_auto:=false` 로 띄운 뒤, 내피의 다른 터미널에서 `ros2 run smart_farm_navigation feeder_dock --ros-args -p use_sim_time:=true -p standoff_m:=0.87` 처럼 바꿔 실행한다. **팔 밑동에서 place 대상까지의 거리 = `standoff_m` + 0.06** 이다(실측 2건 환산). 랙 pick 에서 성공한 자세가 0.932 m 였으므로 그 값을 그대로 재현하려면 0.87 이다. 참고로 `robot_motion.check_base_pose` 의 0.89~1.05 검사는 `start_pick` 에서만 도는 것이라 place 는 통과 여부와 무관하며, place 의 실제 관문은 역기구학과 관절 연속성(한계 20°)이다 |
| 주행 중 시간 초과 | 실시간 배율이 0.2 아래로 떨어진 경우다. 고피에서 다른 GPU 작업을 함께 돌리고 있는지 확인한다 |
| RViz2 클릭으로 손 시험하고 싶음 | 5절을 `dock_auto:=true` 로 띄우고 6·8 절을 생략한다. `FEEDER_APPROACH` 화살표를 Nav2 Goal 로 한 번 클릭하면 도착 후 자동으로 도킹한다 |

## 10. 결과 보고

`results/` 의 `build_gopi_*`, `isaac_*`, `link_*`, `nav2_*`, `navnode_*`, `cmd_*_gopi.txt`, `simresult_*`, `navresult_*`, `gpu_*.csv` 와 bag 디렉터리 이름(`~/.ros/smart_farm_navigation/bags/`), 그리고 8절 마지막의 `[도킹]` 줄을 함께 남긴다.

## 11. 파일과 역할

| 파일 | 실행 PC | 역할 |
|---|---|---|
| `isaacpjt/smart_farm/runtime/standalone_app.py` | 고피 | 장면 실행, 팔레트 집기·내려놓기, 라이다 fullScan, Place 전 차체 정지 확인과 도킹 위치 기록, 운반 중 팔레트 감시 예외 처리(`docs/standalone_app_changes.md` 2-1) |
| `launch/nav2.launch.py` | 내피 | Nav2 · RViz2 · `/scan` 생성 · 정밀 도킹 노드 · 작업점 마커 · 기록 |
| `launch/navigation_node.launch.py` | 내피 | `/navigation/command` 를 받아 NavigateToPose 와 정밀 도킹을 구동. 제한 시간은 sim 시계(300 s / 도킹 150 s) |
| `smart_farm_navigation/feeder_dock.py` | 내피 | 라이다로 TurnTable 앞면을 보며 후진 도킹. `DockLogic`(상태기계, ROS 없음) + `FeederDock`(ROS 배선), `DockParams` 가 파라미터 단일 출처 |
| `sim_test/dock_regression.py` | 내피 | 실측 전 회귀 시험(4-1절). 합성 로봇 + 현장 launch 로 10 시나리오 |
| `sim_test/dock_sim.py` | 내피 | 도킹 상태기계를 ROS 없이 돌리는 오프라인 모의(225 케이스) |
| `sim_test/fake_robot.py` | 내피 | Isaac 대역. 지도에서 점군을 만들고 bag 에서 읽은 굼뜬 차체 응답을 흉내낸다 |
| `sim_test/replay_bag_dock.py` | 내피 | 실측 bag 의 `/scan` 을 노드와 같은 검출기에 다시 넣어 그때 제어기가 본 값을 찍는다 |
| `smart_farm_navigation/cloud_self_filter.py` | 내피 | 결합카터 자기 반사 제거와 점군 합치기 |
| `smart_farm_navigation/nav2_link_check.py` | 내피 | 두 PC 사이 토픽 도달과 자기 반사 점검 |
| `smart_farm_navigation/stations.py` | 내피 | 작업점 읽기와 목표 자세 생성 |
| `config/stations.yaml`, `config/destinations_nav2.yaml` | 내피 | 작업점과 목적지 매핑 |
| `smart_farm_interfaces` | 고피·내피 | 명령·결과·상태 메시지. 명령을 보내는 PC 에 반드시 빌드되어 있어야 한다 |
