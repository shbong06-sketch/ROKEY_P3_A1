# guidance2_27차 — 전 구간 실측 (주행 → 도킹 0.92 → Place → 컨베이어 → 비전 검사 → 솎아내기)

- 작성일: 2026-09-25, 브랜치 `feature/Inspection-Place-nav2`.
- **26차와 달라진 점 세 가지**: ① 장면·지도가 **v014 양배추 씬**으로 바뀌었다 ② 도킹 거리가 **0.92 m** 다 ③ **팀의 PLACE 수정이 우리 브랜치에 들어왔다.** 그래서 Place 이후 컨베이어 반송 → 비전 검사 → 솎아내기까지 자동으로 이어진다. 26차는 `guidance/past/` 로 옮겼다.
- **아직 실측하지 않았다. 검증은 고피3 모의뿐이다** (ADR 2.7: 모의 통과 ≠ 실측 통과).
- **PC 분담**: Nav2 쪽(Nav2·RViz2·정밀 도킹·주행 노드)만 내피(`lwh19180`)에서 돌리고, 나머지(Isaac, 명령 발행)는 모두 고피(`IsaacSim03`)에서 돌린다.
- **고피에도 워크스페이스를 한 번 빌드해야 한다.** 명령에 쓰는 `smart_farm_interfaces` 메시지가 거기 있어야 하기 때문이다.
- `ROS_DOMAIN_ID` 는 두 PC 모두 **101**(고피1 기준). 고피2 를 쓰면 두 PC 모두 102 로 바꾼다. 비전 컨테이너를 띄운다면 그것도 같은 값이다.
- 아래 순서대로 위에서 아래로 실행한다. 블록마다 환경 줄이 들어 있으므로 그대로 복사해 붙이면 된다.

## 0. 이번 판에서 알아 둘 것 (읽기만)

| 항목 | 내용 |
|---|---|
| 장면 | `scenes/Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd`. 그 폴더가 있으면 팀 앱이 `--scene` 없이 **자동으로 이 씬을 연다**. 로메인이 아니라 양배추다 |
| 지도 | `maps/Collected_smartfarm_v014.yaml` 을 `nav2.launch.py` 가 자동으로 쓴다. v011 지도와 다른 픽셀은 36개뿐이고(비전룸 M0609 받침대 이동) 주행·도킹 구역은 같다 |
| 도킹 거리 | 라이다가 검출하는 면(world y −3.645)에서 **0.92 m**. 팀이 올인원을 끝까지 돌려 확인한 값이며 **팀의 place 수정과 한 쌍**이다. 임의로 바꾸지 않는다 |
| Place | 팀 수정으로 네 가지가 고쳐졌다 — 놓는 방향이 도킹 방향과 90° 어긋나던 것, TurnTable 쿼터니언 비정규화(0.507), 높이 25.9 mm 부족, 포크판이 롤러에 걸리던 것. 놓는 동안 줄기 벨트는 인터록으로 멈춘다 |
| Place 이후 | 트레이가 벨트에 놓이면 **컨베이어 반송 → 이송 프레임이 밀어 넣기 → YOLO 검사 → 노랑·갈색 솎아내기(SortBin 1/2 번갈아) → 재검사 → 배출**이 자동으로 진행된다. 사람이 명령을 더 보내지 않는다 |
| YOLO | Isaac 파이썬이 아니라 **별도 `python3` 하위 프로세스**로 돈다(라이브러리 충돌 방지). 그 파이썬에 `ultralytics` 가 있어야 한다. 1-1절에서 미리 확인한다 |
| 가중치 | 씬 폴더의 `best.pt` 를 자동으로 찾는다. 바꾸려면 `SMARTFARM_YOLO_WEIGHTS` |
| Isaac 터미널 주의 | **Isaac 을 띄우는 터미널에서는 워크스페이스를 `source` 하지 않는다.** 팀 앱이 Isaac 번들 ROS 라이브러리를 먼저 쓰도록 되어 있어 시스템 ROS 와 섞이면 죽는다 |
| 주행 실행 방식 | `navigation_node` 가 NavigateToPose 액션을 직접 보내고, 도착하면 명령 식별자를 실어 정밀 도킹을 시작시킨다. 같은 식별자로 돌아온 결과만 인정한다 |
| 도킹 시작 | 자동 시작은 꺼져 있다(`dock_auto` 기본 false). 명령으로만 시작한다. RViz2 클릭으로 손 시험할 때만 `dock_auto:=true` 로 띄운다 |
| 도킹 순서 | `SETTLE` → `ALIGN_TO_GOAL` → `REVERSE` → `CHECK` → (`CREEP` → `DONE`) 또는 `BACKOFF` 후 재시도. Nav2 `/cmd_vel` 이 2 s 조용하고 차체 각속도가 0.03 rad/s 아래여야 정렬을 시작한다(ADR 2.2) |
| 도킹 조향 | 차체 각속도가 명령을 1~3 s 늦게 따라오는 것을 전제로 짰다. 후진 0.10 m/s, 조향 상한 0.12 rad/s 라 화면상 느리게 보인다. 정상이다 |
| 제한 시간 | 주행 노드 600 s, 도킹 240 s(노드 자체는 120 s). 모두 `/clock` 기준이다 |
| 발행 명령 | 모두 `--once --max-wait-time-secs 15`. 15 초 안에 구독자를 못 찾으면 무한 대기 대신 오류로 끝난다 |
| 결과 구독 | 결과 토픽은 보관되지 않는다. **7절 구독 터미널을 명령보다 먼저 띄운다** |
| 실측 전 회귀 | **내피에서 `sim_test/dock_regression.py` 를 먼저 돌린다(4-1절).** Isaac 없이 돈다. 실측 중에는 절대 돌리지 않는다(같은 도메인에서 Nav2 가 두 벌 뜬다) |

### 지금까지 고친 것 (읽기만)

| 판 | 증상 | 조치 |
|---|---|---|
| 26차까지 | PLACE 에서 IK 110° 점프 / `DESCEND_5` 20.7°(한계 20°) | 팀 `feature/cabbage-place-fix` 의 `turntable_place_pose()` 수정 + 도킹 거리 0.92 를 함께 반입(`a95ba6c`) |
| 24차 | 정렬 직후에도 차체가 0.23 rad/s 로 돌아 후진 중 0.16 m 이탈, `CHECK` +21.2° | `SETTLE` 단계, 관성·측정 지연을 뺀 오차로 조향, 면 법선 추종, 횡 0.06 m 초과 시 0.9 m 물러나 재시도 |
| 24차 | 운반 중 팔레트 30 mm 미끄러짐 예외로 Isaac 종료 | main loop 에서 잡아 한 줄 남기고 계속. 다음 명령 때 `MOTION_FAILED` 결과로 보고 |
| 23차 | 후진 0.6 s 만에 `STALLED_REVERSE` | 멈춤 감지가 방향 변화도 이동으로 인정, 단계마다 기준점 재설정 |
| 22차 | 후진 1 m 중 방향 25° 틀어짐 → 컨베이어에 걸림 | 도킹 지점 제자리 회전 금지, 물러나 재진입(최대 2회) |
| 21차 | Place 지시 후 결과가 안 보임 | 결과 구독 터미널을 명령보다 먼저 띄운다(7절) |
| 19차 | 오류가 로그에 안 남음 | `PYTHONUNBUFFERED=1`. `stdbuf` 는 별칭이라 쓸 수 없다 |
| 18차 | 명령에서 메시지 형식 오류 | 고피에도 워크스페이스를 1회 빌드(1절) |

### 검증 상태 (고피3 모의, 실측 아님)

| 시험 | 조건 | 결과 |
|---|---|---|
| ROS 회귀 10 시나리오 (standoff 0.92) | 현장과 같은 launch + 합성 로봇 | **10/10 통과.** 도착 면 거리 0.916~0.950 m, 최대 횡 오차 0.057 m (허용 0.06). `results/regression_20260925_standoff092.txt` |
| v014 지도 교체 후 normal | 같은 launch, 지도만 v014 | 1/1 통과 (면 0.947 m, 횡 −0.006 m) |
| 오프라인 격자 (standoff 0.92) | 시작 자세 45종 × 차체 5종 = 225 | 방향·거리·충돌 225/225. **횡 오차는 224/225** — 1 케이스가 0.063 m 로 허용 0.06 m 를 3 mm 넘음 (시작 world (−2.39, −1.85) yaw 70°, 제자리 회전이 명령의 12% 만 나오는 차체). `results/docksim_20260925_standoff092.txt:52`. 0.90 일 때는 225/225(최대 0.059 m)였음 |

남은 여유: 도킹 거리가 멀수록 횡 오차를 갚을 후진 거리가 줄어 횡 오차가 커진다(ROS 회귀 최대 0.043 → 0.057, 격자 최대 0.059 → 0.063). 위 1 케이스에서는 도킹 노드 자신이 `SUCCEEDED` 로 보고했으므로 노드가 재는 횡과 실제 횡이 3 mm 어긋난 것이다. **실측 전에는 이득을 고치지 않는다**(고치면 ROS 회귀 10/10 의 근거가 무효가 된다). 실측에서 팔 쪽 실제 횡 허용치를 받으면 `lat_tol_m`·`max_retry` 로 줄인다.

## 0-1. 지금은 실행할 수 없다 — 고피3 임시 기간 (2026-09-25 ~ 약 3일)

**아래 1~9절은 교육장의 고피1·고피2 와 내피가 있을 때의 절차다.** 지금은 교육장 밖이라 그 세 대를 모두 쓸 수 없고 임시 GCP VM 한 대(고피3)만 있다. 교육장에 돌아가면 1절부터 그대로 실행하면 된다.

고피3 에서 되는 것과 안 되는 것:

| 항목 | 가능 여부 |
|---|---|
| Isaac Sim 실측 | 장면이 들어왔으므로 기술적으로는 가능. 다만 **실행 전에 사용자 승인을 받는다**(유의미한 트러블슈팅은 사용자가 직접 실측·녹화한다) |
| Nav2 · RViz2 | 가능 |
| 워크스페이스 빌드 | 가능 |
| 오프라인 격자 모의 (`dock_sim.py`) | 가능 |
| ROS 회귀 시험 (`dock_regression.py`) | 가능 |

고피3 에서 회귀를 돌릴 때는 아래 블록을 쓴다. 경로는 심볼릭 링크로 맞춰 두었으므로 **다른 기기와 같은 대문자 경로를 그대로 쓴다.** 화이트리스트 줄만 없다(VM 에는 교육장 랜카드가 없어 켜면 통신이 끊긴다).

```bash
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
colcon build --symlink-install --packages-select smart_farm_interfaces smart_farm_navigation
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
python3 /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/sim_test/dock_regression.py 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/regression_$(date +%Y%m%d_%H%M).txt
```

오프라인 격자만 빠르게 보려면(2 분):

```bash
source /opt/ros/jazzy/setup.bash
python3 /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/sim_test/dock_sim.py
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

**빌드는 한 번이면 되지만 `source` 는 새 터미널을 열 때마다 해야 한다.** 그래서 8절의 모든 블록에 환경 줄과 `source` 가 들어 있다. 블록을 통째로 붙여 쓴다.

- `colcon: command not found` 가 나오면 고피에 colcon 이 없는 것이다. 그 사실을 보고한다.
- 빌드 산출물(`build/`, `install/`, `log/`)은 git 에 올라가지 않는다.

## 1-1. 고피 터미널 1 — YOLO 준비 확인 (이번 판 신규, 1분)

비전 검사는 Isaac 파이썬이 아니라 보통 `python3` 로 따로 돈다. 그 파이썬에 `ultralytics` 가 없으면 **검사 단계에서만** 멈춘다(주행·도킹·Place 는 정상 진행된다). 미리 확인한다.

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
python3 -c "import ultralytics, torch; print('ultralytics', ultralytics.__version__, '| torch', torch.__version__)"
ls -l /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/best.pt
```

기대: 버전 두 개가 찍히고 `best.pt` 가 보인다.

- `ModuleNotFoundError: No module named 'ultralytics'` 가 나오면 **선택**이다. ① 그대로 진행하고 검사 앞 단계까지만 본다(2절에서 `--no-vision-station` 을 붙인다) ② 또는 `python3 -m pip install ultralytics` 로 설치한 뒤 진행한다. 어느 쪽이든 결과 보고에 적는다.
- 다른 파이썬에 들어 있으면 2절 블록에 `export SMARTFARM_YOLO_PYTHON=<그 파이썬 경로>` 를 한 줄 더한다.

## 2. 고피 터미널 2 — Isaac Sim + 팀 올인원 앱

**이 터미널에서는 워크스페이스를 `source` 하지 않는다.** 튜터 지시대로 환경을 준비한 뒤:

```bash
export ROS_DOMAIN_ID=101
export PYTHONUNBUFFERED=1
export SMARTFARM_STATION_OUT=/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/station_$(date +%Y%m%d_%H%M)
isaac_python /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py --autoplay 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/isaac_$(date +%Y%m%d_%H%M).txt
```

기대 순서:

```
[시작] Scene을 불러옵니다: /home/rokey/ROKEY_P3_A1/.../Collected_smartfarm_v014_room_core_cabbage.usd
[라이다] 3D 라이다 fullScan=True (1개 helper)
[컨베이어] 통로 확인됨 — 벽 개구부 이상 없음
[READY] Collected_smartfarm_v014_room_core_cabbage scene ready; TRANSFER/PICK_HARVEST/PLACE_INSPECT physical profiles loaded
[대기] /sim_task/command의 String/JSON 명령을 기다립니다
```

- **첫 줄의 씬 이름에 `_cabbage` 가 없으면 v014 폴더를 못 찾은 것이다.** 그때는 `--scene /home/rokey/ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/Collected_smartfarm_v014_room_core_cabbage.usd` 를 붙여 다시 실행한다.
- `SMARTFARM_STATION_OUT` 은 검사 사진·`station_results.json`·`yolo_worker.err` 가 쌓이는 곳이다. 지정해 두면 나중에 그 폴더만 보면 된다.
- `PYTHONUNBUFFERED=1` 은 출력을 한 줄씩 바로 기록하기 위한 것이다. 없으면 갑자기 종료됐을 때 마지막 출력이 사라져 원인을 볼 수 없다.
- Isaac 의 Stop 버튼은 누르지 않는다(팀 앱이 종료 처리에서 죽는다). 끝낼 때는 Ctrl+C 를 쓴다.
- Isaac 을 다시 실행하면 5·6절의 내피 터미널도 모두 다시 실행한다. 시뮬레이션 시각이 0 으로 돌아가기 때문이다.

**선택 인자** (필요할 때만 위 명령 끝에 붙인다):

| 인자 | 무엇을 위한 것인가 |
|---|---|
| `--no-vision-station` | 비전 검사·솎아내기를 끈다. `ultralytics` 가 없거나, 이번 실측에서 주행·도킹·Place 까지만 보고 싶을 때. 컨베이어는 10 초 뒤 스스로 배출한다 |
| `--no-conveyor` | 컨베이어 반송까지 끈다. Place 직후 상태를 오래 들여다볼 때만 |
| `--human-crossing` | 카터가 통로를 나올 때 작업자가 앞을 막았다가 비키는 돌발상황을 넣는다. Nav2 회피를 보고 싶을 때만. **첫 실측에서는 붙이지 않는다** |

## 3. 고피 터미널 3 — GPU 사용량 감시 (실측 내내 켜 둔다)

Isaac 을 띄운 뒤 다른 터미널에서 실행한다. 종료 원인이 메모리 부족인지 가려내기 위한 것이다.

```bash
nvidia-smi --query-gpu=timestamp,name,utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv -l 2 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/gpu_$(date +%Y%m%d_%H%M).csv
```

2 초마다 한 줄씩 쌓인다. 실측이 끝날 때까지 켜 두고 Ctrl+C 로 끝낸다.

- 이번 판은 카메라 5대와 YOLO 가 함께 도므로 26차보다 GPU 부하가 크다. `memory.used` 가 `memory.total` 에 붙으면 메모리 부족이다. 그때는 이 파일과 Isaac 로그를 함께 보고한다.
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

- **YAML·코드를 고쳤으면 반드시 이 `colcon build` 를 다시 한다.** launch 는 `install/` 복사본을 읽으므로 빌드하지 않으면 옛 값으로 돈다.
- 전부 0 Hz 면 두 PC 의 도메인이 다르거나 화이트리스트에 상대 IP 가 없다.
- `N` 과 `highest z` 를 적어 둔다. 실측 기준값은 35~66점, 0.55 m 다. 카터 경량본(팀이 몸체 속 부품을 뺀 것)을 쓰므로 이 값이 달라질 수 있다. 달라지면 그 숫자를 보고한다.

## 4-1. 내피 터미널 1 — 실측 전 회귀 시험 (선택이지만 강력 권장, 20~30 분)

Isaac 없이 내피 혼자 돈다. 현장과 같은 `nav2.launch.py` 를 띄우고 합성 로봇으로 도킹 10 시나리오를 돌려 통과/실패 표를 낸다. **Isaac 이나 실측용 Nav2 가 떠 있는 동안에는 돌리지 않는다.** 실패가 있으면 실측을 미룬다.

```bash
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
python3 /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/sim_test/dock_regression.py 2>&1 | tee /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/regression_$(date +%Y%m%d_%H%M).txt
```

도메인 77·LOCALHOST 는 실측용(101)과 섞이지 않게 일부러 다르게 둔 것이다. 기대: 마지막에 `10/10` 표가 나온다.

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
- 지도는 `Collected_smartfarm_v014.yaml` 이 자동으로 쓰인다. 다른 지도로 시험하려면 `map:=/home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/maps/Collected_smartfarm_v011.yaml` 처럼 인자를 준다(**선택**, 되돌림 시험용).
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

### 8-1. 팔레트 집기 (PICK_HARVEST)

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String '{data: "{\"task_id\": \"TASK-20260925-001\", \"command_id\": \"TASK-20260925-001-CMD-001\", \"operation\": \"PICK_HARVEST\", \"recipe_id\": \"HARVEST_RACK_L1\", \"pallet_id\": \"PALLET_001\", \"source\": \"RACK_L1\", \"destination\": \"CARRY\"}"}' 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/cmd_$(date +%Y%m%d)_gopi.txt
```

**고피 터미널 4** 에 `"status": "SUCCEEDED"` 와 `"safe_to_navigate": true` 가 나오면 다음으로 간다. 팀 기준 소요 약 72 s(시뮬 시간)다.

### 8-2. 주행 + 정밀 도킹 (NAVIGATION)

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub --once --max-wait-time-secs 15 /navigation/command smart_farm_interfaces/msg/TaskCommand "{task_id: 'TASK-20260925-001', command_id: 'TASK-20260925-001-CMD-002', operation: 'NAVIGATION', destination: 'FEEDER_DOCK'}" 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/cmd_$(date +%Y%m%d)_gopi.txt
```

6절 내피 터미널 3 의 기대 출력:

```
goToPose FEEDER_APPROACH: -2.19, -1.55
정밀 도킹 시작 요청: run_id=TASK-20260925-001-CMD-002
도킹 결과 SUCCEEDED: face_dist=0.9x yaw_err=-0.x lat=0.0x
result SUCCEEDED/NONE for TASK-20260925-001-CMD-002 (phase ARRIVED)
```

`face_dist` 는 **0.92 근처**여야 한다(모의 범위 0.916~0.950). **고피 터미널 5** 에 `status: SUCCEEDED`, `reason: NONE`, `reached_station: FEEDER_DOCK` 이 나와야 한다. 팀 기준 소요 약 37 s(시뮬 시간)다.

### 8-3. 내려놓기 (PLACE_INSPECT) — 이 뒤는 자동으로 이어진다

```bash
export ROS_DOMAIN_ID=101
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rokey/.ros/fastdds_whitelist.xml
source /opt/ros/jazzy/setup.bash
source /home/rokey/ROKEY_P3_A1/cobot3_ws/install/setup.bash
ros2 topic pub --once --max-wait-time-secs 15 /sim_task/command std_msgs/msg/String '{data: "{\"task_id\": \"TASK-20260925-001\", \"command_id\": \"TASK-20260925-001-CMD-003\", \"operation\": \"PLACE_INSPECT\", \"recipe_id\": \"PLACE_AT_INSPECTION\", \"pallet_id\": \"PALLET_001\", \"source\": \"CARRY\", \"destination\": \"INSPECT_STATION\"}"}' 2>&1 | tee -a /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/results/cmd_$(date +%Y%m%d)_gopi.txt
```

2절 고피 터미널(Isaac)에 다음이 차례로 찍힌다. 명령은 더 보내지 않는다.

```
[Place] 차체 정지 확인 (0.5초 대기)
[도킹] 카터 본체 world (…), place 대상까지 x … y … 직선 … m
[컨베이어] 줄기 벨트 정지 (로봇 놓기 중)  →  재가동
[컨베이어] PALLET_001 벨트 감지 — 반송 시작
[컨베이어] PALLET_001 카메라 앞 정지
[솎아내기] PALLET_001 도착 … 이송 프레임이 내려와 …
[비전] YOLO 워커 시작 … / [비전] YOLO 준비 완료 — 클래스 …
[비전] PALLET_001 검사: …
[솎아내기] … (노랑·갈색 포기를 SortBin 1/2 에 버린다)
[비전] PALLET_001 재검사: …
[컨베이어] PALLET_001 검사 완료 — 배출 시작 → 배출 완료
```

- **`[도킹] 카터 본체 world (…)` 줄을 그대로 옮겨 적어 주면** 팔 자세 허용 범위를 숫자로 정할 수 있다. 이번 실측에서 받아야 할 값이다.
- 팀 기준 전 구간 소요는 시뮬 시간 약 4분 19초다(녹화를 켜면 벽시계로 2~3배).

## 9. 문제가 생겼을 때

| 증상 | 조치 |
|---|---|
| 2절 첫 줄 씬 이름에 `_cabbage` 가 없음 | v014 폴더를 못 찾았다. 2절 설명대로 `--scene` 으로 절대경로를 지정한다 |
| `install/setup.bash: No such file or directory` (고피) | 1절 빌드를 하지 않았다. 1절을 먼저 실행한다 |
| `Unknown package 'smart_farm_interfaces'` 또는 `The passed message type is invalid` | 그 터미널에서 `source .../install/setup.bash` 를 하지 않았다. 8절 블록을 통째로 붙인다. 그래도 안 되면 1절 빌드부터 다시 한다 |
| 발행 명령이 15 초 뒤 오류로 끝남 | 구독자를 못 찾았다. `/navigation/command` 면 6절 터미널이, `/sim_task/command` 면 2절 Isaac 이 `[대기]` 인지 본다. 두 PC 의 도메인도 확인한다 |
| 명령을 보냈는데 결과가 안 보임 | 7절 결과 터미널을 명령보다 **먼저** 띄웠는지 확인한다. 나중에 띄우면 지나간 결과를 못 본다 |
| `navigate_to_pose 액션 서버가 없습니다` | 5절 Nav2 가 아직 안 떴다. `Managed nodes are active` 를 본 뒤 명령을 보낸다 |
| 접근 지점에 멈춘 뒤 도킹이 시작되지 않음 | 5절 터미널에 `FEEDER_APPROACH 부근에 서 있으나 자동 시작이 꺼져 있습니다` 경고가 뜬다. 8-2 명령으로 시작한다 |
| 도킹 중 `BACKOFF` 가 보임 | 정상이다. 도착 방향이 틀어져 물러나 다시 맞추는 중이다(최대 2회) |
| 도킹 결과 `STALLED_REVERSE` / `STALLED_CREEP` | 카터가 무언가에 걸렸다. Isaac 화면에서 닿은 곳을 확인하고 bag 이름과 함께 알려 준다 |
| 도킹 결과 `YAW_OFF_..DEG` | 두 번 다시 맞췄는데도 방향이 3° 안으로 안 들어왔다. `navresult_*` 와 bag 을 알려 준다 |
| 도킹 결과 `FACE_NOT_FOUND` | 라이다가 TurnTable 앞면을 못 봤다. 5절 터미널의 `idle:` 줄과 RViz2 의 `/scan` 을 함께 보고한다 |
| Place 가 `BASE_NOT_SETTLED` 로 실패 | 차체가 아직 미세하게 움직인다. 2~3 초 기다렸다 8-3 을 다시 보낸다 |
| Place 에서 관절 한계 실패(`MOTION_FAILED`) | **도킹 거리를 임의로 바꾸지 않는다.** 0.92 는 팀 place 수정과 한 쌍이다. Isaac 로그의 실패 단계 이름(`DESCEND_n`)과 `[도킹] 카터 본체 world` 줄을 그대로 보고한다 |
| `[컨베이어] 고장 — …` (`CONVEYOR_FAILED`) | 트레이가 벨트 어딘가에서 멈췄다. 그 줄과 Isaac 화면의 위치를 보고한다 |
| 검사 단계에서 `YOLO 워커가 종료되었습니다` | `ultralytics` 나 가중치 경로 문제다. `$SMARTFARM_STATION_OUT/yolo_worker.err` 를 보고한다. 1-1절 확인을 건너뛰었으면 거기부터 한다 |
| 검사·솎아내기를 빼고 앞 구간만 보고 싶음 | 2절에 `--no-vision-station` 을 붙인다 |
| Isaac 터미널에서 rclpy 오류로 죽음 | 그 터미널에서 워크스페이스를 `source` 했을 가능성이 크다. 새 터미널에서 워크스페이스 없이 2절만 실행한다 |
| `stdbuf: failed to run command 'isaac_python'` | 외부 명령을 앞에 붙이면 셸 별칭이 풀리지 않는다. 2절대로 `isaac_python` 을 첫 낱말로 두고 `PYTHONUNBUFFERED=1` 만 쓴다 |
| 주행 중 시간 초과 | 실시간 배율이 0.2 아래로 떨어진 경우다. GPU 부하(3절 csv)를 함께 본다. 이번 판은 카메라·YOLO 때문에 더 무겁다 |
| RViz2 클릭으로 손 시험하고 싶음 | 5절을 `dock_auto:=true` 로 띄우고 6·8절을 생략한다. `FEEDER_APPROACH` 를 Nav2 Goal 로 한 번 클릭하면 도착 후 자동으로 도킹한다 |

## 10. 결과 보고

아래를 함께 남긴다.

- `results/` 의 `build_gopi_*`, `isaac_*`, `link_*`, `nav2_*`, `navnode_*`, `cmd_*_gopi.txt`, `simresult_*`, `navresult_*`, `gpu_*.csv`
- `results/station_*/` 폴더 (검사 사진, `station_results.json`, 있으면 `yolo_worker.err`)
- bag 디렉터리 이름 (`~/.ros/smart_farm_navigation/bags/`)
- 8-3 의 `[도킹] 카터 본체 world (…)` 줄 **(필수)**
- 실패했으면 그 파일을 `errored/` 에 둔다

## 11. 파일과 역할

| 파일 | 실행 PC | 역할 |
|---|---|---|
| `isaacpjt/smart_farm/runtime/standalone_app.py` | 고피 | 장면 실행, 팔레트 집기·내려놓기, 라이다 fullScan, Place 전 차체 정지 확인과 도킹 위치 기록, 컨베이어·비전 스테이션 기동. 팀 파일(2026-09-25 반입) |
| `isaacpjt/smart_farm/scripts/conveyor.py`, `conveyor_rollers.py` | 고피 | 트레이 반송, 줄기 벨트 인터록(`hold_stem`), 검사 완료 후 배출. 팀 파일 |
| `isaacpjt/smart_farm/scripts/inspection_cull_station.py` | 고피 | 이송 프레임 생성·구동, YOLO 검사(하위 프로세스), 솎아내기 상태기계. 팀 파일 |
| `isaacpjt/smart_farm/scripts/cull_motion.py`, `human_crossing.py` | 고피 | 솎아내기 팔 모션 실행기, 작업자 돌발상황(선택). 팀 파일 |
| `isaacpjt/smart_farm/scenes/Collected_smartfarm_v014/` | 고피 | v014 양배추 씬·에셋·`best.pt`. git 제외 |
| `isaacpjt/smart_farm/maps/Collected_smartfarm_v014.{png,yaml}` | 내피 | Nav2 지도. `nav2.launch.py` 기본값 |
| `launch/nav2.launch.py` | 내피 | Nav2 · RViz2 · `/scan` 생성 · 정밀 도킹 노드 · 작업점 마커 · bag 기록 |
| `launch/navigation_node.launch.py` | 내피 | `/navigation/command` 를 받아 NavigateToPose 와 정밀 도킹을 구동. 제한 시간 600 s / 도킹 240 s(`/clock` 기준) |
| `smart_farm_navigation/feeder_dock.py` | 내피 | 라이다로 TurnTable 앞면을 보며 후진 도킹. `DockLogic`(상태기계, ROS 없음) + `FeederDock`(ROS 배선), `DockParams` 가 파라미터 단일 출처(`standoff_m` 0.92) |
| `smart_farm_navigation/cloud_self_filter.py` | 내피 | 결합카터 자기 반사 제거와 0.25 s 점군 합치기 |
| `smart_farm_navigation/nav2_link_check.py` | 내피 | 두 PC 사이 토픽 도달과 자기 반사 점검 |
| `smart_farm_navigation/stations.py` | 내피 | 작업점 읽기와 목표 자세 생성 |
| `config/stations.yaml` | 내피 | 작업점 좌표(FEEDER_APPROACH, FEEDER_DOCK)와 기준 장면 이름 |
| `config/nav2_params.yaml` | 내피 | Nav2 설정. Smac Hybrid-A*, RPP `desired_linear_vel` 0.3 m/s |
| `sim_test/dock_regression.py` | 내피 | 실측 전 회귀 시험(4-1절). 합성 로봇 + 현장 launch 로 10 시나리오 |
| `sim_test/dock_sim.py` | 내피 | 도킹 상태기계를 ROS 없이 돌리는 오프라인 격자 모의(225 케이스) |
| `sim_test/fake_robot.py` | 내피 | Isaac 대역. 지도에서 점군을 만들고 굼뜬 차체 응답을 흉내낸다 |
| `sim_test/replay_bag_dock.py` | 내피 | 실측 bag 의 `/scan` 을 같은 검출기에 다시 넣어 그때 제어기가 본 값을 찍는다 |
| `smart_farm_interfaces` | 고피·내피 | 명령·결과·상태 메시지. 명령을 보내는 PC 에 반드시 빌드되어 있어야 한다 |
