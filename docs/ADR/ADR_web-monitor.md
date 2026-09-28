# ADR_web-monitor

관제 웹·DB 트랙의 규칙. 브랜치 `feature/monitor` 와 `smart_farm_monitor` 패키지가 얽힌 모든 작업에 적용함.

- **상위는 `ADR_basic.md` 임.** 충돌하면 ADR_basic 이 이김. ADR_basic 에 이미 있는 범용 규칙(기기 구분, 답변 형식, 실측·모의 구분, git 운용 범위, 저장소 위생, 시간 기준의 원칙)은 이 문서에 다시 적지 않음
- 주행·도킹이 얽히면 `ADR_navigation2.md` 를 함께 적용함
- **이 문서를 임의로 수정하지 않음.** 사용자가 지시했거나, 수정 없이는 업무가 불가능함을 보고하고 승인받은 경우만 함

---

## 1. 이 트랙의 대상과 경계

1.1. **담당과 산출물**: 2026-09-27 팀 회의에서 웹 서비스·DB 구축이 이원호 담당으로 확정됨. 설계의 단일 출처는 `docs/04-monitoring-web-db.md` 임. 이 ADR 은 규칙만 담고 수치·스키마의 근거는 그 문서에 둠.

1.2. **쓰기 권한 추가 경로** (2026-09-28 사용자 승인): `cobot3_ws/src/smart_farm_monitor/`. ADR_basic §3 의 세 경로에 이 경로를 더함. 이 패키지 안은 전부 내 소유이므로 `[navigation YYYY-MM-DD]` 같은 표시가 필요 없음.

1.3. **팀 코드는 계약만 소비함.** 관제는 팀이 이미 발행하는 토픽만 구독하고, 팀 노드·런치·메시지를 고치지 않음. 관제 때문에 팀 파일을 고쳐야 할 상황이 생기면 **고치지 않고 보고**함. 실제로 필요했던 변경 한 건(`CycleStatus` 에 `pallet_locations` 추가)은 팀장 판단 대기 상태이며, 그 값 없이도 관제가 돌도록 대체 기록을 씀(§4.3).

1.4. **관제를 위해 팀 런치를 건드리지 않음.** 별도 `monitor.launch.py` 로 띄움. 시연 절차에 터미널이 하나 늘어나는 대가를 택한 것임(팀장 동의, 2026-09-28).

---

## 2. 구조 기준선 (바꾸려면 보고 후)

2.1. **두 프로세스 + 파일 하나.** `recorder`(rclpy 노드)가 토픽을 받아 SQLite 에 적고, `web`(FastAPI)이 같은 파일을 읽기 전용으로 조회함. **웹 프로세스에 ROS 를 넣지 않음.** 한 프로세스에 "계속 듣는 일" 과 "물어보면 답하는 일" 을 같이 시키면 스레드·asyncio 가 필요해지고 그건 ADR_basic 이 금지한 것임.

2.2. **웹에서 ROS 로 나가는 길은 `web_command` 표 하나뿐임.** 웹이 행을 넣고 recorder 가 읽어 `/start_cycle` 을 대신 호출함. 새 ROS 인터페이스를 만들지 않음. **세 번째 경로(웹이 직접 토픽·서비스를 쓰는 길)를 추가하지 않음.**

2.3. **시간 기준의 관제 쪽 적용** (원칙은 ADR_basic §5-6).
- 기록하는 모든 행에 `*_wall`(컴퓨터 시계)과 `*_sim`(`/clock`)을 **함께** 적음. 사이클 시간 분석은 `*_sim` 으로만 함
- **`/clock` 이 없으면 `*_sim` 은 0 이 아니라 빈 값(`None`)임.** 0 으로 적으면 소요시간이 0 으로 계산돼 분석을 망침
- **우편함 폴링과 상태 keepalive 타이머는 벽시계(`ClockType.STEADY_TIME`)로 돌림.** 공정 시간을 재는 것이 아니라 "웹이 뭔가 넣었는지", "노드가 살아 있는지" 를 보는 것임. 노드 시계로 두면 Isaac 이 없을 때 타이머가 아예 안 뜀(2026-09-28 에 실제로 겪음, §5.2)
- `config/monitor.yaml` 의 `use_sim_time` 은 **`true` 가 기준값**임. Isaac 없이 시험할 때만 내림

2.4. **DB 불변 규칙.**
- 표 10개: `cycle`, `step`, `executor_status`, `detection`, `inspection_verdict`, `pallet_move`, `dock_attempt`, `web_command`, `live`, `anomaly_label`. 정의는 `smart_farm_monitor/schema.sql` 이 출처임
- **쓰는 프로세스는 `recorder` 하나뿐임.** 웹은 `web_command` 와 `anomaly_label` 삽입만 하고 그 밖의 표에 쓰지 않음
- `PRAGMA journal_mode=WAL`. 이것이 있어야 쓰는 중에도 읽힘
- **고빈도 값은 이력으로 쌓지 않음.** 로봇 자세처럼 초당 여러 번 오는 것은 `live` 표의 한 행을 덮어씀
- **heartbeat 를 전부 적지 않음.** 값이 바뀐 순간과 10 초 생존 표본만 적음
- **늦게 온 결과는 지우거나 덮지 않음.** `late_result = 1` 인 별도 행으로 남김. Task Manager 가 TIMEOUT 으로 끝낸 뒤에도 Isaac 동작은 계속 돌기 때문임(팀 PR #12 본문에 명시된 사실)
- 통계 함수(`step_durations()`)는 `late_result = 0` 행만 셈

2.5. **웹 스택.** FastAPI + uvicorn + 순수 자바스크립트. **프레임워크·빌드 도구를 더하지 않음**(팀이 ROS 2 입문자임). 의존은 **apt 로 설치함** — 우분투 24.04 는 PEP 668 이라 `pip install` 이 막힘.
```
sudo apt-get install -y python3-fastapi python3-uvicorn python3-httpx
```
기본 포트는 **8080** 이고 `monitor.launch.py` 의 `port:=` 로 바꿈.

2.6. **실시간 갱신은 SSE 하나로 함.** `/api/events` 가 `version_token()`(각 표의 MAX(id) 묶음)을 0.5 초마다 보고 **바뀐 때만** 상태를 밀어줌. `rosbridge_suite` + `roslibjs` 는 도입하지 않음 — 같은 결과를 얻으면서 이력이 남지 않아 ML·사이클 분석에 쓸 수 없음. 지도 갱신이 체감상 답답할 때만 재검토함.

2.7. **지도 좌표 변환은 두 줄로만 씀.** 출처는 `maps/Collected_smartfarm_v014.yaml` 임.
```
px = (x - origin_x) / resolution
py = height_px - (y - origin_y) / resolution      # 이미지 원점은 좌상단
```
현행값 `resolution 0.05`, `origin (-4.525, -10.025)`, 이미지 285 × 460 px. **수치를 코드에 박지 않고 yaml 에서 읽음.** 좌표는 항상 **map 좌표계 x/y(m)** 로 표기함.

---

## 3. 문서·기록 배치 (2026-09-28 사용자 지시로 확정)

3.1. **가이던스는 그 작업이 속한 패키지 안에 둠.** 관제는 `cobot3_ws/src/smart_farm_monitor/guidance/` 임. 파일명은 **`guidance3_<n>차.md`** 이고 **27차부터** 시작함. 보관 폴더 이름은 사용자가 만든 **`past_monitor_guidance/`** 를 씀(`past/` 가 아님).

3.2. **이번 사례 (되풀이 금지).** 나는 `guidance3_27차.md` 를 `smart_farm_navigation/guidance/` 에 만들었음. 가이던스 경로를 ADR_basic §6.1 의 옛 문구(주행 패키지 고정)로만 읽고, 브랜치·패키지가 바뀐 상황을 반영하지 않은 잘못임. 사용자가 직접 `smart_farm_monitor/guidance/` 와 `past_monitor_guidance/` 를 만들어 옮겨 두었고, 그 구조를 기준으로 삼음. `guidance2_26차.md` 는 주행 트랙의 현행 문서이므로 옮기지 않고 그 자리에 둠.

3.3. **가이던스는 사용자의 실측 메뉴얼임.** 사용자가 **당장 해야 할 일 목록이 아니면** 내가 유용하다고 판단해도 `past_monitor_guidance/` 로 보내고 그곳에서 열람함. 실측을 내가 대행하는 판(ADR_basic 참조)의 가이던스는 작성 직후 보관 폴더로 보냄.

3.4. **실행 기록은 관제 패키지 자신의 `results/` 에 둠.**

| 경로 | 무엇 | git |
|---|---|---|
| `smart_farm_monitor/results/log/` | 터미널 `tee` 로그, 시험 출력, DB 조회 결과 | 추적함 |
| `smart_farm_monitor/results/log_media/<수행한것>_<YYYYMMDD>_<HHMM>/` | 화면 캡처·녹화 | 제외. 그 안의 `*.md` 만 추적 |
| `smart_farm_monitor/data/farm.db` | 기록 원본 | 제외 |

Isaac·Nav2·주행 쪽 로그는 그대로 `smart_farm_navigation/results/log/` 에 둠. **두 패키지의 로그를 섞지 않음.**

---

## 4. 팀과의 계약

4.1. **구독하는 것** (팀이 이미 발행 중. 관제 때문에 팀이 고칠 것은 없음)
`/cycle/status`, `/sim_task/{command,result,status}`, `/sim_task/inspection_data_status`, `/navigation/{command,result,status}`, `/inspection/{command,result,status}`, `/inspection/detections_2d`, `/feeder_dock/result`(latched), `/amcl_pose`.

4.2. **`/start_cycle` 의 response 는 현행 3필드(`accepted`, `task_id`, `reason`)를 그대로 씀** (2026-09-28 팀장과 합의). 웹의 시작 버튼은 수락 여부만 즉시 알면 되고 전체 결과는 `/cycle/status` 로 봄. **서비스 인터페이스 변경을 요청하지 않음.**

4.3. **미발행 1건**: `CycleStateMachine.pallet_locations`. 그때까지 `pallet_move` 의 위치는 명령의 `destination` → `reached_station` → 공정 단계 이름 순으로 대체 기록함. **같은 판정 규칙을 관제에 다시 구현하지 않음** — 규칙이 두 곳에 생기면 팀이 한쪽만 바꿀 때 관제가 틀린 값을 보여줌.

4.4. **공정 단계는 12개임.** `TRANSFER`, `PICK_HARVEST`, `NAVIGATION`, `PLACE_INSPECT`, `CONVEY_TO_INSPECT`, `PREPARE_INSPECT`, `MOVE_TO_INSPECT`, `INSPECT`, `CULL`, `RECHECK`, `RELEASE_INSPECT`, `CONVEYOR_OUT`. 출처는 팀 `scenario.py` 의 `StepDefinition` 임. `CycleState` enum 은 16개(`IDLE`·`PREFLIGHT`·`COMPLETE`·`ERROR` 포함)이므로 **두 수를 섞지 않음.**

4.5. **계약이 바뀌면 recorder 를 먼저 고침.** 팀 브랜치를 반입할 때마다 `scenario.py` 의 단계 수와 `task_manager_node.py` 의 토픽 목록을 대조함.

4.6. **이 DB 가 팀 두 사람의 입력임.** 이상 탐지(봉승현)는 `step`(`duration_sim`·`status`·`reason`·`late_result`)과 `dock_attempt`, 사이클 시간 최적화(백승주)는 같은 `step` 표를 씀. **스키마에서 컬럼을 지우거나 의미를 바꾸기 전에 보고함.**

---

## 5. 내가 틀리기 쉬운 지점 (금지·확인 문구)

5.1. **`/feeder_dock/result` 와 `/feeder_dock/status` 는 latched 임**(`RELIABLE` + `TRANSIENT_LOCAL`, depth 1, `feeder_dock.py:369`). 구독 QoS 를 기본값으로 두면 **한 건도 못 받음.** 손으로 시험 발행할 때는 `ros2 topic pub --qos-durability transient_local --qos-reliability reliable` 를 붙임. 또 latched 라 재접속 시 같은 값이 다시 오므로 `run_id` 로 중복을 걸러냄.

5.2. **`use_sim_time: true` 인데 `/clock` 이 없으면 노드 시계 타이머가 한 번도 뛰지 않음.** 2026-09-28 에 이 때문에 웹 시작 버튼이 영원히 응답을 못 받았음. 새 타이머를 만들 때마다 **"이건 공정 시간인가, 노드 생존·입력 확인인가"** 를 먼저 정함. 회귀는 `test_web_command_drains_without_clock` 이 지킴.

5.3. **설치본과 소스 트리의 경로 깊이가 다름.** `install/smart_farm_monitor/lib/python3.12/site-packages/…` 라 상대 경로가 어긋남. `schema.sql`·`web/` 은 `share/smart_farm_monitor/` 를 **위로 올라가며 찾는** 방식으로만 구함. 깊이를 숫자로 박지 않음.

5.4. **화면이 비어 있는 것은 결함이 아님.** 공정을 한 번도 돌리지 않았으면 표가 비는 것이 정상임. 판단은 `/api/state` 의 값과 recorder 로그로 함.

5.5. **관제만 띄운 상태에서 시작 버튼이 `거절: SERVICE_NOT_AVAILABLE` 인 것은 정상 동작임.** `/start_cycle` 을 제공하는 것은 Task Manager 임. 이것을 결함으로 보고하지 않음.

5.6. **"응답이 오지 않음" 을 "거절" 로 표시하지 않음.** `PENDING`(recorder 가 안 읽음)과 `SENT`(서비스가 답을 안 줌)와 `REJECTED`(실제 거절)를 화면에서 구분함.

5.7. **프로세스를 끌 때 `pkill -f` 를 쓰지 않음.** 포트가 잡혀 있으면 `ss -ltnp | grep <포트>` 로 PID 를 찾아 `kill <PID>` 함. 2026-09-28 에 옛 웹 프로세스가 포트를 잡아 새 판이 안 뜬 일이 있었음.

5.8. **모의와 실측을 섞지 않음**(ADR_basic §5-5). 가짜 발행자(`test_recorder_node.py`, 스모크 스크립트)로 만든 DB 행은 실측이 아님. 실측 판정은 `results/log/` 와 `results/log_media/` 의 회차 폴더로만 함.

---

## 6. 검증 기준선 (2026-09-28, 고피3)

| 항목 | 상태 |
|---|---|
| `colcon build` (`smart_farm_monitor`, `smart_farm_navigation`) | 통과 |
| 시험 **39건** (`test_store` 17 + `test_recorder_node` 7 + `test_web_app` 15) | 통과. **ROS·Isaac 없이 돌아감** |
| 화면 5개 + `/static/*` + `/api/map` | HTTP 200 |
| SSE `/api/events` | 첫 이벤트 수신 확인 |
| 기록 배선 | `/cycle/status`·`/sim_task/{command,result}`·`/feeder_dock/result` 로 `cycle`·`step`·`dock_attempt` 각 1행 생성 확인 |
| 시작 버튼 경로 | `web_command` → recorder → `/start_cycle` 호출까지 확인(Task Manager 없어 `REJECTED/SERVICE_NOT_AVAILABLE`) |
| **전 구간 실측** | **미실시.** Isaac + Nav2 + 주행 + 검사 + Task Manager + 관제를 한 번에 돌린 기록이 아직 없음 |

시험은 도메인 **77** + `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 로 격리해 돌림. 실측 중에는 시험을 돌리지 않음.

---

## 7. 미해결

1. **전 구간 실측 0회.** 지도에 로봇이 찍히고 간트가 그려진 화면이 아직 없음.
2. **검사 노드(Docker) 를 이 기기에서 띄운 기록이 없음.** 컨테이너의 `ROS_DOMAIN_ID` 가 고피3 와 같아야 하며 미확인임.
3. **`--no-vision-station` 을 뺀 Isaac 기동을 이 기기에서 해 본 적 없음.** 로딩 시간·로그가 26차 기록과 다를 수 있음.
4. `pallet_locations` 팀장 판단 대기(§4.3).
5. 이상 탐지 결과 토픽 이름 미정. 정해지면 대시보드 배너에 더함.
6. 검사 디버그 영상(JPEG) 겹치기 미구현. `cv_bridge` 의존이 늘어 선택 항목으로 둠.
7. **교육장 복귀 후 다중 기기 구성에서 미검증.** 관제는 여러 기기의 토픽을 받아야 하므로 복귀 첫 실행에서 `/cycle/status` 수신 여부를 먼저 확인함.
