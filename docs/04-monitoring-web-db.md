# 관제 웹 서비스 및 DB 구축 계획 (04)

- 담당자: 이원호
- 작성일: 2026-09-28
- 근거 문서: `docs/01-architecture.md` §12(미확정 "작업 기록의 저장 위치와 형식"), `docs/02-interfaces.md`(토픽 계약), `docs/고도화_개발_일정_및_발표_준비.md`(2026-09-27 회의 역할 분담), `docs/00-Business_Requirement_Document/비즈니스 정의 …md`
- 상태: **계획안. 사용자 승인 전이며 코드는 아직 만들지 않았음.**

---

## 1. 이 작업이 채우는 빈칸

1. 팀 아키텍처 문서 `01-architecture.md` §12 의 미확정 항목 중 **"작업 기록의 저장 위치와 형식"** 이 아직 비어 있음. Task Manager 는 사이클 상태를 `/cycle/status` 로 **발행만** 하고 어디에도 **저장하지 않음**(`task_manager_node.py` 전체에 파일·DB 쓰기 없음). 즉 지금은 사이클이 끝나면 기록이 사라짐.
2. 2026-09-27 회의에서 **웹 서비스 및 DB 구축이 이원호 담당, 기한 2026-09-28** 로 확정됨.
3. 같은 회의에서 봉승현 담당 **ML 기반 이상 탐지**의 "입력 데이터, 정상·이상 정의" 가 미확정으로 남아 있음. 이상 탐지의 학습·판정 입력은 결국 **공정 기록**이므로, 본 DB 스키마가 그 미확정을 해소하는 선행 조건임.
4. 백승주 담당 **공정 사이클 시간 최적화**도 단계별 소요시간 기록이 있어야 병목을 지목할 수 있음. 같은 DB 의 `step` 테이블 한 개로 두 사람의 입력을 동시에 공급함.
5. 비즈니스 정의 문서의 프로젝트 가치 4번 **"트레이 단위 추적성 확보"** 와 검증 지표 **"트레이 위치·상태 기록 일치율"** 은 현재 어디에도 구현체가 없음. `pallet_move` 테이블이 이 지표의 측정 수단이 됨.

> 요약: 관제 웹·DB 는 부가 기능이 아니라 **팀의 미확정 항목 3개(작업 기록 형식, ML 입력, 트레이 추적성)를 동시에 닫는 작업**임. 발표 자료에서도 챌린지 5번(ML 이상 탐지)의 전제 조건으로 제시할 수 있음.

---

## 2. 원격 레포 확인 결과 (2026-09-28 기준)

`git fetch --all` 로 확인한 사실만 적음.

| 브랜치 | 최신 커밋 | 시각 | 내용 |
|---|---|---|---|
| `origin/feature/task-managed-integration` | `7beec39` | 2026-09-28 01:50 | 봉승현. `test: verify color-based cull bin routing` |
| `origin/feature/synthetic-anomaly-detection` | `7beec39` | 2026-09-28 01:50 | **위와 동일 커밋.** 이상 탐지 작업은 브랜치만 만들어 둔 상태이며 아직 커밋 없음 |
| `origin/feature/cabbage-place-fix` | `3a4ae3a` | 2026-09-27 23:47 | 백승주. 가속·도킹 속도 실험 옵션, 파지·이송 속도 실험 문서 |
| `origin/feature/lwh` (내 브랜치) | `d6125ac` | 2026-09-27 06:16 | 영상 콘티·스토리보드 |
| `origin/development` | 2026-09-22 | | 통합분 미반영. 팀 작업은 아직 `development` 로 올라오지 않았음 |

### 2.1 봉승현 팀장 통합 상태 — "1차 완성 임박" 이 아니라 **구조는 이미 완성되어 검증 단계**임

`origin/feature/task-managed-integration` 의 최근 83 커밋을 확인한 결과, `standalone_app.py` 단일 스크립트 통합을 넘어 **Task Manager 중심 아키텍처가 전부 들어와 있음**.

- 새 ROS 2 패키지 `cobot3_ws/src/smart_farm_manager/`
  - `task_manager_node.py`(872줄), `state_machine.py`, `scenario.py`, `protocol.py`, `mock_executor.py`
  - 런치 3종: `task_manager.launch.py`, `task_manager_fake_test.launch.py`, `task_manager_navigation_test.launch.py`
  - 단위 시험: `test_state_machine.py`, `test_json_protocol.py`
- Isaac Sim 쪽 어댑터 `cobot3_ws/isaacpjt/smart_farm/runtime/sim_task_node.py`(484줄), `inspection_detection_store.py`
- 새 Isaac 모듈: `cull_bin_routing.py`, `cull_slot_mapping.py`, `cull_target_geometry.py`, `conveyor_rollers.py` + 각각의 `test_*.py`
- 최근 커밋의 성격이 `feat:` 가 아니라 **`test:` / `fix:` / `docs:` 중심**임(예: `test: verify color-based cull bin routing`, `fix: clear pending command and inspection data during scene reset`). 골격을 세우는 단계가 아니라 예외 처리와 회귀를 메우는 단계임.

즉 **관제 웹이 붙을 데이터 계약은 이미 고정되어 있음.** 웹·DB 설계를 팀 통합 완료까지 기다릴 이유가 없고, `docs/02-interfaces.md` 의 계약만 지키면 됨.

### 2.2 확정된 데이터 계약 (`docs/02-interfaces.md`, 코드로 재확인)

사이클 상태는 `CycleState` 15종임: `IDLE`, `PREFLIGHT`, `TRANSFER`, `PICK_HARVEST`, `NAVIGATION`, `PLACE_INSPECT`, `CONVEY_TO_INSPECT`, `PREPARE_INSPECT`, `MOVE_TO_INSPECT`, `INSPECT`, `CULL`, `RECHECK`, `RELEASE_INSPECT`, `CONVEYOR_OUT`, `COMPLETE`, `ERROR`.

| 토픽/서비스 | 타입 | 관제 웹에서의 용도 |
|---|---|---|
| `/start_cycle` | `smart_farm_interfaces/srv/StartCycle` | 웹 "시나리오 시작" 버튼 |
| `/cycle/status` | `CycleStatus` msg | 현재 공정 단계, 사이클 최종 결과 |
| `/sim_task/command` `/result` `/status` | `std_msgs/String` JSON | Isaac 작업 지시·결과·phase |
| `/navigation/command` `/result` `/status` | `TaskCommand`·`TaskResult`·`ExecutorStatus` | 주행·도킹 |
| `/inspection/command` `/result` `/status` | 위와 같음 | 검사 |
| `/inspection/detections_2d` | `std_msgs/String` JSON | 슬롯별 class·confidence·bbox(원본 RGB pixel 좌표 u/v) |
| `/feeder_dock/result` `/status` | `std_msgs/String` JSON (latched) | 도킹 품질(우리 모듈) |
| `/clock` | `rosgraph_msgs/Clock` | 공정 소요시간의 기준 시각 |
| `/amcl_pose` | `PoseWithCovarianceStamped` | 도킹 완료 시점의 **map 좌표계 x/y/yaw** 표본 |

---

## 3. 설계

### 3.1 구조 — ROS 프로세스와 웹 프로세스를 분리하고 SQLite 파일 하나로 잇는다

```
[ROS 2 도메인 101]
 Task Manager ──/cycle/status────────┐
 Sim Task     ──/sim_task/*──────────┤
 Navigation   ──/navigation/*────────┤   ┌──────────────────┐
 Inspection   ──/inspection/*────────┼──▶│  recorder_node   │──쓰기──▶ farm.db
 feeder_dock  ──/feeder_dock/*───────┤   │  (rclpy 노드)     │         (SQLite, WAL)
 Isaac Sim    ──/clock, /amcl_pose───┘   └──────────────────┘             │
                                                  ▲                        │ 읽기 전용
                                        /start_cycle 호출                  │
                                                  │                        ▼
                                          web_command 테이블 ◀──삽입── [web_app]
                                                                      (FastAPI, ROS 의존 없음)
                                                                             ▲ HTTP
                                                                        브라우저(1 s 폴링)
```

핵심 결정 3가지와 이유:

1. **웹 프로세스에 ROS 를 넣지 않음.** 웹 서버는 요청을 기다리고 ROS 노드는 `spin` 을 돌아야 하므로 한 프로세스에 두면 스레드나 asyncio 가 필요해짐. ADR_basic 의 "불필요한 스레드/asyncio 금지"에 어긋나고 입문자가 디버깅하기 어려움. 두 프로세스로 나누고 **SQLite 파일을 유일한 접점**으로 삼으면 양쪽 모두 평범한 코드로 유지됨.
2. **웹에서 ROS 로 나가는 명령은 DB 테이블을 우편함으로 씀.** 웹이 `web_command` 테이블에 행 하나를 넣고, `recorder_node` 의 0.5 s `create_timer` 가 그 행을 읽어 `/start_cycle` 서비스를 호출하고 응답을 같은 행에 적음. 웹에 ROS 도, 소켓도 필요 없음. 새 ROS 인터페이스를 만들지 않으므로 팀 계약도 건드리지 않음.
3. **DB 는 SQLite.** 설치가 필요 없고(파이썬 표준 `sqlite3`), 파일 하나라 백업·이전이 `scp` 한 번임. 초당 수십 행 쓰기에 충분함. PostgreSQL 은 서버·계정·백업 관리 부담이 시연 규모에 맞지 않음. 봉승현 팀장 쪽 ML 도 `pandas.read_sql` 한 줄로 읽을 수 있음. 동시 읽기·쓰기를 위해 `PRAGMA journal_mode=WAL` 을 씀.

### 3.2 시간 기준 — 벽시계와 sim 시각을 **둘 다** 기록

- Isaac 실시간 배율이 0.3 전후이므로 **벽시계 경과 ≠ 공정 소요시간**임. 사이클 타임 최적화(백승주)는 반드시 sim 시각 기준이어야 함.
- 따라서 모든 행에 `*_wall`(epoch 초, 행 순서·실제 대기시간용)과 `*_sim`(`/clock` 초, 공정 시간용)을 함께 적음. `recorder_node` 는 `use_sim_time: true` 로 띄우고 `/clock` 을 직접 구독해 최신 sim 초를 들고 있음.
- **확인 필요**: 팀 `demo_harvest.yaml` 의 Task Manager 는 `use_sim_time: false` 이고 timeout 판정에 `time.monotonic()`/`STEADY_TIME` 을 씀. 이는 ADR_basic §5-6("타임아웃 판정은 `/clock` 기준")과 다름. Task Manager 는 Isaac 밖에서 도는 관리자 노드이므로 팀이 의도한 선택일 수 있음. 관제 DB 쪽은 `recorder` 가 `/clock` 을 직접 붙이므로 팀 코드 수정 없이 해결됨. 다만 **"timeout 값의 의미가 벽시계 초" 라는 점은 팀과 합의해 두어야 함**(sim 0.3 배속이면 sim 기준 timeout 이 실질 1/3 로 줄어듦).

### 3.3 DB 스키마 (초안)

```sql
PRAGMA journal_mode=WAL;

-- 1. 사이클 1회 = 1행
CREATE TABLE cycle (
  task_id         TEXT PRIMARY KEY,
  scenario_id     TEXT NOT NULL,
  started_wall    REAL NOT NULL,   -- epoch 초
  started_sim     REAL,            -- /clock 초
  ended_wall      REAL,
  ended_sim       REAL,
  final_state     TEXT,            -- COMPLETE / ERROR / NULL(진행 중)
  terminal_status TEXT,            -- RUNNING / SUCCEEDED / FAILED / IDLE
  failure_reason  TEXT             -- NONE 또는 실패 사유
);

-- 2. 공정 명령 1건 = 1행. 사이클 타임·이상 탐지의 주 입력
CREATE TABLE step (
  id            INTEGER PRIMARY KEY AUTOINCREMENT,
  task_id       TEXT NOT NULL,
  command_id    TEXT NOT NULL,
  seq           INTEGER,           -- command_id 말미 순번
  cycle_state   TEXT,              -- 명령 발행 시점의 CycleState
  executor      TEXT,              -- sim_task / navigation / inspection
  operation     TEXT,
  recipe_id     TEXT,
  pallet_id     TEXT,
  source        TEXT,
  destination   TEXT,
  target_slots  TEXT,              -- JSON 배열 문자열
  issued_wall   REAL, issued_sim  REAL,
  result_wall   REAL, result_sim  REAL,
  duration_wall REAL,              -- result_wall - issued_wall
  duration_sim  REAL,              -- 공정 소요시간(분석 기준)
  status        TEXT,              -- SUCCEEDED / FAILED / NULL(진행 중)
  phase         TEXT,
  reason        TEXT,
  UNIQUE(task_id, command_id)
);

-- 3. executor 상태. heartbeat 0.5 s 전부가 아니라 값이 바뀐 순간만 + 10 s 간격 생존 표본
CREATE TABLE executor_status (
  id         INTEGER PRIMARY KEY AUTOINCREMENT,
  recv_wall  REAL, recv_sim REAL,
  executor   TEXT, state TEXT,     -- STARTING/READY/BUSY/PAUSED/ERROR
  task_id    TEXT, command_id TEXT,
  operation  TEXT, phase TEXT, detail TEXT
);

-- 4. 검사 검출. 슬롯 1개 = 1행
CREATE TABLE detection (
  id           INTEGER PRIMARY KEY AUTOINCREMENT,
  task_id      TEXT, command_id TEXT, pallet_id TEXT,
  pass_no      INTEGER,            -- 1=INSPECT, 2=RECHECK
  stamp_sim    REAL, recv_wall REAL,
  frame_id     TEXT, image_width INTEGER, image_height INTEGER,
  slot_id      TEXT,               -- SLOT_01~06, 미판정은 빈 문자열
  class_name   TEXT, confidence REAL,
  center_u     REAL, center_v REAL,     -- 원본 RGB 영상 pixel 좌표 (u=가로, v=세로)
  bbox_x_min   REAL, bbox_y_min REAL,
  bbox_x_max   REAL, bbox_y_max REAL
);

-- 5. 검사 판정. command 1건 = 1행
CREATE TABLE inspection_verdict (
  id            INTEGER PRIMARY KEY AUTOINCREMENT,
  task_id       TEXT, command_id TEXT, pallet_id TEXT,
  operation     TEXT,              -- INSPECT / RECHECK
  defect_slots  TEXT, unknown_slots TEXT,   -- JSON 배열 문자열
  recv_wall     REAL, recv_sim REAL
);

-- 6. 팔레트(트레이) 위치 이력. 비즈니스 지표 "트레이 위치·상태 기록 일치율" 의 측정 근거
CREATE TABLE pallet_move (
  id         INTEGER PRIMARY KEY AUTOINCREMENT,
  task_id    TEXT, command_id TEXT,
  pallet_id  TEXT,
  location   TEXT,                 -- RACK_L2 / CARRY / INSPECT_STATION / …
  source     TEXT,                 -- 직전 위치(명령의 source)
  recv_wall  REAL, recv_sim REAL
);

-- 7. 도킹 시도. 발표 챌린지 2번(도킹 재시도·예외 처리)의 수치 근거
CREATE TABLE dock_attempt (
  id           INTEGER PRIMARY KEY AUTOINCREMENT,
  task_id      TEXT, command_id TEXT,
  run_id       TEXT, status TEXT, reason TEXT,
  face_dist_m  REAL,   -- TurnTable 라이다 검출 면 기준 상대좌표계, 면 법선 방향 거리 m
  yaw_err_deg  REAL,   -- 같은 면 기준 상대 yaw 오차 도
  lat_m        REAL,   -- 같은 면 접선 방향(횡) 오차 m
  retry        INTEGER,-- BACKOFF 재시도 횟수
  map_x        REAL, map_y REAL, map_yaw_deg REAL,  -- /amcl_pose, map 좌표계
  recv_wall    REAL, recv_sim REAL
);

-- 8. 웹 → ROS 명령 우편함
CREATE TABLE web_command (
  id             INTEGER PRIMARY KEY AUTOINCREMENT,
  kind           TEXT NOT NULL,    -- START_CYCLE
  payload        TEXT,             -- {"scenario_id":"DEMO_HARVEST_01"}
  requested_wall REAL NOT NULL,
  state          TEXT NOT NULL DEFAULT 'PENDING',  -- PENDING/SENT/ACCEPTED/REJECTED
  response       TEXT
);
```

### 3.4 웹 화면 (6개)

| 화면 | 보여주는 것 | 근거 데이터 |
|---|---|---|
| ① 대시보드 | 현재 `CycleState`, 진행 중 명령, executor 3개 상태 램프, 경과 시간(sim·wall 병기), 15단계 체크리스트, 실패 사유 | `cycle`, `step`, `executor_status` |
| ② 사이클 이력 | 사이클 목록(성공·실패·소요시간) → 클릭 시 단계별 소요시간 막대 | `cycle`, `step` |
| ③ 검사 결과 | 슬롯 6칸 그리드, class·confidence, 불량/미판정 표시, RECHECK 전후 비교 | `detection`, `inspection_verdict` |
| ④ 트레이 추적 | 팔레트별 현재 위치와 이동 이력, 기록 일치율 | `pallet_move` |
| ⑤ 도킹 품질 | `face_dist_m`·`yaw_err_deg`·`lat_m` 분포, 재시도 횟수, 성공률 | `dock_attempt` |
| ⑥ 관제 조작 | "DEMO_HARVEST_01 시작" 버튼과 수락·거절(BUSY) 표시 | `web_command` |

KPI 타일(대시보드 상단): 사이클 성공률, 평균 사이클 시간(sim), 단계별 평균·최대 시간, 도킹 성공률과 평균 재시도, 불량 검출 수, 트레이 위치 기록 일치율. 모두 비즈니스 정의 문서의 검증 지표와 1:1 대응함.

### 3.5 파일 배치 (제안)

```
cobot3_ws/src/smart_farm_monitor/          ← 새 ROS 2 패키지 (승인 필요, §5 참조)
├── package.xml
├── setup.py
├── config/monitor.yaml                    # db_path, 폴링 주기, 기록 대상 토픽
├── launch/monitor.launch.py               # recorder + web 동시 기동
├── smart_farm_monitor/
│   ├── __init__.py
│   ├── schema.sql                         # §3.3 그대로
│   ├── store.py                           # sqlite3 래퍼. ROS 의존 없음
│   ├── recorder_node.py                   # rclpy 노드. 토픽 → store, web_command → /start_cycle
│   └── web_app.py                         # FastAPI. store 읽기 전용
├── web/  index.html, history.html, inspection.html, app.js, style.css
├── test/ test_store.py                    # ROS 없이 도는 순수 파이썬 시험
└── data/                                  # farm.db. .gitignore 로 제외
```

`store.py` 를 ROS 의존 없는 순수 파이썬으로 떼어 두면 Isaac·Nav2 없이도 `test_store.py` 로 회귀를 돌릴 수 있음. 팀의 `state_machine.py`(ROS 의존 없음)와 같은 방식이라 팀 관례에도 맞음.

---

## 4. 단계별 일정

| 단계 | 내용 | 산출물 | 소요 |
|---|---|---|---|
| **0** | 스키마 확정 + `store.py` + `test_store.py`. ROS 없이 완결 | `schema.sql`, `store.py`, 시험 통과 | 1~2 h |
| **1** | `recorder_node.py`: `/cycle/status`·`/*/command`·`/*/result`·`/clock` 기록. **기록이 먼저 돌아야 데이터가 쌓임** | 사이클 1회가 `cycle`·`step` 에 남음 | 2~3 h |
| **2** | `web_app.py` + 화면 ①② (대시보드·이력) | 브라우저에서 진행 상황 확인 | 2~3 h |
| **3** | `/inspection/detections_2d`·`/feeder_dock/result`·`/amcl_pose` 기록 + 화면 ③⑤ | 검사·도킹 화면 | 2~3 h |
| **4** | 화면 ⑥ 관제 조작(`web_command` → `/start_cycle`) + 화면 ④ 트레이 추적 | 웹에서 시연 시작 | 2 h |
| **5** | CSV·Parquet 내보내기 API. 봉승현 ML·백승주 사이클 타임에 인계 | `/export/steps.csv` 등 | 1 h |
| **6** (여유 시) | 실시간 갱신(SSE)으로 폴링 대체, 영상 촬영용 화면 정리 | 영상 소스 | 2 h |

- 0~2 단계까지가 **오늘(2026-09-28) 기한의 실질 목표**임. 3단계 이후는 실측 데이터가 쌓이는 속도에 맞춰 진행함.
- 0단계와 2단계는 Isaac·Nav2 없이 진행 가능함. 1·3·4단계 검증만 실행이 필요함.

---

## 5. 승인·합의가 필요한 항목

### 5.1 사용자 승인이 필요한 것 (ADR_basic §3 쓰기 권한)

1. **새 패키지 `cobot3_ws/src/smart_farm_monitor/` 생성 승인.** 현재 내 쓰기 허용 범위는 `smart_farm_navigation/`, `isaacpjt/smart_farm/`, `docs/` 셋뿐임. 관제는 주행과 별개 관심사이므로 형제 패키지로 두는 것이 옳다고 보지만, 승인 없이는 만들 수 없음. 대안은 `smart_farm_navigation/` 안에 하위 모듈로 넣는 것인데, 관제 기능이 주행 패키지에 섞여 팀원이 찾기 어려워짐.
2. **`feeder_dock.py` 결과에 `retry` 필드 1개 추가.** `DockLogic._finish()` 가 만드는 결과 JSON 에 현재 재시도 횟수(`self.retry`)가 빠져 있어 도킹 재시도 통계를 낼 수 없음. 우리 소유 파일이고 기존 필드는 그대로 두는 가산적 변경임.
3. **Isaac Sim 실행 승인**(ADR_basic §2-1). 1·3·4단계 검증에 실제 사이클 실행이 필요함. 다만 이는 처음 보는 실패를 소진하는 성격이 아니라 기록 배선 확인이므로, 사용자가 실측·녹화할 회차와 별도로 짧게 돌리는 것으로 제안함.

### 5.2 봉승현 팀장과 합의할 것 (모두 가산적 변경, 기존 계약 비파괴)

1. **`CycleStatus.msg` 에 `string[] pallet_locations` 추가 제안.** `CycleStateMachine.pallet_locations` 딕셔너리가 트레이의 논리 위치를 이미 들고 있으나 어디에도 발행되지 않음. 관제가 이를 직접 유도 계산하면 상태 머신 규칙이 두 곳에 중복됨. `"PALLET_001:CARRY"` 형태 배열 한 필드만 늘리면 중복 없이 해결됨. **이 필드가 없으면** 관제는 명령의 `pallet_id`·`destination` 필드로만 위치를 기록하고, 빈 `destination` 구간은 `CycleState` 이름으로 대체함(정확도가 떨어짐).
2. **ML 이상 탐지 입력을 이 DB 로 확정할지 합의.** 제안: 이상 탐지의 입력은 `step` 테이블(단계별 `duration_sim`, `status`, `reason`, `phase`)과 `dock_attempt`(횡 오차·재시도), 라벨은 `status`/`reason`. 회의록의 미확정 항목 "ML 이상 탐지의 입력 데이터, 정상·이상 정의" 가 이로써 정리됨. 정상 사이클 표본을 모으려면 **기록 노드를 백승주의 반복 실험 회차에도 같이 띄우는 것**이 가장 효율적임.
3. **Task Manager timeout 의 시간 기준 확인**(§3.2). `use_sim_time: false` + `time.monotonic()` 이 의도된 선택인지 확인만 필요함.
4. **기록 노드를 어느 launch 에 넣을지.** `task_manager.launch.py` 에 포함시키면 항상 기록되지만 팀 소유 파일 수정이 됨. 별도 `monitor.launch.py` 를 따로 띄우는 쪽을 먼저 제안함.

---

## 6. 위험과 대응

| 위험 | 대응 |
|---|---|
| 내 브랜치 `feature/lwh` 가 팀 브랜치보다 83 커밋 뒤처져 있어 실제 사이클을 돌릴 수 없음 | 0~2단계는 `docs/02-interfaces.md` 계약만 쓰므로 브랜치 무관하게 개발 가능. 검증 직전에 `origin/feature/task-managed-integration` 을 반입함. 반입 시 ADR_nav2 §2.4 점검표로 내 추가분 소실 확인 |
| 워킹트리에 미커밋 변경 152건(대부분 `results/` 파일 삭제)이 있어 브랜치 전환·반입이 꼬일 수 있음 | 반입 전에 이 삭제가 의도된 정리인지 확인받고 커밋 또는 복원함. **삭제된 `results/` 파일은 실측 기록이므로 임의로 커밋하지 않음** |
| `/sim_task/status` 0.5 s heartbeat 를 전부 저장하면 DB 가 불필요하게 커짐 | 값이 바뀐 순간만 저장 + 10 s 간격 생존 표본 |
| SQLite 동시 접근 | `journal_mode=WAL`, 웹은 읽기 전용 연결. 쓰기는 `recorder_node` 하나만 |
| 사이클 도중 `recorder_node` 재시작 | `step` 의 `UNIQUE(task_id, command_id)` 로 중복 방지. `result_*` 가 NULL 인 행은 "진행 중 또는 유실" 로 표시하고 지우지 않음 |
| 기한(2026-09-28) 안에 6화면 전부는 무리 | 0~2단계(대시보드·이력)를 기한 산출물로 잡고 나머지는 발표 준비 기간에 이어서 함 |

---

## 7. 파일·역할 요약

| 파일 | 역할 |
|---|---|
| `docs/04-monitoring-web-db.md` | 이 계획서. 관제 웹·DB 의 설계 단일 출처 |
| `smart_farm_monitor/schema.sql` | 테이블 8개 정의 |
| `smart_farm_monitor/store.py` | SQLite 읽기·쓰기. ROS 의존 없음 |
| `smart_farm_monitor/recorder_node.py` | 토픽 → DB 기록, `web_command` → `/start_cycle` 호출 |
| `smart_farm_monitor/web_app.py` | 조회 API 와 화면 제공. ROS 의존 없음 |
| `smart_farm_monitor/data/farm.db` | 기록 원본. git 제외 |
| `smart_farm_monitor/test/test_store.py` | ROS·Isaac 없이 도는 회귀 시험 |

---

## 8. 미해결·가정

- `smart_farm_monitor` 패키지 생성 승인 전임. 승인 전에는 이 문서 외 어떤 파일도 만들지 않음.
- 웹 프레임워크는 FastAPI + uvicorn 을 전제로 적었음. 세 기기(고피1·고피2·내피) 모두에 `pip install` 이 필요함. 설치를 피하려면 파이썬 표준 `http.server` 로도 같은 구조가 되지만 JSON 직렬화와 라우팅을 직접 써야 함. 미확정.
- 검사 디버그 영상(`Image` 토픽, 토픽 이름은 `debug_image_topic` 파라미터)을 JPEG 로 저장해 웹에 띄울지는 미정임. `cv_bridge` 의존이 늘어나므로 3단계 선택 항목으로 둠.
- `/amcl_pose` 를 도킹 완료 시점 표본으로 쓰는 것은 계획이며 아직 검증 안 함. AMCL 이 실제로 발행 중인지 실행으로 확인해야 함.
- 사이클 타임의 "정상 범위" 는 실측 표본이 없어 정할 수 없음. 백승주의 반복 실험 결과가 나온 뒤 확정함.
- 본 문서의 수치·판정은 **모두 코드와 문서 열람 결과이며 실측이 아님.**
