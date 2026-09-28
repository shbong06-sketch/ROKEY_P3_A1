# 관제 웹 서비스 및 DB 구축 계획 (04)

- 담당자: 이원호
- 작성일: 2026-09-28
- 근거 문서: `docs/01-architecture.md` §12(미확정 "작업 기록의 저장 위치와 형식"), `docs/02-interfaces.md`(토픽 계약), `docs/고도화_개발_일정_및_발표_준비.md`(2026-09-27 회의 역할 분담), `docs/00-Business_Requirement_Document/비즈니스 정의 …md`
- 상태: **계획안. 2026-09-28 사용자 지시로 계획만 확정하고 코드 착수는 보류함.** 웹 프레임워크는 FastAPI + uvicorn 으로 확정함.

> **프로그래밍을 모르는 사람이 읽을 부분은 §9 와 §10 임.** §1~§8 은 구현 담당자용 상세 설계임.

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
  late_result   INTEGER DEFAULT 0,   -- Task Manager 가 TIMEOUT 판정한 뒤 늦게 도착한 결과면 1
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
| ① 대시보드 | 현재 `CycleState`, 진행 중 명령, executor 3개 상태 램프, 경과 시간(sim·wall 병기), 12단계 체크리스트, 실패 사유 | `cycle`, `step`, `executor_status` |
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

- **2026-09-28 사용자 지시: 계획만 확정하고 코드 착수는 보류함.** 패키지를 어디에 둘지(`smart_farm_monitor` 신설 vs `smart_farm_navigation` 안)도 함께 보류함.
- 웹 프레임워크는 **FastAPI + uvicorn 으로 확정**(2026-09-28 사용자 결정). 고피1·고피2·내피 세 기기에 `pip install fastapi uvicorn` 이 필요하므로 교육장 복귀 시 설치 단계를 가이던스에 넣어야 함.
- 단계 범위는 **0~6단계 전체를 계획에 포함**함(2026-09-28 사용자 결정). 실제 착수 순서와 기한 배분은 승인 후 다시 정함.
- 검사 디버그 영상(`Image` 토픽, 토픽 이름은 `debug_image_topic` 파라미터)을 JPEG 로 저장해 웹에 띄울지는 미정임. `cv_bridge` 의존이 늘어나므로 3단계 선택 항목으로 둠.
- `/amcl_pose` 를 도킹 완료 시점 표본으로 쓰는 것은 계획이며 아직 검증 안 함. AMCL 이 실제로 발행 중인지 실행으로 확인해야 함.
- 사이클 타임의 "정상 범위" 는 실측 표본이 없어 정할 수 없음. 백승주의 반복 실험 결과가 나온 뒤 확정함.
- 본 문서의 수치·판정은 **모두 코드와 문서 열람 결과이며 실측이 아님.**

---

## 9. 쉬운 설명 — 이게 무슨 작업인가

### 9.1 한 줄 요약

**공정이 돌아갈 때 무슨 일이 언제 있었는지 적어 두는 "작업 일지"(DB)를 만들고, 그 일지를 보기 좋게 보여주면서 시작 버튼도 달린 "관제실 모니터"(웹)를 만드는 일**임.

### 9.2 지금은 왜 필요한가

- 로봇이 공정을 한 바퀴 돌 때 Task Manager 는 "지금 TRANSFER 중", "지금 도킹 중" 같은 알림을 계속 내보냄. 그런데 그 알림을 **받아서 적어 두는 곳이 없음.** 터미널 화면에 흘러가고 사라짐.
- 그래서 지금은 "방금 그 사이클 몇 초 걸렸지?", "어제 도킹이 몇 번 실패했지?" 같은 질문에 답할 수 없음.
- 팀 아키텍처 문서(`01-architecture.md`)의 미확정 목록에도 **"작업 기록의 저장 위치와 형식"** 이 아직 빈칸으로 남아 있음. 이 작업이 그 빈칸임.

### 9.3 왜 프로그램을 두 개로 나누는가

- ROS 프로그램(`recorder_node`)이 하는 일: **계속 귀를 열고 듣기.** 로봇 쪽 알림이 오면 일지에 적음.
- 웹 프로그램(`web_app`)이 하는 일: **누가 물어보면 답하기.** 브라우저가 "지금 상황 어때?" 하고 물으면 일지를 읽어서 보여줌.
- 이 두 가지는 성격이 반대임. 한 프로그램에 둘 다 시키면 "듣는 중이라 답을 못 하고", "답하는 중이라 못 듣는" 상황이 생김. 이걸 피하려고 흔히 쓰는 기법(스레드, asyncio)은 입문자가 고장 원인을 찾기 아주 어렵고, ADR 에서도 쓰지 말라고 정해 두었음.
- 그래서 **프로그램을 두 개로 나누고, 일지 파일(`farm.db`) 하나를 같이 보게 함.** 한쪽은 적기만, 한쪽은 읽기만 함. 양쪽 코드가 둘 다 단순하게 유지됨.

### 9.4 웹의 "시작" 버튼은 어떻게 동작하는가

웹 프로그램은 ROS 를 전혀 모름. 그래서 **우체통 방식**을 씀.

1. 사람이 웹에서 "시나리오 시작" 을 누름
2. 웹은 일지의 `web_command` 칸에 `"시작해 달라"` 한 줄을 적어 둠 (= 우체통에 편지 넣기)
3. ROS 쪽 `recorder_node` 가 0.5초마다 우체통을 확인함
4. 편지가 있으면 대신 `/start_cycle` 을 호출하고, 결과(수락/거절)를 같은 줄에 적음
5. 웹은 그 줄을 읽어 "수락됨" 또는 "이미 실행 중(BUSY)" 을 화면에 띄움

덕분에 웹에 ROS 를 넣지 않아도 되고, **팀장님 쪽 인터페이스(토픽·메시지)를 새로 만들 필요가 전혀 없음.**

### 9.5 시간을 두 가지로 적는 이유

- Isaac Sim 은 실제 시간보다 느리게 돎(대략 0.3배). 즉 **실제 벽시계로 10초 = 시뮬레이션 안에서는 3초쯤.**
- "이 공정이 느린가?" 를 판단할 때는 시뮬레이션 시간(`/clock`)으로 봐야 맞음. 벽시계로 보면 컴퓨터가 버벅인 것까지 공정 지연으로 잘못 세게 됨.
- 그래서 일지의 모든 줄에 **벽시계 시각과 시뮬레이션 시각을 둘 다** 적음. 나중에 어느 쪽으로든 분석할 수 있음.

### 9.6 이 작업이 팀 세 사람에게 동시에 쓰이는 지점

| 팀원 | 담당 | 이 DB 에서 받아 쓰는 것 |
|---|---|---|
| 봉승현 | ML 기반 이상 탐지 | `step` 표의 "단계별 걸린 시간 / 성공·실패 / 실패 이유", `dock_attempt` 표의 "도킹 오차·재시도 횟수" |
| 백승주 | 공정 사이클 시간 최적화 | 같은 `step` 표. 어느 단계가 가장 오래 걸리는지 바로 나옴 |
| 이원호 | 웹·DB, 영상 | 대시보드 화면 자체가 영상 소재가 됨 |

즉 일지 하나로 세 사람의 입력을 동시에 공급함. **기록을 일찍 시작할수록 세 사람 모두에게 데이터가 쌓임.**

---

## 10. 팀장님께 상의할 내용 (그대로 읽어드릴 수 있게 정리)

아래 4개는 모두 **기존 것을 고치는 게 아니라 덧붙이는 변경**이라, 지금 돌아가는 코드가 깨질 일은 없음.

### 상의 1. 팔레트(트레이) 위치를 밖으로 알려 주실 수 있는지 — **가장 중요**

> "Task Manager 안에 `pallet_locations` 라고, PALLET_001 이 지금 랙에 있는지 운반 중인지를 담아 두는 부분이 이미 있는 것으로 봤습니다. 그런데 이 값이 밖으로 발행되지 않아서 관제 웹에서는 볼 수 없습니다.
>
> `CycleStatus` 메시지에 `string[] pallet_locations` 칸 하나만 늘려서 `"PALLET_001:CARRY"` 같은 형태로 같이 보내 주실 수 있을까요? 기존 칸은 그대로 두는 추가이므로 다른 노드는 영향받지 않습니다."

**왜 필요한지 (팀장님이 물으실 때)**
- 우리 기획서의 검증 지표 중 **"트레이 위치·상태 기록 일치율"** 이 이 값 없이는 측정이 안 됨.
- 우리가 밖에서 추측해 다시 계산할 수도 있지만, 그러면 **똑같은 판정 규칙이 Task Manager 와 관제 두 곳에 생김.** 나중에 팀장님이 규칙을 한쪽만 바꾸면 웹이 틀린 값을 보여주게 됨. 그게 가장 피하고 싶은 상황임.
- 이 값 없으면 어떻게 할지: 명령에 들어 있는 `destination` 칸만으로 대충 기록함. 비어 있는 구간이 있어 정확도가 떨어짐.

### 상의 2. 이상 탐지(팀장님 담당)의 입력을 이 DB 로 하실지

> "회의록에 ML 이상 탐지의 입력 데이터가 미확정으로 남아 있어서 여쭙니다.
>
> 제가 만들 일지에는 단계별로 **몇 초 걸렸는지 / 성공인지 실패인지 / 실패면 그 이유(`reason`)** 가 다 남고, 도킹은 **몇 m 틀어졌는지와 몇 번 다시 시도했는지** 까지 남습니다. 이상 탐지 입력으로 이걸 쓰실 수 있을지, 아니면 더 필요한 항목이 있을지 알려 주시면 표에 칸을 미리 넣어 두겠습니다."

**같이 부탁할 것 — 정상 데이터 모으기**

> "이상 탐지는 '정상일 때가 어떤 모습인지' 를 많이 봐야 학습이 됩니다. 백승주님이 사이클 시간 측정하려고 공정을 반복 실행하실 예정인데, 그때 제 기록 프로그램을 터미널 하나에 같이 띄워 두면 정상 표본이 자동으로 쌓입니다. 그렇게 진행해도 괜찮을까요?"

이게 되면 **회의록의 "ML 이상 탐지 입력 데이터 미확정" 항목이 정리되고, 동시에 백승주님 실험 한 번이 팀장님 학습 데이터까지 만들어 줌.**

### 상의 3. 단계 제한시간(timeout)의 시간 기준 확인 — 확인만, 고칠 필요 없음

> "Task Manager 가 단계 제한시간을 셀 때 컴퓨터 실제 시계를 쓰고 계신 것 같습니다(`use_sim_time: false`, `time.monotonic()`).
>
> Isaac 이 실제보다 3배쯤 느리게 도니까, '60초 제한' 이 시뮬레이션 안에서는 20초 정도에 해당합니다. 의도하신 것이면 그대로 두면 되고, 확인만 부탁드립니다. 관제 쪽은 제가 두 시간을 다 기록하므로 팀장님 코드를 고칠 일은 없습니다."

### 상의 4. 기록 프로그램을 어느 launch 로 띄울지

> "기록 프로그램을 팀장님 `task_manager.launch.py` 안에 넣으면 시연할 때 자동으로 항상 기록됩니다. 다만 팀장님 파일을 고쳐야 합니다.
>
> 아니면 제가 `monitor.launch.py` 를 따로 만들어서 터미널 하나를 더 띄우는 방법도 있습니다. 저는 후자가 팀장님 쪽 변경이 없어 낫다고 보는데, 시연 터미널이 하나 늘어나는 게 부담이면 전자로 하겠습니다. 어느 쪽이 편하실까요?"

### 상의 5. (선택) 발표 자료 연계

> "관제 웹·DB 가 챌린지 5번(ML 이상 탐지)의 전제 조건이 되니, PPT 에 '공정 데이터 기록 계층' 을 한 장 넣는 것이 흐름상 자연스러울 것 같습니다. 김민서님 레이아웃에 반영할지 판단 부탁드립니다."

### 팀장님께 굳이 물을 필요 없는 것 (내가 알아서 처리함)

- `feeder_dock.py` 결과에 재시도 횟수(`retry`) 칸 추가 — **우리 소유 파일**이라 팀장님과 상의할 필요 없음. 사용자 승인만 받으면 됨.
- 사이클 소요시간을 두 가지 시간으로 기록하는 것 — 관제 쪽에서만 처리함.
- 웹 화면 구성, 프레임워크 선택 — 관제 담당 재량.

---

## 11. 고도화 기능 목록과 우선순위 (2026-09-28 작성)

> 판단 기준: **발표·영상에서 보이는 효과 ÷ 구현 비용**, 그리고 **팀원 3인의 담당 업무에 직접 연결되는지**. 유행하는 프레임워크를 도입하는 것보다, 관제 화면이 갖춰야 할 다섯 가지(서버 푸시 / 도면 위 오버레이 / 이벤트 타임라인 / 리플레이 / 데이터 내보내기)가 있는지가 화면의 수준을 결정함.

### P0 — 이게 없으면 나머지가 성립하지 않음

| # | 기능 | 내용 | 비용 |
|---|---|---|---|
| 1 | 기록 파이프라인 | `recorder_node` → SQLite. 모든 화면·ML·사이클 분석의 전제 | 중 |
| 2 | 서버 푸시(SSE) | 브라우저 폴링 대신 `text/event-stream` 으로 변경 즉시 밀어줌. `live` 테이블 1행을 UPDATE 로 유지해 고빈도 값(로봇 위치)을 이력 비대 없이 전달 | 낮음 |

### P1 — 발표·영상 효과가 크고 비용이 낮음

| # | 기능 | 내용 | 근거·연결 |
|---|---|---|---|
| 3 | **2D 지도 위 로봇 실시간 위치·경로** | `maps/Collected_smartfarm_v014.png`(285×460 px)을 배경으로 깔고, `map` 좌표계 x/y 를 `resolution 0.05`·`origin [-4.525, -10.025]` 로 픽셀 변환해 로봇 점·`/plan` 경로선·작업점 표시 | 심사자가 RViz 없이 볼 수 있는 유일한 주행 화면. 영상 소재 직결 |
| 4 | 공정 타임라인(간트) | 사이클의 12단계를 가로 막대로. 병목이 눈에 보임 | 백승주 사이클 시간 최적화 직결. "몇 % 개선" 수치 근거 |
| 5 | 검사 결과 오버레이 | 슬롯 6칸 그리드 + bbox·class·confidence. RECHECK 전후 비교 | CULL 챌린지 시각화 |
| 6 | KPI 카드 | 사이클 성공률, 평균 사이클 시간(sim), 도킹 성공률·평균 재시도, 불량 검출 수 | 기획서 검증 지표와 1:1 |
| 7 | 관제실 레이아웃 | 다크 테마, 1080p 고정 그리드 | CSS 뿐이라 비용 최저. 화면이 제품처럼 보임 |

### P2 — 데이터가 쌓인 뒤

| # | 기능 | 내용 | 근거·연결 |
|---|---|---|---|
| 8 | 사이클 A/B 비교 | 두 사이클의 단계별 시간을 나란히 | 최적화 전·후 대조 |
| 9 | 리플레이(타임라인 스크럽) | 기록을 되감아 지도·상태를 재생 | 지도 뷰 재사용. 발표 데모로 강력 |
| 10 | 이상 탐지 배너 + 사람 라벨링 | 팀장 ML 결과를 배너로. 사람이 "이건 이상" 클릭하면 학습 라벨 축적 | 봉승현 담당 직결, 챌린지 5번 |
| 11 | 영상 타임코드 내보내기 | 단계별 시작 시각을 벽시계로 CSV 출력. 녹화 시작 시각만 넣으면 컷 지점 목록이 나옴 | **영상 편집 담당 업무 시간 단축.** 비용 아주 낮음 |
| 12 | CSV·Parquet 내보내기 | `pandas.read_sql` 없이도 바로 받게 | 팀장·백승주 인계 |
| 13 | 라이브 카메라 | `/rgb` MJPEG 스트림 | 대역폭 부담 큼. 검사 순간 스냅샷으로 대체 가능 |

### P3 — 트렌드지만 이 프로젝트에는 실익이 낮음

| # | 기능 | 왜 뒤로 미루는지 |
|---|---|---|
| 14 | `rosbridge_suite` + `roslibjs` (웹이 토픽 직접 구독) | 표준적인 방법이지만 SSE + `live` 테이블로 같은 결과를 얻음. 도입하면 의존·보안 설정이 늘고 **이력이 남지 않아** ML·사이클 분석에 쓸 수 없음. 지도 갱신이 체감상 답답할 때만 도입 |
| 15 | three.js 3D 디지털 트윈 뷰 | Isaac Sim 화면이 이미 그 역할을 함. 비용 대비 중복 |
| 16 | WebRTC 저지연 영상 | 녹화 영상으로 충분 |
| 17 | 사용자 인증·권한, PWA, 모바일 반응형 | 단일 시연 환경이라 불필요 |
| 18 | Grafana / Prometheus 도입 | 우리 데이터는 시계열 메트릭이 아니라 공정 이벤트라 궁합이 나쁨. 설치 부담이 크고, 직접 만드는 쪽이 발표에도 유리 |

### 착수 순서 (내 판단)

**2026-09-28 사용자 결정: P0 전체 + P1 전체 + P2 10번만 구현. P2 의 나머지와 P3 는 구현하지 않음.**

착수 순서: **P0 1 → P0 2 → P1 3(지도) → P1 6·7 → P1 4 → P1 5 → P2 10.**
P1 3번을 먼저 두는 이유는 지도 뷰가 그 자체로 영상 소재가 되기 때문임. P2 10번(이상 탐지 배너·라벨링)은 봉승현 팀장님 ML 결과 토픽이 정해진 뒤에야 배선이 확정되므로 마지막임. 그때까지는 **사람 라벨링 UI 부분만 먼저 만들어 정상·이상 라벨을 모아 둠**(팀장님 학습 입력).

구현하지 않기로 한 것: P2 8·9·11·12·13, P3 14~18. 다만 **P2 12(CSV 내보내기)는 팀장님·백승주님 인계 수단이므로, 필요해지면 `/export/steps.csv` 한 줄로 추가 가능하도록 `store.py` 에 조회 함수를 남겨 둠.**

---

## 12. Task Manager 의 시간 기준 — 위치와 변경 방법 (팀장님 요청, 2026-09-28)

### 12.1 지금 시스템에서 시간 기준이 갈려 있음

- **Nav2 전체와 `navigation_node` 는 `use_sim_time: True`** 임(`launch/nav2.launch.py` 여러 줄, `launch/navigation_node.launch.py:27`).
- **Task Manager 만 `use_sim_time: false`** 임(`smart_farm_manager/config/demo_harvest.yaml`).
- 즉 주행 쪽은 시뮬레이션 시각으로, 공정 관리 쪽은 컴퓨터 시계로 판단하고 있음.

### 12.2 컴퓨터 실제 시계를 쓰는 곳 (파일·줄·용도)

파일: `cobot3_ws/src/smart_farm_manager/smart_farm_manager/task_manager_node.py` (브랜치 `origin/feature/task-managed-integration`, `7beec39` 기준)

| 줄 | 코드 | 용도 | 시각 기준을 바꿔야 하나 |
|---|---|---|---|
| 178~185 | `Clock(clock_type=ClockType.STEADY_TIME)` + `create_timer(..., clock=self.steady_clock)` | 0.1 s 주기 tick 타이머 | **아니오. 그대로 둠** |
| 230 | `self.preflight_started_at = time.monotonic()` | PREFLIGHT 시작 시각 | **아니오. 그대로 둠** |
| 271 | `received_at=time.monotonic()` | executor heartbeat 수신 시각 | **아니오. 그대로 둠** |
| 451 | `now = time.monotonic()` (`_tick_preflight`) | PREFLIGHT 경과, heartbeat age | **아니오. 그대로 둠** |
| 592 | `self.command_deadline = time.monotonic() + step.timeout_sec` | **공정 제한시간 설정** | **예** |
| 613 | `if time.monotonic() < self.command_deadline` | **공정 제한시간 판정** | **예** |
| 375 | `self.cull_data_deadline = time.monotonic() + cull_data_timeout_sec` | 검사 데이터 저장 대기 | **예** |
| 438 | `if ... time.monotonic() >= self.cull_data_deadline` | 위 판정 | **예** |

### 12.3 왜 일부만 바꾸는가 — 중요한 구분

- **바꿀 것(592·613·375·438)**: 이 네 곳은 **Isaac 안에서 로봇이 실제로 움직이는 시간**을 재고 있음. 시뮬레이션 배율이 기기마다 다르면(고피1·고피2·고피3의 GPU 성능 차이) 같은 파라미터가 서로 다른 여유를 주게 되어 **timeout 판정이 실행마다 달라짐.** 재현성이 없으면 반복 실험(백승주)과 이상 탐지(봉승현)의 입력에 기기 성능이 노이즈로 섞임.
- **바꾸지 말 것(178·230·271·451)**: 이 네 곳은 **"상대 노드가 살아 있는가"** 를 판정함. 이건 시뮬레이션 밖의 사실임. 시뮬레이션 시각으로 바꾸면, **Isaac 이 죽어 `/clock` 이 끊긴 순간 시각이 멈춰서 Task Manager 가 영원히 기다림.** tick 타이머까지 `/clock` 에 걸면 Isaac 이 뜨기 전에는 노드가 아무 일도 못 하고, `task_manager_fake_test.launch.py`(Isaac 없이 mock_executor 만 쓰는 시험)가 아예 돌지 않음.

### 12.4 구체적 변경 내용 (5군데)

**(1) 헬퍼 한 개 추가** — `task_manager_node.py`

```python
    def _process_clock_sec(self) -> float:
        """공정 제한시간 판정 기준 시각(초). use_sim_time 이 true 면 /clock 시각이다."""
        return self.get_clock().now().nanoseconds * 1e-9
```

**(2) 네 곳의 `time.monotonic()` 을 교체**

```python
# :592
self.command_deadline = self._process_clock_sec() + step.timeout_sec
# :613
if self._process_clock_sec() < self.command_deadline:
# :375
self.cull_data_deadline = (
    self._process_clock_sec()
    + float(self.get_parameter("cull_data_timeout_sec").value)
)
# :438
if self.cull_data_deadline is not None and self._process_clock_sec() >= self.cull_data_deadline:
```

**(3) `/clock` 미수신 가드 두 줄** — `_dispatch_current_step()` 앞부분

`use_sim_time: true` 인데 `/clock` 이 아직 안 오면 시각이 0 이라 제한시간이 걸리지 않음.

```python
        # /clock 이 오기 전에는 제한시간을 걸 수 없으므로 명령을 보내지 않는다.
        if self._process_clock_sec() <= 0.0:
            return
```

**(4) 런치에서만 `use_sim_time` 을 덮음.** `demo_harvest.yaml` 을 세 런치가 공유하므로 yaml 을 고치면 `fake_test` 가 깨짐.

```python
# task_manager.launch.py, task_manager_navigation_test.launch.py
parameters=[parameter_file, {"use_sim_time": True}],
# task_manager_fake_test.launch.py 는 그대로 둠 (Isaac 없음)
```

**(5) 장면 Stop → Play 되감김 가드 한 줄.** `/clock` 이 0 으로 돌아가면 기존 deadline 이 먼 미래가 되어 timeout 이 안 걸림. 팀이 이미 장면 리셋 시 대기 명령을 폐기하고 있으므로(`fix: clear pending command and inspection data during scene reset`), 그 경로에서 `self.command_deadline = None`·`self.cull_data_deadline = None` 만 함께 해 주면 됨.

### 12.5 바꾸는 순서 — 지금 바로 바꾸면 안 되는 이유

현재 `timeout_sec` 값(TRANSFER 400 s, CULL 900 s 등, `scenario.py`)은 **벽시계 기준으로 맞춰 둔 값**임. 시뮬레이션 시각으로 바꾸면 배율 0.3 에서 **실질 여유가 약 3배 늘어남.** 즉 지금 잡히던 실패도 안 잡히게 됨.

따라서 순서는 다음이 맞음.

1. 관제 DB 가 `step.duration_sim`(단계별 시뮬레이션 소요시간)을 실측으로 모음
2. 그 값으로 `timeout_sec` 을 시뮬레이션 기준으로 다시 잡음
3. 위 (1)~(5) 변경을 함께 반영

**즉 이 변경은 관제 DB 기록이 먼저 돌아야 근거를 가짐.**

---

## 13. 팀장님 2차 답변 반영 (2026-09-28)

### 13.1 받은 답변 요지

1. `feature/task-managed-integration` 이 통합 브랜치이며 `development` 로 **PR 을 이미 보냈음.** 내일 GPU 노트북 실환경 검증 후 merge 예정.
2. Task Manager 는 **고정 시나리오를 내부에 저장해 FSM 으로 단계 실행**하는 구조라 시나리오가 외부로 발행되지 않음. "지금 구조로는 관제 웹에서 확인이 안 될 것" 이라고 보심.
3. `_start_cycle_callback` 과 인터페이스를 교체하면 발행은 가능하나, **현재 service response 는 "명령이 잘 전달되었는지" 만 즉시 반환하는 용도**임. 어떤 response 가 필요한지 구체적으로 알려 달라고 하심.
4. 시간 기준이 두 개가 된 것은 **의도가 아님.** 설계 당시 USD 에서 `/clock` 발행이 안 되던 상태라 자체 시각을 썼고, 이후 구조 변경 없이 기능만 붙였음. 지금은 **timeout 을 넉넉하게 주는 임시방편**으로 두었으며, sim 내부·외부 노드에 따라 시간 기준을 나누는 것이 구조적으로 더 안정적이라는 데 동의하심.
5. 관제 기록 노드는 **별도 패키지로 두는 편이 안전**하다고 보시며, 선택은 이쪽에 맡기심.

### 13.2 2번·3번에 대한 답 — **팀장님 코드 변경 없이 관제 웹은 동작함**

이건 내가 질문을 팔레트 위치에 한정해 적어 오해를 산 부분임. 사실 관계는 다음과 같음.

관제가 필요한 것은 전부 **이미 발행되고 있음.**

| 관제가 얻는 것 | 어디서 |
|---|---|
| 현재 공정 단계, 사이클 최종 상태, 실패 사유 | `/cycle/status` (`state`, `status`, `reason`, `active_command_id`) |
| 단계별 operation·팔레트·대상 슬롯 | `/sim_task/command`(String JSON), `/navigation/command`, `/inspection/command` (TaskCommand) |
| 단계별 성공·실패·phase·사유 | `/sim_task/result`, `/navigation/result`, `/inspection/result` |
| 단계별 소요시간 | 위 command 발행 시각과 result 수신 시각의 차 (관제가 직접 잼) |
| executor 준비·진행 상태 | `/sim_task/status`, `/navigation/status`, `/inspection/status` |
| 검사 결과 슬롯·클래스·bbox | `/inspection/detections_2d` |
| 검사 데이터 저장 성공·거절 | `/sim_task/inspection_data_status` |
| 도킹 품질 | `/feeder_dock/result` (navigation 쪽 우리 모듈) |

발행되지 않는 것은 **`CycleStateMachine.pallet_locations`(팔레트 논리 위치) 하나뿐**이고, 그것도 없으면 정확도만 떨어지고 관제 자체는 돌아감.

**따라서 `/start_cycle` 의 response 는 지금 그대로가 정확히 필요한 형태임.** 웹의 시작 버튼은 "수락되었는지" 만 즉시 알면 되고(`accepted`, `task_id`, `reason=BUSY`), 전체 결과는 `/cycle/status` 로 봄. 이 분담은 이미 `docs/02-interfaces.md` §4 에 그대로 적혀 있음. **인터페이스 교체는 필요 없음.**

시나리오 정의(12단계 순서) 자체도 외부 발행이 안 되지만, 관제는 `CycleState` 순서를 화면에 그려 두면 되므로 문제 없음. 다만 **시나리오가 여러 개가 되거나 단계가 바뀌면 관제 화면도 같이 고쳐야 함.** 그때가 오면 `/cycle/scenario`(String JSON, latched)로 단계 목록을 한 번만 발행하는 방안을 제안함. 지금은 불필요함.

### 13.3 4번 — 시간 기준: 순서만 맞추면 됨

`/clock` 이 발행되지 않던 시절의 잔재라는 설명으로 §12 의 진단이 확인됨. 지금 `timeout_sec` 이 넉넉한 값(TRANSFER 400 s, CULL 900 s)인 것도 그 임시방편의 결과임.

바꾸는 순서는 §12.5 그대로임. **관제 DB 가 `step.duration_sim` 을 모으는 것이 선행 조건**이며, 그 값 없이 지금 시간 기준만 바꾸면 여유가 약 3배 늘어 실패를 더 못 잡게 됨.

### 13.4 PR 본문에서 확인한 미해결 항목이 DB 스키마에 미치는 영향

PR #12 본문의 리뷰 요청 사항 중 다음이 관제에 직접 걸림.

> "Task Manager 의 시간 초과가 진행 중인 Isaac 동작을 직접 취소하지 않으므로, 시간 초과 후 재시작 절차 확인"

즉 Task Manager 가 TIMEOUT 으로 실패 처리한 뒤에도 **Isaac 안에서는 그 동작이 계속 돌고 있고, 늦게 도착한 result 가 토픽에 실제로 올라옴.** Task Manager 는 이를 `NO_ACTIVE_COMMAND`·`MISMATCHED_RESULT` 로 버리지만 **관제는 토픽을 구독하므로 그 늦은 결과를 받게 됨.**

대응: `step` 표에 **`late_result` 플래그**를 둠(§3.3 반영 완료). 이러면 다음이 가능해짐.

- 화면에서 "제한시간 초과로 실패 처리됐지만 실제로는 N초 뒤 성공했음" 을 구분해 표시함
- **`timeout_sec` 값이 너무 짧다는 직접 증거**가 됨 → §12.5 의 재설정 근거가 데이터로 남음
- 이상 탐지(봉승현) 라벨에서 "진짜 실패" 와 "제한시간만 짧았던 것" 을 분리할 수 있음

---

## 14. PR #12 분석 — 우리 워크스페이스와 관제 계획에 미치는 영향

확인 방법: GitHub API 로 PR 메타데이터 조회 + `git diff` 로 브랜치 대조. (`gh` CLI 는 이 기기에 없음.)

### 14.1 PR 개요

| 항목 | 값 |
|---|---|
| 번호·제목 | #12 `feat: Task Manager 기반 스마트팜 전체 작업 흐름 통합` |
| 상태 | **open**, `mergeable: true`, `mergeable_state: clean` |
| base ← head | `development` ← `feature/task-managed-integration` (`7beec39`) |
| 규모 | 커밋 309개, 변경 파일 176개, +16,616 / −1,147 |
| 생성 | 2026-09-28 02:06 UTC |
| 본문의 검증 문구 | "Ubuntu 실환경에서 전체 통합 흐름 실기동 확인" |

**PR 본문의 검증 문구에 대한 정리 (2026-09-28 사용자 확인)**: 팀장님도 VM 인스턴스에서 작업 중이고 **교육장 고피1·고피2 와 사양을 일부러 비슷하게 맞춘 환경**이므로, 통합 자체는 완료로 간주하신 것임. 따라서 PR 본문의 "실환경 확인" 과 "내일 검증 후 merge" 는 서로 어긋난 진술이 아니라 같은 상태를 뭉뚱그린 것으로 봄. **이 건에 무게를 두지 않음.**

남는 위험은 하나뿐임: 교육장 복귀 후에는 **내피와 원격 통신하는 구성(도메인·`FASTRTPS_DEFAULT_PROFILES_FILE` 화이트리스트·다중 기기)으로 돌아가므로**, VM 한 대에서 확인된 것이 그 구성에서 그대로 재현되는지는 아직 모름. 관제 기록 노드는 이 위험의 영향을 받음(토픽을 여러 기기에서 받아야 함). 복귀 첫 실행 때 `/cycle/status` 수신 여부를 먼저 확인함.

브랜치 `7beec39` 는 2026-09-28 01:50 이후 갱신이 없음. 즉 **내가 앞서 분석한 내용이 곧 최종본임.**

### 14.2 우리 navigation 패키지 — 영향 없음

`origin/feature/lwh` 와 `origin/feature/task-managed-integration` 의 `smart_farm_navigation` 핵심 파일 blob 해시가 **전부 동일**함.

| 파일 | 대조 결과 |
|---|---|
| `smart_farm_navigation/feeder_dock.py` | 같음 |
| `smart_farm_navigation/navigation_node.py` | 같음 |
| `smart_farm_navigation/geometry.py` | 같음 |
| `launch/nav2.launch.py` | 같음 |
| `launch/navigation_node.launch.py` | 같음 |
| `config/nav2_params.yaml` | 같음 |
| `config/stations.yaml` | 같음 |

두 브랜치 차이는 **277개 파일이 전부 "우리 브랜치에만 있고 팀 브랜치에 없음"** 이며 내용은 다음뿐임.

- `results/log/*` 실측 기록 (팀이 가져가지 않은 것이 정상)
- 옛 스크립트·설정: `nav_to_pose.py`, `scripts/odom_to_tf.py`, `scripts/inject_action_graph.py`, `scripts/run_all.sh`, `scripts/ros2_command_수정*차.txt`, `rviz2/mir100_navigation.rviz`

즉 **팀 쪽에서 우리 코드를 고친 곳은 없음.** merge 후 `development` 를 우리 브랜치로 반입할 때 navigation 쪽 충돌 위험은 낮음. 다만 ADR_nav2 §2.4 점검표는 그대로 수행함.

### 14.3 관제 계획에 반영해야 할 차이 — **여기가 중요**

우리 브랜치가 들고 있는 `smart_farm_manager` 는 2026-09-26 판이고 **팀 최신판과 다름.** 관제 기록 노드는 **팀 최신판 기준으로 작성해야 함.**

| 항목 | 우리 브랜치(구판) | 팀 최신판 |
|---|---|---|
| 공정 단계 수 (`scenario.py` 의 `StepDefinition`) | **7개** | **12개** |
| Task Manager 가 다루는 토픽 | 11개 | **13개** (`/sim_task/inspection_context`, `/sim_task/inspection_data_status` 추가) |
| `smart_farm_interfaces` msg·srv | — | **완전히 동일** |

결론 3가지.

1. **`smart_farm_interfaces` 가 동일하므로 관제 기록 노드의 메시지 계약은 안전함.** 팀 merge 를 기다리지 않고 작성 가능함.
2. **대시보드 체크리스트는 12단계로 그림**(TRANSFER, PICK_HARVEST, NAVIGATION, PLACE_INSPECT, CONVEY_TO_INSPECT, PREPARE_INSPECT, MOVE_TO_INSPECT, INSPECT, CULL, RECHECK, RELEASE_INSPECT, CONVEYOR_OUT). 앞서 15단계로 적었던 것은 `CycleState` enum 값 수(16개, IDLE·PREFLIGHT·COMPLETE·ERROR 포함)와 공정 단계 수를 섞은 것이었음. 문서에서 정정했음.
3. **`/sim_task/inspection_data_status` 를 기록 대상에 추가함.** 값이 `STORED`/`REJECTED` 이고, Task Manager 의 `INSPECTION_DATA_REJECTED`·`INSPECTION_DATA_TIMEOUT` 실패와 직결되므로 이상 탐지 입력으로 가치가 있음. `executor_status` 표에 `executor='sim_task_data'` 로 넣음.

### 14.4 장면·지도 — 관제 지도 뷰의 데이터 출처는 유지됨

- PR 이 지우는 `scenes/` 항목은 `rack_pick_test` 하위뿐임. `scenes/` 최상위 목록은 두 브랜치가 동일함.
- **`maps/Collected_smartfarm_v014.png`·`.yaml` 은 팀 브랜치에 그대로 있음.** 즉 P1-3 지도 뷰의 배경 이미지와 좌표 변환 파라미터(`resolution 0.05`, `origin [-4.525, -10.025]`, 285×460 px)가 유지됨.
- v014 USD 본체는 양쪽 모두 git 에 없음(ADR_nav2 2.4 의 대용량 USD 제외 규칙대로 로컬 파일임).

### 14.5 merge 후 해야 할 일 (순서)

1. 팀장님이 `development` 로 merge 완료했음을 확인함
2. `development` 를 `feature/lwh` 로 반입함. `git pull` 은 사용자 확인 후 진행함
3. ADR_nav2 §2.4 점검표로 navigation 추가분이 살아 있는지 확인함
4. `smart_farm_manager` 12단계·13토픽 기준으로 기록 노드를 맞춤(이미 그 기준으로 작성하므로 확인만)
5. `colcon build` 후 `test_store.py` 회귀 실행

---

## 15. 승인 반영 (2026-09-28)

| 항목 | 결과 |
|---|---|
| 새 패키지 `cobot3_ws/src/smart_farm_monitor/` 생성 | **승인됨.** 팀장님도 별도 패키지를 권함 |
| `.gitignore` 의 `/cobot3_ws/src/smart_farm_monitor/*` | `…/smart_farm_monitor/data/*` 로 좁혀 코드는 추적, DB 파일만 제외 |
| 웹 프레임워크 | FastAPI + uvicorn |
| 구현 범위 | P0 전체 + P1 전체 + P2 10번 |
| Isaac Sim 실행 | 승인됨. **단, 아래 기록 의무를 지킴** |

### 15.1 Isaac 실행 시 미디어 기록 의무 (사용자 지시)

짧은 확인용 실행이라도 **미디어 자료를 반드시 남김.** 최소 스냅샷.

- 저장 경로: `cobot3_ws/src/smart_farm_navigation/results/log_media/<수행한것>_<YYYYMMDD>_<HHMM>/`
  - 예: `results/log_media/monitor_recorder_check_20260929_1430/`
- 이 경로는 `.gitignore` 로 git 제외임(134 MB 미디어 사고 방지). **파일 목록과 무엇을 찍었는지는 답변과 가이던스에 표로 남김.**

### 15.2 ADR_monitor.md 작성 절차 (사용자 지시)

새 패키지 생성은 새 ADR 을 필요로 함. 절차는 다음으로 정함.

1. 1차 코드 착수를 수행함
2. 착수 중 겪은 것(막힌 지점, 구조 결정, 예상과 달랐던 점)을 **진단 내용으로 보고**함
3. 사용자가 방향을 디렉팅함
4. 그 방향을 `docs/ADR/ADR_monitor.md` 로 작성함

**즉 ADR_monitor.md 를 미리 만들지 않음.** 그때까지 이 문서(`04-monitoring-web-db.md`)가 관제의 단일 출처임.

---

## 16. `feature/monitor` 브랜치 구성 — 무엇을 가져오고 무엇을 지켜야 하는가 (2026-09-28)

### 16.1 먼저 확인한 위험 — 팀 정리 커밋 `19ca11d` 가 우리 문서 자산 329개를 지웠음

`git merge-tree` 로 시험 병합해 본 결과, 팀 브랜치를 그냥 병합하면 **우리 문서 자산이 조용히 사라짐.**

커밋 `19ca11d` (2026-09-26, shbong) `chore: 통합 브랜치의 과거 실험 파일과 중복 문서 정리` 가 **329개 파일을 삭제**했고, 그 안에 다음이 들어 있음.

| 사라지는 것 | 개수 | 성격 |
|---|---|---|
| `docs/ADR/ADR_basic.md`, `ADR_navigation2.md` | 2 | 에이전트 상시 규칙의 원본 |
| `docs/prompt/*.txt` | 8 | 사용자 지시 기록 |
| `docs/reference/**` (강의자료·노션 정리) | 약 110 | 우리 정리물 |
| `cobot3_ws/src/smart_farm_navigation/guidance/past/**` | 다수 | 지난 차수 가이던스 |
| `cobot3_ws/src/smart_farm_navigation/results/**` | 144+ | **실측 기록** (ADR_basic §5-5 의 실측 판정 근거) |
| `config/past/`, `smart_farm_navigation/past/` 일부 | 다수 | 미사용 보관분 |

팀 브랜치에 남은 것은 `guidance/guidance2_26차.md`(현행 1개)와 팀 자신의 `docs/*.md` 8개뿐임. `results/` 는 통째로 없음.

**병합 충돌로 드러나는 것은 142개뿐임**(`results/log/*` 141개 rename/delete + `docs/ADR/ADR_basic.md` 1개). 나머지 삭제는 **우리가 base 이후 손대지 않은 파일이라 충돌 없이 조용히 지워짐.** 이것이 ADR_nav2 §2.4("병합 뒤 내 추가분이 사라졌는지 확인") 가 있는 이유임.

반대로 **코드는 충돌이 0임.** `smart_farm_navigation`, `smart_farm_manager`, `smart_farm_interfaces`, `isaacpjt/smart_farm` 전부 무충돌로 팀 최신판이 들어옴. 우리 브랜치가 base 이후 코드를 건드리지 않았기 때문임(문서만 커밋했음).

### 16.2 결론 — `feature/lwh` 에서 파고, 팀 브랜치를 병합한 뒤 우리 문서를 되살림

질문이 "`feature/lwh` 와 `feature/task-managed-integration` 둘 다 pull 해야 하나" 였음. 답은 **둘 다 필요하고, 순서와 충돌 처리 규칙이 있음.**

```
# 1) 우리 브랜치에서 새 브랜치를 팜 (우리 문서·실측 기록을 기준으로 삼기 위해)
git checkout -b feature/monitor feature/lwh

# 2) 팀 통합 최종본을 병합함 (코드는 무충돌로 최신이 됨)
git merge origin/feature/task-managed-integration

# 3) 충돌 142개를 규칙으로 해소함
#    - results/log/* 141개 → 우리 것 유지 (실측 기록)
#    - docs/ADR/ADR_basic.md → 우리 것 유지 (2026-09-28 갱신분 포함)

# 4) 조용히 지워진 우리 문서 자산을 되살림
git checkout feature/lwh -- docs/ADR docs/prompt docs/reference \
    cobot3_ws/src/smart_farm_navigation/guidance \
    cobot3_ws/src/smart_farm_navigation/results

# 5) 병합 커밋을 마무리하고 푸시함
```

**왜 이 방향인가**

- **`feature/lwh` 를 바탕으로 해야** 우리 문서·실측 기록이 기본값이 되고, 되살릴 목록이 짧아짐. 반대로 팀 브랜치를 바탕으로 하면 329개를 하나씩 확인해야 함
- **병합(merge)으로 해야** 계보가 남아서, 나중에 `development` 를 다시 가져올 때 같은 변경이 두 번 충돌하지 않음. 파일만 복사해 오면 계보가 끊김
- 관제 개발에 실제로 필요한 팀 코드는 `smart_farm_manager`(12단계·13토픽), `smart_farm_interfaces`, `runtime/sim_task_node.py`, `smart_farm_vision` 이고 **전부 이 병합으로 들어옴**

### 16.3 병합 뒤 점검표

1. `docs/ADR/ADR_basic.md`·`ADR_navigation2.md` 존재 확인
2. `cobot3_ws/src/smart_farm_navigation/results/log/` 파일 수가 병합 전과 같은지 확인
3. `smart_farm_navigation` 핵심 7파일(§14.2)이 팀 브랜치와 같은 blob 인지 확인
4. `smart_farm_manager/smart_farm_manager/scenario.py` 의 `StepDefinition` 이 **12개**인지 확인
5. `.gitignore` 에서 `/cobot3_ws/src/smart_farm_monitor/*` 를 `…/data/*` 로 좁혔는지 확인
6. `colcon build --packages-select smart_farm_interfaces smart_farm_manager smart_farm_navigation` 통과 확인

### 16.4 팀장님께 알려야 할 것

PR #12 가 `development` 로 merge 되면 **`development` 에서도 위 문서 자산이 사라짐.** 우리 브랜치에는 남으므로 작업에 지장은 없으나, 공유 저장소 기준에서는 실측 기록과 지난 가이던스가 없어짐. 통보 문안은 답변에 포함함.

### 16.5 실행 결과 (2026-09-28 수행 완료)

브랜치 `feature/monitor` 를 만들어 위 순서대로 수행했음. 커밋 `29b32a4`, `origin/feature/monitor` 로 푸시 완료.

| 항목 | 결과 |
|---|---|
| 충돌 | **142건** (`results/log/*` 141 rename/delete + `docs/ADR/ADR_basic.md` 1 modify/delete) → 전부 우리 것 유지로 해소 |
| 조용히 지워질 뻔한 것 | **174건 복원**: `docs/ADR`, `docs/prompt`, `docs/reference`(강의자료 103), `guidance/`(past 27), `results/`, `errored/`(9), `smart_farm_navigation/docs/`(9), `config/past`, `launch/past` |
| 삭제를 그대로 둔 것 | 저장소 루트의 `frames_2026-09-18_20.02.43.{gv,pdf}` (tf 트리 덤프, 루트 어지럽힘) |
| 코드 충돌 | **0건** |
| `docs/ADR/` | `ADR_basic.md`, `ADR_navigation2.md` 둘 다 존재 |
| `results/log/` | **144개** (병합 전과 같음) |
| `scenario.py` 의 `StepDefinition` | **12개** |
| `task_manager_node.py` 토픽 | **13개** |
| navigation 핵심 7파일 | 팀 브랜치와 blob 동일 |
| `.gitignore` | `/cobot3_ws/src/smart_farm_monitor/*` → `…/data/*` 로 축소 |
| `colcon build` (`smart_farm_interfaces`, `smart_farm_manager`, `smart_farm_navigation`) | **3 packages finished, 6.31 s, 실패 0** |
| 팀 단위시험 (`test_state_machine.py`, `test_json_protocol.py`) | **34 passed** |

즉 **관제 개발 기준선이 팀 통합 최종본과 동일하고, 우리 문서·실측 기록은 하나도 잃지 않았음.**
