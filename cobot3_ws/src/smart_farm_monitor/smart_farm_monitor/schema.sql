-- 스마트팜 관제 DB 스키마.
-- 시각은 두 가지를 함께 적는다.
--   *_wall : 컴퓨터 시계(epoch 초). 행의 순서와 실제 대기시간을 본다.
--   *_sim  : /clock 시각(초). 공정 소요시간 분석은 이 값으로만 한다.
-- Isaac 실시간 배율이 0.3 전후라 두 값은 서로 다르다.

-- 1. 사이클 1회 = 1행
CREATE TABLE IF NOT EXISTS cycle (
  task_id         TEXT PRIMARY KEY,
  scenario_id     TEXT NOT NULL,
  started_wall    REAL NOT NULL,
  started_sim     REAL,
  ended_wall      REAL,
  ended_sim       REAL,
  final_state     TEXT,            -- COMPLETE / ERROR / NULL(진행 중)
  terminal_status TEXT,            -- RUNNING / SUCCEEDED / FAILED / IDLE
  failure_reason  TEXT
);

-- 2. 공정 명령 1건 = 1행. 사이클 타임 분석과 이상 탐지의 주 입력
CREATE TABLE IF NOT EXISTS step (
  id            INTEGER PRIMARY KEY AUTOINCREMENT,
  task_id       TEXT NOT NULL,
  command_id    TEXT NOT NULL,
  seq           INTEGER,
  cycle_state   TEXT,              -- 명령 발행 시점의 CycleState
  executor      TEXT,              -- sim_task / navigation / inspection
  operation     TEXT,
  recipe_id     TEXT,
  pallet_id     TEXT,
  source        TEXT,
  destination   TEXT,
  target_slots  TEXT,              -- JSON 배열 문자열
  issued_wall   REAL,
  issued_sim    REAL,
  result_wall   REAL,
  result_sim    REAL,
  duration_wall REAL,
  duration_sim  REAL,              -- 공정 소요시간(분석 기준)
  status        TEXT,              -- SUCCEEDED / FAILED / NULL(진행 중)
  phase         TEXT,
  reason        TEXT,
  reached_station TEXT,
  late_result   INTEGER DEFAULT 0, -- 이미 결과가 확정된 뒤 늦게 도착한 결과면 1
  UNIQUE(task_id, command_id)
);

CREATE INDEX IF NOT EXISTS idx_step_task ON step(task_id, id);

-- 3. executor 상태. heartbeat 전부가 아니라 값이 바뀐 순간과 생존 표본만
CREATE TABLE IF NOT EXISTS executor_status (
  id         INTEGER PRIMARY KEY AUTOINCREMENT,
  recv_wall  REAL,
  recv_sim   REAL,
  executor   TEXT,                 -- sim_task / navigation / inspection / sim_task_data
  state      TEXT,                 -- STARTING / READY / BUSY / PAUSED / ERROR
  task_id    TEXT,
  command_id TEXT,
  operation  TEXT,
  phase      TEXT,
  detail     TEXT
);

CREATE INDEX IF NOT EXISTS idx_exec_recv ON executor_status(executor, id);

-- 4. 검사 검출. 슬롯 1개 = 1행. 좌표는 원본 RGB 영상 pixel (u=가로, v=세로)
CREATE TABLE IF NOT EXISTS detection (
  id           INTEGER PRIMARY KEY AUTOINCREMENT,
  task_id      TEXT,
  command_id   TEXT,
  pallet_id    TEXT,
  pass_no      INTEGER,            -- 1=INSPECT, 2=RECHECK
  stamp_sim    REAL,
  recv_wall    REAL,
  frame_id     TEXT,
  image_width  INTEGER,
  image_height INTEGER,
  slot_id      TEXT,               -- SLOT_01~06, 미판정은 빈 문자열
  class_name   TEXT,
  confidence   REAL,
  center_u     REAL,
  center_v     REAL,
  bbox_x_min   REAL,
  bbox_y_min   REAL,
  bbox_x_max   REAL,
  bbox_y_max   REAL
);

CREATE INDEX IF NOT EXISTS idx_det_task ON detection(task_id, command_id);

-- 5. 검사 판정. command 1건 = 1행
CREATE TABLE IF NOT EXISTS inspection_verdict (
  id            INTEGER PRIMARY KEY AUTOINCREMENT,
  task_id       TEXT,
  command_id    TEXT,
  pallet_id     TEXT,
  operation     TEXT,              -- INSPECT / RECHECK
  defect_slots  TEXT,              -- JSON 배열 문자열
  unknown_slots TEXT,
  recv_wall     REAL,
  recv_sim      REAL
);

-- 6. 팔레트(트레이) 위치 이력. "트레이 위치·상태 기록 일치율" 의 측정 근거
CREATE TABLE IF NOT EXISTS pallet_move (
  id         INTEGER PRIMARY KEY AUTOINCREMENT,
  task_id    TEXT,
  command_id TEXT,
  pallet_id  TEXT,
  location   TEXT,
  source     TEXT,
  recv_wall  REAL,
  recv_sim   REAL
);

-- 7. 도킹 시도. 거리·각도는 TurnTable 라이다 검출 면 기준 상대좌표계
CREATE TABLE IF NOT EXISTS dock_attempt (
  id          INTEGER PRIMARY KEY AUTOINCREMENT,
  task_id     TEXT,
  command_id  TEXT,
  run_id      TEXT,
  status      TEXT,
  reason      TEXT,
  face_dist_m REAL,                -- 면 법선 방향 거리 m
  yaw_err_deg REAL,                -- 면 기준 상대 yaw 오차 도
  lat_m       REAL,                -- 면 접선 방향(횡) 오차 m
  retry       INTEGER,
  map_x       REAL,                -- /amcl_pose, map 좌표계 x m
  map_y       REAL,                -- map 좌표계 y m
  map_yaw_deg REAL,                -- map 좌표계 yaw 도
  recv_wall   REAL,
  recv_sim    REAL
);

-- 8. 웹 -> ROS 명령 우편함. 웹은 행을 넣고 recorder 가 읽어 서비스를 호출한다
CREATE TABLE IF NOT EXISTS web_command (
  id             INTEGER PRIMARY KEY AUTOINCREMENT,
  kind           TEXT NOT NULL,    -- START_CYCLE
  payload        TEXT,             -- {"scenario_id": "DEMO_HARVEST_01"}
  requested_wall REAL NOT NULL,
  state          TEXT NOT NULL DEFAULT 'PENDING',
  response       TEXT
);

-- 9. 최신값 1행씩. 고빈도 값(로봇 위치)을 이력으로 쌓지 않고 덮어쓴다
CREATE TABLE IF NOT EXISTS live (
  key          TEXT PRIMARY KEY,
  value        TEXT,
  updated_wall REAL,
  updated_sim  REAL
);

-- 10. 사람이 붙이는 정상·이상 라벨 (이상 탐지 학습 입력)
CREATE TABLE IF NOT EXISTS anomaly_label (
  id          INTEGER PRIMARY KEY AUTOINCREMENT,
  task_id     TEXT,
  command_id  TEXT,
  label       TEXT,                -- NORMAL / ANOMALY
  note        TEXT,
  labeled_wall REAL
);
