# 실측 1차 · 시도 1 — `NAV_FAILED` 로 중단 (2026-09-28)

- 기기 **고피3** · `task_id` **`TASK-20260928-123736`** · 시작 `2026-09-28T12:34:49Z`
- 파일럿과 달라진 점: **검사 노드를 진짜 비전 노드(Docker + GPU)로 교체**, **창별 영상 녹화 추가**
- 결과: **3단계 NAVIGATION 에서 `NAV_FAILED` 로 사이클 중단.** 원인은 공정이 아니라 **환경 정리 누락**이었음

## 1. 구성 (전부 실측, 모의 없음)

| 구성 | 상태 |
|---|---|
| Isaac Sim (`:99`) | READY. v014 cabbage, `fullScan=True` |
| Nav2 + RViz2 (`:98`) | `Managed nodes are active`, bag 기록 |
| 주행 노드 | READY |
| **검사 노드 (Docker, GPU)** | READY. `torch 2.4.1+cu124`, `cuda.is_available()=True`, `NVIDIA L4`, `model=/models/best.pt`, `device=cuda:0`, `camera=/rgb` |
| 관제 (recorder + web) | 기동 |
| Task Manager | READY |
| 웹 시작 버튼 | **ACCEPTED** |

## 2. 진행과 중단

| # | operation | 결과 | 벽시계 | sim |
|---|---|---|---|---|
| 1 | TRANSFER | SUCCEEDED | 280.5 s | 95.0 s |
| 2 | PICK_HARVEST | SUCCEEDED | 182.1 s | 60.3 s |
| 3 | NAVIGATION | **FAILED (`NAV_FAILED`)** | **0.6 s** | 0.3 s |
| 3 | NAVIGATION (늦게 도착) | SUCCEEDED, `late_result=1` | 94.8 s | 30.8 s |

## 3. 트러블슈팅 — 원인은 파일럿 프로세스의 잔존

### 증상

`NAVIGATION` 명령이 나간 지 **0.64 초 만에** `Cycle failed: status=FAILED, reason=NAV_FAILED`. 주행 실패가 아니라 즉시 실패였음. 이후 Task Manager 로그에 `navigation` 의 `BUSY → READY(last FAILED)` 가 1.5 초 간격으로 반복됐음.

### 관찰

`nav2` 로그에 결정적인 줄이 있었음.

```
[bt_navigator] Begin navigating from current location (-0.42, 1.01) to (-2.19, -1.55)
[bt_navigator] Received goal preemption request          <-- 같은 목표가 한 번 더 들어옴
[bt_navigator] Begin navigating from current location (-0.42, 1.01) to (-2.19, -1.55)
[bt_navigator] Goal succeeded                            <-- 두 번째 목표는 성공
```

`ros2 node list` 로 세어 보니 **`/navigation_node` 가 2개**였음.

```
PID 96947, 96957   11:57 시작  <- 파일럿(monitor_pilot_20260928_1151)의 잔존 프로세스
PID 122155, 122250 12:35 시작  <- 이번 실측 1차의 것
```

### 원인

**파일럿을 정리할 때 `navigation_node` 를 놓쳤음.** 정리에 쓴 프로세스 패턴 목록에 `navigation_node` 가 빠져 있었고, launch 래퍼에 보낸 `SIGINT` 는 자식 노드까지 전파되지 않았음.

그 결과 이번 실측에서 **같은 `/navigation/command` 를 두 노드가 각각 처리**했음. 두 노드가 같은 목표를 `NavigateToPose` 로 보냈고, 두 번째 목표가 첫 번째를 **선점(preempt)** 했음. 선점당한 첫 목표의 결과가 실패로 돌아와 `navigation_node` 가 `NAV_FAILED` 를 발행했고, Task Manager 는 그것을 terminal 결과로 받아 사이클을 끝냈음.

**로봇 자체는 정상이었음.** 두 번째 목표는 성공했고 정밀 도킹까지 마쳤음(아래 4절). 즉 물리 동작의 문제가 아니라 **환경 정리의 문제**였음.

### 조치

1. 잔존 프로세스를 **PID 로 종료**함 (`kill 96957 96947`). `pkill -f` 는 쓰지 않음
2. 정리 절차를 고침 — 종료 패턴 목록에 `navigation_node`·`smart_farm_*/lib`·`nav2_*`·`lifecycle_manager`·`rosbag2`·`opennav`·`smart_farm_vision/lib` 을 모두 넣고, **launch 래퍼가 아니라 자식 PID 를 직접** 종료한 뒤 **남은 개수가 0 인지 확인**하도록 함
3. 다음 시도 전에 `ros2 node list` 로 **중복 노드가 없는지 먼저 확인**하는 단계를 둠

### 증거 스냅샷

| 파일 | 시점 |
|---|---|
| `ts01_isaac_NAV_FAILED.png`, `ts01_rviz_NAV_FAILED.png` | `NAV_FAILED` 직후 |
| `ts02_isaac_duplicate_node.png`, `ts02_rviz_duplicate_node.png` | 중복 노드 확인 시점 |

## 4. 그래도 얻은 것 — 도킹 2번째 표본과 `late_result` 실증

### 도킹 (검출 면 기준 상대좌표계)

| 항목 | 시도 1 | (참고) 파일럿 |
|---|---|---|
| `face_dist_m` | **0.927** | 0.938 |
| `yaw_err_deg` | **0.36** | 0.78 |
| `lat_m` | **0.022** | 0.017 |
| `retry` | **0** | 0 |
| map 좌표 | (−2.229, −2.739) yaw 89.13° | (−2.235, −2.680) yaw 88.42° |

두 표본 모두 `standoff_m` 0.92 목표에 근접했고 재시도 0 임.

### `late_result` 가 실데이터로 검증됨

Task Manager 가 `NAV_FAILED` 로 끝낸 뒤 **두 번째 목표의 성공 결과가 늦게 도착**했고, Task Manager 는 `Result ignored: no active command` 로 버렸음. 관제 DB 는 그것을 **`late_result=1` 의 별도 행**으로 남겼음(94.8 s / sim 30.8 s). 설계 의도대로 동작함.

## 5. 파일

| 파일 | 내용 |
|---|---|
| `isaac.mp4` | `:99` Isaac 창. 1920×1080, 10 fps, **812.9 s**, 48 MB |
| `rviz.mp4` | `:98` RViz2 창. 1920×1080, 10 fps, **812.9 s**, 11 MB |
| `ts01_*`, `ts02_*` (4장) | 트러블슈팅 시점 창별 스냅샷 |
| `isaac_<단계>.png`, `rviz_<단계>.png` | 단계별 창별 스냅샷 |
| `state_trace.txt` | 공정 단계·map 좌표 전이 |
| `ffmpeg_*.log`, `xvfb_*.log` | 녹화·가상 디스플레이 로그 |

관련 로그는 `smart_farm_navigation/results/log/*_20260928_1234.txt`(Isaac·Nav2·주행)와 `smart_farm_monitor/results/log/*_20260928_1234.*`(Task Manager·관제·비전)에 있음.
