# standalone_app.py 수정 내역 보고 (navigation 파트)

- 대상 파일: `cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py`
- 작성일: 2026-09-23, 브랜치 `feature/Inspection-Place-nav2`
- 관련 커밋: `c381ef2`(추가), `66e4af1`(수정)
- **요약: 41줄 추가, 삭제·변경 0줄.** 기존 코드는 한 줄도 건드리지 않았고 전부 새 줄만 넣었음. 모든 추가 줄에 `[navigation 2026-09-23]` 주석을 달아 두어 찾기 쉬움.

## 1. 왜 손댔는가

인터페이스 설계 문서의 통합 검증 항목 7이 **"Nav2 성공 후 베이스 정지 검증 없이 PLACE 가 시작되지 않는가"** 임. 기존 `PLACE_INSPECT` 는 명령을 받으면 바로 바퀴 브레이크를 걸고 팔을 움직였으므로, 카터가 아직 미끄러지는 중이어도 Place 가 시작될 수 있었음. 그 검증 한 가지를 넣는 것이 목적임.

함께 **도킹이 실제로 어디에 멈췄는지 기록**하는 출력도 넣었음. Navigation 이 보고하는 값은 라이다로 잰 상대 거리라, Isaac 안의 실제 좌표와 대조할 자료가 필요함. 이 값이 쌓이면 팔이 닿는 범위를 숫자로 정할 수 있음. **판정은 하지 않고 기록만 함**.

## 2. 고친 곳 여섯 군데

| 위치 | 추가한 것 | 하는 일 |
|---|---|---|
| 상수부 | `BASE_SETTLE_SECONDS = 0.5` | Place 전에 차체가 멈춰 있어야 하는 시간 |
| `SimulationRuntime` | 필드 `base_watcher`, `arm_base` | 정지 감시기와 팔 밑동 객체를 런타임에 보관 |
| `create_simulation_runtime()` | `place_watcher = BaseWatcher()` 한 줄과 전달 | **Place 검사 전용 감시기를 따로 만듦** |
| 함수 추가 | `report_dock_pose()` | 카터 본체 world 좌표와 place 대상까지의 거리를 한 줄 출력 |
| `start_operation()` 의 `PLACE_INSPECT` | 정지 확인 5줄 + `report_dock_pose()` 호출 1줄 | 안 멈췄으면 `FAILED / BASE_NOT_SETTLED` 로 돌려보내고, 멈췄으면 기록 후 기존 흐름 그대로 진행 |
| `run()` 메인 루프 | `runtime.base_watcher.update(...)` 한 줄 | 매 스텝 정지 여부 갱신 |

### 감시기를 따로 만든 이유

`BaseWatcher` 는 `PalletTransferController` 가 소유해서 자기 흐름 안에서 `update()` 와 `reset()` 을 부르는 객체임. 처음에는 그 객체를 그대로 썼는데, 메인 루프에서 또 갱신하니 **대기 시간이 두 배 속도로 흐르고 정지 판정이 실제보다 일찍 나는** 문제가 생겼음. 실측 로그의 `[대기] 팔 베이스 정지를 기다리는 중 (181.0초)` 가 그 증상임. 그래서 Place 검사용 감시기를 별도 객체로 분리했고, 이송 제어기 쪽 동작에는 영향이 없음.

## 3. 출력이 어떻게 달라지는가

Place 명령을 받으면 기존 출력 사이에 아래 한 줄이 추가됨.

```
[도킹] 카터 본체 world (-2.170, -2.830), place 대상까지 x -0.020 m, y -1.080 m, 직선 1.080 m
```

그리고 차체가 아직 흔들리는 중이면 팔을 움직이지 않고 `FAILED / BASE_NOT_SETTLED / CHECK_BASE_STOPPED` 로 결과를 돌려줌. 이때는 2~3 초 뒤 같은 명령을 다시 보내면 됨.

## 4. 중간에 제가 낸 결함과 조치 (보고)

9월 23일 19차 실측에서 **Place 명령을 보내자마자 Isaac 이 종료**되었음. 원인은 처음 넣은 `report_dock_pose()` 가 이 파일에 없는 이름 두 가지를 부른 것임.

- `robot_motion.prim_world_pose(...)` — 이 파일은 `from robot_motion import (...)` 만 하므로 `robot_motion` 이라는 모듈 이름이 없고, 그 함수 자체도 프로젝트에 존재하지 않음
- `numpy` — 이 파일은 numpy 를 import 하지 않음

예외가 최상위까지 올라가 `app.close()` 로 이어졌음. 커밋 `66e4af1` 에서 표준 API(`runtime.robot.get_world_pose()`)와 순수 파이썬 연산으로 바꿨고, 바뀐 파일 전체를 정적 검사(`pyflakes`)로 돌려 같은 유형이 더 없는 것을 확인했음. 제 확인 절차가 빠진 탓이며, 앞으로 이 파일을 건드릴 때는 정적 검사를 반드시 거치겠음.

## 5. 되돌리기와 영향 범위

- 이 41줄을 모두 지워도 기존 동작은 그대로임. 추가만 했고 기존 줄은 손대지 않았음.
- `PLACE_INSPECT` 외의 연산(`TRANSFER`, `PICK_HARVEST`)에는 변화가 없음.
- 팀에서 쓰는 `BaseWatcher`, `PalletTransferController`, `turntable_place_pose`, `brake_wheels_at_current_position` 의 동작은 바뀌지 않았음.
- 되돌리려면: `git revert 66e4af1 c381ef2` 가 아니라, 해당 파일에서 `[navigation 2026-09-23]` 주석이 달린 블록만 지우면 됨(다른 파일의 변경과 섞이지 않게).

## 6. 참고: 같은 파일에 9월 22일에 넣었던 것

라이다 관련으로 `open_scene()` 에 `fullScan=True` 6줄을 넣은 적이 있음(커밋 `49d24d8`). 그 부분은 이후 팀에서 `1e7fbc7` 로 다시 정리했으므로 **현재 파일의 해당 코드는 팀 코드**이며 제 변경분이 아님.
