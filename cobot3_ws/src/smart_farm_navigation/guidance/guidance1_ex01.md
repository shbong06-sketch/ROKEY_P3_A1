# [guidance1 ex01] smart_farm_navigation 구조 파악용 — 파일 위치와 역할 (2026-09-20 갱신)

## 현재 실행 파일
| 실행 파일 | launch | 설정 | 용도 |
| --- | --- | --- | --- |
| `path_runner` | `path.launch.py`, `carter2_dock.launch.py` | `config/path_runner.yaml`, `config/carter2_dock.yaml` | 직선·제자리회전 경유지 주행. carter2 도킹은 두 번째 로봇이 생기면 사용 |
| `path_runner_smooth` | `path_smooth.launch.py` | `config/path_runner_smooth.yaml` | 곡선 주행 → INSPECTION_DOCK (1차 시연 기본) |
| `scene_check` | — | — | 장면 통신 점검 |
| `scripts/launch_scene.py` | — | — | 주행 단독 시험용 Isaac standalone. 통합에서는 팀장 run_world.py 사용 |
| `scripts/env_check.sh` | — | — | 터미널 환경 점검 |
| `smart_farm_navigation/geometry.py` | — | — | wrap_angle, yaw_from_odom 공용 함수 |

## 문서
- `docs/integration_plan.md` 통합 계획, `docs/architecture.md` 노드·상태 그림, `docs/*.mmd` 그림 원본.
- `guidance/guidance1_11차.md` 곡선 주행 + ros_set 실측, `guidance/guidance1_12차.md` 통합 리허설(최신).

## 실효 파일 보관 위치 (각 모듈 최상단의 past/)
| 위치 | 내용 |
| --- | --- |
| `guidance/past/` | 1차~10차 가이드(smart_farm_nav2_01.usd 시절) |
| `launch/past/escape.launch.py`, `config/past/escape_controller.yaml`, `smart_farm_navigation/past/escape_controller.py` | 초기 탈출 제어기(원호 후진). 통합 후 미사용 |
| `scripts/past/gopi_run.sh` | 일괄 자동 실행 스크립트 |
| `params/past/smart_farm/123.py` | 수업용 Nav2 params 사본 |

- Nav2·RViz2 관련 파일(nav2.launch.py, nav2_params.yaml, stations.yaml, maps/, make_map_from_usd.py, go_to_station.py)은 feature/navigation2 에만 있음.
- `results/` 는 고피 실행 로그, `errored/` 는 초기 오류 기록.
