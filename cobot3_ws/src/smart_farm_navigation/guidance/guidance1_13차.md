# [guidance1 13차] 브랜치 정리 확인 — Nav2 제거, past/ 이동, 공용 모듈 분리

## 변경 내용
- 삭제(feature/navigation2 에 보존됨): `launch/nav2.launch.py`, `config/nav2_params.yaml`, `config/stations.yaml`, `maps/`, `scripts/make_map_from_usd.py`, `smart_farm_navigation/go_to_station.py`, results 의 nav2_·goto_ 로그.
- `past/` 로 이동(통합 후 미사용): `past/escape/`(escape_controller.py, escape.launch.py, escape_controller.yaml), `past/gopi_run.sh`, `past/params/`(수업용 Nav2 params), `past/guidance/`(1차~10차 가이드).
- `smart_farm_navigation/geometry.py` 신설: path_runner·path_runner_smooth 가 escape_controller 에서 가져오던 wrap_angle, yaw_from_odom 을 여기서 가져옴.
- `env_check.sh` 에서 Nav2 패키지 검사 절 제거. `package.xml` 의 Nav2 의존성 제거.
- 명명은 새 아키텍처 문서 기준(RACK_DOCK, INSPECTION_DOCK)으로 가이드·설계 문서에 반영함. 디렉터리 구조는 바꾸지 않음.

## 남는 실행 파일
| 실행 파일 | 용도 |
| --- | --- |
| `path_runner` (`path.launch.py`, `carter2_dock.launch.py`) | 직선·제자리회전 경유지 주행, carter2 RACK_DOCK 도킹 |
| `path_runner_smooth` (`path_smooth.launch.py`) | carter1 곡선 주행 → INSPECTION_DOCK |
| `scene_check` | 장면 통신 점검 |
| `scripts/launch_scene.py` | 주행 단독 시험용 Isaac standalone |
| `scripts/env_check.sh` | 터미널 환경 점검 |

## 고피 확인 (터미널 1개)
```bash
ros_set
cd /home/rokey/ROKEY_P3_A1/cobot3_ws
rm -rf build/smart_farm_navigation install/smart_farm_navigation
colcon build --packages-select smart_farm_navigation
source install/setup.bash
ros2 pkg executables smart_farm_navigation
```
- 기대: `path_runner`, `path_runner_smooth`, `scene_check` 3개만 출력. 이후 절차는 guidance1 11차(곡선 주행·ros_set 실측)와 12차(통합 리허설)를 그대로 따름.
