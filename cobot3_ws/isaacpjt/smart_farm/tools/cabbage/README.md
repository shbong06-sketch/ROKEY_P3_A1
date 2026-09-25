# tools/cabbage — 양배추 6구 트레이 에셋 · 검증 · 올인원(Windows) 도구

에셋 결과물은 `assets/cabbage_pallet_6/`, 설명·검증 결과는 `docs/cabbage_asset_and_allinone.md`.
스크립트 안의 경로는 작성 PC 기준(`D:\isaacsim`, `D:\smartfarm-sim`)이다. 다른 PC 에서는 경로만 바꾼다.

| 파일 | 실행 | 하는 일 |
|---|---|---|
| `01_model_cabbage_blender.py` | `blender -b --factory-startup -P 01_... -- OUT_DIR [HEAD_SCALE]` | 양배추 머리 3종(A/B/C) 시각 모델(.blend + .usda) |
| `02_build_cabbage_pallet.py` | numpy·scipy·manifold3d·pillow·usd-core 가 있는 파이썬 | 트레이·포기 강체, 충돌체, 재질, `condition` variant(.usda) |
| `03_finalize_isaac.py` | `isaacsim\python.bat` | Isaac 5.1 USD 로 .usd 저장 + 구조 검사 + Asset Validator |
| `04_physics_tests.py` | `isaacsim\python.bat 04_... ASSET OUT MODE` | settle / transport / release 물리 시험 |
| `05_m0609_pick_place_test.py` | `isaacsim\python.bat` | 팀 `cull_motion.CullMotion`(무수정)으로 M0609 + RG2 픽앤플레이스 |
| `06_make_cabbage_scene.py` | `isaacsim\python.bat run_in_isaac.py 06_... SCENE ASSET_DIR` | 원본은 두고 `<씬>_cabbage.usd` 복사본: 로메인→양배추(랙 3개는 검사용 패턴), 칸 색 이식, 벨트 위 시험 큐브 3개 제거 |
| `07_scene_settle_diag.py` | `isaacsim\python.bat` | 명령 없이 씬 전체 강체가 스스로 움직이는지 |
| `08_yolo_color_check.py` | ultralytics + torch + usd-core | `env_capture.py` 사진을 best.pt 로 채점(칸 색 정답과 대조) |
| `aio_wrapper.py` | `isaacsim\python.bat aio_wrapper.py runtime/standalone_app.py ...` | standalone_app 을 수정 없이 실행 + 포기 이동·차체 궤적 기록 |
| `run_allinone_live.ps1` | PowerShell | Isaac GUI + WSL Nav2/RViz2 + PICK_HARVEST → NAVIGATION → PLACE_INSPECT → 컨베이어 |
| `run_isaac_for_nav2.ps1` | PowerShell | Isaac GUI + Windows 쪽 중계기 |
| `wsl/ros_tcp_relay.py` | 양쪽 | Windows(Isaac) ↔ WSL(Nav2) ROS 토픽 중계 (DDS 가 WSL 경계를 못 넘음) |
| `wsl/start_nav2_stack.sh` | WSL | 중계기 서버 + nav2.launch + navigation_node (`STANDOFF=0.92` 로 feeder_dock 재기동) |
| `wsl/run_flow.py` | WSL | guidance2_25 8절 순서의 명령 발행 |
| `wsl/setup_ros2_nav2.sh`, `wsl/nav2_env.sh` | WSL | ROS 2 Jazzy + Nav2 + 팀 패키지 설치 / 환경 |

팀 Linux 두 대 구성(고피·내피)에서는 `wsl/` 과 `run_*.ps1` 이 필요 없다. guidance2_25 그대로.
