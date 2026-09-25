# tools/cabbage — 양배추 트레이 에셋 · 올인원(수확→Nav2→컨베이어→비전 검사→솎아내기) 도구

에셋: `assets/cabbage_pallet_6/` · 설명·검증: `docs/cabbage_asset_and_allinone.md`
스크립트 안의 경로는 작성 PC 기준(`D:\isaacsim`, `D:\smartfarm-sim`)이다. 다른 PC 에서는 경로만 바꾼다.

## 공정별 파일 (어디를 보면 되는지)

| 공정 | 런타임 코드 (smart_farm/) | 시험·도구 (tools/cabbage/) |
|---|---|---|
| ① 랙 수확 PICK_HARVEST | `runtime/standalone_app.py` (팀 코드) | `run_allinone_live.ps1` |
| ② Nav2 주행 → 컨베이어 앞 PLACE_INSPECT | `runtime/standalone_app.py` (놓기 자세 수정) | `wsl/start_nav2_stack.sh`, `wsl/run_flow.py`, `wsl/ros_tcp_relay.py` |
| ③ 컨베이어 반송 → 비전룸 정지 | `scripts/conveyor.py`, `scripts/conveyor_rollers.py` (refactor/lift-robot-motion + 인터록) | — |
| ④ 비전 검사 (YOLO best.pt) | `scripts/inspection_cull_station.py`, `scripts/inspection_yolo_worker.py` | `08_yolo_color_check.py` |
| ⑤ 솎아내기 픽앤플레이스 → SortBin 1/2 번갈아 | `scripts/inspection_cull_station.py` + `scripts/cull_motion.py`(feature/cull-motion 원본) | `09_inspection_cull_station_test.py`, `11_vision_arm_reach_check.py` |
| ⑥ 컨베이어 재가동 (솎아내기 끝나면) | `inspection_cull_station` → `conveyor.inspection_done()` | — |
| 씬 (양배추 교체·배치·카메라) | — | `06_make_cabbage_scene.py` |
| 녹화 | — | `aio_wrapper.py`(CABBAGE_CAPTURE_CAMS) · `10_make_process_videos.py` · `save_view_camera.ps1` |

팀원 진행 항목과의 대응

| 팀원 항목 | 이 브랜치 |
|---|---|
| pick_harvest → navigation → place_inspect 통합 | `standalone_app.py` PLACE 수정 + `run_allinone_live.ps1` (실제 Nav2) |
| cull 모션 완성 (pick and place 단위) | `cull_motion.py` 원본 + `inspection_cull_station._next_cull` (OPEN~RETREAT, 중간점 VIA) |
| detection 과 cull 연결 | `inspection_cull_station._assign/_judge` (YOLO → 칸 → 제거 목록) |
| inspect_place 와 conveyor 통합 | `standalone_app.py` (install/attach/update, `hold_stem`) |
| conveyor 운송과 inspect 통합 (검사 위치 정지) | `conveyor.install(vision_x=-0.69)` + 스테이션 |
| 검출 좌표 기반 cull (좌표 변환·도달 범위) | 카메라 모델로 픽셀→월드(대조용), 집는 좌표는 트레이 자세+칸 배치, 도달 범위 `11_vision_arm_reach_check.py` |
| conveyor 재가동 조건 = cull 완료 | `CONVEYOR_AUTO_RESUME_SECONDS` 대신 스테이션이 `inspection_done()` |

## 파일

| 파일 | 실행 | 하는 일 |
|---|---|---|
| `01_model_cabbage_blender.py` | `blender -b --factory-startup -P 01_... -- OUT_DIR [HEAD_SCALE]` | 양배추 머리 3종 시각 모델 |
| `02_build_cabbage_pallet.py` | numpy·scipy·manifold3d·pillow·usd-core 파이썬 | 트레이·포기 강체, 충돌체, 재질, `condition` variant |
| `03_finalize_isaac.py` | `isaacsim\python.bat` | Isaac USD 로 저장 + Asset Validator |
| `04_physics_tests.py` | `isaacsim\python.bat 04_... ASSET OUT MODE` | settle / transport / release |
| `05_m0609_pick_place_test.py` | `isaacsim\python.bat` | 팀 CullMotion 으로 RG2 픽앤플레이스 단위 시험 |
| `06_make_cabbage_scene.py` | `isaacsim\python.bat run_in_isaac.py 06_... SCENE ASSET_DIR` | `<씬>_cabbage.usd` 복사본: 양배추 교체, 비전룸 배치(받침대·SortBin), 벽 통로 윗부분 평판, 컨베이어 교차점 걸림 수정, 리프트 망루 텔레스코픽, 공정 카메라, 시험 트레이·Lettuce 삭제 |
| `07_scene_settle_diag.py` | `isaacsim\python.bat` | 씬 강체 무개입 안정성 |
| `08_yolo_color_check.py` | ultralytics + torch + usd-core | 사진을 best.pt 로 채점 |
| `09_inspection_cull_station_test.py` | `isaacsim\python.bat 09_... --scene S --out D [--pallet P] [--start=x,y]` | 로봇 없이 컨베이어→푸셔→검사→솎아내기→배출만 |
| `10_make_process_videos.py` | pylib 파이썬 (opencv, pillow) | 녹화 프레임 → 공정별 mp4 + 합본 |
| `11_vision_arm_reach_check.py` | `isaacsim\python.bat` | 비전 M0609 위에서 집기 도달 범위 (Lula IK) |
| `aio_wrapper.py` | `isaacsim\python.bat aio_wrapper.py runtime/standalone_app.py ...` | standalone_app 무수정 실행 + 포기·트레이 감시 + 공정 카메라 녹화 + 뷰 저장 |
| `run_allinone_live.ps1` | PowerShell | Isaac GUI + WSL Nav2/RViz2 + 전체 흐름 |
| `run_isaac_for_nav2.ps1` | PowerShell | Isaac GUI + Windows 쪽 중계기 |
| `save_view_camera.ps1 [-Name X]` | 실행 중에 | GUI 뷰포트의 현재 시점을 `/World/ProcessCameras/X` 카메라로 저장 |
| `run_in_isaac.py` | | pxr 스크립트를 Isaac USD 로 실행 (예외는 `<스크립트>.error.txt`) |
| `wsl/*` | WSL | Nav2 스택·중계기·흐름 명령 |

팀 Linux 두 대 구성에서는 `wsl/` 과 `run_*.ps1` 이 필요 없다.
