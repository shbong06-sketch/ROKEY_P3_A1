# 양배추 6구 트레이 에셋 · 올인원 통합 정리 (2026-09-25)

브랜치 `feature/cabbage-place-fix` · Isaac Sim 5.1 · 검증 PC: Windows 11 + RTX 3060 12GB (+ WSL2 Ubuntu 24.04 / ROS 2 Jazzy Nav2)

> 에셋 파일은 저장소에서 빼고 공유 zip(`cabbage_smartfarm_share_2026-09-25.zip`)으로 전달한다. 이 문서의 제작·시험 도구 설명은
> 기록용이며 도구 파일은 커밋 `ee1043d` 에 있다. 최종 파일 구성은 `docs/allinone_debug_and_changes_2026-09-25.md` 맨 위 참고.
> 양배추 판정은 이후 결정으로 **yellow·brown 모두 제거**(아래 표의 "보류"는 옛 규칙 표기).

## 1. 한눈에

| 항목 | 내용 |
|---|---|
| 에셋 | `cabbage_pallet_6.usd`(랙용, 전부 초록) · `cabbage_pallet_6_inspect.usd`(검사용 기본 배치) · `cabbage_pallet_empty.usd` · `cabbage_A/B/C.usd`(낱개) · `textures/` · `build_info.json` |
| 계층 | 로메인과 같음: `/<root>/Cube_011_001`(트레이 강체) · `/<root>/root_001/Cabbage_01~06`(포기 강체). 팀 코드의 `Pallet_0N/Cube_011_001` 경로가 그대로 맞음 |
| 배치 | 씬의 x 0.6 배율을 에셋에 반영(0.252 x 0.552 m) → **scale (1,1,1)** 로 놓는다. 위치·회전은 로메인과 같음 |
| 포기 | 3 모양(A/B/C) x 3 상태(green/yellow/brown), 지름 약 85~90 mm(잎 끝) / 파지 밴드 78 mm, 0.3 kg |
| 씬 교체 | `tools/cabbage/06_make_cabbage_scene.py` 가 원본은 두고 `<씬>_cabbage.usd` 복사본을 만든다(검사 트레이 칸 색 이식, 벨트 위 시험 큐브 3개 제거) |
| 랙 트레이 | 포크로 드는 랙 트레이도 비전 선별 대상이라 **검사용 에셋 + 검사 패턴**: Pallet_01 G Y B G Y G (불량 03, 보류 02·05) · Pallet_02 G Y B G Y B (불량 03·06) · Pallet_03 G G Y G Y G (보류 03·05) |

## 2. 양배추 모델 버전별 차이

| | ① 최초 모델 | ② 구버전 (색 작업 전, 시뮬레이션 돌렸던 것) | ③ 단색 버전 (중간, 폐기) | ④ **수정 버전 (최종)** |
|---|---|---|---|---|
| 형상 | Blender 절차 모델: 속통(둥근 윗면) + 겉잎 6장 + 꼭지, 모양 3종 | 같음 | 같음 | 같음 |
| 크기(잎 끝 최대 폭) | 약 112 mm (배율 0.86) | **85~90 mm** (배율 0.60) | 같음 | 같음 |
| 크기를 바꾼 이유 | – | RG2 완전 개방 패드 간격 **99.6 mm**(Isaac 실측). 밴드 78 mm 면 양쪽 10.8 mm 여유 | – | – |
| 겉모습 | 잎맥 텍스처 초록 잎 + 연한 속통 | 같음 | 잎·속통 모두 팀 기준색 단색(명암만) → 윗면 둥근 속통 구분이 사라짐 | **②의 두 톤 모습 유지**. 노랑/갈색은 같은 텍스처에 평균색만 입힘 |
| 불량 표현(비전) | 없음 | 없음 | green/yellow/brown | **green(원래 텍스처) / yellow / brown**, variantSet `condition` + 라벨 `condition` |
| 충돌체 | 선반(lathe) 볼록 조각 | 조각 5개: 시트(트레이 원뿔과 동일) · 하부(밴드 아래 4 mm 들여 턱) · **파지 밴드(적도 ±12 mm 수직 원통)** · 상부 · 바닥, 각 ≤ 64 정점 | 같음 | 같음 |
| PhysX 설정 | 기본 | **contactOffset 4 mm, restOffset 0, 위치 반복 64, 최대 탈침투 10 m/s** (RG2 가 빠르게 닫힐 때 밴드를 파고드는 문제 해결) | 같음 | 같음 |
| 질량·마찰 | 0.3 kg · 0.8/0.6 | 같음 | 같음 | 같음 |
| YOLO(`romaine3_v012_640sq_yolo11n_best.pt`, 검사 자세 19장) | – | – | 96.2 %, 색 오판 0 | **95.7 %**, 갈색 98.1 %(오판 0), 노랑 92.9 %(오판 0), 초록 97.1 %(2건 노랑) |

로메인 원본 같은 조건: 91.8 %(색 오판 18건). 미검출은 모두 화면 가장자리·가림.

## 3. 로메인 트레이(`romaine_pallet_6_v005`)와 다른 점

| | 로메인 v005 | 양배추 |
|---|---|---|
| 트레이 크기 | 0.42 x 0.552 원본을 씬에서 x 0.6 | 0.252 x 0.552 로 만들어 둠 → scale 1 |
| 포기 질량 | 1.0 kg | 0.3 kg (트레이 1.0 kg 동일, 총 2.8 kg) |
| 칸 시트 | 원뿔 39.5 mm 삽입 | 원뿔 림 r 26 mm, 약 9 mm 삽입 (위로 들면 바로 빠짐) |
| 파지면 | 불규칙 볼록(가시 밴드 추가 이력) | 적도 수직 밴드(RG2 용) |
| 색 배치 | 재질 바인딩(M_Romaine_Yellow/Brown) | variant `condition`, 교체 스크립트가 원본 칸 색을 그대로 옮김 |

## 4. 코드 변경 (이 브랜치 커밋)

| 커밋 | 파일 | 내용 |
|---|---|---|
| `31960a8` | `runtime/standalone_app.py` | **PLACE_INSPECT 수정** (로메인 원본에서도 실패하던 것): ① 놓는 방향을 '놓을 자리 → 팔 베이스' 로(턴테이블 +x 와 도킹 방향 90° 어긋나 IK 110° 뒤집힘) ② TurnTable 쿼터니언 정규화(배율 섞여 0.507) ③ 놓는 높이 = 벨트 윗면 + 25.9 mm(팔레트 원점 규칙) ④ 포크판이 벨트 가장자리보다 35 mm 밖에 오게 놓기(롤러 사이에 포크판이 걸려 EXIT 에서 팔레트가 20 mm 따라 올라옴) |
| `2f744ef` | `runtime/standalone_app.py`, `scripts/conveyor.py`, `scripts/conveyor_rollers.py` | **컨베이어 올인원 통합** (conveyor 는 refactor/lift-robot-motion 에서 가져옴): install/attach/update 계약대로 연결, 벨트에 걸친 시험 트레이 `Pallet_Inspect*` 는 벨트 밖에 세우고 시작(가이드가 관통해 작물이 튀던 문제), 로봇 놓기 중 줄기 벨트 인터록(`hold_stem`), 로봇이 드는 트레이는 벨트에 올라온 뒤 요 고정(`carried_paths`), `--no-conveyor` |

## 5. 팀에서 정할 것 / 바꿀 값

| 항목 | 현재 | 권장 | 근거 |
|---|---|---|---|
| 솎아내기 그리퍼 힘 `cull_standalone.GRIPPER_DRIVE_MAX_FORCE` | 1e4 N·m | **약 8** | 1e4 는 손끝 수백 kN → 어떤 물체도 관통. 8 N·m 에서 6칸 모두 파지 성공(미끄럼 ≤ 1 mm) |
| `feeder_dock` `standoff_m` | 0.85 | **0.92** (가이드 허용 0.78~0.94) | 0.85 에서 팔 베이스→놓을 자리 0.72 m, DESCEND_5 관절 20.7°(한계 20°)로 한 번 실패. 0.92 → 0.79 m 성공 |
| 비전 연동 | 반영됨: 스테이션이 끝나면 `inspection_done()` (`--no-vision-station` 이면 예전처럼 10 s 자동 배출) | — | 6-1 절 |
| 컨베이어 줄 ↔ 비전 로봇 거리 (R-C04) | 줄 y -6.73 은 도달 밖 | 푸셔 이송(현재) 또는 줄/로봇 배치 재합의 | 6-1 절 |
| v011 씬 `/clock` | 없음 | v013 사용 | Nav2 는 use_sim_time |

## 6. 검증 (이 PC, 양배추 v013 room core)

| 단계 | 결과 |
|---|---|
| 에셋 | Asset Validator 실제 위반 0 (variant 구조에서 검사기 3종 내부 오류만 발생) · 안정화 0.001 mm · 운반(1.5 m/s², 5°) 0.038 mm · 위로 빼기 걸림 없음 |
| TRANSFER `--demo` | v011 / v013 room core 모두 SUCCEEDED |
| PICK_HARVEST → **실제 Nav2** FEEDER_DOCK → PLACE_INSPECT | SUCCEEDED (도킹 면 0.869 m, 방향 1.55°, 좌우 5 mm) |
| 컨베이어 | 줄기 → 교차점 → 비전룸 정지 (-0.495, -6.727) → 10 s 뒤 배출 |
| 운반 중 포기 | Nav2 주행까지 최대 1.5 mm / 1.8°, 컨베이어 교차점 전환에서 최대 6.6 mm / 8.7° 후 복귀 |

## 6-1. 비전 검사 → 솎아내기 → 컨베이어 재가동 (2026-09-25 추가)

`scripts/inspection_cull_station.py` (+ `inspection_yolo_worker.py`, `cull_motion.py` 원본). 컨베이어가 카메라 앞에 세운 팔레트를 스테이션이 받아 끝까지 처리하고 `inspection_done()` 으로 다시 내보낸다 (시간이 아니라 솎아내기 완료가 재가동 조건).

| 순서 | 내용 |
|---|---|
| 정지 | `conveyor.install(vision_x=-0.69)` → 트레이 중심이 비전 M0609 base 와 같은 x 에 섬 |
| 푸셔 이송 | 컨베이어 줄(y -6.73)은 팔이 닿지 않음(위에서 집기 + 접근 18 cm 는 트레이 중심 0.48 m 까지). 로우폴리 푸셔(`/World/VisionTransfer`, 앞 푸셔 판+로드+하우징, 롤러 아래에서 올라오는 뒤 푸셔)가 트레이를 물리로 밀어 y -7.00 으로. 이동 중 포기 칸 이탈 0.1 mm |
| 검사 | 손목 RealSense(640x640) 3 프레임 → YOLO `romaine3_v012_640sq_yolo11n_best.pt` → 카메라 모델로 포기 투영 후 칸 배정 → 다수결 |
| 판정 | green = 정상, **yellow · brown = 제거** (양배추 결정) |
| 솎아내기 | 팀 `CullMotion` 실행기 + 계획 OPEN→APPROACH→DESCEND→GRASP→LIFT→**VIA**→PLACE→RELEASE→RETREAT. 집는 좌표 = 트레이 자세 + 칸 배치(비전 좌표는 대조, 오차 2~10 mm). 버리는 곳 SortBin_1 / SortBin_2 번갈아, 같은 통은 긴 변을 따라 자리 바꿈. 한 포기 끝나면 검사 자세로 복귀 |
| 재검사 | 제거 칸이 비었는지 다시 촬영 |
| 복귀·배출 | 앞 푸셔 물림 → 뒤 푸셔가 벨트 줄로 되밈 → `inspection_done()` |

시험(`09_inspection_cull_station_test.py`, v013 양배추 씬): 불량 4칸 트레이 4/4 제거·통 안 착지, 3칸 트레이 3/3, 비전 판정 6/6 일치, 트레이 벨트 복귀 후 배출.

씬 복사본에서 바뀐 것 (`06_make_cabbage_scene.py`, 원본 씬은 그대로)

| 항목 | 내용 |
|---|---|
| 비전 M0609 받침대 | 컨베이어 쪽으로 +0.146 m (프레임에 붙음), 로봇 base (-0.672, -7.480) |
| SortBox → SortBin | 속이 찬 큐브라 포기가 뚜껑에 얹혔음 → 같은 크기·색의 윗면 열린 통(바닥+벽 4장), 받침대 양옆 (-1.017 / -0.327, -7.70) |
| 비전룸 벽 통로 | 윗부분(BackWall_03/04_Head)에 가운데가 파인 홈 → 평판으로 교체 |
| 컨베이어 교차점 | `Feeder/Geometry/SM_ConveyorBelt_A49_01` 충돌체가 롤러보다 2 mm 높아 교차점 출구(x -1.42)에서 트레이가 걸림(접촉 보고로 확인) → 이 프레임 부품 충돌 끔 |
| 카터 리프트 프레임 | 기둥+윗가로대(z 1.48)를 팔이 관통 → 텔레스코픽: 바깥 기둥 0.78 m 로 낮춤 + 리프트 판에 안쪽 기둥 0.63 m. 로봇 위치·리프트 행정 그대로 |
| 공정 카메라 | `/World/ProcessCameras/Cam0_Perspective, Cam1_Harvest, Cam2_Nav2Place, Cam4_CullPickPlace` (검사 화면 = 손목 RealSense) |
| 삭제 | 벨트 위 시험 트레이 Pallet_Inspect/_01/_02/_03, Lettuce_1~3 |

## 7. Windows 한 대로 돌리는 법 (참고)

- Isaac 과 Nav2 사이는 DDS 가 WSL 경계를 넘지 못해 `tools/allinone_live/wsl/ros_tcp_relay.py`(localhost TCP 중계) 사용. WSL 은 NAT 모드(mirrored 에서는 WSL 안에서도 늦게 뜬 노드 간 데이터가 안 옴).
- 한 번에 실행: `smart_farm\tools\allinone_live\run_allinone.ps1` (Isaac GUI + Nav2 + RViz2 + 명령 흐름, 옵션 `-Human` `-Lane`). 설명: 그 폴더 README.md
- 팀 Linux PC 두 대 구성(고피/내피)은 기존 guidance2_25 그대로 쓰면 되고 중계기는 필요 없다.
