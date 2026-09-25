# 올인원 공정 녹화 (2026-09-25, Isaac Sim 5.1 GUI, v013 양배추 씬)

한 번의 올인원 실행(Pallet_01: G Y B G Y G → 노랑 2 + 갈색 1 제거)을 공정별 씬 카메라로 녹화한 것. 2배속.

| 파일 | 카메라 | 내용 |
|---|---|---|
| `00_all_processes.mp4` | — | 01~05 합본 |
| `01_harvest_rack.mp4` | Cam1_Harvest | 랙에서 양배추 팔레트 수확 (PICK_HARVEST) |
| `02_nav2_drive_place.mp4` | Cam2_Nav2Place | Nav2 주행 → 컨베이어 앞 도킹 → 내려놓기 (PLACE_INSPECT) → 컨베이어 반송 |
| `03_vision_inspection.mp4` | 손목 RealSense | 비전룸 검사 화면 + YOLO 판정 결과 |
| `04_cull_pick_place.mp4` | Cam4 / Cam5 | 이송 프레임 → 솎아내기(SortBin 1/2 번갈아) → 재검사 → 되밀기 |
| `05_transfer_frame_pusher.mp4` | Cam5_Pusher | 이송 프레임: PlateN(주황)이 로봇 쪽으로, PlateS(빨강)가 벨트로 되밀기 |
| `06_vision_detection_boxes.mp4` | 손목 RealSense | 프레임마다 YOLO 박스·클래스·신뢰도 (rqt_image_view 형식). 실제 판정은 3프레임 다수결 |

녹화 당시 가중치는 `romaine3_v012_640sq_yolo11n_best.pt`. 현재 코드 기본값은 `best.pt`(= romaine3_v011, 같은 검사 사진 90/90 일치).
