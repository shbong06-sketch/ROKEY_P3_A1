# tools/cabbage — 양배추 씬 만들기

`make_cabbage_scene.py` 하나. 원본 v013 씬에서 양배추 씬 복사본 `<씬>_cabbage.usd` 를 만든다 (원본은 그대로).

```
isaacsim/python.sh make_cabbage_scene.py <v013>/Collected_smartfarm_v013_room_core.usd <cabbage_pallet_6 에셋 폴더>
```

공유 zip 에 만들어 둔 양배추 씬이 들어 있어서 보통은 돌릴 필요가 없다.

하는 일: 로메인 트레이 → 양배추(랙 트레이는 검사 패턴), 비전룸 배치(받침대·SortBin), 벽 통로 윗부분 평판,
컨베이어 교차점 걸림 수정(A49 충돌 끔), 카터 리프트 텔레스코픽, 공정 카메라, 시험 트레이·Lettuce 삭제, 카터 속 부품 끄기.

## 올인원 실행

```
isaacsim/python.sh cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py --autoplay
```

- 양배추 씬이 `smart_farm/scenes/Collected_smartfarm_v013/Collected_smartfarm_v013_room_core_cabbage.usd` 에 있으면
  `--scene` 없이 그 씬을 연다.
- 컨베이어·비전 검사·솎아내기는 트레이가 벨트에 놓이면 자동 진행.
- YOLO: 씬 폴더의 `*best*.pt` 를 찾아 쓴다 (또는 `SMARTFARM_YOLO_WEIGHTS`). ultralytics 가 있는 파이썬은
  `SMARTFARM_YOLO_PYTHON` (없으면 `python3`).
- 코드: `runtime/standalone_app.py` + `scripts/` (conveyor, conveyor_rollers, cull_motion 은 팀원 코드 그대로,
  inspection_cull_station 이 검사·솎아내기·이송 프레임·YOLO 워커 전부).
