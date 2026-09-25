# tools/cabbage — 양배추 씬 만들기

| 파일 | 하는 일 |
|---|---|
| `06_make_cabbage_scene.py` | 원본 v013 씬에서 양배추 씬 복사본 `<씬>_cabbage.usd` 를 만든다 (원본은 그대로) |
| `run_in_isaac.py` | pxr 스크립트를 Isaac Sim 5.1 의 USD 로 실행 (예외는 `<스크립트>.error.txt`) |

```
isaacsim/python.sh run_in_isaac.py 06_make_cabbage_scene.py <v013>/Collected_smartfarm_v013_room_core.usd <cabbage_pallet_6 에셋 폴더>
```

양배추 에셋 폴더(`cabbage_pallet_6`)는 저장소에 넣지 않고 공유 zip 으로 전달한다. zip 에는 이미 만들어 둔 양배추 씬도 들어 있어 보통은 이 스크립트를 돌릴 필요가 없다.

06 이 하는 일: 로메인 트레이 → 양배추(랙 트레이는 검사 패턴), 비전룸 배치(받침대·SortBin), 벽 통로 윗부분 평판,
컨베이어 교차점 걸림 수정(A49 충돌 끔), 카터 리프트 텔레스코픽, 공정 카메라, 시험 트레이·Lettuce 삭제, 카터 속 부품 끄기.

올인원 실행에 쓰는 코드는 `runtime/standalone_app.py` 와 `scripts/` (conveyor, conveyor_rollers, cull_motion,
inspection_cull_station, inspection_yolo_worker). 설명은 `docs/allinone_debug_and_changes_2026-09-25.md`.

에셋 제작·시험·녹화용 도구(01~05, 07~13, Windows/WSL 실행 스크립트)는 정리하면서 뺐다. 필요하면 커밋 `ee1043d` 에 있다.
