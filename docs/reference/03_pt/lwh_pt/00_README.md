# Navigation 파트 PT 산출물 — 파일 안내

- 작성일: 2026-09-27 · 담당 이원행
- 담당 구간: 랙 팔레트 파지 → 자율주행 → 턴테이블 정밀 도킹 → Place 인계

## 파일 목록

| 파일 | 역할 | 언제 쓰는가 |
|---|---|---|
| `01_Navigation_PT_슬라이드구성.md` | **원고 본문(24장).** 슬라이드마다 목적 / 본문 / 도해 지시 / 근거(파일:줄) 4칸 | PPTX 로 다시 만들 때, 수치 출처를 확인할 때 |
| `02_Navigation_발표대본.md` | 15분 5부 대본, 예상 질문 11건, 회의 후 할 일 | 회의·발표 직전에 읽음 |
| `03_Navigation_슬라이드.html` | **시각화 슬라이드 25장.** 실행 영상 캡처 15장 + 슬라이드마다 인포그래픽 1개 이상 | 화면으로 보여줄 때, PDF 로 뽑을 때 |
| `img/` | 위 슬라이드가 쓰는 캡처 이미지 15장 | HTML 과 같은 위치에 있어야 함 |

`01` 과 `03` 은 같은 내용이며 `03` 이 요약본임. 수치가 어긋나면 `01` 의 `근거` 칸이 기준임.

## 슬라이드 보는 법

브라우저로 `03_Navigation_슬라이드.html` 을 열면 됨. 외부 서버·폰트·CDN 을 쓰지 않으므로 인터넷 없이 동작함.

- `←` `→` 키 또는 하단 버튼으로 이동. `Home` / `End` 로 처음·끝
- **PDF 로 저장**: 하단 `PDF 저장` 버튼 또는 `Ctrl+P` → 용지 **가로**, 배율 **100%**, **배경 그래픽 켜기**, 여백 **없음**
- **PPTX 가 필요하면**: 위 PDF 를 PowerPoint 에서 열거나, 각 슬라이드를 이미지로 내보내 한 장씩 배치함. 레이아웃을 그대로 쓰려면 PDF 가 안전함
- 내용이 슬라이드 높이를 넘으면 그 슬라이드만 자동으로 축소됨(잘리지 않음). 글씨가 작아 보이면 해당 슬라이드의 항목을 줄이는 것이 맞음

## 이미지 출처

모두 **2026-09-26 전 구간 통과 실행**의 기록임. 원본은 저장소의
`cobot3_ws/src/smart_farm_navigation/results/media_log/` 에 있으며 git 에는 올리지 않음(용량).

| 파일 | 원본 | 시점 |
|---|---|---|
| `isaac_01_pick.jpg` ~ `isaac_08_conveyor.jpg` | `run_20260926_0132/isaac.mp4` | 영상 266 / 332 / 354 / 387 / 405 / 432 / 504 / 537초 |
| `isaac_09_after_place.jpg` | `run2_isaac_after_place.png` | Place 완료 직후 |
| `rviz_01_drive.jpg`, `rviz_02_docked.jpg` | `run_20260926_0132/rviz.mp4` | 영상 312 / 405초 |
| `rviz_03_full.jpg` | `live_rviz_map.png` | RViz2 전체 화면 |
| `term_01_cmd.jpg` | `run_20260926_0132/terminal.mp4` | 영상 405초 |
| `term_02_three_cmds.jpg` | `run2_terminal_after_place.png` | 세 명령 발행 후 |
| `term_03_font.jpg` | `font_check_after_fix.png` | 녹화 글꼴 수정 확인 |

영상 시각 ↔ 로그 시각 대응은 `run_20260926_0132/start_time.txt`(2026-09-26T01:32:43Z)를 기준으로 계산함.

## 수치 출처 (질문 받으면 여는 곳)

| 수치 | 출처 |
|---|---|
| 도킹 0.937 m · +1.08° · 좌우 0.000 m | `results/nav2_20260926_0133.txt:460` |
| 카터 정지 world (−2.161, −2.738) | `results/isaac_20260926_0132.txt:665` |
| 팔 베이스 ~ 놓을 자리 0.809 m | `results/isaac_20260926_0132.txt:666` |
| 단계 소요 36 / 58 / 73초 | `results/navnode_20260926_0133.txt`, `isaac_20260926_0132.txt` 로그 시각 차 |
| ROS 회귀 10/10 | `results/regression_20260925_standoff092.txt` |
| 격자 225 케이스 · 이전 이득 206/225 | `results/docksim_20260925_standoff092.txt`, `smart_farm_navigation/docs/troubleshooting.md` §5-5 |
| 전 구간 10분 9초 | `docs/0번_단계통합_완료보고_및_수정내역_260926.md` §6-1 |
