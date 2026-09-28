# 공정 실험 도구 (2026-09-26 ~ 28)

Isaac Sim 5.1 올인원·스테이션을 조건별로 무인 실행하고, 결과를 DB·그림·DES 로 정리하는 도구 모음.
결과 문서: `docs/cabbage_grasp_speed_experiments_2026-09-27.md`, `docs/cabbage_followup_2026-09-28.md`

| 파일 | 하는 일 | 실행 |
|---|---|---|
| `station_test.py` | 스테이션 단독 시험 1회 (시험 트레이 + 컨베이어 + 검사·솎아내기). `--grip-force` `--head-mass` `--head-friction` `--truth-labels` `--pattern` | Isaac `python.bat station_test.py --scene <v014 씬> --out <폴더>` |
| `station_run.ps1` | `station_test.py` 여러 조건 묶음 실행, 검은 화면 회차 자동 재시도 | `station_run.ps1 -Root <폴더> -Specs @("이름\|인자\|환경변수")` |
| `speed_runs.ps1` | 운반 속도 실험 (수확→주행·도킹→내려놓기까지), 공정 카메라 녹화 → `mkvid.sh` 로 mp4 | `speed_runs.ps1 -Conds v06d1,v10 -Rep 1` |
| `run_experiments.ps1` | 9/26 사업 검토 실험 31회 묶음 | |
| `analyze.py` · `build_db.py` | 9/26 실행 로그 → 요약·SQLite(표 9 + 뷰 3) | 로컬 python |
| `grasp_analyze.py` | 집기 실험 → `grasp.sqlite` (선별 시도 1행, lift_slip / drop_out 구분) | |
| `speed_analyze.py` · `speed_clips.py` | 운반 실험 → `speed.sqlite`, 주행~내려놓기 클립 | |
| `des_fleet.py` + `des_inputs.json` | 설비 구성 이산사건 시뮬레이션 (표준 라이브러리만). `--conds default_v0.3,fast_v0.6_d1` | `python des_fleet.py out.csv --q 0.99,1.0` |
| `export_webdb.py` | 실험 결과 → 웹 DB 적재용 (PostgreSQL 스키마, CSV, JSON, SQLite) | |
| `make_figures.py` | 슬라이드·문서용 그림 16장 (PIL, Isaac `kit\python\python.exe` 로 실행) | |

- 조건표(CSV) 로 올인원을 돌리는 일반 실행기는 `../allinone_live/run_conditions.ps1`.
- 분석 스크립트의 기본 경로는 작성 PC(`D:\smartfarm-sim\out\...`) 기준이다. 다른 PC 에서는 인자로 폴더를 준다.
- 모든 수치는 시뮬레이션 측정이다. 실물 보정 전 절대값을 설비 사양으로 쓰지 않는다.
