# smart_farm_monitor/results

관제 패키지의 실행 기록을 두는 곳. `smart_farm_navigation/results` 와 같은 구조임.

| 경로 | 무엇 | git |
|---|---|---|
| `log/` | 터미널 `tee` 로그, 시험 출력, DB 조회 결과 | **추적함** (실측 판정 근거, ADR_basic §5-5) |
| `log_media/` | 화면 캡처·녹화. 회차별 하위 폴더 `<수행한것>_<YYYYMMDD>_<HHMM>/` | 제외 (용량). 단 이 폴더 안의 `*.md` 문서는 추적함 |

DB 파일 `farm.db` 는 `results/` 가 아니라 `data/` 에 있고 git 제외임.
