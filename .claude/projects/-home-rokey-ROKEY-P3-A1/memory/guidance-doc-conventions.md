---
name: guidance-doc-conventions
description: User's rules for guidance_n차.txt files beyond the ADR — self-contained per 차수, absolute paths, no 개루프 wording
metadata:
  type: feedback
---

Each `guidance_n차.txt` (in cobot3_ws/src/smart_farm_navigation/guidance/) must be readable on its own: restate 목적, 고정 경로, 사전 조건, 명령, 기대 결과, 중단 조건 every time. Increment 차수 by one per Q&A turn. Since 2026-09-20 the two tracks have separate branches and file names: feature/navigation (/cmd_vel motion) uses `guidance1_<n>차.md` starting at 11차; feature/navigation2 (Nav2+RViz2) uses `guidance2_<n>차.md` starting at 12차. Format is Markdown. Use absolute paths only, never shell variables like PROJECT_ROOT. Never include `git pull` or branch switching in a guidance: the user already checks out feature/navigation and pulls before they can read the file. Commands must work from any cwd. Refer to the target as "smart_farm_nav2_01.usd 탈출 동작", not "개루프".

**Why:** The user pastes command blocks into 고피 terminals and sometimes skips the first lines; on 2026-09-18 a `PROJECT_ROOT=` line was lost and colcon build ran at the repo root. They also said they should not need to open earlier 차수 files, and they dislike the term 개루프.

**How to apply:** Write every new guidance as a standalone document with literal `/home/rokey/ROKEY_P3_A1/...` paths, commit and push it to feature/navigation, and name the goal by the USD file. See [[gopi-naepi-split]].

**2026-09-22 update:** the user wants guidance files strictly linear (execute top to bottom, no jumping back), and every terminal block fully copy-pasteable on its own (repeat the 5 env lines in each block instead of "same 5 lines as before"). Explain what each 방법/step is for; mark optional steps as optional.
