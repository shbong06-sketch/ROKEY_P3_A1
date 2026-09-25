---
name: minimize-gopi-work
description: User demands minimal manual work on 고피 — one-command scripts, no GUI reading, no copy-pasting outputs; do analysis on 내피 instead
metadata:
  type: feedback
---

Every 고피 request must be as few commands as possible, ideally one line that pulls and runs a script. The script must capture its own output to a file and commit/push it. Never ask the user to read values from the Isaac Sim GUI (Stage/Property panels) or to paste terminal output by hand. Anything that can be computed on 내피 (USD inspection with usd-core in a scratchpad venv, static checks) must be done here.

**Why:** On 2026-09-19 the user rejected a guidance step that asked for manual GUI readings and output copy-paste, saying their physical-space work time is the bottleneck and process runtime overruns cause real harm.

**How to apply:** Ship a shell script in cobot3_ws/src/smart_farm_navigation/guidance/ (or scripts/) that does env setup, build, checks, logging via tee, and git commit+push. The guidance_n차.txt then holds one command block. Use the real path /home/rokey/ROKEY_P3_A1/cobot3_ws consistently, never mixed with /home/rokey/cobot3_ws, because colcon --symlink-install breaks when the workspace path string changes. See [[guidance-doc-conventions]] and [[gopi-naepi-split]].
