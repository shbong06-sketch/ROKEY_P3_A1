---
name: project-milestones
description: 1차 시연 done 2026-09-21 on Collected_smartfarm_v004; /cmd_vel track closed, next is Nav2 (feature/navigation2); jazzy vs humble bridge permanent
metadata:
  type: project
---

- 1차 시연 finished 2026-09-21 (Collected_smartfarm_v004.usd, /cmd_vel path_runner_smooth + team integration_v1.py). User declared the /cmd_vel track finished; next work is Nav2-based (feature/navigation2, guidance2_<n>차 naming). Isaac Sim time on 고피 is scarce; the user confirms values by prompt instead of scene files.
- v004 confirmed values (2026-09-21): carter start world (−0.40, 1.20) yaw +90°, drives toward world −Y as it faces (drive_direction_sign −1.0). INSPECTION_DOCK = center of Conveyor/Seg_6 (pivot (2.328, −5.121), 2 m long) with 1.5 m standoff → world (3.328, −3.621); waypoints [1.5, 4.821]/[0.0, 3.728], final_heading 0 (perpendicular to belt, facing it). integration_v1.py CONVEYOR_LATERAL_OFFSET_X_M / STANDOFF must match the yaml.
- 고피 has system ROS 2 Jazzy, while Isaac Sim's bundled ros2 bridge libs are Humble. Never put the bridge lib in LD_LIBRARY_PATH of a jazzy shell; scripts/env_check.sh checks it.
- Architecture docs in docs/reference describe the MiR100+M0617 plan; actual robots are nova_carter_ros and M0609.

**Why:** Dates, confirmed geometry and the track switch are not in the repo code.

**How to apply:** Do not reopen /cmd_vel tuning; keep yaml and integration_v1.py constants paired. See [[carter-visual-front]], [[integration-design]].
