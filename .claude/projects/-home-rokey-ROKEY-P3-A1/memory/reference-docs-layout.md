---
name: reference-docs-layout
description: docs/reference is split into 01_Lecture (course notes), 02_notion (team Notion exports: architecture, interfaces, git rules) and past/; newest doc wins by 마지막 수정; never restructure the repo tree unless told
metadata:
  type: project
---

- `docs/reference/01_Lecture/` = course material (my Nova-Carter notes moved here by the user), `02_notion/` = team Notion exports (01_시스템아키텍쳐, 02_인터페이스설계, 03_Git관리규칙), `past/` = superseded material. `docs/*.md` may be older than 02_notion copies: compare the `마지막 수정` line and use the newest. ADR §8: do not browse docs/reference unless the topic needs it (token cost).
- Architecture (2026-09-19 23:32): Task Manager + Navigation Node + Inspection Node outside; Sim Task Executor inside one Isaac standalone (single owner of SimulationApp/World/step). Names: RACK_DOCK, INSPECTION_DOCK, INSPECT_STATION, PACK_OUT, PALLET_001.., TASK-YYYYMMDD-NNN(-CMD-NNN); operations TRANSFER, PICK_HARVEST, NAVIGATION, PLACE_INSPECT, INSPECT, CULL, CONVEYOR_OUT; custom msgs TaskCommand/TaskResult/ExecutorStatus/CycleStatus planned in smart_farm_interfaces.
- Git rules: commit prefixes feat/fix/test/refactor/docs/ci/chore, feature/* → development → main via PR, branch names lowercase-hyphen.
- The architecture doc's directory 기준안 (runtime/, motion/, config/ under isaacpjt) must NOT be applied until the user explicitly says so (2026-09-20).
- feature/navigation now holds only /cmd_vel runners (path_runner, path_runner_smooth, scene_check); Nav2 files live on feature/navigation2; superseded files sit in smart_farm_navigation/past/.

**Why:** Prevents using stale docs, re-adding Nav2 files to the wrong branch, or restructuring the tree prematurely.

**How to apply:** Read 02_notion docs when designing interfaces; follow the naming; keep tree as is. See [[integration-design]] and [[guidance-doc-conventions]].
