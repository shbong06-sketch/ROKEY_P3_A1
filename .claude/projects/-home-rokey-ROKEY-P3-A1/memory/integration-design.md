---
name: integration-design
description: 1st-integration design (2026-09-20) — team's robot_motion.py is the single Isaac standalone; carter motion via /cmd_vel path_runner nodes; handshake = AMR stillness in docking window, no rclpy inside Isaac
metadata:
  type: project
---

- Integration scene will be `Collected_smartfarm_v001.usd` (not yet in git). Backbone spec: `cobot3_ws/isaacpjt/smart_farm/scripts/robot_motion.py` (team lead, still changing): standalone that opens the scene, builds World, waits for the AMR base (SingleManipulator base pose) to be still inside BASE_X/Y_WINDOW (x −0.70..−0.55, y ±0.15, yaw ±15°, world frame), then plans M0609 pick&place with Lula IK. No ROS inside.
- My side: carter1 via path_runner_smooth (path_smooth.launch.py), carter2 via carter2_dock.launch.py (path_runner on /carter2 topics, YAML uses `/**:` because the node is namespaced). Only one standalone may own the sim, so launch_scene.py is for solo driving tests; robot_motion.py needs `enable_extension("isaacsim.ros2.bridge")` right after SimulationApp. Plan lives in `smart_farm_navigation/docs/integration_plan.md`, procedure in guidance1_12차.md.
- User chose /cmd_vel control (not Nav2) for the 1st integration/demo; Nav2 track continues on feature/navigation2.

**Why:** Prevents re-deriving the handshake design and the "one standalone" constraint.

**How to apply:** Keep rclpy out of Isaac scripts; sync via physics state or Action Graph topics. See [[project-milestones]] and [[carter-visual-front]].
