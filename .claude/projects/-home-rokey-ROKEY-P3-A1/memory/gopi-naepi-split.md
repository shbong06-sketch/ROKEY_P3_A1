---
name: gopi-naepi-split
description: This machine is 내피 (code only, no ROS/Isaac); 고피 (host IsaacSim03) runs Isaac Sim and ROS 2 Jazzy, syncs via feature/navigation
metadata:
  type: project
---

This machine (hostname lwh19180, git user TGS10218) is 내피: cannot run Isaac Sim, but /opt/ros/jazzy IS installed (rclpy, colcon, ros2 launch work after `source /opt/ros/jazzy/setup.bash`), so ROS nodes can be built and integration-tested here with a fake odom publisher (scratchpad/fake_carter.py) on ROS_DOMAIN_ID=77. 고피 (hostname IsaacSim03, git user shbong) runs Isaac Sim + ROS 2 Jazzy with ROS_DOMAIN_ID=101 and pulls feature/navigation to test. Results come back as commits containing errored/*.txt or appended output under "---" in guidance files.

**Why:** Isaac Sim and the real scene exist only on 고피, but the ROS side can be verified here first, which saves 고피 round trips. usd-core in the scratchpad venv reads USD files. Expect a one-turn round trip per verification.

**How to apply:** After writing code or guidance, commit and push to feature/navigation, then wait for the user's next message. Do NOT run `git pull` on 내피: the user pulls 고피 results themselves before giving the next instruction (stated 2026-09-19). Just read the working tree as-is. See [[guidance-doc-conventions]].

**2026-09-23 correction:** only the Nav2 side (nav2.launch.py = Nav2/RViz2/feeder_dock/bag, and navigation_node) runs on 내피. Everything else — Isaac, all command publishing (/sim_task/command and /navigation/command), and later the Task Manager — runs on 고피. Therefore 고피 needs `colcon build --packages-select smart_farm_interfaces smart_farm_manager` once, or `smart_farm_interfaces/msg/TaskCommand` cannot be published there. Never source the workspace in the Isaac terminal (the team app puts the Isaac ROS bundle first and mixing with /opt/ros kills it).
