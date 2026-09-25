---
name: stepwise-observable-workflow
description: User wants 고피 procedures split by terminal (1 Isaac Sim, 2 build/check, 3 ros2 launch/run) so they can watch each step; monolithic bash is optional only
metadata:
  type: feedback
---

Guidance must follow the user's learned flow: 터미널 1 = env check, ros_set, isaac_ros, launch Isaac Sim and Play; 터미널 2 = build, source, checks (ros2 run scene_check / later Nav2+rviz2); 터미널 3 = ros2 launch/run of the motion node. Checks and motion are ROS 2 python nodes, not shell logic. scripts/gopi_run.sh stays as an optional one-shot.

**Why:** On 2026-09-19 the user said the one-line gopi_run.sh worked but hid the mechanism; they could not map it onto the ROS 2/Nav2 workflow they learned, and the final goal is Nav2 where that flow matters.

**How to apply:** Keep automation inside ROS nodes and launch files; keep the shell layer to a few visible commands per terminal with tee to results/. The user commits results themselves. See [[minimize-gopi-work]] and [[guidance-doc-conventions]].
