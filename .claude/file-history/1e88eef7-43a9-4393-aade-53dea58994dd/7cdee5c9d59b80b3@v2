---
name: ros2-bootcamp-level
description: "Team are ROS 2 bootcamp students — keep code within plain ROS 2 idioms (nodes, topics, params, timers) and simple Python; avoid clever constructs they can't read"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 1e88eef7-43a9-4393-aade-53dea58994dd
  modified: 2026-09-23T23:33:47.810Z
---

The team (including the user) are bootcamp students in a ROS-based robot-control course. Code must stay inside what they can read and maintain: rclpy nodes, subscriptions/timers, parameters, launch files, plain Python classes and functions.

**Why:** Said on 2026-09-24 while I was refactoring `feeder_dock`: "배운 범위를 넘어서는 코드는 우리가 보고 숙지를 하기 어려움… ros2에 기반하고 있으니 이를 벗어나지는 말아주기 바람."

**How to apply:** No metaprogramming, no dict-unpacking tricks, no decorators beyond `@dataclass`/`@property`, no asyncio/threads unless ROS requires it. Explicit loops over clever one-liners. Test tools in `sim_test/` may be plain Python but should also stay simple and commented. Related: [[integration-design]], [[stepwise-observable-workflow]].
