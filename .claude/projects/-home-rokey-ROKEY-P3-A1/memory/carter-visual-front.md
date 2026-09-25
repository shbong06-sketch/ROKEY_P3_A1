---
name: carter-visual-front
description: Nova Carter front = base_link +x (big drive wheels); scene places the REAR toward the corridor exit on purpose; exit by backing up, never rotate in the corridor
metadata:
  type: project
---

Settled 2026-09-21 by the user after the 1st demo: the carter's real front is the drive-wheel (big wheel) side = `base_link` +x, casters and the M0609/lift are at the rear (−x). Earlier "visible front = −x" was the user's mistake. In Collected_smartfarm_v008/v011 the rig at (−0.42, 1.01) yaw 90 has its rear toward the corridor exit (−y) because the arm works from the rear; the team will not rotate it.

**Why:** Changing the rig yaw would break the M0609 pick/place parameters.

**How to apply:** /cmd_vel runners keep drive_direction_sign −1 for "drive toward the exit". Nav2 track: go_to_station backs up (Nav2 BackUp) out of reverse_out_zones before any goal; RPP allow_reversing false; never plan an in-place turn inside the rack corridor or 0.55 m in front of the Feeder TurnTable. See [[nav2-track-v008]].
