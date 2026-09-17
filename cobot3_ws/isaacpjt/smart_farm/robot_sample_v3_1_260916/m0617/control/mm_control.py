"""Parameter-based control for the robot_sample package.
Made on Isaac Sim 5.1; targeted at Isaac Sim 5.0.1 (isaacsim.core.* legacy API, unchanged 4.5-5.x).

Works for both assets:
  - mobile_manipulator.usda  (MiR100 + M0617 + RG6)          -> MobileManipulatorControl(root, has_base=True)
  - m0617_fixed_base.usda    (M0617 + RG6, base fixed)       -> MobileManipulatorControl(root, has_base=False)

Units used by THIS API (converted internally):
  base  : v [m/s], w [rad/s]  (w > 0 = turn left / counter-clockwise)
  arm   : joint angles [deg] in joint_1..joint_6 order (NOT the PhysX DOF order)
  gripper: RG6 l_out angle [deg], -30 = closed (~65 mm between tips), +30 = open (~194 mm)

Base commands:
  set_base_velocity(v, w)      open-loop wheel speeds. Straight driving is accurate (~97%), turning is NOT
                               (wheel slip / caster drag: ~50-85% of the commanded yaw).
  turn_by_deg(deg), drive_distance(m)
                               closed-loop goals using the measured base pose -> use these for accurate moves.

Call initialize() after the simulation is playing (World.reset() or timeline play),
then call step(dt) every physics step.
"""
import math

import numpy as np
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction

WHEEL_RADIUS = 0.0625          # MiR100 drive wheel radius [m]  (from mir100.usd collider)
WHEEL_SEPARATION = 0.4452      # distance between drive wheels [m]
ARM_JOINTS = [f"joint_{i}_joint" for i in range(1, 7)]
# Doosan M0617 official joint speed limits (dsr_description2 URDF): 100,100,150,225,225,225 deg/s
ARM_MAX_SPEED_DEG = np.array([100.0, 100.0, 150.0, 225.0, 225.0, 225.0])
GRIPPER_DRIVE = "l_out_joint"  # the only driven RG6 joint; the 5 others follow via mimic
GRIPPER_RANGE_DEG = (-30.0, 30.0)  # usable range (mimic followers are limited to +-30)


class MobileManipulatorControl:
    def __init__(self, root: str, has_base: bool = True, arm_speed_deg: float = 45.0,
                 max_linear: float = 1.0, max_angular: float = 1.5):
        self.root = root.rstrip("/")
        self.has_base = has_base
        self.arm_speed_deg = float(arm_speed_deg)
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.warnings = []
        self._arm_target = None
        self._arm_cmd = None
        self._base_goals = []      # queue of ("turn", target_yaw_deg) / ("drive", start_xy, heading_deg, distance)

    # ------------------------------------------------------------------ setup
    def initialize(self):
        self.arm = SingleArticulation(f"{self.root}/m0617", name=self.root.replace("/", "_") + "_arm")
        self.gripper = SingleArticulation(f"{self.root}/rg6", name=self.root.replace("/", "_") + "_gripper")
        self.arm.initialize()
        self.gripper.initialize()
        names = self.arm.dof_names
        missing = [j for j in ARM_JOINTS if j not in names]
        if missing:
            raise RuntimeError(f"arm joints not found in {self.root}/m0617: {missing} (dof_names={names})")
        self._arm_idx = np.array([names.index(j) for j in ARM_JOINTS])
        props = self.arm.dof_properties
        self._arm_lo = np.degrees(np.array([props["lower"][i] for i in self._arm_idx]))
        self._arm_hi = np.degrees(np.array([props["upper"][i] for i in self._arm_idx]))
        if GRIPPER_DRIVE not in self.gripper.dof_names:
            raise RuntimeError(f"{GRIPPER_DRIVE} not found in {self.root}/rg6")
        self._grip_idx = self.gripper.dof_names.index(GRIPPER_DRIVE)
        if self.has_base:
            self.base = SingleArticulation(f"{self.root}/mir100", name=self.root.replace("/", "_") + "_base")
            self.base.initialize()
            bn = self.base.dof_names
            self._wheel_idx = np.array([bn.index("left_wheel_joint_joint"), bn.index("right_wheel_joint_joint")])
        self._arm_cmd = self.arm_joints_deg()
        self._arm_target = self._arm_cmd.copy()

    def _warn(self, msg):
        self.warnings.append(msg)
        print(f"[mm_control] WARNING: {msg}")

    # ------------------------------------------------------------------ base
    def set_base_velocity(self, v: float, w: float):
        """v [m/s] forward, w [rad/s] yaw rate (+ = left)."""
        if not self.has_base:
            self._warn("set_base_velocity ignored: this robot has no mobile base")
            return
        vc = float(np.clip(v, -self.max_linear, self.max_linear))
        wc = float(np.clip(w, -self.max_angular, self.max_angular))
        if vc != v or wc != w:
            self._warn(f"base command clipped: v {v}->{vc} m/s, w {w}->{wc} rad/s")
        wl = (vc - wc * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
        wr = (vc + wc * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
        self.base.apply_action(ArticulationAction(joint_velocities=np.array([wl, wr]), joint_indices=self._wheel_idx))

    def stop_base(self):
        if self.has_base:
            self._base_goals = []
            self.set_base_velocity(0.0, 0.0)

    def turn_by_deg(self, angle_deg: float, max_w: float = 0.8):
        """Closed-loop in-place turn relative to the heading at the time the goal starts (+ = left)."""
        if not self.has_base:
            self._warn("turn_by_deg ignored: this robot has no mobile base"); return
        self._base_goals.append(["turn", float(angle_deg), float(max_w), None])

    def drive_distance(self, distance_m: float, max_v: float = 0.4):
        """Closed-loop straight drive (negative = backwards) with heading hold."""
        if not self.has_base:
            self._warn("drive_distance ignored: this robot has no mobile base"); return
        self._base_goals.append(["drive", float(distance_m), float(max_v), None])

    def base_done(self):
        return not self._base_goals

    def _base_step(self):
        if not self.has_base or not self._base_goals:
            return
        g = self._base_goals[0]
        x, y, yaw = self.base_pose()
        if g[3] is None:                       # latch start state when the goal becomes active
            g[3] = (x, y, yaw)
        x0, y0, yaw0 = g[3]
        if g[0] == "turn":
            target = yaw0 + g[1]
            err = (target - yaw + 180.0) % 360.0 - 180.0
            if abs(err) < 0.5:
                self.set_base_velocity(0.0, 0.0); self._base_goals.pop(0); return
            w = float(np.clip(math.radians(err) * 2.0, -g[2], g[2]))
            if abs(w) < 0.08:
                w = math.copysign(0.08, w)
            self.set_base_velocity(0.0, w)
        else:
            hx, hy = math.cos(math.radians(yaw0)), math.sin(math.radians(yaw0))
            travelled = (x - x0) * hx + (y - y0) * hy
            remaining = g[1] - travelled
            if abs(remaining) < 0.005:
                self.set_base_velocity(0.0, 0.0); self._base_goals.pop(0); return
            v = float(np.clip(remaining * 1.5, -g[2], g[2]))
            if abs(v) < 0.03:
                v = math.copysign(0.03, v)
            herr = math.radians((yaw0 - yaw + 180.0) % 360.0 - 180.0)
            self.set_base_velocity(v, float(np.clip(herr * 2.0, -0.5, 0.5)))

    def base_pose(self):
        """(x, y, yaw_deg) of base_footprint (mobile) or arm base (fixed)."""
        art = self.base if self.has_base else self.arm
        pos, quat = art.get_world_pose()   # quat = (w, x, y, z)
        w_, x_, y_, z_ = [float(q) for q in quat]
        yaw = math.degrees(math.atan2(2 * (w_ * z_ + x_ * y_), 1 - 2 * (y_ * y_ + z_ * z_)))
        return float(pos[0]), float(pos[1]), yaw

    # ------------------------------------------------------------------ arm
    def set_arm_joints_deg(self, q_deg, speed_deg: float = None):
        """Target joint_1..joint_6 [deg]. Motion is ramped in step(); limits are clamped."""
        q = np.array(q_deg, dtype=float)
        if q.shape != (6,):
            raise ValueError("set_arm_joints_deg expects 6 values (joint_1..joint_6)")
        qc = np.clip(q, self._arm_lo, self._arm_hi)
        if not np.allclose(qc, q):
            self._warn(f"arm target clamped to joint limits: {q.tolist()} -> {qc.tolist()}")
        self._arm_target = qc
        if speed_deg is not None:
            self.arm_speed_deg = float(speed_deg)

    def arm_joints_deg(self):
        return np.degrees(self.arm.get_joint_positions()[self._arm_idx])

    def arm_done(self, tol_deg: float = 0.5):
        return bool(np.all(np.abs(self._arm_cmd - self._arm_target) < 1e-6) and np.all(np.abs(self.arm_joints_deg() - self._arm_target) < tol_deg))

    # ------------------------------------------------------------------ gripper
    def set_gripper_deg(self, angle_deg: float):
        a = float(np.clip(angle_deg, *GRIPPER_RANGE_DEG))
        if a != angle_deg:
            self._warn(f"gripper angle clipped {angle_deg}->{a} deg (usable {GRIPPER_RANGE_DEG})")
        self.gripper.apply_action(ArticulationAction(joint_positions=np.array([math.radians(a)]), joint_indices=np.array([self._grip_idx])))

    def open_gripper(self):
        self.set_gripper_deg(GRIPPER_RANGE_DEG[1])

    def close_gripper(self):
        self.set_gripper_deg(GRIPPER_RANGE_DEG[0])

    def gripper_deg(self):
        return math.degrees(float(self.gripper.get_joint_positions()[self._grip_idx]))

    # ------------------------------------------------------------------ loop
    def step(self, dt: float):
        """Call once per physics step."""
        self._base_step()
        if self._arm_target is None:
            return
        max_step = np.minimum(self.arm_speed_deg, ARM_MAX_SPEED_DEG) * dt
        delta = np.clip(self._arm_target - self._arm_cmd, -max_step, max_step)
        self._arm_cmd = self._arm_cmd + delta
        full = np.zeros(len(self.arm.dof_names))
        full[self._arm_idx] = np.radians(self._arm_cmd)
        self.arm.apply_action(ArticulationAction(joint_positions=full))
