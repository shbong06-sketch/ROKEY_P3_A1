"""Top-down / tilted IK reachability of the vision M0609 for tray slot positions (Lula, base frame)."""
import sys, json, math
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver
M = r"D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\M0609"
ks = LulaKinematicsSolver(robot_description_path=M + r"\rmpflow\m0609_description.yaml",
                          urdf_path=M + r"\doosan-robot2\urdf\m0609_isaac_sim.urdf")
TCP = np.array([0, 0, 0.19671])
TOOL_Q = (0.0, 1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0), 0.0)   # team top-down, fingers across
out = open(sys.argv[1], "w")
def ik(tcp_base, tilt_deg=0.0):
    q = Rot.from_quat([TOOL_Q[1], TOOL_Q[2], TOOL_Q[3], TOOL_Q[0]])
    if tilt_deg:
        yaw = math.atan2(tcp_base[1], tcp_base[0])
        axis = np.array([-math.sin(yaw), math.cos(yaw), 0.0])      # tip the tool toward the target (outward)
        q = Rot.from_rotvec(axis * math.radians(tilt_deg)) * q
    flange = np.asarray(tcp_base) - q.apply(TCP)
    xyzw = q.as_quat()
    sol, ok = ks.compute_inverse_kinematics("link_6", flange, np.array([xyzw[3], *xyzw[:3]]))
    return ok
BASE_Z = 0.77
GRIP_Z = float(sys.argv[2]) if len(sys.argv) > 2 else 0.885   # world z of the head grip band
for dy_c in (0.894, 0.80, 0.75, 0.70, 0.626, 0.55, 0.48):
    for dx_c in (0.176, 0.0):
        row = []
        for sx in (-0.189, 0.0, 0.189):
            for sy in (-0.075, 0.075):
                p = (dx_c + sx, dy_c + sy, GRIP_Z - BASE_Z)
                row.append((round(math.hypot(p[0], p[1]), 3), ik(p), ik((p[0], p[1], p[2] + 0.18)), ik(p, 20), ik(p, 35)))
        allok = all(r[1] and r[2] for r in row)
        tilt = all(r[3] for r in row), all(r[4] for r in row)
        print(f"centre dx={dx_c:.3f} dy={dy_c:.3f}: topdown(all pick+approach)={allok} tilt20={tilt[0]} tilt35={tilt[1]} ", row, file=out, flush=True)
app.close()
