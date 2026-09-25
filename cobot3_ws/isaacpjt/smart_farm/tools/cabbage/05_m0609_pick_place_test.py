"""M0609 + OnRobot RG2 pick-and-place of one cabbage with the team's unmodified CullMotion (RMPflow).

  D:\\isaacsim\\python.bat 05_m0609_pick_place_test.py --team-dir DIR --robot-usd USD --asset A --empty B --out JSON
      [--slot 3] [--pick-z-offset M] [--place-slot 3]

team-dir must contain cull_motion.py and rmpflow/{m0609_rmpflow_controller.py, m0609_description.yaml,
m0609_rmpflow_common.yaml} and urdf/m0609_isaac_sim.urdf (from shbong06-sketch/ROKEY_P3_A1, feature/cull-motion).

Layout copies the v011 scene: robot base 0.77 m, tray origin 0.027 m above the base, tray yawed 90 deg
(as /World/SmartFarm/Placed/Pallet_Inspect), same prim paths as cull_standalone.py, same drive gains,
gripper close target 1.18 rad and tool orientation.
Plan = cull_motion.build_cull_plan: OPEN, PICK_APPROACH, PICK_DESCEND, GRASP, LIFT, PLACE_APPROACH,
PLACE_DESCEND, RELEASE, RETREAT.
"""
import argparse, json, math, sys, traceback
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--team-dir", required=True)
ap.add_argument("--robot-usd", required=True)
ap.add_argument("--asset", required=True)
ap.add_argument("--empty", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--slot", type=int, default=3)
ap.add_argument("--place-slot", type=int, default=3)
ap.add_argument("--pick-z-offset", type=float, default=None, help="relative to the head AABB centre; default = grip band")
ap.add_argument("--headless", type=int, default=1)
ap.add_argument("--video-dir", default="")
ap.add_argument("--gripper-gains", default="team", help="team = 1e5/1e3/1e4 (cull_standalone), asset = values in the RG2 USD, or k,d,f")
ap.add_argument("--trace", type=int, default=1)
ap.add_argument("--head-physx", default="", help="iters,depen,contact_offset override on every head (session only), e.g. 64,5,0.004; depen<=0 -> remove")
ap.add_argument("--aim", default="aabb", help="aabb = visual AABB centre (what vision reports), axis = the head's stem axis")
ap.add_argument("--tcp-offset", default="team", help="team = (0, 0, 0.19671) as in CullPickConfig, measured = pad centre measured in link_6, or x,y,z")
args = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": bool(args.headless)})
res = {"args": vars(args)}
try:
    import numpy as np
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
    from isaacsim.core.utils.xforms import get_world_pose
    from isaacsim.robot.manipulators.grippers import ParallelGripper
    from scipy.spatial.transform import Rotation as Rot

    TEAM = Path(args.team_dir)
    sys.path.insert(0, str(TEAM)); sys.path.insert(0, str(TEAM / "rmpflow"))
    from m0609_rmpflow_controller import RMPFlowController
    from cull_motion import CullMotion, CullPickConfig, CullConfig, build_cull_plan

    # ---- constants copied from cull_standalone.py (feature/cull-motion)
    ROBOT_PATH = "/World/SmartFarm/Placed/M0609/Asset"
    BASE_PATH = f"{ROBOT_PATH}/base_link"; EE_PATH = f"{ROBOT_PATH}/link_6"
    GRIPPER_ROOT_PATH = f"{ROBOT_PATH}/onrobot_rg2ft"
    ARM_JOINTS = tuple(f"joint_{i}" for i in range(1, 7))
    READY_JOINTS_DEG = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
    GRIPPER_JOINTS = ("finger_joint", "right_inner_knuckle_joint")
    GRIPPER_OPEN_POSITION, GRIPPER_CLOSE_POSITION = 0.0, 1.18
    PHYSICS_DT = 1.0 / 60.0
    TOOL_Q = (0.0, 1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0), 0.0)

    stage_ctx = omni.usd.get_context()
    stage_ctx.new_stage()
    stage = stage_ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT, physics_prim_path="/physicsScene")

    def xform(path, t=(0, 0, 0), rz=0.0):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp().Set(Gf.Vec3d(*t)); x.AddRotateZOp().Set(float(rz))
        return x

    def static_box(path, center, size):
        c = UsdGeom.Cube.Define(stage, path); c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*center)); c.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())

    BASE_Z = 0.77
    PAL_Z = BASE_Z + 0.02692913                      # v011: Pallet_Inspect origin 0.79692913
    Z0 = -0.02592913
    static_box("/World/ground", (0, 0, -0.05), (6, 6, 0.1))
    xform("/World/SmartFarm"); xform("/World/SmartFarm/Placed")
    xform("/World/SmartFarm/Placed/M0609", (0, 0, BASE_Z))
    stage.DefinePrim(ROBOT_PATH, "Xform").GetReferences().AddReference(args.robot_usd, "/World/m0609")
    static_box("/World/SmartFarm/Placed/Pedestal", (0, 0, BASE_Z / 2 - 0.0005), (0.2, 0.2, BASE_Z - 0.001))
    # source tray centred 0.42 m in front of the base (cull_standalone pick point y = 0.4192), yawed 90 deg
    SRC_XY, DST_XY, DST_YAW = (0.0, 0.42), (0.45, 0.0), 0.0
    xform("/World/SmartFarm/Placed/Pallet_Inspect", (SRC_XY[0], SRC_XY[1], PAL_Z), 90)
    stage.DefinePrim("/World/SmartFarm/Placed/Pallet_Inspect/Asset", "Xform").GetReferences().AddReference(args.asset)
    xform("/World/SmartFarm/Placed/Pallet_Place", (DST_XY[0], DST_XY[1], PAL_Z), DST_YAW)
    stage.DefinePrim("/World/SmartFarm/Placed/Pallet_Place/Asset", "Xform").GetReferences().AddReference(args.empty)
    for xy in (SRC_XY, DST_XY):                       # conveyor/table surface under each tray
        static_box("/World/SmartFarm/Placed/Table_%d" % (xy == DST_XY), (xy[0], xy[1], PAL_Z + Z0 - 0.02), (0.6, 0.6, 0.04))

    SRC_ROOT = "/World/SmartFarm/Placed/Pallet_Inspect/Asset"
    PALLET_BODY_PATH = SRC_ROOT + "/Cube_011_001"
    HEAD_PATHS = tuple(f"{SRC_ROOT}/root_001/Cabbage_{i:02d}" for i in range(1, 7))
    TARGET_PATH = HEAD_PATHS[args.slot - 1]
    DST_BODY_PATH = "/World/SmartFarm/Placed/Pallet_Place/Asset/Cube_011_001"

    # ---- drives exactly as cull_standalone.configure_drives
    for name in ARM_JOINTS:
        d = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{ROBOT_PATH}/joints/{name}"), "angular")
        d.GetStiffnessAttr().Set(1.0e8); d.GetDampingAttr().Set(1.0e4); d.GetMaxForceAttr().Set(1.0e8)
    fd = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{GRIPPER_ROOT_PATH}/joints/finger_joint"), "angular")
    if args.gripper_gains == "team":
        fd.GetStiffnessAttr().Set(1.0e5); fd.GetDampingAttr().Set(1.0e3); fd.GetMaxForceAttr().Set(1.0e4)
    elif args.gripper_gains != "asset":
        k, d, f_ = (float(v) for v in args.gripper_gains.split(","))
        fd.GetStiffnessAttr().Set(k); fd.GetDampingAttr().Set(d); fd.GetMaxForceAttr().Set(f_)
    res["gripper_drive"] = [fd.GetStiffnessAttr().Get(), fd.GetDampingAttr().Get(), fd.GetMaxForceAttr().Get()]

    if args.trace:   # contact report on both inner fingers: who touches them during GRASP / LIFT
        from pxr import PhysxSchema
        for n in ("left_inner_finger", "right_inner_finger", "left_outer_knuckle", "right_outer_knuckle", "left_inner_knuckle", "right_inner_knuckle"):
            cr = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(f"{GRIPPER_ROOT_PATH}/{n}"))
            cr.CreateThresholdAttr().Set(0.0)
    if args.head_physx:
        it_, dp_, co_ = (float(v) for v in args.head_physx.split(","))
        for hpth in HEAD_PATHS:
            hpr = stage.GetPrimAtPath(hpth)
            hpr.GetAttribute("physxRigidBody:solverPositionIterationCount").Set(int(it_))
            if dp_ > 0:
                hpr.GetAttribute("physxRigidBody:maxDepenetrationVelocity").Set(dp_)
            else:
                hpr.RemoveProperty("physxRigidBody:maxDepenetrationVelocity")
            if co_ > 0:
                for ch in hpr.GetChildren():
                    if ch.HasAPI(UsdPhysics.CollisionAPI):
                        ch.AddAppliedSchema("PhysxCollisionAPI")
                        ch.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(co_)
                        ch.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
    gripper = ParallelGripper(end_effector_prim_path=EE_PATH, joint_prim_names=list(GRIPPER_JOINTS),
                              joint_opened_positions=np.array([GRIPPER_OPEN_POSITION] * 2),
                              joint_closed_positions=np.array([GRIPPER_CLOSE_POSITION] * 2), action_deltas=None)
    robot = world.scene.add(SingleArticulation(prim_path=ROBOT_PATH, name="cull_m0609"))
    target = world.scene.add(SingleRigidPrim(prim_path=TARGET_PATH, name="cull_target"))
    world.reset()
    robot.initialize(physics_sim_view=world.physics_sim_view)
    gripper.initialize(physics_sim_view=world.physics_sim_view, articulation_apply_action_func=robot.apply_action,
                       get_joint_positions_func=robot.get_joint_positions, set_joint_positions_func=robot.set_joint_positions,
                       dof_names=robot.dof_names)
    gripper.set_default_state(np.array([GRIPPER_OPEN_POSITION] * 2))
    q = np.zeros(robot.num_dof)
    for name, deg in zip(ARM_JOINTS, READY_JOINTS_DEG):
        q[robot.get_dof_index(name)] = np.deg2rad(deg)
    robot.set_joints_default_state(positions=q, velocities=np.zeros(robot.num_dof))
    robot.set_joint_positions(q); robot.set_joint_velocities(np.zeros(robot.num_dof))
    controller = RMPFlowController(name="cull_m0609_rmpflow", robot_articulation=robot, physics_dt=PHYSICS_DT,
                                   urdf_path=str(TEAM / "urdf" / "m0609_isaac_sim.urdf"),
                                   robot_description_path=str(TEAM / "rmpflow" / "m0609_description.yaml"),
                                   rmpflow_config_path=str(TEAM / "rmpflow" / "m0609_rmpflow_common.yaml"),
                                   end_effector_frame_name="link_6")
    world.play()
    for _ in range(120):                               # SETTLE_STEPS
        world.step(render=not args.headless)

    # ---- where is the RG2 pad centre in link_6? (open fingers, collider points of both inner fingers)
    def body_pts(body):
        xc = UsdGeom.XformCache(); bw = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(body))
        pts = []
        for p in Usd.PrimRange(stage.GetPrimAtPath(body), Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if "/collisions/" in str(p.GetPath()) and p.IsA(UsdGeom.Mesh):
                m = xc.GetLocalToWorldTransform(p)
                pts += [np.array(m.Transform(Gf.Vec3d(*v))) for v in UsdGeom.Mesh(p).GetPointsAttr().Get()]
        return np.array(pts)
    ee_p, ee_q = get_world_pose(EE_PATH); R_ee = Rot.from_quat(np.roll(np.asarray(ee_q, float), -1))
    Lp = R_ee.inv().apply(body_pts(GRIPPER_ROOT_PATH + "/left_inner_finger") - ee_p)
    Rp = R_ee.inv().apply(body_pts(GRIPPER_ROOT_PATH + "/right_inner_finger") - ee_p)
    tip = max(Lp[:, 2].max(), Rp[:, 2].max())
    pad_c = (Lp[Lp[:, 2] > tip - 0.01].mean(0) + Rp[Rp[:, 2] > tip - 0.01].mean(0)) / 2
    res["pad_centre_link6"] = pad_c.round(4).tolist(); res["pad_tip_z_link6"] = round(float(tip), 4)
    if args.tcp_offset == "team":
        TCP_OFF = (0.0, 0.0, 0.19671)
    elif args.tcp_offset == "measured":
        TCP_OFF = (float(pad_c[0]), float(pad_c[1]), 0.19671)
    else:
        TCP_OFF = tuple(float(v) for v in args.tcp_offset.split(","))
    res["tcp_offset_used"] = [round(v, 4) for v in TCP_OFF]
    # ---- targets in base_link coordinates
    def wp(path):
        p, qq = get_world_pose(path); return np.asarray(p, float), np.asarray(qq, float)   # q = w x y z
    base_p, base_q = wp(BASE_PATH)
    R_wb = Rot.from_quat([base_q[1], base_q[2], base_q[3], base_q[0]])
    to_base = lambda p: R_wb.inv().apply(np.asarray(p) - base_p)
    info = json.load(open(Path(args.asset).with_name("build_info.json")))
    var = info["slots"][args.slot - 1]["variant"]; hg = info["heads"][var]
    head_p, head_q = wp(TARGET_PATH)
    head_up = Rot.from_quat([head_q[1], head_q[2], head_q[3], head_q[0]]).apply([0, 0, 1])
    band_c = head_p + head_up * hg["z_eq"]                                  # grip band centre (world)
    tcp_pick_w = band_c - np.array([0, 0, 0.002])                           # pad tip at the band centre (tip drops ~3 mm closing)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeWorldBound(stage.GetPrimAtPath(TARGET_PATH)).ComputeAlignedRange()
    aabb_c = (np.array(rng.GetMin()) + np.array(rng.GetMax())) / 2
    pick_z_offset = float(tcp_pick_w[2] - aabb_c[2]) if args.pick_z_offset is None else args.pick_z_offset
    aim_w = aabb_c if args.aim == "aabb" else np.array([head_p[0], head_p[1], aabb_c[2]])
    detected_base = to_base(aim_w)                                          # what vision would hand over
    # place: same head->TCP offset, into the empty tray's slot, 3 mm above the seat
    slot_xy = info["slots"][args.place_slot - 1]["slot_xy"]
    dst_origin = np.array([DST_XY[0], DST_XY[1], PAL_Z])
    slot_w = dst_origin + Rot.from_euler("z", DST_YAW, degrees=True).apply([slot_xy[0], slot_xy[1], 0])
    head_seat_w = slot_w + np.array([0, 0, info["slots"][args.place_slot - 1]["origin_z"]])
    place_tcp_w = head_seat_w + (tcp_pick_w - head_p) + np.array([0, 0, 0.003])
    place_base = to_base(place_tcp_w)
    res["setup"] = {"variant": var, "detected_aabb_centre_base": detected_base.round(4).tolist(),
                    "pick_z_offset": round(pick_z_offset, 4), "place_tcp_base": place_base.round(4).tolist()}

    motion = CullMotion(robot=robot, gripper=gripper, arm_controller=controller,
                        get_end_effector_world_pose=lambda: get_world_pose(EE_PATH),
                        get_base_world_pose=lambda: get_world_pose(BASE_PATH),
                        get_target_world_pose=target.get_world_pose,
                        config=CullPickConfig(approach_clearance=0.18, lift_clearance=0.22, tcp_offset_local=TCP_OFF,
                                              tool_orientation_base=TOOL_Q, min_target_rise=0.05))
    motion.start_pick(tuple(detected_base))
    motion._plan = build_cull_plan(tuple(detected_base), CullConfig(       # team's full pick -> place plan
        place_position_base=tuple(place_base), approach_clearance=0.18, transit_clearance=0.18, pick_z_offset=pick_z_offset))

    watch = (PALLET_BODY_PATH,) + HEAD_PATHS + (DST_BODY_PATH,)
    start = {p: wp(p) for p in watch}
    tcp = lambda: wp(EE_PATH)[0] + Rot.from_quat(np.roll(wp(EE_PATH)[1], -1)).apply(TCP_OFF)
    log, stage_rel, step = [], {}, 0
    prev_stage = None
    while motion.is_running and step < 6000:
        try:
            motion.update()
        except Exception as e:
            res["motion_error"] = str(e)
            res["tcp_at_fail_base"] = to_base(tcp()).round(4).tolist()
            break
        world.step(render=not args.headless)
        step += 1
        cur = motion.current_stage
        hp, hq = wp(TARGET_PATH)
        rel = hp - tcp()
        if cur != prev_stage and cur == "GRASP":   # clearance of each open pad to the head axis, along the closing axis
            e_p, e_q = get_world_pose(EE_PATH); R_e = Rot.from_quat(np.roll(np.asarray(e_q, float), -1))
            Lw = R_e.apply(Lp) + e_p; Rw = R_e.apply(Rp) + e_p
            ax_ = (Lw.mean(0) - Rw.mean(0)); ax_[2] = 0; ax_ /= np.linalg.norm(ax_)
            band_z = hp[2] + hg["z_eq"]
            Lb = Lw[np.abs(Lw[:, 2] - band_z) < 0.012]; Rb = Rw[np.abs(Rw[:, 2] - band_z) < 0.012]
            res["at_grasp"] = {"tcp_minus_head_axis_xy_mm": ((tcp() - hp)[:2] * 1000).round(2).tolist(),
                               "aabb_minus_axis_xy_mm": ((aabb_c - head_p)[:2] * 1000).round(2).tolist(),
                               "left_pad_inner_mm": round(float(((Lb - hp) @ ax_).min()) * 1000, 1) if len(Lb) else None,
                               "right_pad_inner_mm": round(float(((Rb - hp) @ ax_).max()) * 1000, 1) if len(Rb) else None,
                               "band_r_mm": round(hg["r_eq"] * 1000, 1), "pad_pts_in_band": [len(Lb), len(Rb)]}
        if cur != prev_stage:
            log.append({"stage": cur, "step": step, "head_z_mm": round((hp[2] - start[TARGET_PATH][0][2]) * 1000, 1),
                        "finger": round(float(robot.get_joint_positions()[robot.get_dof_index("finger_joint")]), 3)})
            prev_stage = cur
        if args.trace and cur in ("PICK_DESCEND", "GRASP", "LIFT"):
            from omni.physx import get_physx_simulation_interface
            from pxr import PhysicsSchemaTools
            hdrs, data = get_physx_simulation_interface().get_contact_report()
            for h in hdrs:
                a0 = str(PhysicsSchemaTools.intToSdfPath(h.collider0)).replace(ROBOT_PATH, "R").replace(SRC_ROOT, "S")
                a1 = str(PhysicsSchemaTools.intToSdfPath(h.collider1)).replace(ROBOT_PATH, "R").replace(SRC_ROOT, "S")
                imp = sum(np.linalg.norm(np.array(data[h.contact_data_offset + k].impulse)) for k in range(h.num_contact_data))
                key = cur[:5] + " " + " | ".join(sorted((a0, a1)))
                cs = res.setdefault("contacts", {}).setdefault(key, {"first_step": step, "frames": 0, "max_impulse": 0.0})
                cs["frames"] += 1; cs["max_impulse"] = round(max(cs["max_impulse"], float(imp)), 4)
        if args.trace and cur in ("PICK_DESCEND", "GRASP", "LIFT") and step % 5 == 0:
            lf = wp(GRIPPER_ROOT_PATH + "/left_inner_finger")[0]; rf = wp(GRIPPER_ROOT_PATH + "/right_inner_finger")[0]
            res.setdefault("trace", []).append({"s": step, "st": cur[:6], "fing": round(float(robot.get_joint_positions()[robot.get_dof_index("finger_joint")]), 3),
                "head_mm": ((hp - start[TARGET_PATH][0]) * 1000).round(1).tolist(), "tcp_z_mm": round(float(tcp()[2] - start[TARGET_PATH][0][2]) * 1000, 1),
                "fingers_gap_mm": round(float(np.linalg.norm((lf - rf)[:2])) * 1000, 1), "pal_mm": round(float(np.linalg.norm(wp(PALLET_BODY_PATH)[0] - start[PALLET_BODY_PATH][0])) * 1000, 2)})
        if cur in ("LIFT", "PLACE_APPROACH", "PLACE_DESCEND"):
            stage_rel.setdefault("ref", rel)
            stage_rel["max_slip_mm"] = max(stage_rel.get("max_slip_mm", 0.0), float(np.linalg.norm(rel - stage_rel["ref"])) * 1000)
            stage_rel["max_rise_mm"] = max(stage_rel.get("max_rise_mm", 0.0), (hp[2] - start[TARGET_PATH][0][2]) * 1000)
    res["stages"] = log
    res["done"] = motion.is_done; res["failed"] = motion.is_failed; res["error"] = motion.error; res["steps"] = step
    res["carry"] = {k: round(float(v), 2) for k, v in stage_rel.items() if k != "ref"}
    for _ in range(90):                                                      # 1.5 s after RETREAT
        world.step(render=not args.headless)
    hp, hq = wp(TARGET_PATH)
    up = Rot.from_quat(np.roll(hq, -1)).apply([0, 0, 1])
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))            # body axis vs world Z
    res["placed"] = {"xy_err_mm": round(float(np.linalg.norm(hp[:2] - head_seat_w[:2])) * 1000, 2),
                     "z_err_mm": round(float(hp[2] - (head_seat_w[2])) * 1000, 2), "tilt_deg": round(tilt, 2)}
    res["others_mm"] = {p.split("/")[-1] + ("(dst)" if p == DST_BODY_PATH else ""): round(float(np.linalg.norm(wp(p)[0] - start[p][0])) * 1000, 2)
                        for p in watch if p != TARGET_PATH}
except Exception:
    res["exception"] = traceback.format_exc()
open(args.out, "w").write(json.dumps(res, indent=1))
app.close()
