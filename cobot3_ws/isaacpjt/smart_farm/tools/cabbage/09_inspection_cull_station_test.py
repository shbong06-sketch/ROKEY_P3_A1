"""Vision-room station test without the fork robot: one inspection tray is put on the conveyor line, the conveyor
stops it at the vision line, inspection_cull_station inspects it with YOLO and culls yellow/brown heads to the SortBoxes.

  D:\\isaacsim\\python.bat 09_inspection_cull_station_test.py --scene SCENE --out DIR [--pallet Pallet_Inspect] [--gui]
"""
import argparse, json, os, sys, time, traceback
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--scene", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--pallet", default="Pallet_Inspect")
ap.add_argument("--start", default="-1.25,-6.75")
ap.add_argument("--gui", action="store_true")
ap.add_argument("--max-seconds", type=float, default=240.0)
args = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": not args.gui, "width": 1280, "height": 720})
REPO = Path(r"D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt")
sys.path.insert(0, str(REPO / "smart_farm" / "scripts"))
out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
res = {"scene": args.scene, "pallet": args.pallet}
try:
    import numpy as np
    import omni.usd
    from pxr import UsdGeom, Gf
    from isaacsim.core.api import World
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("omni.replicator.core")
    omni.usd.get_context().open_stage(args.scene)
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        app.update()
    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    for prim in stage.Traverse():                     # ROS graphs are not needed here
        if prim.GetTypeName() == "OmniGraph":
            prim.SetActive(False)
    placed = "/World/SmartFarm/Placed"
    test = f"{placed}/{args.pallet}"
    sx, sy = (float(v) for v in args.start.split(","))
    others = [f"{placed}/Pallet_Inspect{s}" for s in ("", "_01", "_02", "_03") if f"{placed}/Pallet_Inspect{s}" != test]
    for i, path in enumerate([test] + others):       # the test tray on the line, the rest parked off the belt
        op = next(o for o in UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps() if o.GetOpName() == "xformOp:translate")
        v = op.Get()
        op.Set(type(v)(sx, sy, 0.769 + 0.0259) if path == test else type(v)(-3.40, -5.60 + 0.6 * i, 0.03))
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60, physics_prim_path="/physicsScene")
    from conveyor import install as install_conveyor
    import inspection_cull_station
    conveyor = install_conveyor(stage, [test], vision_x=-0.69, auto_resume=None)
    station = inspection_cull_station.install(stage, world, REPO / "M0609", out)
    for path in filter(None, os.environ.get("CAB_TEST_ENABLE", "").split(",")):    # bisecting helper
        stage.GetPrimAtPath(path).SetActive(True)
        print("[test] activated", path, flush=True)
    for path in filter(None, os.environ.get("CAB_TEST_DISABLE", "").split(",")):   # bisecting helper
        stage.GetPrimAtPath(path).SetActive(False)
        print("[test] deactivated", path, flush=True)
    if os.environ.get("CAB_TEST_LIPFIX"):         # zero-friction material on the junction exit frame
        from pxr import UsdShade, UsdPhysics as UP, Sdf as S
        mat = UsdShade.Material.Define(stage, "/World/_LipMat")
        pm = UP.MaterialAPI.Apply(mat.GetPrim())
        pm.CreateStaticFrictionAttr(0.0); pm.CreateDynamicFrictionAttr(0.0); pm.CreateRestitutionAttr(0.0)
        lip = stage.GetPrimAtPath("/World/SmartFarm/Placed/Conveyor/Feeder/Geometry/SM_ConveyorBelt_A49_01")
        UsdShade.MaterialBindingAPI.Apply(lip).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        print("[test] zero-friction lip", lip.GetPath(), flush=True)
    for path in filter(None, os.environ.get("CAB_TEST_NOCOLL", "").split(",")):
        from pxr import UsdPhysics as UP
        UP.CollisionAPI(stage.GetPrimAtPath(path)).CreateCollisionEnabledAttr().Set(False)
        print("[test] collision off", path, flush=True)
    contact_bodies = []
    if os.environ.get("CAB_TEST_CONTACTS"):      # PhysX contact report on the tray + heads (what blocks the tray?)
        from pxr import PhysxSchema, UsdPhysics as UP, Usd as U
        for p in U.PrimRange(stage.GetPrimAtPath(test)):
            if p.HasAPI(UP.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr().Set(0.0)
                contact_bodies.append(str(p.GetPath()))
    world.reset()
    conveyor.attach()
    station.attach(conveyor)
    world.play()
    t0, steps = time.time(), 0
    released = None
    while app.is_running() and steps < args.max_seconds * 60:
        world.step(render=(steps % 3 == 0))
        conveyor.update(1 / 60)
        station.update(1 / 60)
        steps += 1
        if os.environ.get("CAB_TEST_STATE") and steps % 120 == 0:
            d = conveyor._drive
            held = conveyor._held()
            print(f"[test:state] t={steps / 60:.0f} zone={conveyor.zone_of(test)} held={held.name if held else None} "
                  f"line={d.line._speed:+.2f} cross={d.cross._speed:+.2f} lift={d.cross._lift:.3f} stem={d.stem._speed:+.2f} "
                  f"sorter={d.sorter._speed:+.2f} n_api_line={len(d.line._api)}", flush=True)
        if contact_bodies and steps % 30 == 0:
            from omni.physx import get_physx_simulation_interface
            from pxr import PhysicsSchemaTools
            hdrs, data = get_physx_simulation_interface().get_contact_report()
            tray_x = UsdGeom.Xformable(stage.GetPrimAtPath(test + "/Cube_011_001")).ComputeLocalToWorldTransform(0).ExtractTranslation()[0]
            for h in hdrs:
                a = str(PhysicsSchemaTools.intToSdfPath(h.actor0)); b = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
                c0 = str(PhysicsSchemaTools.intToSdfPath(h.collider0)); c1 = str(PhysicsSchemaTools.intToSdfPath(h.collider1))
                if test in a and test in b:
                    continue
                pts = [list(data[h.contact_data_offset + k].position) for k in range(h.num_contact_data)]
                key = " | ".join(sorted((c0, c1)))
                rec = res.setdefault("contacts", {}).setdefault(key, {"first_t": round(steps / 60, 1), "n": 0})
                rec["n"] += 1; rec["last_t"] = round(steps / 60, 1); rec["tray_x"] = round(float(tray_x), 3)
                if pts:
                    rec["pt"] = [round(float(v), 3) for v in pts[0]]
        if station.results and released is None:
            released = steps
        if released is not None and steps - released > 5 * 60:   # watch the tray leave for 5 s
            break
    res["sim_seconds"] = round(steps / 60, 1); res["wall_seconds"] = round(time.time() - t0, 1)
    res["station"] = station.results
    res["tray_final"] = [round(v, 3) for v in UsdGeom.Xformable(stage.GetPrimAtPath(test + "/Cube_011_001")).ComputeLocalToWorldTransform(0).ExtractTranslation()]
    res["zone_final"] = str(conveyor.zone_of(test))
    station.close()
except Exception:
    res["exception"] = traceback.format_exc()
    print(res["exception"], flush=True)
(out / "result.json").write_text(json.dumps(res, indent=1, ensure_ascii=False), encoding="utf-8")
app.close()
