"""비전룸 스테이션 단독 시험 (포크 로봇·Nav2 없이).

검사용 양배추 트레이 하나를 컨베이어 줄기(또는 가로줄기)에 올려 두면
  컨베이어 반송 -> 카메라 앞 정지 -> 이송 프레임이 로봇 앞으로 밀기 -> YOLO 검사 -> 노랑·갈색 솎아내기(SortBin 1/2)
  -> 재검사 -> 벨트로 되밀기 -> 배출
까지 inspection_cull_station 이 올인원과 같은 코드로 처리한다.

  D:\\isaacsim\\python.bat 09_inspection_cull_station_test.py --scene SCENE --out DIR
        [--pattern G,Y,B,G,Y,B] [--start=-2.186,-4.2] [--snap /World/ProcessCameras/Cam5_Pusher] [--contacts] [--gui]

  --pattern   칸 01~06 색 (G/Y/B). 기본은 검사용 에셋 기본 배치 G,Y,B,G,Y,G
  --start     트레이를 놓을 x,y. 기본은 줄기 위(-2.186,-4.2) — 교차점까지 실제 경로로 지난다
  --snap      이 카메라로 1초마다 사진 (snap_XXXX.jpg)
  --contacts  트레이가 무엇에 닿는지 PhysX 접촉 보고를 result.json 에 남김 (걸림 진단)
결과: DIR/result.json, DIR/station_results.json, DIR/station_events.json, DIR/*_yolo.jpg
"""
import argparse
import json
import sys
import time
import traceback
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--scene", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--pattern", default="G,Y,B,G,Y,G")
ap.add_argument("--start", default="-2.186,-4.2")
ap.add_argument("--snap", default="")
ap.add_argument("--contacts", action="store_true")
ap.add_argument("--no-instance", action="store_true", help="시험 트레이 포기의 instanceable 을 끔 (렌더 이상 진단용)")
ap.add_argument("--grip-force", type=float, default=None, help="RG2 finger_joint 드라이브 maxForce (N·m, 기본 8)")
ap.add_argument("--head-mass", type=float, default=None, help="포기 질량 kg (기본 0.3)")
ap.add_argument("--truth-labels", action="store_true", help="판정을 정답 라벨로 대신 (집기 실험을 렌더·비전과 분리)")
ap.add_argument("--head-friction", default="", help="포기 마찰 '정지,동적' (예 0.4,0.3). 기본 에셋 0.8/0.6")
ap.add_argument("--gui", action="store_true")
ap.add_argument("--max-seconds", type=float, default=360.0)
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": not args.gui, "width": 1280, "height": 720})
HERE = Path(__file__).resolve()
REPO = HERE.parents[3]                                   # 저장소 smart_farm/tools/cabbage/ 안에서 실행 -> isaacpjt
if not (REPO / "smart_farm").exists():                  # 작성 PC 의 작업 폴더에서 실행한 경우
    REPO = HERE.parents[2] / "ROKEY_P3_A1" / "cobot3_ws" / "isaacpjt"
sys.path.insert(0, str(REPO / "smart_farm" / "scripts"))
out = Path(args.out)
out.mkdir(parents=True, exist_ok=True)
res = {"scene": args.scene, "pattern": args.pattern}
COLOURS = {"G": "green", "Y": "yellow", "B": "brown"}
TEST = "/World/SmartFarm/Placed/Pallet_Test"
BELT_TOP, PALLET_ORIGIN_ABOVE_BOTTOM = 0.769, 0.0259

try:
    import numpy as np
    import omni.usd
    from pxr import Gf, Usd, UsdGeom
    from isaacsim.core.api import World
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.replicator.core")
    omni.usd.get_context().open_stage(args.scene)
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        app.update()
    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())          # 씬 파일은 건드리지 않는다
    for prim in stage.Traverse():                          # ROS 그래프는 이 시험에 필요 없다
        if prim.GetTypeName() == "OmniGraph":
            prim.SetActive(False)

    # 시험 트레이: 씬 옆 assets 의 검사용 양배추 트레이를 벨트 방향(긴 변 = x)으로 놓는다
    asset = Path(args.scene).parent / "assets" / "cabbage_pallet_6" / "cabbage_pallet_6_inspect.usd"
    sx, sy = (float(v) for v in args.start.split(","))
    tray = UsdGeom.Xform.Define(stage, TEST)
    tray.AddTranslateOp().Set(Gf.Vec3d(sx, sy, BELT_TOP + PALLET_ORIGIN_ABOVE_BOTTOM))
    tray.AddRotateZOp().Set(90.0)
    tray.GetPrim().GetReferences().AddReference(str(asset))
    for i, c in enumerate(args.pattern.split(","), start=1):
        head = stage.GetPrimAtPath(f"{TEST}/root_001/Cabbage_{i:02d}")
        head.GetVariantSets().GetVariantSet("condition").SetVariantSelection(COLOURS[c.strip().upper()])
    if args.head_mass:
        from pxr import UsdPhysics as _UP
        for i in range(1, 7):
            _UP.MassAPI.Apply(stage.GetPrimAtPath(f"{TEST}/root_001/Cabbage_{i:02d}")).CreateMassAttr().Set(float(args.head_mass))
    if args.no_instance or args.head_friction:
        for p in Usd.PrimRange(stage.GetPrimAtPath(TEST)):
            if p.IsInstanceable():
                p.SetInstanceable(False)
    if args.head_friction:
        # 새 물리 재질을 만들어 포기 루트에 '하위보다 강하게' 묶는다 (에셋 안의 재질 결합을 덮어씀)
        from pxr import UsdPhysics as _UP, UsdShade as _US
        fs, fd = (float(v) for v in args.head_friction.split(","))
        mat = _US.Material.Define(stage, "/World/TestHeadFriction")
        api = _UP.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr().Set(fs)
        api.CreateDynamicFrictionAttr().Set(fd)
        api.CreateRestitutionAttr().Set(0.0)
        n = 0
        for i in range(1, 7):
            head = stage.GetPrimAtPath(f"{TEST}/root_001/Cabbage_{i:02d}")
            for p in Usd.PrimRange(head):
                if p.HasAPI(_UP.CollisionAPI):
                    _US.MaterialBindingAPI.Apply(p).Bind(mat, _US.Tokens.strongerThanDescendants, "physics")
                    n += 1
        print(f"[test] head friction {fs}/{fd} bound to {n} collider prims", flush=True)
        res["head_friction"] = [fs, fd]
        res["head_friction_colliders"] = n

    world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60, physics_prim_path="/physicsScene")
    from conveyor import install as install_conveyor
    import inspection_cull_station
    S = inspection_cull_station
    if args.grip_force:
        S.GRIPPER_DRIVE = (S.GRIPPER_DRIVE[0], S.GRIPPER_DRIVE[1], float(args.grip_force))
    if args.truth_labels:
        S.BAD_FRAME_RETRY = 0
        _CMAP = {"green": "lettuce_dark_green", "yellow": "lettuce_yellow", "brown": "lettuce_brown"}

        def _truth_assign(self):
            tray_z = S._world_matrix(self._stage, str(self._tray.GetPath()))[2, 3]
            out_ = {}
            for h in self._heads:
                hp = S._world_matrix(self._stage, str(h.GetPath()))[:3, 3]
                if hp[2] < tray_z:
                    out_[h.GetName()] = ("UNKNOWN", None)
                    continue
                aim = hp.copy()
                aim[2] = tray_z + S.AIM_ABOVE_TRAY
                out_[h.GetName()] = (_CMAP.get(self._truth(h), "UNKNOWN"), aim)
            return out_
        S.VisionCullStation._assign = _truth_assign

    conveyor = install_conveyor(stage, [TEST], vision_x=-0.69, auto_resume=None)
    station = inspection_cull_station.install(stage, world, REPO / "M0609", out)

    if args.contacts:
        from pxr import PhysxSchema, UsdPhysics
        for p in Usd.PrimRange(stage.GetPrimAtPath(TEST)):
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr().Set(0.0)
    snap = None
    if args.snap:
        import omni.replicator.core as rep
        snap = rep.AnnotatorRegistry.get_annotator("rgb")
        snap.attach([rep.create.render_product(args.snap, (960, 540))])

    world.reset()
    conveyor.attach()
    station.attach(conveyor)
    world.play()

    t0, steps, released = time.time(), 0, None
    while app.is_running() and steps < args.max_seconds * 60:
        world.step(render=(snap is not None or steps % 3 == 0))
        conveyor.update(1 / 60)
        station.update(1 / 60)
        steps += 1
        if snap is not None and steps % 60 == 0:
            data = snap.get_data()
            if data is not None and getattr(data, "size", 0):
                from PIL import Image
                Image.fromarray(np.asarray(data)[..., :3]).save(str(out / f"snap_{steps // 60:04d}.jpg"), quality=85)
        if args.contacts and steps % 30 == 0:
            from omni.physx import get_physx_simulation_interface
            from pxr import PhysicsSchemaTools
            headers, data = get_physx_simulation_interface().get_contact_report()
            for h in headers:
                a = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
                b = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
                if TEST in a and TEST in b:
                    continue
                key = " | ".join(sorted((str(PhysicsSchemaTools.intToSdfPath(h.collider0)),
                                         str(PhysicsSchemaTools.intToSdfPath(h.collider1)))))
                rec = res.setdefault("contacts", {}).setdefault(key, {"first_t": round(steps / 60, 1), "n": 0})
                rec["n"] += 1
                rec["last_t"] = round(steps / 60, 1)
        if conveyor.fault and "fault" not in res:
            res["fault"] = list(conveyor.fault)
            print("[test] conveyor fault:", conveyor.fault, flush=True)
        if station.results and released is None:
            released = steps
        if released is not None and steps - released > 5 * 60:   # 배출되어 떠나는 모습 5초
            break
    tray_body = UsdGeom.Xformable(stage.GetPrimAtPath(TEST + "/Cube_011_001"))
    res.update({"grip_force": float(S.GRIPPER_DRIVE[2]), "head_mass": args.head_mass or 0.3, "truth_labels": args.truth_labels,
                "cull_retry": S.CULL_RETRY, "vision_only": S.VISION_ONLY, "aim_above_tray": S.AIM_ABOVE_TRAY})
    res.update({"sim_seconds": round(steps / 60, 1), "wall_seconds": round(time.time() - t0, 1),
                "station": station.results,
                "tray_final": [round(v, 3) for v in tray_body.ComputeLocalToWorldTransform(0).ExtractTranslation()],
                "zone_final": str(conveyor.zone_of(TEST))})
    station.close()
except Exception:
    res["exception"] = traceback.format_exc()
    print(res["exception"], flush=True)
(out / "result.json").write_text(json.dumps(res, indent=1, ensure_ascii=False), encoding="utf-8")
app.close()
