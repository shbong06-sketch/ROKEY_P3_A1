"""Re-save the builder's .usda layers as .usd with Isaac Sim 5.1's own USD, then check them.

  D:\\isaacsim\\python.bat 03_finalize_isaac.py BUILD_DIR OUT_DIR REPORT_JSON

Checks per file: default prim / units / up axis, every rigid body has mass + colliders, no nested rigid bodies,
colliders are purpose=guide with a physics material, convex colliders <= 64 verts, visual material bindings
resolve through the instanced prototypes, textures resolve. Then runs the Isaac Sim asset validator rules when
the omni.asset_validator extension is available.
"""
import sys, os, json, shutil, traceback
BUILD, OUT, REPORT = sys.argv[1], sys.argv[2], sys.argv[3]
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
report = {}
try:
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Ar
    os.makedirs(os.path.join(OUT, "textures"), exist_ok=True)
    for f in os.listdir(os.path.join(BUILD, "textures")):
        shutil.copy2(os.path.join(BUILD, "textures", f), os.path.join(OUT, "textures", f))
    shutil.copy2(os.path.join(BUILD, "build_info.json"), os.path.join(OUT, "build_info.json"))
    for name in ("cabbage_pallet_6", "cabbage_pallet_6_inspect", "cabbage_pallet_empty", "cabbage_A", "cabbage_B", "cabbage_C"):
        src = os.path.join(BUILD, name + ".usda")
        dst = os.path.join(OUT, name + ".usd")
        Sdf.Layer.FindOrOpen(src).Export(dst)
        st = Usd.Stage.Open(dst)
        r = {"defaultPrim": str(st.GetDefaultPrim().GetPath()), "upAxis": UsdGeom.GetStageUpAxis(st),
             "metersPerUnit": UsdGeom.GetStageMetersPerUnit(st), "kgPerUnit": UsdPhysics.GetStageKilogramsPerUnit(st),
             "kind": Usd.ModelAPI(st.GetDefaultPrim()).GetKind(), "problems": []}
        prob = r["problems"]
        it = Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate))
        prims = list(it)
        bodies = [p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        r["rigid_bodies"] = len(bodies)
        for b in bodies:
            anc = b.GetParent()
            while anc and anc.GetPath() != Sdf.Path.absoluteRootPath:
                if anc.HasAPI(UsdPhysics.RigidBodyAPI):
                    prob.append("nested rigid body " + str(b.GetPath()))
                anc = anc.GetParent()
            m = UsdPhysics.MassAPI(b).GetMassAttr().Get()
            if not m or m <= 0:
                prob.append("no mass " + str(b.GetPath()))
            cols = [p for p in Usd.PrimRange(b) if p.HasAPI(UsdPhysics.CollisionAPI)]
            if not cols:
                prob.append("no collider " + str(b.GetPath()))
        cols = [p for p in prims if p.HasAPI(UsdPhysics.CollisionAPI)]
        r["colliders"] = len(cols)
        for c in cols:
            if UsdGeom.Imageable(c).ComputePurpose() != UsdGeom.Tokens.guide:
                prob.append("collider not guide " + str(c.GetPath()))
            pm, _ = UsdShade.MaterialBindingAPI(c).ComputeBoundMaterial("physics")
            if not pm or not pm.GetPrim().HasAPI(UsdPhysics.MaterialAPI):
                prob.append("no physics material " + str(c.GetPath()))
            if c.IsA(UsdGeom.Mesh) and len(UsdGeom.Mesh(c).GetPointsAttr().Get()) > 64:
                prob.append("hull > 64 verts " + str(c.GetPath()))
        vis = [p for p in prims if p.IsA(UsdGeom.Mesh) and not p.HasAPI(UsdPhysics.CollisionAPI)]
        r["visual_meshes"] = len(vis)
        r["visual_tris"] = int(sum(sum(n - 2 for n in UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get()) for p in vis))
        tex = set()
        for p in vis:
            mat, _ = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()
            if not mat:
                prob.append("unbound visual " + str(p.GetPath()))
                continue
            for sh in Usd.PrimRange(mat.GetPrim()):
                a = sh.GetAttribute("inputs:file")
                if a and a.Get():
                    tex.add(a.Get().resolvedPath or ("UNRESOLVED " + a.Get().path))
        r["textures"] = sorted(tex)
        prob += ["texture not found " + t for t in tex if t.startswith("UNRESOLVED")]
        bb = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(st.GetDefaultPrim()).ComputeAlignedRange()
        r["visual_bbox"] = [list(bb.GetMin()), list(bb.GetMax())]
        # Isaac Sim asset validator (omni.asset_validator), if the extension can be enabled headless
        try:
            from isaacsim.core.utils.extensions import enable_extension
            enable_extension("omni.asset_validator.core")
            app.update()
            import omni.asset_validator.core as av
            engine = av.ValidationEngine(init_rules=True)
            res = engine.validate(dst)
            issues = [str(i) for i in res.issues()]
            r["asset_validator_issues"] = issues[:60]
            r["asset_validator_issue_count"] = len(issues)
            by = {}
            for s in issues:
                kind = "INTERNAL_CHECKER_CRASH" if "Uncaught error" in s else "REAL"
                rule = s.split("rule ")[1].split(":")[0] if "rule " in s else s[:40]
                by[kind + " " + rule] = by.get(kind + " " + rule, 0) + 1
            r["asset_validator_by_rule"] = by
            r["asset_validator_real_issues"] = [s for s in issues if "Uncaught error" not in s][:40]
        except Exception as e:
            r["asset_validator"] = "unavailable: %s" % e
        report[name] = r
except Exception:
    report["error"] = traceback.format_exc()
open(REPORT, "w").write(json.dumps(report, indent=1))
app.close()
