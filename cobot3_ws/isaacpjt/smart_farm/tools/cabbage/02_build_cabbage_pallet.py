"""Physics-ready cabbage assets for Isaac Sim 5.1 (PhysX), built from the Blender visual heads.

  python 02_build_cabbage_pallet.py VISUAL_USDA OUT_DIR
  (needs numpy, scipy, manifold3d, pillow, usd-core; writes .usda text layers that 03_finalize_isaac.py
   re-saves as .usd with Isaac Sim's own USD)

Outputs in OUT_DIR:
  cabbage_pallet_6.usda   6-slot tray + 6 cabbages (same hierarchy as romaine_pallet_6_v005:
                          /<root>/Cube_011_001 = tray body, /<root>/root_001/Cabbage_0N = heads)
  cabbage_pallet_empty.usda  the tray alone
  cabbage_A|B|C.usda      one free cabbage per shape (root = rigid body)
  textures/*.png

Physics layout (Isaac Sim 5.1 asset structure / SimReady rules):
  * root Xform = defaultPrim, kind=component, Z up, 1 m/unit, 1 kg/unit, no RigidBodyAPI on the root
  * every rigid body is its own Xform (tray, each head) with RigidBodyAPI + MassAPI(mass only; PhysX
    derives COM/inertia from the colliders)
  * visuals and colliders are separate prims; colliders are purpose=guide
  * tray: compound of Cube primitives + 16 convex wedges per slot that form an exact conical seat
  * head: 5 convex pieces (<= 64 verts each, the GPU hull limit): base / seat (matches the tray cone) /
    lower / grip band (vertical cylinder at the equator for the RG2 pads) / upper
  * physics materials bound with the "physics" purpose
"""
import os, sys, json
import numpy as np
import manifold3d as mf
from scipy.spatial import ConvexHull
from scipy.ndimage import gaussian_filter
from PIL import Image
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Vt, Kind

SRC, OUT = sys.argv[1], sys.argv[2]
os.makedirs(os.path.join(OUT, "textures"), exist_ok=True)

# ---------------------------------------------------------------- parameters
HEAD_MASS = 0.30            # kg, ~88 mm mini cabbage (ellipsoid volume x 0.9 g/cm3)
PALLET_MASS = 1.0           # kg, same as romaine_pallet_6_v005 (fork transfer tuning)
FRICTION = (0.8, 0.6)       # static, dynamic - same as romaine_pallet_6_v005 PhysicsMaterial
HOLE_R = 0.026              # seat rim radius on the deck top (m)
CONE_H = 0.008              # conical part of the seat; below it the hole is a straight R_BOT bore
CLEAR = 0.0003              # initial head <-> seat gap
SEG = 16                    # facets of seat cone / head lathe (wedges align with head facets)
BAND_HALF = 0.012           # grip band half height (m)
# RG2 grasp robustness (measured with 05_m0609_pick_place_test.py): without a contact offset the closing pads
# tunnel into the band before contacts exist; 4 mm + 64 position iterations held 6/6 slots at an 8 N.m finger drive
HEAD_CONTACT_OFFSET = 0.004
HEAD_POS_ITERS = 64
HEAD_MAX_DEPEN = 10.0
SLOT_YAW = [0, 150, 70, 250, 20, 200]      # visual yaw per slot (deg), heads are rotationally symmetric colliders
SLOT_VARIANT = ["A", "B", "C", "A", "B", "C"]

# tray envelope = romaine_pallet_6_v005 as placed in the v011/v013 scenes (scale 0.6, 1, 1) baked to scale 1
X1, Y1 = 0.2102097 * 0.6, 0.276125
Z0, Z1 = -0.02592913, 0.054070868
END_WALL_Y = 0.2577
T_SIDE, T_BOT, T_TOP = 0.006, 0.005, 0.012
WIN_Y = (0.0105, 0.2353)          # side windows the fork tines enter through
WIN_Z = (-0.0059, 0.0338)
HOLES = [(sx * 0.125 * 0.6, y) for y in (-0.189, 0.0, 0.189) for sx in (-1, 1)]
CELL_X, CELL_Y = 0.045, 0.060     # half size of the square cell around each slot (x limited by the side wall)
ZT0 = Z1 - T_TOP
assert HOLES[1][0] + CELL_X <= X1 - T_SIDE + 1e-4

# ---------------------------------------------------------------- read visual heads
src = Usd.Stage.Open(SRC)
xc = UsdGeom.XformCache()
heads = {}
for var in ("A", "B", "C"):
    root = src.GetPrimAtPath("/cabbage_heads/Cabbage_" + var)
    parts = {}
    for p in Usd.PrimRange(root):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        M = np.array(xc.GetLocalToWorldTransform(p), dtype=np.float64)
        P = np.array(m.GetPointsAttr().Get(), dtype=np.float64)
        P = P @ M[:3, :3] + M[3, :3]
        N = m.GetNormalsAttr().Get()
        N = None if N is None else np.array(N, dtype=np.float64) @ np.linalg.inv(M[:3, :3]).T
        if N is not None:
            N /= np.linalg.norm(N, axis=1, keepdims=True)
        st = UsdGeom.PrimvarsAPI(p).GetPrimvar("st")
        uv = np.array(st.Get(), dtype=np.float64) if st and st.HasValue() else None
        parts[p.GetName().split("_")[0]] = dict(P=P, fvc=np.array(m.GetFaceVertexCountsAttr().Get()),
                                                fvi=np.array(m.GetFaceVertexIndicesAttr().Get()), N=N,
                                                N_interp=m.GetNormalsInterpolation(), uv=uv)
    heads[var] = parts

# ---------------------------------------------------------------- head profile (convex hull sections)
def profile(pts):
    hull = ConvexHull(pts)
    e = set()
    for s in hull.simplices:
        for a, b in ((s[0], s[1]), (s[1], s[2]), (s[0], s[2])):
            e.add((min(a, b), max(a, b)))
    E = np.array(sorted(e)); A, B = pts[E[:, 0]], pts[E[:, 1]]
    def section(hz):
        m = (np.minimum(A[:, 2], B[:, 2]) <= hz) & (np.maximum(A[:, 2], B[:, 2]) >= hz) & (A[:, 2] != B[:, 2])
        t = (hz - A[m, 2]) / (B[m, 2] - A[m, 2])
        return A[m, :2] + t[:, None] * (B[m, :2] - A[m, :2])
    return section, pts[:, 2].min(), pts[:, 2].max()

def bisect(f, lo, hi, target, n=50):
    for _ in range(n):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if f(mid) < target else (lo, mid)
    return (lo + hi) / 2

geo = {}
for var, parts in heads.items():
    pts = np.vstack([p["P"] for p in parts.values()])
    section, zmin, zmax = profile(pts)
    axis = section(zmin + 0.01).mean(0)                          # stem / head axis
    for p in parts.values():                                     # centre the head on its axis
        p["P"][:, :2] -= axis
    pts[:, :2] -= axis
    section, zmin, zmax = profile(pts)
    rmean = lambda hz, s=section: float(np.linalg.norm(s(hz), axis=1).mean()) if len(s(hz)) else 0.0
    zs = np.linspace(zmin + 0.002, zmax - 0.002, 200)
    rs = np.array([rmean(z) for z in zs])
    z_eq = float(zs[rs.argmax()]); r_eq = float(rs.max()) + 0.0005
    h_seat = bisect(rmean, zmin + 0.001, z_eq, HOLE_R)          # where the hull is HOLE_R wide
    geo[var] = dict(zmin=zmin, zmax=zmax, rmean=rmean, z_eq=z_eq, r_eq=r_eq, h_seat=h_seat,
                    width=float(2 * np.linalg.norm(pts[:, :2], axis=1).max()))
    print("head %s: height %.1f mm, max width %.1f mm, equator z %.1f mm (r %.1f), seat z %.1f mm (embed %.1f mm)" % (
        var, (zmax - zmin) * 1000, geo[var]["width"] * 1000, z_eq * 1000, r_eq * 1000, h_seat * 1000, (h_seat - zmin) * 1000))

# one tray for all variants: bore radius from the narrowest head under its own seat
R_BOT = min(g["rmean"](g["h_seat"] - CONE_H) for g in geo.values())
assert R_BOT > 0.004
print("seat: rim r %.1f mm -> r %.1f mm over %.1f mm, bore below" % (HOLE_R * 1000, R_BOT * 1000, CONE_H * 1000))
# fork tines run at |y| in [0.054, 0.081] below WIN_Z[1] through the middle row: heads must stay above the windows
for var, g in geo.items():
    bottom = Z1 - (g["h_seat"] - g["zmin"]) + CLEAR
    assert bottom > WIN_Z[1] + 0.001 or g["rmean"](g["zmin"] + (WIN_Z[1] - bottom)) < 0.034, var

# ---------------------------------------------------------------- mesh helpers
def hull_mesh(pts):
    pts = np.asarray(pts, dtype=np.float64)
    h = ConvexHull(pts)
    tris = [s if np.dot(np.cross(pts[s[1]] - pts[s[0]], pts[s[2]] - pts[s[0]]), eq[:3]) > 0 else s[::-1]
            for s, eq in zip(h.simplices, h.equations)]
    used = np.unique(np.array(tris))
    remap = -np.ones(len(pts), int); remap[used] = np.arange(len(used))
    return pts[used], remap[np.array(tris)]

ANG = np.arange(SEG) * 2 * np.pi / SEG

def lathe(rings, tip=None):
    pts = [(r * np.cos(t), r * np.sin(t), z) for z, r in rings for t in ANG]
    if tip is not None:
        pts.append((0.0, 0.0, tip))
    return hull_mesh(pts)

def head_proxies(g):
    zmin, hs, rmean = g["zmin"], g["h_seat"], g["rmean"]
    zb = hs - CONE_H
    band_lo, band_hi = g["z_eq"] - BAND_HALF, g["z_eq"] + BAND_HALF
    base = [(zmin + 1e-4, min(rmean(zmin + 1e-4), R_BOT - 0.0005)), ((zmin + zb) / 2, min(rmean((zmin + zb) / 2), R_BOT - 0.0005)),
            (zb, R_BOT - 0.0005)]
    if zb - zmin < 0.002:
        base = None
    # lower body stops 1 mm under the band and 4 mm inside it: a ledge, so RG2 fingertips that dip below the
    # band find no sloped face to wedge the head down into its seat
    lower = [(hs, HOLE_R), ((hs + band_lo) / 2, min(max(rmean((hs + band_lo) / 2), HOLE_R), g["r_eq"] - 0.004)),
             (band_lo - 0.001, g["r_eq"] - 0.004)]
    top = g["zmax"] - 1e-4
    upper_z = [band_hi + f * (top - 0.004 - band_hi) for f in (0.0, 0.45, 0.8)]
    upper = [(z, min(rmean(z), g["r_eq"])) for z in upper_z]
    upper[0] = (band_hi, g["r_eq"])
    out = {"collision_seat": lathe([(zb, R_BOT), (hs, HOLE_R)]),          # identical to the tray cone
           "collision_lower": lathe(lower),
           "collision_grip": lathe([(band_lo, g["r_eq"]), (band_hi, g["r_eq"])]),   # vertical band for RG2 pads
           "collision_upper": lathe(upper, tip=top)}
    if base:
        out["collision_base"] = lathe(base)
    for k, (pp, tt) in out.items():
        assert len(pp) <= 64, (k, len(pp))
    return out

# ---------------------------------------------------------------- textures
rng = np.random.default_rng(7)
def smooth_noise(h, w, sigma):
    n = gaussian_filter(rng.standard_normal((h, w)), sigma, mode="wrap")
    return n / (np.abs(n).max() + 1e-9)

def srgb(img):
    return Image.fromarray((np.clip(img, 0, 1) * 255 + 0.5).astype(np.uint8))

H = W = 512
v, u = np.mgrid[0:H, 0:W] / (H - 1.0)
v = 1.0 - v                                  # image row 0 = v 1 (USD st origin bottom-left)
au = np.abs(u - 0.5) * 2                     # 0 midrib .. 1 leaf edge
base_c = np.array([0.80, 0.87, 0.66]); tip_c = np.array([0.42, 0.63, 0.38]); edge_c = np.array([0.33, 0.52, 0.30])
t = np.clip(v ** 0.8, 0, 1)[..., None]
leaf = base_c * (1 - t) + tip_c * t
leaf = leaf * (1 - 0.35 * (au ** 3)[..., None]) + edge_c * 0.35 * (au ** 3)[..., None]
mid_w = 0.045 * (1 - 0.7 * v)
leaf = np.where((au < mid_w)[..., None], leaf * 0.35 + np.array([0.90, 0.93, 0.80]) * 0.65, leaf)
f = ((v + 0.55 * au) * 7.0) % 1.0
vein = np.clip(1 - np.minimum(f, 1 - f) / 0.035, 0, 1) * (au > mid_w) * (1 - 0.6 * au)
leaf = leaf * (1 - 0.35 * vein[..., None]) + np.array([0.86, 0.91, 0.74]) * 0.35 * vein[..., None]
leaf *= (1 + 0.06 * smooth_noise(H, W, 6))[..., None]
srgb(leaf).save(os.path.join(OUT, "textures", "cabbage_leaf_albedo.png"))

core = np.array([0.72, 0.84, 0.52]) * (1 - 0.25 * v[..., None]) + np.array([0.86, 0.90, 0.66]) * 0.25 * v[..., None]
g2 = ((u * 22 + 0.8 * smooth_noise(H, W, 20)) % 1.0)
core_vein = np.clip(1 - np.minimum(g2, 1 - g2) / 0.05, 0, 1) * (0.3 + 0.7 * (1 - v))
core = core * (1 - 0.18 * core_vein[..., None]) + np.array([0.90, 0.93, 0.78]) * 0.18 * core_vein[..., None]
core *= (1 + 0.05 * smooth_noise(H, W, 5))[..., None]
srgb(core).save(os.path.join(OUT, "textures", "cabbage_core_albedo.png"))

# Vision conditions (team YOLO romaine3_v012_640sq_yolo11n colour classes; Inspection_Node 설계정리: green NORMAL,
# yellow HOLD, brown DEFECT). The head keeps its modelled two-tone look - veined green outer leaves around a pale
# round core - in every condition:
#   green  : the original albedo textures, unchanged
#   yellow / brown : the same textures tinted with UsdUVTexture scale so the outer-leaf mean albedo equals the team
#            colour exactly and the core stays a lighter shade of it
CONDITIONS = {"green": None, "yellow": (0.85, 0.72, 0.12), "brown": (0.38, 0.22, 0.08)}
CORE_LIGHTER = 1.25
def lin_mean(img):   # mean linear albedo of an sRGB-encoded image
    c = np.clip(img, 0, 1)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4).reshape(-1, 3).mean(0)
LEAF_MEAN, CORE_MEAN = lin_mean(leaf), lin_mean(core)
def cond_scale(cond, part):
    col = CONDITIONS[cond]
    if col is None:
        return None
    target = np.array(col) * (CORE_LIGHTER if part == "core" else 1.0)
    return tuple(np.clip(target, 0, 1) / (CORE_MEAN if part == "core" else LEAF_MEAN))
print("natural mean albedo (linear): leaf", LEAF_MEAN.round(3), "core", CORE_MEAN.round(3))

# ---------------------------------------------------------------- tray solid (visual)
def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))

EXT = 0.001
SLOPE = (HOLE_R - R_BOT) / CONE_H
solid = box(-X1, X1, -Y1, Y1, Z0, Z1)
solid -= box(-X1 + T_SIDE, X1 - T_SIDE, -END_WALL_Y, END_WALL_Y, Z0 + T_BOT, ZT0)
for sx in (-1, 1):
    for y0, y1 in ((WIN_Y[0], WIN_Y[1]), (-WIN_Y[1], -WIN_Y[0])):
        xa = sx * (X1 + 0.01); xb = sx * (X1 - T_SIDE - 0.001)
        solid -= box(min(xa, xb), max(xa, xb), y0, y1, WIN_Z[0], WIN_Z[1])
for hx, hy in HOLES:
    solid -= mf.Manifold.cylinder(CONE_H + EXT, R_BOT, HOLE_R + SLOPE * EXT, 128).translate((hx, hy, Z1 - CONE_H))
    solid -= mf.Manifold.cylinder(T_TOP - CONE_H + 2 * EXT, R_BOT, R_BOT, 128).translate((hx, hy, ZT0 - EXT))
assert solid.status() == mf.Error.NoError
mesh = solid.to_mesh()
PV = np.asarray(mesh.vert_properties)[:, :3].astype(np.float64)
PT = np.asarray(mesh.tri_verts).astype(np.int64)
tri = PV[PT]
fn = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); fn /= np.linalg.norm(fn, axis=1, keepdims=True)
ax_ = np.abs(fn).argmax(1)
PUV = np.array([tri[k, j, [i for i in range(3) if i != ax_[k]]] / 0.5 for k in range(len(PT)) for j in range(3)])
print("tray: %d verts %d tris, genus %d, bbox %s %s" % (len(PV), len(PT), solid.genus(), PV.min(0).round(4), PV.max(0).round(4)))

# ---------------------------------------------------------------- USD authoring
def new_stage(path, root_name):
    st = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(st, 1.0)
    root = UsdGeom.Xform.Define(st, "/" + root_name).GetPrim()
    st.SetDefaultPrim(root)
    Usd.ModelAPI(root).SetKind(Kind.Tokens.component)
    st.GetRootLayer().customLayerData = {"generator": "smartfarm-sim/scripts/cabbage/02_build_cabbage_pallet.py",
                                         "isaacsim": "5.1.0"}
    return st, root

def preview_material(st, path, color=None, tex=None, rough=0.5, tint=None):
    mat = UsdShade.Material.Define(st, path)
    sh = UsdShade.Shader.Define(st, path + "/PreviewSurface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(0.4)
    if tex:
        rd = UsdShade.Shader.Define(st, path + "/stReader")
        rd.CreateIdAttr("UsdPrimvarReader_float2")
        rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tx = UsdShade.Shader.Define(st, path + "/Albedo")
        tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("./textures/" + tex))
        tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        if tint:   # per-channel multiplier on the decoded (linear) texture -> condition colour
            tx.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(float(tint[0]), float(tint[1]), float(tint[2]), 1.0))
        tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
        rgb = tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    else:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat

def physics_material(st, path, sf, df):
    mat = UsdShade.Material.Define(st, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr().Set(sf); api.CreateDynamicFrictionAttr().Set(df)
    api.CreateRestitutionAttr().Set(0.0)
    return mat

def looks(st, root):
    lk = root.GetPath().AppendChild("Looks")
    UsdGeom.Scope.Define(st, lk)
    return dict(
        pallet=preview_material(st, str(lk) + "/M_Pallet", color=(0.62, 0.45, 0.24), rough=0.75),
        phys_pallet=physics_material(st, str(lk) + "/PhysicsMaterial_Pallet", *FRICTION),
        phys_cabbage=physics_material(st, str(lk) + "/PhysicsMaterial_Cabbage", *FRICTION))

def rigid_body(prim, mass, label, grasped=False):
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.CreateRigidBodyEnabledAttr().Set(True)
    rb.CreateKinematicEnabledAttr().Set(False)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(mass)
    if grasped:   # PhysxRigidBodyAPI (not in usd-core): stiffer contact solve against the RG2 finger drive
        prim.AddAppliedSchema("PhysxRigidBodyAPI")
        prim.CreateAttribute("physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(HEAD_POS_ITERS)
        prim.CreateAttribute("physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(4)
        prim.CreateAttribute("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float).Set(HEAD_MAX_DEPEN)
    prim.AddAppliedSchema("SemanticsLabelsAPI:class")                     # Isaac Sim 5 UsdSemantics
    prim.CreateAttribute("semantics:labels:class", Sdf.ValueTypeNames.TokenArray).Set(Vt.TokenArray([label]))

def mesh_prim(st, path, P, fvc, fvi, N=None, N_interp="faceVarying", uv=None, mat=None, double=False):
    m = UsdGeom.Mesh.Define(st, path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(P, np.float32)))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([int(x) for x in fvc]))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([int(x) for x in fvi]))
    m.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*np.min(P, 0)), Gf.Vec3f(*np.max(P, 0))]))
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateDoubleSidedAttr(double)
    if N is not None:
        m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(N, np.float32)))
        m.SetNormalsInterpolation(N_interp)
    if uv is not None:
        pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        uq, inv = np.unique(np.round(np.asarray(uv, np.float64), 6), axis=0, return_inverse=True)   # indexed primvar
        pv.Set(Vt.Vec2fArray.FromNumpy(uq.astype(np.float32)))
        pv.SetIndices(Vt.IntArray([int(i) for i in inv.reshape(-1)]))
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    return m

def collider(prim, phys_mat, approx=None, contact_offset=None):
    UsdGeom.Imageable(prim).CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr().Set(True)
    if contact_offset:   # PhysxCollisionAPI: contacts are generated before the fast-closing RG2 pads reach the band
        prim.AddAppliedSchema("PhysxCollisionAPI")
        prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(contact_offset)
        prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
    if approx:
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set(approx)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(phys_mat, UsdShade.Tokens.weakerThanDescendants, "physics")

def core_uv(parts):
    c = parts["core"]
    P = c["P"][c["fvi"]]
    az = (np.arctan2(P[:, 1], P[:, 0]) / (2 * np.pi)) % 1.0
    zz = (P[:, 2] - P[:, 2].min()) / np.ptp(P[:, 2])
    # fix the seam: faces straddling az 0/1 get consistent u
    uv = np.stack([az, zz], 1)
    k = 0
    for n in c["fvc"]:
        us = uv[k:k + n, 0]
        if us.max() - us.min() > 0.5:
            us[us < 0.5] += 1.0
        k += n
    return uv

def write_prototypes(st, root_path, only=None):
    """Visual-only head prototypes: abstract class prims under <root>/_prototypes, instanced by every head.
    Materials live inside each prototype so bindings survive the internal reference remapping."""
    holder = UsdGeom.Scope.Define(st, root_path.AppendChild("_prototypes")).GetPrim()
    holder.SetSpecifier(Sdf.SpecifierClass)          # abstract: never rendered / simulated, only referenced
    for var, parts in heads.items():
        if only and var != only:
            continue
        for cond, col in CONDITIONS.items():     # one prototype per shape x condition (instancing keeps it cheap)
            proto = UsdGeom.Xform.Define(st, holder.GetPath().AppendChild("Cabbage_%s_%s" % (var, cond))).GetPrim()
            lk = str(proto.GetPath()) + "/Looks"
            UsdGeom.Scope.Define(st, lk)
            stem_col = (0.86, 0.88, 0.70) if col is None else tuple(np.clip(np.array(col) * 1.3, 0, 1))
            L = dict(leaf=preview_material(st, lk + "/M_Cabbage_Leaf", tex="cabbage_leaf_albedo.png", rough=0.45,
                                           tint=cond_scale(cond, "leaf")),
                     core=preview_material(st, lk + "/M_Cabbage_Core", tex="cabbage_core_albedo.png", rough=0.5,
                                           tint=cond_scale(cond, "core")),
                     stem=preview_material(st, lk + "/M_Cabbage_Stem", color=stem_col, rough=0.6))
            for name, key, dbl in (("leaves", "leaf", True), ("core", "core", False), ("stem", "stem", False)):
                p = parts[name]
                uv = core_uv(parts) if name == "core" else p["uv"]
                x = UsdGeom.Xform.Define(st, proto.GetPath().AppendChild(name))       # mesh under its own Xform (instancing rule)
                mesh_prim(st, x.GetPath().AppendChild("mesh"), p["P"], p["fvc"], p["fvi"], p["N"], p["N_interp"], uv, L[key], dbl)


def add_condition_variants(st, body, geo_path, proto_base, default):
    """variantSet 'condition' (green / yellow / brown) on a head body: swaps the instanced visual prototype and the
    'condition' semantic label; physics is identical in every variant."""
    vs = body.GetVariantSets().AddVariantSet("condition")
    for cond in CONDITIONS:
        vs.AddVariant(cond)
        vs.SetVariantSelection(cond)
        with vs.GetVariantEditContext():
            st.OverridePrim(geo_path).GetReferences().AddInternalReference(Sdf.Path(proto_base + "_" + cond))
            body.AddAppliedSchema("SemanticsLabelsAPI:condition")
            body.CreateAttribute("semantics:labels:condition", Sdf.ValueTypeNames.TokenArray).Set(Vt.TokenArray([cond]))
            body.CreateAttribute("farm:condition", Sdf.ValueTypeNames.String).Set(cond)
    vs.SetVariantSelection(default)

def write_head(st, parent, name, var, L, yaw=0.0, proto_root=None, cond="green"):
    body = UsdGeom.Xform.Define(st, parent.AppendChild(name))
    rigid_body(body.GetPrim(), HEAD_MASS, "cabbage", grasped=True)
    body.GetPrim().CreateAttribute("farm:crop", Sdf.ValueTypeNames.String).Set("cabbage")
    body.GetPrim().CreateAttribute("farm:variant", Sdf.ValueTypeNames.String).Set(var)
    geo_x = UsdGeom.Xform.Define(st, body.GetPath().AppendChild("geo"))
    geo_x.AddRotateZOp().Set(float(yaw))
    geo_x.GetPrim().SetInstanceable(True)
    add_condition_variants(st, body.GetPrim(), geo_x.GetPath(), str(proto_root.AppendPath("_prototypes/Cabbage_" + var)), cond)
    for cname, (pp, tt) in head_proxies(geo[var]).items():
        m = mesh_prim(st, body.GetPath().AppendChild(cname), pp, [3] * len(tt), tt.reshape(-1))
        collider(m.GetPrim(), L["phys_cabbage"], UsdPhysics.Tokens.convexHull, HEAD_CONTACT_OFFSET)
    return body

def write_tray(st, root, L):
    body = UsdGeom.Xform.Define(st, root.GetPath().AppendChild("Cube_011_001"))
    rigid_body(body.GetPrim(), PALLET_MASS, "pallet")
    normals = np.repeat(fn, 3, axis=0)
    mesh_prim(st, body.GetPath().AppendChild("Cube_011_001"), PV, [3] * len(PT), PT.reshape(-1), normals, "faceVarying",
              PUV, L["pallet"])
    col = UsdGeom.Scope.Define(st, body.GetPath().AppendChild("collision")).GetPath()
    n = [0]
    def cbox(x0, x1, y0, y1, z0, z1):
        c = UsdGeom.Cube.Define(st, col.AppendChild("box_%02d" % n[0])); n[0] += 1
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
        c.AddScaleOp().Set(Gf.Vec3f(x1 - x0, y1 - y0, z1 - z0))
        c.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(-0.5), Gf.Vec3f(0.5)]))
        collider(c.GetPrim(), L["phys_pallet"])
    cbox(-X1, X1, -Y1, Y1, Z0, Z0 + T_BOT)                                     # bottom deck
    for sy in (-1, 1):                                                         # end walls
        cbox(-X1, X1, *sorted((sy * END_WALL_Y, sy * Y1)), Z0, Z1)
    for sx in (-1, 1):                                                         # side walls with tine windows
        xa, xb = sorted((sx * X1, sx * (X1 - T_SIDE)))
        cbox(xa, xb, -END_WALL_Y, END_WALL_Y, Z0, WIN_Z[0])
        cbox(xa, xb, -END_WALL_Y, END_WALL_Y, WIN_Z[1], ZT0)
        for y0, y1 in ((-END_WALL_Y, -WIN_Y[1]), (-WIN_Y[0], WIN_Y[0]), (WIN_Y[1], END_WALL_Y)):
            cbox(xa, xb, y0, y1, WIN_Z[0], WIN_Z[1])
    xs = sorted({-X1, X1} | {hx + d for hx, _ in HOLES for d in (-CELL_X, CELL_X)})
    ys = sorted({-END_WALL_Y, END_WALL_Y} | {hy + d for _, hy in HOLES for d in (-CELL_Y, CELL_Y)})
    for y0, y1 in zip(ys[:-1], ys[1:]):                                        # top deck minus slot cells
        run = None
        for x0, x1 in zip(xs[:-1], xs[1:]):
            inside = any(abs((x0 + x1) / 2 - hx) < CELL_X and abs((y0 + y1) / 2 - hy) < CELL_Y for hx, hy in HOLES)
            if not inside:
                run = (run[0], x1) if run else (x0, x1)
            elif run:
                cbox(run[0], run[1], y0, y1, ZT0, Z1); run = None
        if run:
            cbox(run[0], run[1], y0, y1, ZT0, Z1)
    def to_cell(th):                                                            # ray from slot centre to the cell border
        return min(CELL_X / max(abs(np.cos(th)), 1e-9), CELL_Y / max(abs(np.sin(th)), 1e-9))
    corners = [np.arctan2(sy * CELL_Y, sx * CELL_X) % (2 * np.pi) for sx in (-1, 1) for sy in (-1, 1)]
    nw = 0
    for hi, (hx, hy) in enumerate(HOLES):                                      # convex wedges = conical seat + bore
        for k in range(SEG):
            t0, t1 = ANG[k], ANG[k] + 2 * np.pi / SEG
            pts = []
            for r, z in ((HOLE_R, Z1), (R_BOT, Z1 - CONE_H), (R_BOT, ZT0)):
                pts += [(hx + r * np.cos(t), hy + r * np.sin(t), z) for t in (t0, t1)]
            for t in [t0, t1] + [c for c in corners if t0 < c < t1]:
                d = to_cell(t)
                pts += [(hx + d * np.cos(t), hy + d * np.sin(t), z) for z in (Z1, ZT0)]
            pp, tt = hull_mesh(pts)
            m = mesh_prim(st, col.AppendChild("hole%d_w%02d" % (hi, k)), pp, [3] * len(tt), tt.reshape(-1))
            collider(m.GetPrim(), L["phys_pallet"], UsdPhysics.Tokens.convexHull); nw += 1
    print("tray colliders: %d boxes + %d wedges" % (n[0], nw))
    return body

def head_origin_z(var):
    return Z1 - geo[var]["h_seat"] + CLEAR

# --- 1) tray + 6 cabbages: rack version (all green) and inspection version (team default slot pattern,
#        Inspection_Node 설계정리: SLOT_01 green, 02 yellow, 03 brown, 04 green, 05 yellow, 06 green)
PALLET_FILES = {"cabbage_pallet_6": ["green"] * 6,
                "cabbage_pallet_6_inspect": ["green", "yellow", "brown", "green", "yellow", "green"]}
for fname, conds in PALLET_FILES.items():
    st, root = new_stage(os.path.join(OUT, fname + ".usda"), "cabbage_pallet_6")
    L = looks(st, root)
    write_prototypes(st, root.GetPath())
    write_tray(st, root, L)
    grp = UsdGeom.Xform.Define(st, root.GetPath().AppendChild("root_001")).GetPath()
    slots = []
    for i, ((hx, hy), var, yaw, cond) in enumerate(zip(HOLES, SLOT_VARIANT, SLOT_YAW, conds)):
        b = write_head(st, grp, "Cabbage_%02d" % (i + 1), var, L, yaw, root.GetPath(), cond)
        b.AddTranslateOp().Set(Gf.Vec3d(hx, hy, head_origin_z(var)))
        slots.append(dict(name="Cabbage_%02d" % (i + 1), slot_xy=[hx, hy], variant=var, origin_z=head_origin_z(var)))
    st.GetRootLayer().Save()

# --- 2) empty tray
st2, root2 = new_stage(os.path.join(OUT, "cabbage_pallet_empty.usda"), "cabbage_pallet_empty")
L2 = looks(st2, root2)
write_tray(st2, root2, L2)
st2.GetRootLayer().Save()

# --- 3) single free cabbages, one file per shape (root = rigid body, origin at the stem cut)
for var in ("A", "B", "C"):
    st3, cab = new_stage(os.path.join(OUT, "cabbage_%s.usda" % var), "cabbage")
    L3 = looks(st3, cab)
    write_prototypes(st3, cab.GetPath(), only=var)
    rigid_body(cab, HEAD_MASS, "cabbage", grasped=True)
    cab.CreateAttribute("farm:crop", Sdf.ValueTypeNames.String).Set("cabbage")
    cab.CreateAttribute("farm:variant", Sdf.ValueTypeNames.String).Set(var)
    g = UsdGeom.Xform.Define(st3, "/cabbage/geo")
    g.GetPrim().SetInstanceable(True)
    add_condition_variants(st3, cab, g.GetPath(), "/cabbage/_prototypes/Cabbage_" + var, "green")
    for cname, (pp, tt) in head_proxies(geo[var]).items():
        m = mesh_prim(st3, "/cabbage/" + cname, pp, [3] * len(tt), tt.reshape(-1))
        collider(m.GetPrim(), L3["phys_cabbage"], UsdPhysics.Tokens.convexHull, HEAD_CONTACT_OFFSET)
    st3.GetRootLayer().Save()

info = dict(head_mass=HEAD_MASS, pallet_mass=PALLET_MASS, friction=FRICTION, hole_rim_r=HOLE_R, bore_r=R_BOT,
            cone_h=CONE_H, tray_bbox=[PV.min(0).tolist(), PV.max(0).tolist()], deck_top_z=Z1,
            heads={k: {kk: (vv if not callable(vv) else None) for kk, vv in g.items() if kk != "rmean"} for k, g in geo.items()},
            slots=slots, grip_band_half=BAND_HALF)
json.dump(info, open(os.path.join(OUT, "build_info.json"), "w"), indent=1)
print("wrote", OUT)
