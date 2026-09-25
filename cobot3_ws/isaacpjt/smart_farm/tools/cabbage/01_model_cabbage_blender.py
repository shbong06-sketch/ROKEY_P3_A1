"""Procedural green cabbage heads (visual only) for Isaac Sim 5.1.

Run headless:
  blender -b --factory-startup -P 01_model_cabbage_blender.py -- OUT_DIR [HEAD_SCALE]

Writes OUT_DIR/cabbage_heads.blend (editable source) and OUT_DIR/cabbage_heads_visual.usd
with three variants /cabbage_heads/Cabbage_A|B|C, each made of three meshes:
  core   - tightly wrapped inner head (UV sphere, layered leaf-edge ridges)
  leaves - 6 outer wrapper leaves (single-sided patches, midrib + veins, flared tips)
  stem   - cut stem stub
Units: metres, Z up, head bottom (stem cut) at z = 0, head axis on +Z.
Physics is NOT authored here (see 02_build_cabbage_pallet.py).
"""
import bpy, bmesh, math, random, sys, os
from mathutils import Vector, noise

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
OUT_DIR = argv[0] if argv else os.path.dirname(os.path.abspath(__file__))
os.makedirs(OUT_DIR, exist_ok=True)

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 1.0

VARIANTS = {"A": 11, "B": 23, "C": 37}   # name -> seed
Z_FLAT = 0.006                          # flattened underside of the head (stem sticks out below it)
# Overall size. The head must pass between the open OnRobot RG2 pads (99.6 mm measured in Isaac Sim,
# rg2_measure.py) with room for ~8 mm TCP error, so the widest leaf tip stays <= ~90 mm and the grip band ~78 mm (mini cabbage).
HEAD_SCALE = float(argv[1]) if len(argv) > 1 else 0.60


def smoothstep(e0, e1, x):
    t = min(max((x - e0) / (e1 - e0), 0.0), 1.0)
    return t * t * (3 - 2 * t)


def ang_diff(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def ellip(az, el, rx, rz, cz):
    return Vector((rx * math.cos(el) * math.cos(az), rx * math.cos(el) * math.sin(az), cz + rz * math.sin(el)))


def get_mat(name, rgb):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.diffuse_color = (*rgb, 1.0)
    return m


def new_obj(name, bm, mat, parent):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    for p in me.polygons:
        p.use_smooth = True
    me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    scene.collection.objects.link(ob)
    ob.parent = parent
    return ob


def build(tag, seed):
    rnd = random.Random(seed)
    RX = 0.0535 + rnd.uniform(-0.0015, 0.0015)   # head radius  (~0.13 m across with outer leaves)
    RZ = 0.049 + rnd.uniform(-0.002, 0.002)      # half height  (~0.11 m tall)
    CZ = Z_FLAT + RZ * 0.93
    nofs = Vector((rnd.uniform(0, 50), rnd.uniform(0, 50), rnd.uniform(0, 50)))

    root = bpy.data.objects.new("Cabbage_" + tag, None)
    scene.collection.objects.link(root)
    root.scale = (HEAD_SCALE,) * 3

    # ---------------- core: layered inner leaves ----------------
    inner = []
    for k in range(9):
        inner.append(dict(a=rnd.uniform(0, 2 * math.pi), w=rnd.uniform(0.7, 1.25),
                          e=rnd.uniform(0.35, 1.25), h=rnd.uniform(0.0008, 0.0016)))
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=48, v_segments=28, radius=1.0, calc_uvs=True)
    for v in bm.verts:
        d = v.co.normalized()
        az, el = math.atan2(d.y, d.x), math.asin(max(-1.0, min(1.0, d.z)))
        off = 0.0
        for L in inner:                                     # each wrapped leaf adds one ridge step
            du = abs(ang_diff(az, L["a"])) / L["w"]
            edge = L["e"] - 0.55 * du * du                   # rounded leaf top edge
            off += L["h"] * smoothstep(0.0, 0.06, edge - el) * smoothstep(1.0, 0.85, du)
        off += 0.0012 * noise.noise(d * 4.0 + nofs)
        p = ellip(az, el, RX - 0.004 + off, RZ - 0.004 + off, CZ)
        p.z = max(p.z, Z_FLAT)
        v.co = p
    new_obj("core_" + tag, bm, get_mat("M_Cabbage_Core", (0.74, 0.83, 0.52)), root)

    # ---------------- outer wrapper leaves ----------------
    NU, NV = 21, 20
    n_leaf = 6
    bm = bmesh.new()
    uv_layer = bm.loops.layers.uv.new("UVMap")
    for k in range(n_leaf):
        a0 = k * 2 * math.pi / n_leaf + rnd.uniform(-0.25, 0.25)
        W = rnd.uniform(0.95, 1.25)                          # angular half width
        el0, el1 = -1.30, rnd.uniform(0.15, 0.85)            # how far up the head the leaf wraps
        gap = 0.0015 + 0.0012 * (k % 3)
        flare = rnd.uniform(0.003, 0.011)                    # outward curl of the leaf top
        curl = rnd.uniform(0.002, 0.006)                     # edge curl
        lofs = Vector((rnd.uniform(0, 90), k * 7.3, 3.1))
        grid = []
        for j in range(NV):
            t = j / (NV - 1)
            hw = W * (0.25 + 0.75 * math.sin(math.pi * (0.10 + 0.55 * t))) * math.sqrt(max(0.0, 1.0 - t ** 3)) + 0.02  # ovate, rounded tip
            row = []
            for i in range(NU):
                u = -1.0 + 2.0 * i / (NU - 1)
                az = a0 + u * hw
                el = el0 + (el1 - el0) * t
                off = gap + flare * t ** 2.5 + curl * abs(u) ** 3 * t
                off += 0.0016 * max(0.0, 1.0 - abs(u) / 0.10) * (1.0 - 0.6 * t)       # midrib
                f = ((t + 0.55 * abs(u)) * 7.0) % 1.0                                   # chevron side veins
                off += 0.0005 * max(0.0, 1.0 - min(f, 1.0 - f) / 0.12) * (abs(u) > 0.1)
                off += 0.0025 * noise.noise(Vector((u * 2.5, t * 3.0, 0)) + lofs) * abs(u) ** 2   # ruffled edge
                p = ellip(az, el, RX + off, RZ + off, CZ)
                p.z = max(p.z, Z_FLAT - 0.0005)
                row.append((bm.verts.new(p), (0.5 + 0.5 * u, t)))
            grid.append(row)
        for j in range(NV - 1):
            for i in range(NU - 1):
                quad = (grid[j][i], grid[j][i + 1], grid[j + 1][i + 1], grid[j + 1][i])
                f = bm.faces.new([q[0] for q in quad])
                for loop, q in zip(f.loops, quad):
                    loop[uv_layer].uv = q[1]
    bm.normal_update()
    new_obj("leaves_" + tag, bm, get_mat("M_Cabbage_Leaf", (0.42, 0.62, 0.38)), root)

    # ---------------- stem stub ----------------
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=16, radius1=0.0115, radius2=0.0135, depth=Z_FLAT + 0.006,
                          calc_uvs=True)
    bmesh.ops.translate(bm, verts=bm.verts, vec=Vector((0, 0, (Z_FLAT + 0.006) / 2)))
    new_obj("stem_" + tag, bm, get_mat("M_Cabbage_Stem", (0.86, 0.88, 0.70)), root)
    return root


offset_x = 0.0
roots = []
for tag, seed in VARIANTS.items():
    r = build(tag, seed)
    r.location.x = offset_x          # spread out in the .blend only; reset before export
    offset_x += 0.2
    roots.append(r)

bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUT_DIR, "cabbage_heads.blend"))

for r in roots:
    r.location.x = 0.0
bpy.context.view_layer.update()
kw = dict(filepath=os.path.join(OUT_DIR, "cabbage_heads_visual.usda"), selected_objects_only=False,
          export_materials=True, generate_preview_surface=True, export_uvmaps=True, export_normals=True,
          evaluation_mode="RENDER", root_prim_path="/cabbage_heads")
try:
    bpy.ops.wm.usd_export(**kw)
except TypeError as e:          # parameter names differ between Blender versions
    print("usd_export fallback:", e)
    kw.pop("root_prim_path"); kw.pop("generate_preview_surface")
    bpy.ops.wm.usd_export(**kw)
for ob in bpy.data.objects:
    if ob.type == "MESH":
        print("MESH", ob.name, len(ob.data.vertices), "verts", len(ob.data.polygons), "faces",
              "dims", tuple(round(x, 4) for x in ob.dimensions))
print("DONE", OUT_DIR)
