"""Remove the Nova Carter's never-shown internal parts from its layer (file size 193 MB -> small).

  D:\\isaacsim\\python.bat run_in_isaac.py 12_slim_carter_layer.py SRC_nova_carter_sim_optimized.usd DST.usd

/chassis_link/internal_components (1.52 M points) sits inside the closed body shell and the v013 scene never composes
it (it is not rendered, has no collider). Everything else in the layer is kept as is.
"""
import sys
from pxr import Sdf

src, dst = sys.argv[1], sys.argv[2]
layer = Sdf.Layer.FindOrOpen(src)
out = Sdf.Layer.CreateNew(dst) if not Sdf.Layer.Find(dst) else Sdf.Layer.Find(dst)
out.TransferContent(layer)
removed = []
for path in ("/chassis_link/internal_components",):
    spec = out.GetPrimAtPath(path)
    if spec is not None:
        del out.GetPrimAtPath(Sdf.Path(path).GetParentPath()).nameChildren[spec.name]
        removed.append(path)
out.Save()
open(dst + ".slim.txt", "w").write("removed: " + ", ".join(removed) + "\n")
