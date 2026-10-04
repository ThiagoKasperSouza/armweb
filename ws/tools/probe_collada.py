#!/usr/bin/env python3
"""Probe the pycollada API against a real franka DAE file."""
import sys
from collada import Collada

path = sys.argv[1]
doc = Collada(path)
print("class:", type(doc).__name__)
print("top-level attrs:", [a for a in dir(doc) if not a.startswith("_")])
print()
for name in ("library_geometries", "meshes", "scenes", "scene"):
    v = getattr(doc, name, "MISSING")
    if isinstance(v, dict):
        print(f"  {name}: dict with {len(v)} keys -> {list(v)[:3]}")
    elif isinstance(v, list):
        print(f"  {name}: list[{len(v)}]")
    else:
        print(f"  {name}: {type(v).__name__}")

"""Find which geometry/primitive in a DAE breaks pycollada."""
import sys
import traceback

from collada import Collada

path = sys.argv[1]
doc = Collada(path)

for gi, geo in enumerate(doc.geometries):
    print(f"\ngeo[{gi}] {geo.__class__.__name__} "
          f"nprims={len(geo.primitives)}")
    for pi, prim in enumerate(geo.primitives):
        print(f"  prim[{pi}] {prim.__class__.__name__} "
              f"nvertex={len(prim.vertex)} nindex={len(prim.index)} "
              f"normal={0 if prim.normal is None else len(prim.normal)}")
        try:
            fl = prim.index
            is_pairs = len(fl[0]) == 2
            print(f"    is_pairs={is_pairs} shape={getattr(fl, 'shape', '?')}")
            n = 0
            for tri in fl:
                for p in tri:
                    _ = float(prim.vertex[int(p[0])][0])
                n += 1
                if n > 200:
                    break
            print("    indexing first 200 tris OK")
        except Exception:
            print("    FAIL:")
            traceback.print_exc()
        break
    if gi >= 3:
        break
if geo_lib is None:
    print("\nNO doc.geometries")
    sys.exit(0)

print("\n  geometries type:", type(geo_lib).__name__)
items = list(geo_lib.items()) if hasattr(geo_lib, "items") else list(enumerate(geo_lib))
print("  count:", len(items))

for name, geo in items[:1]:
    print(f"\ngeometry {name!r}: {geo.__class__.__name__}")
    print("  geom attrs:",
          [a for a in dir(geo) if not a.startswith("_")])
    for pi, prim in enumerate(geo.primitives):
        print(f"\n  prim[{pi}] {prim.__class__.__name__}")
        print("    attrs:", [a for a in dir(prim) if not a.startswith("_")])
        for attr in ("v", "vertices", "f", "triangles", "index",
                     "normal", "normalindex", "__len__"):
            if not hasattr(prim, attr):
                continue
            try:
                val = getattr(prim, attr)
            except Exception as exc:
                print(f"    {attr}: error {exc}")
                continue
            if callable(val):
                continue
            try:
                ln = len(val)
            except TypeError:
                ln = "n/a"
            print(f"    {attr}: type={type(val).__name__} len={ln}")
            if attr in ("v", "f", "triangles") and ln not in ("n/a", 0):
                print(f"      first = {val[0]}")
        break