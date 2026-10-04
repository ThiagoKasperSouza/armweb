#!/usr/bin/env python3
"""Repair the mangled _load_dae body in load_mesh.py (indentation fix)."""
import re
import sys

PATH = "/home/thiag/armweb/ws/tools/load_mesh.py"
src = open(PATH).read()

start = src.index("def _load_dae(path):")
end = src.index("def _load_trimesh(path):")

body = '''def _load_dae(path):
    """Read a COLLADA file into a flat triangle soup."""
    from collada import Collada

    doc = Collada(path)
    geometries = getattr(doc, "geometries", None)
    if not geometries:
        return None

    positions, normals, indices = [], [], []
    for geo in geometries:
        try:
            for prim in getattr(geo, "primitives", []):
                _emit_primitive(prim, positions, normals, indices)
        except Exception as exc:
            print(f"[load_mesh] skipped a geometry: {exc}", file=sys.stderr)

    if not positions or not indices:
        return None
    if any(n is None for n in normals):
        computed = compute_normals(positions, indices)
        normals = [c if n is None else n for n, c in zip(normals, computed)]
    return positions, [_unit(n) for n in normals], indices


'''

open(PATH, "w").write(src[:start] + body + src[end:])
print("repaired _load_dae")