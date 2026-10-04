"""Probe how pycollada actually exposes node transforms and geometry.

Answers, for a given .dae:
  * which scene attribute holds the root nodes
  * whether node.objects('geometry') composes the parent chain itself
  * what the composed matrix looks like

Run with a DAE path as the only argument.
"""
import sys

from collada import Collada

path = sys.argv[1]
doc = Collada(path)

scene = doc.scene
print(f"file       : {path}")
print(f"scene type : {type(scene).__name__}")
print(f"has .node  : {hasattr(scene, 'node')}")
print(f"has .nodes : {hasattr(scene, 'nodes')}")

roots = list(getattr(scene, "nodes", []) or [])
print(f"scene roots: {[r.name for r in roots]}")

print("\n=== geometry via node.objects('geometry') ===")
for r in roots:
    print(f"  root {r.name!r}: local matrix")
    for row in r.matrix.tolist():
        print("   ", [round(v, 4) for v in row])
    try:
        items = list(r.objects("geometry"))
    except Exception as exc:  # pragma: no cover
        print(f"    objects() failed: {exc}")
        continue
    for bg in items:
        print(f"    object type: {type(bg).__name__}")
        print(f"      attrs: "
              f"{[a for a in dir(bg) if not a.startswith('_')]}")
        geom = bg.original
        print(f"      original geometry: {getattr(geom, 'name', None)}")
        mat = getattr(bg, "matrix", None)
        if mat is not None:
            print("      bound matrix:")
            for row in mat.tolist():
                print("       ", [round(v, 4) for v in row])
        verts = []
        for prim in bg.primitives:
            src = getattr(prim, "vertex", None)
            if src is None:
                continue
            for v in src:
                verts.append((float(v[0]), float(v[1]), float(v[2])))
        if verts:
            lo = tuple(min(v[i] for v in verts) for i in range(3))
            hi = tuple(max(v[i] for v in verts) for i in range(3))
            print(f"      raw bbox lo={tuple(round(x, 4) for x in lo)}"
                  f" hi={tuple(round(x, 4) for x in hi)}")
            if mat is not None:
                tl = mat.tolist()
                moved = tuple(
                    sum(tl[r][c] * verts[0][c] for c in range(3)) + tl[r][3]
                    for r in range(3))
                print(f"      first vertex after transform: "
                      f"{tuple(round(x, 4) for x in moved)}")