#!/usr/bin/env python3
"""Verify load_mesh honours COLLADA node transforms on real DAE files.

Uses the synthetic files from make_test_dae.py:

  identity.dae  geometry at the origin, node transform is identity.
                _load_dae must leave it at the origin (no-op).
  nested.dae    geometry under a node translated to (10, 0, 0).
                _load_dae must move it there. Without the scene walk it
                would stay at the origin -- which is exactly the FR3 bug.

Requires pycollada.
"""
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "load_mesh", os.path.join(HERE, "load_mesh.py"))
lm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lm)

try:
    from collada import Collada
except ImportError:
    print("pycollada not installed; run: python3 -m pip install pycollada")
    sys.exit(2)

fails = []


def check(name, cond, extra=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}{extra}")
    if not cond:
        fails.append(name)


def bbox(positions):
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    zs = [p[2] for p in positions]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


print("=== identity.dae: transform is identity, geometry stays put ===")
p = os.path.join(HERE, "identity.dae")
soup = lm._load_dae(p)
check("loaded", soup is not None)
if soup:
    pos = soup[0]
    lo, hi = bbox(pos)
    check("box spans the unit cube",
          all(abs(a - b) < 1e-6 for a, b in
              zip((lo[0], hi[0]), (0.0, 1.0))) and
          all(abs(a - b) < 1e-6 for a, b in
              zip((lo[1], hi[1]), (0.0, 1.0))),
          f" x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}]")
    # A triangle soup duplicates every corner: 12 triangles -> 36 positions.
    check("36 corners / 36 indices (12 triangles)",
          len(pos) == 36 and len(soup[2]) == 36, f" {len(pos)}/{len(soup[2])}")

print("\n=== nested.dae: node walk must move the box to x=10 ===")
p = os.path.join(HERE, "nested.dae")
soup = lm._load_dae(p)
check("loaded", soup is not None)
if soup:
    pos = soup[0]
    lo, hi = bbox(pos)
    check("x range is 10..11 (transform applied)",
          abs(lo[0] - 10.0) < 1e-6 and abs(hi[0] - 11.0) < 1e-6,
          f" x[{lo[0]:.3f},{hi[0]:.3f}]")
    check("y range still 0..1",
          abs(lo[1]) < 1e-6 and abs(hi[1] - 1.0) < 1e-6,
          f" y[{lo[1]:.3f},{hi[1]:.3f}]")
    check("normals still unit length",
          all(abs(sum(c * c for c in n) - 1.0) < 1e-6 for n in soup[1]))

    # The decisive assertion: without the node walk this box sits at 0..1.
    check("NOT left at the origin (the bug)", lo[0] > 5.0,
          f" lo.x={lo[0]:.3f}")

print("\n=== node walk reports the translation it applied ===")
doc = Collada(p)
roots = list(getattr(doc.scene, "nodes", []) or [])
check("scene.nodes is the root list", len(roots) == 1, f" {len(roots)}")
found = []
for r in roots:
    found.extend(r.objects("geometry"))
check("objects('geometry') yields the box", len(found) == 1, f" {len(found)}")
if found:
    bg = found[0]
    mat = bg.matrix.tolist()
    check("BoundGeometry.matrix has x translation 10",
          abs(mat[0][3] - 10.0) < 1e-9, f" {mat[0][3]}")
    check("BoundGeometry has primitives() method",
          callable(getattr(bg, "primitives", None)))

print()
if fails:
    print(f"FAILED ({len(fails)}): " + ", ".join(fails))
    sys.exit(1)
print("All DAE node-transform checks passed.")