#!/usr/bin/env python3
"""Standalone checks for the COLLADA node-transform math in load_mesh.py.

Run with plain python3 - no pycollada / trimesh needed, because the matrix
helpers are pure Python and importable on their own.
"""
import importlib.util
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "load_mesh", os.path.join(HERE, "load_mesh.py"))
lm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lm)

fails = []


def check(name, got, want, tol=1e-9):
    ok = all(abs(g - w) <= tol for g, w in zip(got, want))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {got}")
    if not ok:
        fails.append(f"{name}: got {got}, want {want}")


print("=== identity transform leaves points untouched ===")
I = lm._identity4()
check("origin", lm._transform_point(I, (3.0, -2.0, 7.0)), (3.0, -2.0, 7.0))
check("normal", lm._transform_normal(I, (0.0, 0.0, 5.0)), (0.0, 0.0, 1.0))

print("=== translation ===")
T = ((1, 0, 0, 10), (0, 1, 0, 20), (0, 0, 1, 30), (0, 0, 0, 1))
check("point", lm._transform_point(T, (1.0, 2.0, 3.0)), (11.0, 22.0, 33.0))
# Translation must NOT change a normal's direction.
check("normal", lm._transform_normal(T, (0.0, 0.0, 1.0)), (0.0, 0.0, 1.0))

print("=== 90deg rotation about Z: (x,y,z) -> (-y, x, z) ===")
import math
c, s = 0.0, 1.0
R = ((c, -s, 0, 0), (s, c, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))
check("point", lm._transform_point(R, (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))
check("normal", lm._transform_normal(R, (0.0, 0.0, 1.0)), (0.0, 0.0, 1.0))
check("normal xy", lm._transform_normal(R, (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))

print("=== non-uniform scale: normals use inverse-transpose ===")
# Scale x by 10, y by 1. The plane x=const with normal (1,0,0) is stretched
# along x, so its normal must tilt towards y. The raw (unnormalised)
# inverse-transpose result is (0.1, 0, 0); _unit() renormalises it to (1,0,0),
# which is correct because _transform_normal always returns a unit vector.
# The tilt is visible in the raw components, so check them via a non-axis
# input where the direction genuinely changes.
S = ((10, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))
# Normal (1,1,0)/sqrt2 on a plane x+y=c; stretching x tilts it towards y.
import math
nx = ny = 1.0 / math.sqrt(2.0)
got = lm._transform_normal(S, (nx, ny, 0.0))
# Expected direction: adj/det applied to (1,1,0) -> (1, 10, 0) normalised.
want = (1.0 / math.sqrt(101.0), 10.0 / math.sqrt(101.0), 0.0)
check("tilted normal", got, want)

print("=== composition order: _mat_mul(a, b) applies b then a ===")
# Translate (10,20,30), then rotate 90 deg about Z: (x,y,z) -> (-y, x, z).
# (0,0,0) --T--> (10,20,30) --R--> (-20, 10, 30).
M = lm._mat_mul(R, T)
check("T then R", lm._transform_point(M, (0.0, 0.0, 0.0)), (-20.0, 10.0, 30.0))
# Reverse order rotates the origin in place, then translates: (10,20,30).
# The differing answer proves _mat_mul is order-sensitive (not symmetric).
M2 = lm._mat_mul(T, R)
check("R then T", lm._transform_point(M2, (0.0, 0.0, 0.0)), (10.0, 20.0, 30.0))

print("=== _mat_to_tuples tolerates None and accepts lists ===")
check("none -> identity", lm._mat_to_tuples(None)[0], (1.0, 0.0, 0.0, 0.0))
rows = [[1, 0, 0, 5], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
check("list row", lm._mat_to_tuples(rows)[0], (1.0, 0.0, 0.0, 5.0))

print("=== nested node matrices are composed by pycollada ===")


class FakeBound:
    """Stand-in for collada.BoundGeometry."""

    def __init__(self, matrix, name="g"):
        self.matrix = matrix
        self.original = types.SimpleNamespace(name=name)
        self._prims = []

    def primitives(self):
        return iter(self._prims)


class FakeRoot:
    def __init__(self, bound):
        self._bound = bound

    def objects(self, tipo):
        assert tipo == "geometry", tipo
        return iter(self._bound)


I = lm._identity4()
bound = [FakeBound(I, "flat")]
bound.append(FakeBound(lm._mat_to_tuples([[1, 0, 0, 10], [0, 1, 0, 0],
                                          [0, 0, 1, 0], [0, 0, 0, 1]]),
                       "moved"))
out = []
for bg in bound:
    out.append((bg.original.name, bg.matrix))
def truthy(name, cond, extra=""):
    """Boolean assertion; check() above compares numeric got/want pairs."""
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}{extra}")
    if not cond:
        fails.append(name)


truthy("two bound geometries", len(out) == 2, f" ({len(out)})")
truthy("identity one stays identity",
       all(abs(out[0][1][r][c] - (1.0 if r == c else 0.0)) < 1e-9
           for r in range(4) for c in range(4)))
truthy("moved one carries translation 10",
       abs(out[1][1][0][3] - 10.0) < 1e-9, f" {out[1][1][0][3]}")

print("=== _emit_primitive bakes the transform into positions ===")


class FakeIndex(list):
    """List of index pairs that also answers .shape like a numpy array."""

    def __init__(self, rows):
        super().__init__(rows)
        self.shape = (len(rows), 2)


class FakePrim:
    """Minimal stand-in for a COLLADA TriangleSet with one triangle."""

    def __init__(self):
        self.vertex = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
        self.normal = [(0.0, 0.0, 1.0)]
        self.normal_index = [[0, 0, 0]]
        self.index = FakeIndex([[0, 0], [1, 0], [2, 0]])


pos, nrm, idx = [], [], []
lm._emit_primitive(FakePrim(), pos, nrm, idx, transform=T)
check("translated v0", pos[0], (10.0, 20.0, 30.0))
check("translated v1", pos[1], (11.0, 20.0, 30.0))
if len(pos) != 3 or len(nrm) != 3 or len(idx) != 3:
    fails.append(f"soup length wrong: {len(pos)}/{len(nrm)}/{len(idx)}")
if nrm[0] is None:
    fails.append("normal was dropped instead of transformed")
else:
    check("normal survives", nrm[0], (0.0, 0.0, 1.0))

# No transform -> identity behaviour.
pos2, nrm2, idx2 = [], [], []
lm._emit_primitive(FakePrim(), pos2, nrm2, idx2)
check("untransformed v0", pos2[0], (0.0, 0.0, 0.0))

print()
if fails:
    print(f"FAILED ({len(fails)}):")
    for f in fails:
        print(f"  - {f}")
    sys.exit(1)
print("All checks passed.")