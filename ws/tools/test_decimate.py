#!/usr/bin/env python3
"""Unit tests for the two export bugs found in the FR3 render.

Bug 1 -- decimate() splits the index stream at arbitrary positions instead of
at triangle boundaries. With `indices[::step]` and step=4, the slice starts at
index 0 but lands mid-triangle, so each emitted triangle is built from
vertices that are not a face. That is exactly what a long thin spike looks
like: a cone shooting off the arm.

Bug 2 -- decimate() returns a mesh whose positions list is rebuilt from the
indices that survived, but the caller checks `if positions and indices`. If the
slicing drops every index (small meshes, large step) the link silently
disappears from the GLB with no error at all -- which is why fr3_link0/6/7/8
rendered as nothing.

Run: python3 ws/tools/test_decimate.py
"""
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "export_gltf", os.path.join(HERE, "export_gltf.py"))
ex = importlib.util.module_from_spec(spec)
try:
    spec.loader.exec_module(ex)
except Exception as exc:  # pragma: no cover
    print(f"could not import export_gltf: {exc}")
    sys.exit(2)

fails = []


def check(name, cond, extra=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}{extra}")
    if not cond:
        fails.append(name)


def make_grid(n_quads):
    """A simple n_quads-wide strip of quads (2 triangles each)."""
    positions = []
    for i in range(n_quads + 1):
        positions.append((float(i), 0.0, 0.0))
        positions.append((float(i), 1.0, 0.0))
    indices = []
    for i in range(n_quads):
        a, b = 2 * i, 2 * i + 1
        c, d = 2 * i + 2, 2 * i + 3
        indices += [a, b, c, b, d, c]
    normals = [(0.0, 0.0, 1.0)] * len(positions)
    return positions, normals, indices


print("=== indices stay 16-bit safe ===")
pos, nrm, idx = make_grid(40000)
print(f"  input: {len(pos)} positions, {len(idx)} indices")
out_p, out_n, out_i = ex.decimate(pos, nrm, idx, 65000)
check("output fits in UNSIGNED_SHORT", len(out_p) <= 65535,
      f" ({len(out_p)})")
check("no dangling indices", all(0 <= i < len(out_p) for i in out_i))

print("\n=== BUG 1: triangles must not be cut across boundaries ===")
pos, nrm, idx = make_grid(40000)
out_p, out_n, out_i = ex.decimate(pos, nrm, idx, 65000)
# Every emitted triangle must be a real face of the original strip, i.e. its
# vertices must have been within one quad of each other in x.
xs = [p[0] for p in out_p]
bad = 0
for t in range(0, len(out_i) - 2, 3):
    txs = [xs[out_i[t]], xs[out_i[t + 1]], xs[out_i[t + 2]]]
    if max(txs) - min(txs) > 1.5:
        bad += 1
check("no stretched triangles", bad == 0, f" ({bad} bad of {len(out_i)//3})")
check("output non-empty", len(out_i) > 0, f" ({len(out_i)} indices)")

print("\n=== BUG 2: a small mesh must never vanish ===")
for nq in (2, 3, 5, 8):
    pos, nrm, idx = make_grid(nq)
    op, on, oi = ex.decimate(pos, nrm, idx, 65000)
    ok = len(op) > 0 and len(oi) >= 3
    check(f"strip of {nq} quads survives", ok,
          f" -> {len(op)} pos, {len(oi)} idx")

print("\n=== decimate is a no-op when already small ===")
pos, nrm, idx = make_grid(5)
op, on, oi = ex.decimate(pos, nrm, idx, 65000)
check("untouched positions", len(op) == len(pos))
check("untouched indices", oi == idx)

print("\n=== remap is monotonic and covers every used vertex ===")
pos, nrm, idx = make_grid(40000)
op, on, oi = ex.decimate(pos, nrm, idx, 65000)
check("indices are a permutation of 0..n-1",
      sorted(set(oi)) == list(range(len(op))),
      f" ({len(set(oi))} used of {len(op)})")

print()
if fails:
    print(f"FAILED ({len(fails)}): " + ", ".join(fails))
    sys.exit(1)
print("All decimate checks passed.")