#!/usr/bin/env python3
"""Regression tests for the three bugs behind the broken FR3 render.

Bug A -- the exporter wrote UNSIGNED_SHORT indices unconditionally, capping
every mesh at 65535 vertices. The FR3 links are bigger than that, so each was
run through decimate(), which dropped two thirds of its triangles and left the
arm looking shredded. Fix: weld the soup, then emit UNSIGNED_INT when needed.

Bug B -- bake_animation swapped the quaternion components a second time.
read_animation_from_usda already emits (x, y, z, w), so the extra swap turned
the identity (0,0,0,1) into (0,0,1,0): a 180 deg flip about Z on every joint at
frame 0. Fix: pass the samples straight through.

Bug C -- prismatic joints ignored their origin_rpy. FR3's fr3_finger_joint2 is
prismatic with rpy=(0,0,pi), so the right finger landed 180 deg from its socket.

Run: python3 ws/tools/test_gltf_quality.py
"""
import importlib.util
import json
import math
import os
import struct
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "export_gltf", os.path.join(HERE, "export_gltf.py"))
ex = importlib.util.module_from_spec(spec)
try:
    spec.loader.exec_module(ex)
except Exception as exc:  # pragma: no cover
    print(f"could not import export_gltf: {exc}")
    sys.exit(2)

sys.path.insert(0, HERE)
from load_mesh import weld

fails = []


def check(name, cond, extra=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}{extra}")
    if not cond:
        fails.append(name)


def ang_between(a, b):
    """Angle in degrees between two quaternions (sign-insensitive)."""
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2 * math.acos(min(1.0, dot)))


# --------------------------------------------------------------- Bug A: weld
print("=== BUG A: welding collapses the soup without losing triangles ===")
W = 40  # W x W quad grid, emitted as a soup
pos, nrm, idx = [], [], []
for i in range(W):
    for j in range(W):
        p = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)]
        for a, b, c in ((0, 1, 2), (0, 2, 3)):
            for v in (a, b, c):
                pos.append((float(p[v][0]), float(p[v][1]), 0.0))
                nrm.append((0.0, 0.0, 1.0))
                idx.append(len(pos) - 1)
soup_tris = len(idx) // 3

wp, wn, wi = weld(pos, nrm, idx)
check("triangle count preserved", len(wi) // 3 == soup_tris,
      f" ({soup_tris} -> {len(wi) // 3})")
check("vertices collapsed", len(wp) < len(pos), f" ({len(pos)} -> {len(wp)})")
check("expected grid size", len(wp) == (W + 1) * (W + 1),
      f" ({len(wp)} vs {(W + 1) ** 2})")
check("indices in range", all(0 <= i < len(wp) for i in wi))
check("no unused vertices", sorted(set(wi)) == list(range(len(wp))))

print("\n=== BUG A: hard edges are NOT welded away ===")
# Two corners at the same position but different normals must stay separate,
# otherwise welding silently smooths the shading of the CAD mesh.
hp = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
hn = [(0.0, 0.0, 1.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)]
ap, an, ai = weld(hp, hn, [0, 1, 2])
check("differing normals stay split", len(ap) == 3, f" ({len(ap)} verts)")

print("\n=== BUG A: a big mesh gets 32-bit indices, not a decimate ===")
big_n = 100000
big_pos = [(float(i % 1000), float(i // 1000), 0.0) for i in range(big_n)]
big_nrm = [(0.0, 0.0, 1.0)] * big_n
b = ex.GltfBuilder()
mi = b.add_mesh(big_pos, big_nrm, list(range(big_n)),
                b.material_index([1, 1, 1]))
acc = b.accessors[b.meshes[mi]["primitives"][0]["indices"]]
check("index componentType is UNSIGNED_INT", acc["componentType"] == 5125,
      f" ({acc['componentType']})")
check("all indices survive", acc["count"] == big_n, f" ({acc['count']})")

print("\n=== BUG A: small meshes still use 16-bit (smaller files) ===")
s = ex.GltfBuilder()
mi = s.add_mesh([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)],
                [(0.0, 0.0, 1.0)] * 3, [0, 1, 2], s.material_index([1, 1, 1]))
acc = s.accessors[s.meshes[mi]["primitives"][0]["indices"]]
check("index componentType is UNSIGNED_SHORT", acc["componentType"] == 5123)
# ------------------------------------------------------- Bug B: quaternions
print("\n=== BUG B: identity quaternion survives the bake unswapped ===")
tracks = {"j": {"kind": "rotation", "times": [0.0, 1.0],
                "values": [[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]]}}
bb = ex.GltfBuilder()
bb.node_index = {"joint_j": bb.add_node("joint_j")}
n_s, _ = ex.bake_animation(bb, tracks, 20.0, {})
check("one sampler produced", n_s == 1, f" ({n_s})")
out = bb.accessors[bb.animations[0]["samplers"][0]["output"]]
raw = bb.blob[bb.buffer_views[out["bufferView"]]["byteOffset"]:]
q = struct.unpack("<4f", raw[:16])
check("frame0 is identity",
      all(abs(a - b) < 1e-6 for a, b in zip(q, (0.0, 0.0, 0.0, 1.0))),
      f" {tuple(round(v, 4) for v in q)}")

print("\n=== BUG B: origin_rpy is folded back in ===")
bb2 = ex.GltfBuilder()
bb2.node_index = {"joint_j2": bb2.add_node("joint_j2")}
ex.bake_animation(bb2, {"j2": {"kind": "rotation", "times": [0.0],
                               "values": [[0.0, 0.0, 0.0, 1.0]]}},
                  20.0, {"j2": (math.pi / 2, 0.0, 0.0)})
out2 = bb2.accessors[bb2.animations[0]["samplers"][0]["output"]]
raw2 = bb2.blob[bb2.buffer_views[out2["bufferView"]]["byteOffset"]:][:16]
ang = ang_between(struct.unpack("<4f", raw2), ex.rpy_quat(math.pi / 2, 0.0, 0.0))
check("baked frame0 == URDF rest rotation", ang < 0.5, f" (off by {ang:.3f} deg)")

print("\n=== BUG B: a foreign recording is reported, not silently dropped ===")
bb3 = ex.GltfBuilder()
bb3.node_index = {"joint_real": bb3.add_node("joint_real")}
n3, _ = ex.bake_animation(bb3, {"shoulder_pan_joint": {
    "kind": "rotation", "times": [0.0], "values": [[0.0, 0.0, 0.0, 1.0]]}},
    20.0, {})
check("no animation emitted", not bb3.animations)
check("no sampler emitted", n3 == 0)
# ------------------------------------------------------ Bug C: prismatic rpy
print("\n=== BUG C: prismatic joint keeps its origin rotation ===")
URDF = """<?xml version="1.0"?>
<robot name="t">
  <link name="base"/>
  <link name="slide"/>
  <joint name="slider" type="prismatic">
    <parent link="base"/><child link="slide"/>
    <origin xyz="0 0 0.05" rpy="0 0 3.141592653589793"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="0.04" effort="10" velocity="1"/>
  </joint>
</robot>"""
up = os.path.join(tempfile.mkdtemp(), "r.urdf")
open(up, "w").write(URDF)
gb = ex.GltfBuilder()
ex._convert_into(up, gb, None)
jn = next((nd for nd in gb.nodes if nd.get("name") == "joint_slider"), None)
check("prismatic node exists", jn is not None)
check("prismatic node has a rotation", bool(jn) and "rotation" in jn)
if jn and "rotation" in jn:
    ang = ang_between(jn["rotation"], ex.rpy_quat(0.0, 0.0, math.pi))
    check("rotation is the 180 deg origin yaw", ang < 0.5, f" (off by {ang:.3f})")
    check("translation still offset along the axis",
          abs(jn["translation"][2] - 0.05) < 1e-6,
          f" (z={jn['translation'][2]:.4f})")

print()
if fails:
    print(f"FAILED ({len(fails)}): " + ", ".join(fails))
    sys.exit(1)
print("All glTF quality checks passed.")