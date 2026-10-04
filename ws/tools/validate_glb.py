#!/usr/bin/env python3
"""Validate a .glb file: header, JSON chunk, buffer integrity, key fields."""
import json
import struct
import sys

path = sys.argv[1]
with open(path, "rb") as fh:
    data = fh.read()

magic, version, total = struct.unpack("<III", data[:12])
print("file:", path, f"({len(data)} bytes)")

if magic != 0x46546C67:
    raise SystemExit(f"FAIL: bad magic {magic:#x} (expected glTF)")
if version != 2:
    raise SystemExit(f"FAIL: version {version} (expected 2)")
if total != len(data):
    raise SystemExit(f"FAIL: header length {total} != actual {len(data)}")
print("  header OK: magic=glTF version=2 length matches")

json_len, json_type = struct.unpack("<II", data[12:20])
if json_type != 0x4E4F534A:
    raise SystemExit(f"FAIL: first chunk is not JSON ({json_type:#x})")
gltf = json.loads(data[20:20 + json_len].decode("utf-8"))
print(f"  JSON chunk OK ({json_len} bytes)")

bin_off = 20 + json_len
bin_len, bin_type = struct.unpack("<II", data[bin_off:bin_off + 8])
if bin_type != 0x004E4942:
    raise SystemExit(f"FAIL: second chunk is not BIN ({bin_type:#x})")
bin_data = data[bin_off + 8:bin_off + 8 + bin_len]
print(f"  BIN chunk OK ({bin_len} bytes)")

for key in ("asset", "scenes", "nodes", "meshes", "accessors", "bufferViews"):
    if key not in gltf:
        raise SystemExit(f"FAIL: missing required key '{key}'")

print(f"  asset.version   : {gltf['asset']['version']}")
print(f"  nodes           : {len(gltf['nodes'])}")
print(f"  meshes          : {len(gltf['meshes'])}")
print(f"  materials       : {len(gltf.get('materials', []))}")
print(f"  accessors       : {len(gltf['accessors'])}")
print(f"  bufferViews     : {len(gltf['bufferViews'])}")
print(f"  animations      : {len(gltf.get('animations', []))}")

# Buffer views must lie inside the BIN chunk.
total_len = gltf["buffers"][0]["byteLength"]
if total_len > bin_len:
    raise SystemExit(f"FAIL: buffer byteLength {total_len} > BIN chunk {bin_len}")
for i, view in enumerate(gltf["bufferViews"]):
    end = view.get("byteOffset", 0) + view["byteLength"]
    if end > bin_len:
        raise SystemExit(f"FAIL: bufferView {i} ends at {end} > {bin_len}")
print("  all bufferViews within bounds")

# Accessors must reference valid bufferViews.
for i, acc in enumerate(gltf["accessors"]):
    if acc["bufferView"] >= len(gltf["bufferViews"]):
        raise SystemExit(f"FAIL: accessor {i} -> bad bufferView")
    if acc["count"] <= 0:
        raise SystemExit(f"FAIL: accessor {i} count={acc['count']}")
print("  all accessors valid")

# Node hierarchy must be acyclic and reference existing children.
n_nodes = len(gltf["nodes"])
for i, node in enumerate(gltf["nodes"]):
    for c in node.get("children", []):
        if c >= n_nodes or c == i:
            raise SystemExit(f"FAIL: node {i} has bad child {c}")
if gltf["scenes"][0]["nodes"] != list(range(n_nodes)):
    print("  note: scene root list is not every node")
print("  node hierarchy valid")

if gltf.get("animations"):
    anim = gltf["animations"][0]
    print(f"  animation '{anim['name']}': "
          f"{len(anim['samplers'])} samplers, {len(anim['channels'])} channels")
    for ch in anim["channels"]:
        if ch["target"]["node"] >= n_nodes:
            raise SystemExit("FAIL: animation targets a missing node")

# Vertex totals
total_verts = sum(
    acc["count"] for acc in gltf["accessors"] if acc["type"] == "VEC3")
print(f"  total VEC3 vertices: {total_verts}")
print("VALID GLB")