#!/usr/bin/env python3
"""Confirm a GLB carries the joint metadata the pose panel needs.

Reads the JSON chunk of a .glb and reports, per joint node, whether
extras.armweb_joint survived the export -- the panel in viewer.html builds its
sliders from that, and a .glb exported before the extras existed will show
"nenhuma junta encontrada".

    python3 ws/tools/check_extras.py data/gltf/fr3_rest.glb
"""
import json
import os
import struct
import sys


def load(path):
    d = open(path, "rb").read()
    magic, version, total = struct.unpack("<III", d[0:12])
    if magic != 0x46546C67:
        raise SystemExit(f"{path}: not a GLB (magic {magic:#x})")
    jlen, jtype = struct.unpack("<II", d[12:20])
    if jtype != 0x4E4F534A:
        raise SystemExit(f"{path}: first chunk is not JSON")
    return json.loads(d[20:20 + jlen].decode("utf-8"))


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "data/gltf/fr3_rest.glb"
    if not os.path.isfile(path):
        raise SystemExit(f"not found: {path}")

    g = load(path)
    nodes = g.get("nodes", [])
    joints = [n for n in nodes if n.get("name", "").startswith("joint_")]
    print(f"file        : {path}")
    print(f"nodes       : {len(nodes)}")
    print(f"joint nodes : {len(joints)}")

    with_extras = [n for n in joints if "extras" in n]
    print(f"with extras : {len(with_extras)}")

    if not with_extras:
        print("\n  !! no joint metadata -- the pose panel will stay empty.")
        print("  Re-export with a current export_gltf.py (it writes extras).")
        return 1

    ok_limits = 0
    print(f"\n{'joint':<24} {'type':<11} {'lower':>9} {'upper':>9}")
    for n in joints:
        meta = n.get("extras", {}).get("armweb_joint")
        if not meta:
            print(f"{n.get('name','?'):<24} {'(no metadata)':<11}")
            continue
        lo = meta.get("lower")
        hi = meta.get("upper")
        if lo is not None and hi is not None:
            ok_limits += 1
        print(f"{meta.get('name','?'):<24} {meta.get('type','?'):<11} "
              f"{('-' if lo is None else f'{lo:9.3f}'):>9} "
              f"{('-' if hi is None else f'{hi:9.3f}'):>9}")

    axisless = [n["extras"]["armweb_joint"]["name"] for n in with_extras
                if not n["extras"]["armweb_joint"].get("axis")]
    if axisless:
        print(f"\n  !! no axis for: {axisless}")
        return 1

    print(f"\nOK: {len(with_extras)}/{len(joints)} joint nodes carry metadata, "
          f"{ok_limits} with limits.")
    return 0


if __name__ == "__main__":
    sys.exit(main())