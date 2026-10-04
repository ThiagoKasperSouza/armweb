#!/usr/bin/env python3
"""Print each visual node's local POSITION accessor min/max in a GLB.

Used to check where each part actually sits before the node transforms are
applied, which separates "the mesh is wrong" from "the node is misplaced".
"""
import json
import os
import struct
import sys


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "data/gltf/fr3_rest.glb"
    d = open(path, "rb").read()
    jlen, _ = struct.unpack("<II", d[12:20])
    g = json.loads(d[20:20 + jlen].decode("utf-8"))

    print(f"file: {path}\n")
    print(f"{'visual node':<26} {'local min':<26} {'local max':<26}")
    print("-" * 80)
    for n in g.get("nodes", []):
        if "mesh" not in n or not n.get("name", "").endswith("_visual"):
            continue
        mesh = g["meshes"][n["mesh"]]
        prim = mesh["primitives"][0]
        acc = g["accessors"][prim["attributes"]["POSITION"]]
        lo = [round(x, 3) for x in acc.get("min", [])]
        hi = [round(x, 3) for x in acc.get("max", [])]
        print(f"{n['name']:<26} {str(lo):<26} {str(hi):<26}")

    print("\n=== links with no mesh node (silently dropped?) ===")
    vis = {n["name"][:-len("_visual")] for n in g.get("nodes", [])
           if n.get("name", "").endswith("_visual")}
    links = {n["name"] for n in g.get("nodes", [])
             if n.get("name", "").startswith("fr3_")
             and not n.get("name", "").endswith("_visual")
             and not n.get("name", "").startswith("joint_")}
    missing = sorted(links - vis)
    print(f"  {len(vis)} meshes / {len(links)} links")
    print(f"  missing: {missing if missing else 'none'}")

    print("\n=== which visual node owns the lowest vertex? ===")
    # The bbox min/max come straight from the POSITION accessors, so a single
    # stray vertex can push the whole model below the floor. Find it.
    worst = []
    for n in g.get("nodes", []):
        if "mesh" not in n:
            continue
        for prim in g["meshes"][n["mesh"]]["primitives"]:
            acc = g["accessors"][prim["attributes"]["POSITION"]]
            if "min" in acc:
                worst.append((acc["min"][1], acc["max"][1],
                              n.get("name", "?")))
    worst.sort()
    for lo, hi, nm in worst[:5]:
        print(f"  {nm:<28} y [{lo:7.3f} .. {hi:7.3f}]")
    print("  ...")
    for lo, hi, nm in worst[-3:]:
        print(f"  {nm:<28} y [{lo:7.3f} .. {hi:7.3f}]")

    print(f"\n=== triangle counts per visual (decimation check) ===")
    for n in g.get("nodes", []):
        if "mesh" not in n or not n.get("name", "").endswith("_visual"):
            continue
        tris = 0
        for prim in g["meshes"][n["mesh"]]["primitives"]:
            if "indices" in prim:
                tris += g["accessors"][prim["indices"]]["count"] // 3
        verts = g["accessors"][
            g["meshes"][n["mesh"]]["primitives"][0]["attributes"]["POSITION"]]
        flag = "  <-- OVER 65535!" if verts["count"] > 65535 else ""
        print(f"  {n['name']:<28} tris={tris:>7} verts={verts['count']:>7}{flag}")


if __name__ == "__main__":
    main()