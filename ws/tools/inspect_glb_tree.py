#!/usr/bin/env python3
"""Inspect an exported GLB: node tree, world placement, and outliers.

The arm articulates now but two pieces render detached (a dark cone and the
gripper). Every source mesh loads cleanly, so the fault is in how the URDF
tree was turned into glTF nodes. This walks the exported scene, computes each
node's world matrix, and reports which meshes sit far from the kinematic
chain -- the signature of a node re-parented under the wrong parent.

    python3 ws/tools/inspect_glb_tree.py data/gltf/fr3_animated.glb
"""
import json
import os
import struct
import sys


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def mat_mul(a, b):
    out = [[0.0] * 4 for _ in range(4)]
    for r in range(4):
        for c in range(4):
            out[r][c] = sum(a[r][k] * b[k][c] for k in range(4))
    return out


def trs_matrix(node):
    t = node.get("translation", [0.0, 0.0, 0.0])
    r = node.get("rotation", [0.0, 0.0, 0.0, 1.0])
    s = node.get("scale", [1.0, 1.0, 1.0])
    x, y, z, w = r
    # Column-major glTF rotation -> 4x4.
    rot = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0.0],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0.0],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    for i in range(3):
        for j in range(3):
            rot[i][j] *= s[i]
    for i in range(3):
        rot[i][3] = t[i]
    return rot


def load_glb(path):
    d = open(path, "rb").read()
    magic, _ver, _total = struct.unpack("<III", d[0:12])
    if magic != 0x46546C67:
        raise SystemExit("not a GLB")
    jlen, _jtype = struct.unpack("<II", d[12:20])
    g = json.loads(d[20:20 + jlen].decode("utf-8"))
    return g, d, 20 + jlen


def accessor_min_max(g, d, bin_start, acc_index):
    acc = g["accessors"][acc_index]
    if "min" not in acc or "max" not in acc:
        return None
    return tuple(acc["min"]), tuple(acc["max"])


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "data/gltf/fr3_animated.glb"
    if not os.path.isfile(path):
        raise SystemExit(f"not found: {path}")
    g, d, bin_start = load_glb(path)
    nodes = g.get("nodes", [])
    meshes = g.get("meshes", [])

    parent = {}
    for i, n in enumerate(nodes):
        for c in n.get("children", []):
            parent[c] = i

    def world(i):
        """Compose the world matrix by walking up to the root."""
        chain = []
        cur = i
        guard = 0
        while cur is not None and guard < 64:
            chain.append(cur)
            cur = parent.get(cur)
            guard += 1
        m = trs_matrix(nodes[chain[-1]])
        for idx in reversed(chain[:-1]):
            m = mat_mul(m, trs_matrix(nodes[idx]))
        return m

    def chain_names(i):
        names = []
        cur = i
        guard = 0
        while cur is not None and guard < 64:
            names.append(nodes[cur].get("name", f"?{cur}"))
            cur = parent.get(cur)
            guard += 1
        return " <- ".join(names)

    print(f"file   : {path}")
    print(f"nodes  : {len(nodes)}  meshes: {len(meshes)}")
    roots = g["scenes"][g.get("scene", 0)]["nodes"]
    print(f"roots  : {[nodes[r].get('name') for r in roots]}\n")

    print(f"{'mesh node':<28} {'world translation':<30} {'bbox size':<24}")
    print("-" * 84)
    rows = []
    for i, n in enumerate(nodes):
        if "mesh" not in n:
            continue
        w = world(i)
        t = (w[0][3], w[1][3], w[2][3])
        mesh = meshes[n["mesh"]]
        mins = [1e9] * 3
        maxs = [-1e9] * 3
        for prim in mesh["primitives"]:
            acc = g["accessors"][prim["attributes"]["POSITION"]]
            if "min" in acc:
                for k in range(3):
                    mins[k] = min(mins[k], acc["min"][k])
                    maxs[k] = max(maxs[k], acc["max"][k])
        size = tuple(maxs[k] - mins[k] for k in range(3)) if mins[0] < 1e9 \
            else (0.0, 0.0, 0.0)
        # World position of the mesh bbox centre.
        centre = tuple((mins[k] + maxs[k]) / 2 for k in range(3))
        wc = tuple(w[r][0] * centre[0] + w[r][1] * centre[1]
                   + w[r][2] * centre[2] + w[r][3] for r in range(3))
        rows.append((n.get("name"), wc, size, i))
        print(f"{n.get('name',''):<28} "
              f"({wc[0]:8.3f},{wc[1]:8.3f},{wc[2]:8.3f})   "
              f"{size[0]:7.3f} x {size[1]:6.3f} x {size[2]:6.3f}")

    print("\n=== outlier check: distance from the arm's main axis ===")
    # The arm body should form one connected cluster. Anything far from it is
    # the detached piece.
    if rows:
        cx = sum(r[1][0] for r in rows) / len(rows)
        cy = sum(r[1][1] for r in rows) / len(rows)
        cz = sum(r[1][2] for r in rows) / len(rows)
        print(f"  centroid of all meshes: "
              f"({cx:.3f}, {cy:.3f}, {cz:.3f})")
        for name, wc, size, idx in rows:
            d = ((wc[0] - cx) ** 2 + (wc[1] - cy) ** 2
                 + (wc[2] - cz) ** 2) ** 0.5
            mark = "  <-- OUTLIER" if d > 0.35 else ""
            print(f"  {name:<28} |d|={d:7.3f}{mark}")
            if mark:
                print(f"    chain: {chain_names(idx)}")

    print("\n=== links missing a mesh ===")
    present = set()
    for n in nodes:
        nm = n.get("name", "")
        if nm.endswith("_visual"):
            present.add(nm[:-len("_visual")])
    all_links = set()
    for n in nodes:
        nm = n.get("name", "")
        if nm.startswith("fr3_") and not nm.endswith("_visual") \
                and not nm.startswith("joint_"):
            all_links.add(nm)
    missing = sorted(all_links - present)
    print(f"  meshes present : {len(present)}")
    print(f"  links in tree  : {len(all_links)}")
    print(f"  MISSING meshes : {missing if missing else 'none'}")

    print("\n=== tiny primitives (cone artefacts) ===")
    for n in nodes:
        if "mesh" not in n:
            continue
        tris = []
        for prim in meshes[n["mesh"]]["primitives"]:
            if "indices" in prim:
                tris.append(g["accessors"][prim["indices"]]["count"] // 3)
        tiny = [c for c in tris if c < 20]
        if tiny:
            print(f"  {n.get('name'):<26} prims={tris}")
            print(f"    tiny={tiny}  <-- candidate cone parts")

    print("\n=== node tree ===")

    def show(i, depth=0):
        n = nodes[i]
        tag = "[mesh]" if "mesh" in n else ""
        if "extras" in n and "armweb_joint" in n["extras"]:
            tag += "[joint]"
        print("   " + "  " * depth + n.get("name", str(i)) + " " + tag)
        for c in n.get("children", []):
            show(c, depth + 1)

    for r in roots:
        show(r)


if __name__ == "__main__":
    main()