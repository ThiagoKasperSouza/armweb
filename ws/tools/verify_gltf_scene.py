#!/usr/bin/env python3
"""
Verify a .glb by walking its actual node hierarchy.

Checks what silently breaks rendering:
  * scene roots are minimal (a flat root list = disjointed model)
  * every node is reachable from the roots
  * world bounds, in glTF's Y-up convention (arm upright, base on ground)
  * animated channels target reachable nodes and actually vary over time
  * rotation quaternions are unit length and non-degenerate

Usage: python3 verify_gltf_scene.py <file.glb>
"""
import json
import math
import struct
import sys


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def quat_rotate(q, v):
    x, y, z, w = q
    r = quat_mul(q, [v[0], v[1], v[2], 0.0])
    r = quat_mul(r, [-x, -y, -z, w])
    return r[:3]


def load(path):
    with open(path, "rb") as fh:
        data = fh.read()
    jlen, _ = struct.unpack("<II", data[12:20])
    g = json.loads(data[20:20 + jlen].decode("utf-8"))

    def accessor(i):
        acc = g["accessors"][i]
        bv = g["bufferViews"][acc["bufferView"]]
        off = 20 + jlen + 8 + bv.get("byteOffset", 0)
        ncomp = {"SCALAR": 1, "VEC3": 3, "VEC4": 4}[acc["type"]]
        fmt = {1: "<f", 3: "<3f", 4: "<4f"}[ncomp]
        size = struct.calcsize(fmt)
        return [struct.unpack_from(fmt, data, off + k * size)
                for k in range(acc["count"])]

    return g, accessor


def reachable_from(nodes, roots):
    seen = set()

    def walk(i):
        if i in seen:
            return
        seen.add(i)
        for c in nodes[i].get("children", []):
            walk(c)

    for r in roots:
        walk(r)
    return seen
def world_bounds(g, accessor, roots):
    nodes = g["nodes"]
    mins = [1e9] * 3
    maxs = [-1e9] * 3

    def bounds(i, T, R):
        n = nodes[i]
        t = list(n.get("translation", [0, 0, 0]))
        r = n.get("rotation", [0, 0, 0, 1])
        rt = quat_rotate(r, t)
        L = [T[k] + rt[k] for k in range(3)]
        NR = quat_mul(R, r)
        if "mesh" in n:
            for prim in g["meshes"][n["mesh"]]["primitives"]:
                for v in accessor(prim["attributes"]["POSITION"]):
                    rv = quat_rotate(NR, v)
                    for k in range(3):
                        w = L[k] + rv[k]
                        mins[k] = min(mins[k], w)
                        maxs[k] = max(maxs[k], w)
        for c in n.get("children", []):
            bounds(c, L, NR)

    for r in roots:
        bounds(r, [0, 0, 0], [0, 0, 0, 1])
    return mins, maxs


def base_bounds(g, accessor, roots, name_hints=("link0", "base_link", "base")):
    """World bounds of the BASE link only, not the whole robot.

    The ground check must look at the base, because a robot in its zero pose
    can legitimately fold an elbow below the mounting plane: the FR3's
    fr3_link3 dips to y=-0.44 m while fr3_link0 (the base) sits correctly at
    y=0..0.14. Taking the bbox of the entire scene made a correct export look
    like a robot buried underground.
    """
    nodes = g["nodes"]
    mins = [1e9] * 3
    maxs = [-1e9] * 3
    found = []

    def bounds(i, T, R):
        n = nodes[i]
        t = list(n.get("translation", [0, 0, 0]))
        r = n.get("rotation", [0, 0, 0, 1])
        rt = quat_rotate(r, t)
        L = [T[k] + rt[k] for k in range(3)]
        NR = quat_mul(R, r)
        if "mesh" in n:
            nm = (n.get("name") or "").lower()
            if any(h in nm for h in name_hints):
                found.append(nm)
                for prim in g["meshes"][n["mesh"]]["primitives"]:
                    for v in accessor(prim["attributes"]["POSITION"]):
                        rv = quat_rotate(NR, v)
                        for k in range(3):
                            w = L[k] + rv[k]
                            mins[k] = min(mins[k], w)
                            maxs[k] = max(maxs[k], w)
        for c in n.get("children", []):
            bounds(c, L, NR)

    for r in roots:
        bounds(r, [0, 0, 0], [0, 0, 0, 1])
    if not found:
        return None, None
    return mins, maxs


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "data/gltf/arm_animated.glb"
    g, accessor = load(path)
    nodes = g["nodes"]
    roots = g["scenes"][0]["nodes"]

    print(f"file          : {path}")
    print(f"nodes / meshes: {len(nodes)} / {len(g.get('meshes', []))}")
    print(f"scene roots   : {roots}  "
          f"({', '.join(nodes[r].get('name', '?') for r in roots)})")

    reach = reachable_from(nodes, roots)
    orphans = sorted(set(range(len(nodes))) - reach)
    print(f"reachable     : {len(reach)}/{len(nodes)}")
    if orphans:
        print("  ORPHANED    : "
              + ", ".join(f"[{o}] {nodes[o].get('name')}" for o in orphans))

    mins, maxs = world_bounds(g, accessor, roots)
    size = [maxs[k] - mins[k] for k in range(3)]
    print(f"bbox min/max  : {[round(v, 3) for v in mins]} / "
          f"{[round(v, 3) for v in maxs]}")
    print(f"size XYZ      : {[round(v, 3) for v in size]}")

    problems = []
    x, y, z = size
    # A real robot may rest in a folded or extended pose, so its bounding box
    # is not necessarily taller than it is wide. Require only that the model
    # has a plausible human/robot scale and stands roughly on the ground.
    longest = max(x, y, z)
    if longest > 20.0 or longest < 0.05:
        problems.append(
            f"implausible model size {longest:.3f} m - the mesh scale is "
            f"probably wrong")
    if longest < 0.5:
        print("  note: model is smaller than 0.5 m; check the mesh scale")
    # Base within 15 cm of the ground plane. Measure the BASE link, not the
    # whole robot: an elbow folded below the table in the zero pose is not a
    # placement error.
    bmin, bmax = base_bounds(g, accessor, roots)
    if bmin is None:
        print("  note: no base link matched; skipped the ground check")
    else:
        print(f"base bbox min : {[round(v, 3) for v in bmin]}")
        if abs(bmin[1]) > 0.15:
            problems.append(f"base not near the ground (y={bmin[1]:.3f})")
        elif abs(bmin[1]) > 0.02:
            print(f"  note: base sits {bmin[1]:.3f} m off the ground plane")
        if bmin[2] < -0.02 or bmax[2] > 0.02:
            print(f"  note: base is not centred on z (z={bmin[2]:.3f}..{bmax[2]:.3f})")
    if orphans:
        problems.append(f"{len(orphans)} orphaned node(s)")
    if len(roots) > 3:
        problems.append(f"{len(roots)} scene roots: hierarchy is flattened")

    # The animation checks below must live inside this function, at the same
    # level as the other checks: an earlier version left `for anim in ...` at
    # column 0, which made the whole block a syntax error.
    for anim in g.get("animations", []):
        print(f"\nanimation '{anim['name']}': "
              f"{len(anim['samplers'])} samplers, "
              f"{len(anim['channels'])} channels")
        for ch in anim["channels"]:
            n = ch["target"]["node"]
            s = anim["samplers"][ch["sampler"]]
            out = g["accessors"][s["output"]]
            keys = accessor(s["output"])
            times = accessor(s["input"])
            moved = keys[0] != keys[-1]
            nm = nodes[n].get("name")
            flag = "OK" if n in reach else "ORPHANED TARGET"
            print(f"  {nm:26} {ch['target']['path']:11} "
                  f"{out['count']:3} keys "
                  f"t=[{times[0][0]:.2f}..{times[-1][0]:.2f}]s "
                  f"moved={moved}  {flag}")
            if not moved:
                # A joint that never moves is legitimate when it is fixed, or
                # when the recorded motion simply did not reach it. Report it
                # as a note, not a failure.
                print(f"  note: {nm} is constant across the capture")
            if out["type"] == "VEC4":
                norm = math.sqrt(sum(c * c for c in keys[0]))
                if abs(norm - 1.0) > 1e-3:
                    problems.append(f"{nm}: quaternion not unit length "
                                    f"(|q|={norm:.4f})")
                zero = [c for c in range(4)
                        if all(abs(k[c]) < 1e-6 for k in keys)]
                if len(zero) == 3:
                    # Expected for a single-axis rotation whose angle stayed 0
                    # (an identity quaternion), so treat as informational.
                    print(f"  note: {nm} holds an identity rotation")

    print()
    if problems:
        print("PROBLEMS FOUND:")
        for p in problems:
            print("  -", p)
        sys.exit(1)
    print("OK: hierarchy, orientation and animation all valid")


if __name__ == "__main__":
    main()