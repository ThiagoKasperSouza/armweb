#!/usr/bin/env python3
"""Find the mesh that renders as a stray black cone / detached hand.

The arm now articulates correctly, but two artefacts remain:
  * a black cone floating away from the body
  * the gripper sitting detached from link7

A cone that shades BLACK means the normals are wrong (zero-length or pointing
inward), and a cone *shape* usually means a primitive whose index buffer is
misread -- e.g. a 1-channel (N,3,1) index array reinterpreted as vertex pairs,
which pairs up unrelated vertices and stretches triangles into spikes.

So this reports, per link:
  * bbox / centroid relative to the link frame
  * how many normals are zero-length (renders black)
  * the node matrix determinant (negative = mirrored, flips normals)
  * the index array shape actually emitted by the loader
"""
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import load_mesh as lm  # noqa: E402

URDF = os.environ.get("ARMWEB_URDF", "/ws/assets/urdf/robot.urdf")


def parse_vec(s, default=(0.0, 0.0, 0.0)):
    if not s:
        return default
    try:
        vals = [float(p) for p in s.replace(",", " ").split()[:3]]
    except ValueError:
        return default
    while len(vals) < 3:
        vals.append(0.0)
    return tuple(vals)


def bbox(pos):
    lo = tuple(min(p[i] for p in pos) for i in range(3))
    hi = tuple(max(p[i] for p in pos) for i in range(3))
    return lo, hi


def det3(m):
    a, b, c = m[0][0], m[0][1], m[0][2]
    d, e, f = m[1][0], m[1][1], m[1][2]
    g, h, i = m[2][0], m[2][1], m[2][2]
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)


print(f"URDF: {URDF}")
root = ET.parse(URDF).getroot()

# Which joint drives each link, and where.
driven = {}
for j in root.findall("joint"):
    cel = j.find("child")
    if cel is not None:
        driven[cel.get("link")] = (j.get("name"), parse_vec(
            j.find("origin").get("xyz") if j.find("origin") is not None else None))

problems = []
print(f"\n{'link':<20} {'driver':<20} {'bbox size':<26} {'badN':>6} {'det':>8}")
print("-" * 84)

for link in sorted(root.findall("link"), key=lambda l: l.get("name")):
    name = link.get("name")
    vis = link.find("visual")
    if vis is None:
        continue
    mesh_el = vis.find("geometry/mesh")
    if mesh_el is None:
        continue
    path = lm.resolve_package_uri(mesh_el.get("filename"))
    if not path:
        print(f"{name:<20} {'(unresolved mesh)':<20}")
        problems.append((name, "mesh file unresolved"))
        continue

    ext = os.path.splitext(path)[1].lower()
    soup = lm._load_dae(path) if ext == ".dae" else lm._load_trimesh(path)
    if soup is None:
        print(f"{name:<20} {'(load failed)':<20}")
        problems.append((name, "load failed"))
        continue
    pos, nrm, idx = soup
    lo, hi = bbox(pos)
    size = tuple(hi[i] - lo[i] for i in range(3))

    zero_n = sum(1 for n in nrm
                 if sum(c * c for c in n) < 0.5)
    # Mirrored node -> normals flip -> inside-out / black shading.
    det = float("nan")
    if ext == ".dae":
        try:
            from collada import Collada
            doc = Collada(path)
            dets = []
            for r in (getattr(doc.scene, "nodes", None) or []):
                for bg in r.objects("geometry"):
                    dets.append(det3(lm._mat_to_tuples(bg.matrix)))
            if dets:
                det = min(dets)
        except Exception:
            pass

    jname, jorg = driven.get(name, ("-", (0.0, 0.0, 0.0)))
    print(f"{name:<20} {jname:<20} "
          f"{size[0]:7.3f} x {size[1]:6.3f} x {size[2]:6.3f} "
          f"{zero_n:>6} {det:>8.3f}")

    if zero_n:
        problems.append((name, f"{zero_n} zero-length normals"))
    if det == det and det < 0:
        problems.append((name, f"negative determinant {det:.3f} (mirrored)"))
    # A link much larger than an arm segment usually means a bad index read.
    if max(size) > 0.6:
        problems.append((name, f"oversized mesh {max(size):.3f} m"))

print("\n=== per-primitive index shapes (source of cone artefacts) ===")
try:
    from collada import Collada
except ImportError:
    print("  pycollada unavailable")
    raise SystemExit(0)

for link in sorted(root.findall("link"), key=lambda l: l.get("name")):
    vis = link.find("visual")
    if vis is None:
        continue
    mesh_el = vis.find("geometry/mesh")
    if mesh_el is None:
        continue
    path = lm.resolve_package_uri(mesh_el.get("filename"))
    if not path or not path.lower().endswith(".dae"):
        continue
    try:
        doc = Collada(path)
    except Exception:
        continue
    shapes = []
    for geo in doc.geometries:
        for prim in getattr(geo, "primitives", []):
            flat = getattr(prim, "index", None)
            shapes.append(tuple(getattr(flat, "shape", ()) or ()))
    if shapes:
        uniq = sorted(set(shapes))
        flag = "" if all(len(s) == 3 and s[-1] == 2 for s in uniq) else "  <-- unusual"
        print(f"  {link.get('name'):<20} {uniq}{flag}")

print("\n=== per-link raw DAE breakdown (why is a mesh missing?) ===")
from collada import Collada  # noqa: E402

for name in sorted(root.findall("link"), key=lambda l: l.get("name")):
    vis = name.find("visual")
    if vis is None:
        print(f"  {name.get('name'):<20} no <visual>")
        continue
    me = vis.find("geometry/mesh")
    if me is None:
        print(f"  {name.get('name'):<20} visual without <mesh> (primitive?)")
        continue
    p = lm.resolve_package_uri(me.get("filename"))
    if not p:
        print(f"  {name.get('name'):<20} UNRESOLVED {me.get('filename')}")
        continue
    try:
        doc = Collada(p)
    except Exception as exc:
        print(f"  {name.get('name'):<20} Collada() FAILED: {exc}")
        continue

    ngeo = len(list(doc.geometries))
    prims = []
    for g in doc.geometries:
        prims += list(getattr(g, "primitives", []))
    def n_of(obj):
        # numpy arrays are ambiguous in a boolean context, so compare to None.
        v = getattr(obj, "vertex", None)
        if v is None:
            return 0
        try:
            return len(v)
        except TypeError:
            return 0

    vtx = sum(n_of(pr) for pr in prims)
    tri = 0
    for pr in prims:
        flat = getattr(pr, "index", None)
        if flat is None:
            continue
        try:
            tri += len(flat)
        except TypeError:
            pass

    soup = lm._load_dae(p)
    emitted = len(soup[0]) if soup else 0

    if not emitted:
        print(f"  {name.get('name'):<20} *** LOADER RETURNED NOTHING ***")
        print(f"      {os.path.basename(p)} geoms={ngeo} prims={len(prims)} "
              f"verts={vtx} indices={tri}")
        for i, pr in enumerate(prims):
            flat = getattr(pr, "index", None)
            sh = tuple(getattr(flat, "shape", ()) or ()) if flat is not None else ()
            ni = 0
            if flat is not None:
                try:
                    ni = len(flat)
                except TypeError:
                    ni = -1
            print(f"      prim[{i}] {type(pr).__name__} shape={sh} "
                  f"nvertex={n_of(pr)} nindex={ni}")
        problems.append((name.get("name"), "loader produced no triangles"))
    else:
        print(f"  {name.get('name'):<20} ok: {emitted} corners "
              f"(geoms={ngeo}, prims={len(prims)})")