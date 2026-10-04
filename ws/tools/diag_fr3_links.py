#!/usr/bin/env python3
"""Locate where the FR3 meshes collapse, link by link.

The symptom (every part bunched at the origin, plus a stray cone) has several
possible causes, and each needs a different fix. This reports the facts that
separate them:

  1. Does the COLLADA file even have a scene with node transforms?
  2. Per geometry: raw vertex extent (is it authored in mm?), and the bbox
     AFTER the node walk (did the walk actually move anything?).
  3. Where does each link's mesh centroid end up relative to the joint origin
     it is supposed to hang off?

Run from the project root on the HOST (/home/thiag/armweb), not in the
container. The compose file bind-mounts ./ws as /ws, so the host path
ws/tools/... is /ws/tools/... inside the container:

    docker compose exec -T sim python3 /ws/tools/diag_fr3_links.py

(or just ./scripts/diag_meshes.sh, which wraps exactly that)
"""
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import load_mesh as lm  # noqa: E402

URDF = os.environ.get("ARMWEB_URDF", "/ws/assets/urdf/robot.urdf")


HERE = os.path.dirname(os.path.abspath(__file__))


def find_urdf():
    """Fall back to the other known locations instead of failing outright."""
    cands = [
        URDF,
        "/ws/assets/urdf/robot.urdf",
        "/ws/assets/urdf/demo_arm.urdf",
        os.path.join(HERE, "..", "assets", "urdf", "robot.urdf"),
        os.path.join(HERE, "..", "assets", "urdf", "demo_arm.urdf"),
    ]
    for cand in cands:
        if cand and os.path.isfile(cand):
            return cand
    return None


def bbox(positions):
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    zs = [p[2] for p in positions]
    lo = (min(xs), min(ys), min(zs))
    hi = (max(xs), max(ys), max(zs))
    return lo, hi


print(f"ARMWEB_URDF env: {os.environ.get('ARMWEB_URDF', '(unset)')}")
resolved = find_urdf()
if resolved is None:
    print("no URDF found. Tried:")
    print(f"  {URDF}")
    print("  /ws/assets/urdf/demo_arm.urdf")
    print("")
    print("Prepare one first:  make arm ARM=fr3")
    raise SystemExit(1)
if resolved != URDF:
    print(f"falling back to:  {resolved}")
URDF = resolved
print(f"URDF: {URDF}")

root = ET.parse(URDF).getroot()


def parse_vec(s, default=(0.0, 0.0, 0.0)):
    """Parse a whitespace/comma separated "x y z" attribute."""
    if not s:
        return default
    parts = [p for p in s.replace(",", " ").split() if p]
    if not parts:
        return default
    vals = [float(p) for p in parts[:3]]
    while len(vals) < 3:
        vals.append(0.0)
    return tuple(vals)


# Joint origins, so we know where each link SHOULD sit.
joint_origin = {}
for j in root.findall("joint"):
    o = j.find("origin")
    if o is not None and o.get("xyz"):
        joint_origin[j.find("child").get("link")] = parse_vec(o.get("xyz"))

links = {}
for link in root.findall("link"):
    vis = link.find("visual")
    if vis is None:
        continue
    mesh_el = vis.find("geometry/mesh")
    if mesh_el is None:
        continue
    uri = mesh_el.get("filename")
    path = lm.resolve_package_uri(uri)
    links[link.get("name")] = {
        "uri": uri, "path": path,
        "scale": mesh_el.get("scale"),
        "origin": vis.find("origin"),
    }

print(f"\n{len(links)} links carry a visual mesh\n")
print("=== per-link: does the mesh file exist and how big is it raw? ===")
print(f"  {'link':<24} {'file':<16} {'raw extent (file units)':>28}")
missing = []
for name, info in sorted(links.items()):
    p = info["path"]
    if not p:
        missing.append(name)
        print(f"  {name:<24} {'MISSING':<16} {'-':>28}  ({info['uri']})")
        continue
    ext = os.path.splitext(p)[1].lower()
    soup = lm._load_dae(p) if ext == ".dae" else lm._load_trimesh(p)
    if soup is None:
        print(f"  {name:<24} {ext:<16} {'LOAD FAILED':>28}")
        continue
    pos = soup[0]
    lo, hi = bbox(pos)
    e = (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])
    print(f"  {name:<24} {ext:<16} "
          f"{e[0]:8.1f} x {e[1]:8.1f} x {e[2]:8.1f}")

if missing:
    print(f"\n  !! {len(missing)} mesh file(s) unresolved: {missing}")

# --- the decisive test: does the node walk change the geometry at all? ---
print("=== does the COLLADA scene walk move anything? ===")
try:
    from collada import Collada
except ImportError:
    print("  pycollada unavailable; skipping")
    raise SystemExit(0)

dae_links = [n for n, i in links.items()
             if i["path"] and i["path"].lower().endswith(".dae")]
if not dae_links:
    print("  no .dae visuals resolved; nothing to inspect")
    raise SystemExit(0)

for name in sorted(dae_links):
    p = links[name]["path"]
    try:
        doc = Collada(p)
    except Exception as exc:
        print(f"\n  {name}: COLLADA LOAD FAILED: {exc}")
        continue
    roots = list(getattr(doc.scene, "nodes", None) or [])
    bound = []
    for r in roots:
        bound.extend(r.objects("geometry"))

    ident = 0
    moved = 0
    for bg in bound:
        m = bg.matrix.tolist()
        same = all(abs(m[r][c] - (1.0 if r == c else 0.0)) < 1e-9
                   for r in range(4) for c in range(4))
        if same:
            ident += 1
        else:
            moved += 1

    print(f"\n  {name}  ({os.path.basename(p)})")
    print(f"    geometries={len(list(doc.geometries))}"
          f"  scene_roots={len(roots)}"
          f"  bound_geometries={len(bound)}"
          f"  identity={ident}  transformed={moved}")
    if not bound:
        print("    !! no bound geometry -> _load_dae fell back to raw")
    if ident and not moved:
        print("    all transforms are identity: the node walk cannot be")
        print("    the cause of a collapse for THIS file")
    for bg in bound[:3]:
        m = bg.matrix.tolist()
        t = (m[0][3], m[1][3], m[2][3])
        if any(abs(v) > 1e-9 for v in t):
            print(f"    {bg.original.name!r} translation"
                  f"=({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f})")

# --- where do the meshes actually end up? ---
print("\n=== visual centroid vs. joint origin (collapse detector) ===")
print("  A link whose visual sits far from its joint origin is the")
print("  signature of a missing/ignored transform.\n")
for name in sorted(links):
    info = links[name]
    if not info["path"]:
        continue
    ext = os.path.splitext(info["path"])[1].lower()
    soup = lm._load_dae(info["path"]) if ext == ".dae" \
        else lm._load_trimesh(info["path"])
    if soup is None:
        continue
    pos = soup[0]
    lo, hi = bbox(pos)
    mid = tuple((lo[i] + hi[i]) / 2 for i in range(3))
    org = joint_origin.get(name, (0.0, 0.0, 0.0))
    dist = sum((mid[i] - org[i]) ** 2 for i in range(3)) ** 0.5
    print(f"  {name:<24} centroid=({mid[0]:9.3f},{mid[1]:9.3f},"
          f"{mid[2]:9.3f})  joint_origin=({org[0]:6.3f},{org[1]:6.3f},"
          f"{org[2]:6.3f})  |d|={dist:8.3f}")