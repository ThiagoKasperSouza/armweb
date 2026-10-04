#!/usr/bin/env python3
"""Print each joint's child link, origin and visual offset from the URDF.

The diagnostic showed fr3_link1's visual sitting ~0.40 m from its joint
origin. That is expected *only* if the mesh is authored around its own
centre; if the URDF already places the link frame, an extra offset means the
transform is being applied twice. This makes the numbers comparable:

    link        joint origin (where the link frame sits)
               visual origin (offset applied to the mesh inside that frame)

A visual origin close to (0,0,0) means the DAE must supply the placement.
"""
import os
import xml.etree.ElementTree as ET

URDF = os.environ.get("ARMWEB_URDF", "/ws/assets/urdf/robot.urdf")


def vec(el, attr="xyz", default=(0.0, 0.0, 0.0)):
    if el is None:
        return default
    raw = el.get(attr)
    if not raw:
        return default
    parts = [p for p in raw.replace(",", " ").split() if p]
    try:
        vals = [float(p) for p in parts[:3]]
    except ValueError:
        return default
    while len(vals) < 3:
        vals.append(0.0)
    return tuple(vals)


def fmt(v):
    if v is None:
        return "(   none)"
    return "(" + ", ".join(f"{x:7.3f}" for x in v) + ")"


root = ET.parse(URDF).getroot()

joint_child = {}
print(f"{'joint':<22} {'child link':<22} {'type':<11} joint origin")
for j in root.findall("joint"):
    pel, cel = j.find("parent"), j.find("child")
    if pel is None or cel is None:
        continue
    child = cel.get("link")
    joint_child[child] = j.get("name")
    print(f"{j.get('name'):<22} {child:<22} "
          f"{j.get('type','fixed'):<11} {fmt(vec(j.find('origin')))}")

print(f"\n{'link':<22} {'driven by joint':<22} visual origin")
links = {l.get("name"): l for l in root.findall("link")}
for name in sorted(links):
    vis = links[name].find("visual")
    if vis is None:
        continue
    print(f"{name:<22} {joint_child.get(name, '-'):<22} "
          f"{fmt(vis.find('origin'))}")

print("\n=== links with a visual origin far from (0,0,0) ===")
print("  A non-zero visual origin is combined with the DAE node transform;")
print("  if BOTH place the same part you get a doubled offset.\n")
for name in sorted(links):
    vis = links[name].find("visual")
    if vis is None:
        continue
    o = vec(vis.find("origin"))
    mag = sum(c * c for c in o) ** 0.5
    if mag > 1e-6:
        print(f"  {name:<22} {fmt(o)}   |d|={mag:.3f}")