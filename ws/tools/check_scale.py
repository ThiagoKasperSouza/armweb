#!/usr/bin/env python3
"""Report mesh scale factors and extents in the generated URDF."""
import xml.etree.ElementTree as ET

r = ET.parse("/home/thiag/armweb/ws/assets/urdf/robot.urdf").getroot()
scales = {}
for m in r.findall(".//mesh"):
    fn = m.get("filename", "")
    s = m.find("scale")
    xyz = s.get("xyz") if s is not None else "(none)"
    scales.setdefault(xyz, []).append(fn.split("/")[-1])

print("scale values found:")
for xyz, files in scales.items():
    print(f"  {xyz:22} -> {len(files)} meshes, e.g. {files[:3]}")
if "(none)" in scales:
    print(f"  (none)                  -> {len(scales['(none)'])} meshes (unit scale)")
print()
print("Interpretation: franka_description authors meshes in millimetres,")
print("so a 0.001 scale is required to get metres (URDF convention).")