#!/usr/bin/env python3
"""Probe the available pxr export/glTF facilities and joint-state schema."""
from pxr import Usd, UsdGeom, UsdShade, UsdUtils

print("pxr modules importable: OK")

print("\n--- UsdUtils export-ish symbols ---")
interesting = [n for n in dir(UsdUtils) if any(
    k in n.lower() for k in ("gltf", "usdz", "export", "writer", "cook", "urdf", "exportable"))]
for n in sorted(interesting):
    print("  ", n)

print("\n--- GLTFWriter present? ---")
for cand in ("GLTFWriter", "GLTFExporter", "GLTFExportContext"):
    print(f"  {cand}: {hasattr(UsdUtils, cand)}")

print("\n--- UsdUtilsUsdUrdfParser ---")
print("  ", hasattr(UsdUtils, "UsdUrdfParser"))

print("\n--- UsdGeom prim types available ---")
for t in ("Cube", "Cylinder", "Sphere", "Mesh", "Xform", "Scope"):
    print(f"  {t}: {hasattr(UsdGeom, t)}")

print("\n--- sensor_msgs JointState fields ---")
import sys
sys.path.insert(0, "/opt/ros/humble/lib/python3.10/site-packages")
try:
    from sensor_msgs.msg import JointState
    print("  ", JointState.get_fields_and_field_types())
except Exception as e:
    print("   (sensor_msgs unavailable here):", e)