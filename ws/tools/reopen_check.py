#!/usr/bin/env python3
"""Reopen a .usda and report its structure - validates the export."""
import sys
from pxr import Usd, UsdGeom

path = sys.argv[1] if len(sys.argv) > 1 else "/ws/data/usd/arm.usda"
stage = Usd.Stage.Open(path)
if stage is None:
    raise SystemExit(f"FAILED to reopen {path}")

prims = list(stage.Traverse())
print("reopened OK:", path)
print("prim count:", len(prims))
print("prim types:", sorted({p.GetTypeName() for p in prims}))

bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
b = bbox.ComputeWorldBound(stage.GetPrimAtPath("/arm"))
rng = b.ComputeAlignedRange()
print("world bbox min:", [round(v, 3) for v in rng.GetMin()])
print("world bbox max:", [round(v, 3) for v in rng.GetMax()])