#!/usr/bin/env python3
"""Verify a recorded USD animation: time samples present, values vary."""
import sys
from pxr import Usd, UsdGeom

path = sys.argv[1] if len(sys.argv) > 1 else "/ws/data/usd/arm_anim.usda"
stage = Usd.Stage.Open(path)
if stage is None:
    raise SystemExit(f"FAILED to open {path}")

print("stage:", path)
print("timeCodesPerSecond:", stage.GetTimeCodesPerSecond())
print("startTimeCode:", stage.GetStartTimeCode())

total_samples = 0
animated = 0
for prim in stage.Traverse():
    if not prim.IsA(UsdGeom.Xform):
        continue
    xf = UsdGeom.Xform(prim)
    # This pxr build has no GetOrientAttr/GetTranslateAttr on UsdGeom.Xform,
    # so read the attributes through the ordered xform ops.
    for kind, label in ((UsdGeom.XformOp.TypeOrient, "orient"),
                        (UsdGeom.XformOp.TypeTranslate, "translate")):
        attr = None
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == kind:
                attr = op.GetAttr()
                break
        if attr is None:
            continue
        samples = attr.GetTimeSamples()
        if not samples:
            continue
        total_samples += len(samples)
        animated += 1
        first = attr.Get(samples[0])
        last = attr.Get(samples[-1])
        moved = str(first) != str(last)
        print(f"  {prim.GetName()}.{label}: {len(samples)} samples, "
              f"t=[{samples[0]}..{samples[-1]}], changes={moved}")

print(f"\ntotal time samples: {total_samples}")
print(f"animated attributes: {animated}")
if total_samples == 0:
    raise SystemExit("FAIL: no time samples were authored")
print("OK: animation data present")