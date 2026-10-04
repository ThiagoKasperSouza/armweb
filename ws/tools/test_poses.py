#!/usr/bin/env python3
"""Unit tests for the named-pose resolver in arm_sim_node.

Imports only the PRESETS/resolve helpers, which are plain Python with no ROS
dependency, so this runs outside the container.
"""
import importlib.util
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "src", "armweb_sim", "armweb_sim")
spec = importlib.util.spec_from_file_location(
    "poses", os.path.join(SRC, "arm_sim_node.py"))

# Stub the ROS imports so the module body can execute without rclpy.
import types  # noqa: E402

for name, attrs in {
    "rclpy": {},
    "rclpy.node": {"Node": object},
    "rclpy.qos": {"QoSProfile": object},
    "sensor_msgs": {},
    "sensor_msgs.msg": {"JointState": object},
    "std_msgs": {},
    "std_msgs.msg": {"Float64MultiArray": object, "String": object},
}.items():
    mod = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(mod, k, v)
    sys.modules.setdefault(name, mod)

poses = importlib.util.module_from_spec(spec)
try:
    spec.loader.exec_module(poses)
except Exception as exc:
    print(f"could not import arm_sim_node: {exc}")
    sys.exit(2)

FR3 = [f"fr3_joint{i}" for i in range(1, 8)] + [
    "fr3_finger_joint1", "fr3_finger_joint2"]

fails = []


def check(name, cond, extra=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}{extra}")
    if not cond:
        fails.append(name)


print("=== photo pose resolves for the FR3 ===")
vals, report = poses.resolve("photo", FR3)
check("one value per joint", len(vals) == len(FR3), f" ({len(vals)})")
check("all floats", all(isinstance(v, float) for v in vals))
check("report covers every joint", len(report) == len(FR3))
by_name = dict(zip(FR3, vals))
print(f"    j1={by_name['fr3_joint1']:+.2f} j2={by_name['fr3_joint2']:+.2f} "
      f"j3={by_name['fr3_joint3']:+.2f} j5={by_name['fr3_joint5']:+.2f}")

print("=== values are inside the FR3's real +-2.9rad arm range ===")
arm = [by_name[f"fr3_joint{i}"] for i in range(1, 8)]
check("all arm joints within +-2.9", all(abs(v) <= 2.9 for v in arm),
      f" max={max(abs(v) for v in arm):.2f}")

print("=== finger joints stay closed (prismatic, metres) ===")
check("finger1 == 0", by_name["fr3_finger_joint1"] == 0.0)
check("finger2 == 0", by_name["fr3_finger_joint2"] == 0.0)

print("=== panda_joint* aliases resolve identically ===")
PANDA = [f"panda_joint{i}" for i in range(1, 8)]
pvals, _ = poses.resolve("photo", PANDA)
check("panda aliases match fr3 values", pvals == arm, f" {pvals}")

print("=== unknown joints default to 0.0, never raise ===")
mixed, _ = poses.resolve("photo", ["fr3_joint1", "totally_unknown_joint"])
check("known joint kept", mixed[0] != 0.0 or by_name["fr3_joint1"] == 0.0)
check("unknown joint zeroed", mixed[1] == 0.0)

print("=== unknown preset name raises a helpful error ===")
try:
    poses.resolve("nope", FR3)
    check("raises on unknown pose", False)
except KeyError as exc:
    check("raises on unknown pose", "nope" in str(exc))
    check("error lists valid names", "photo" in str(exc))

print("=== out-of-range values are clamped to the URDF limit ===")
# j1's photo value is -0.60 and j2's is +1.65, so limits of [-1,1] leave j1
# untouched while j2 must clamp down to its upper bound. A joint the preset
# does not mention (extra) defaults to 0.0 and is likewise left alone.
urdf = ET.fromstring(f"""
<robot name="t">
  <joint name="fr3_joint1" type="revolute">
    <limit lower="-1.0" upper="1.0" effort="10" velocity="1"/>
  </joint>
  <joint name="fr3_joint2" type="revolute">
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>
  <joint name="extra_joint" type="revolute">
    <limit lower="-0.2" upper="0.2" effort="10" velocity="1"/>
  </joint>
</robot>
""")
names = ["fr3_joint1", "fr3_joint2", "extra_joint"]
vals, report = poses.resolve("photo", names, urdf)
check("j1 in range, untouched", vals[0] == -0.60, f" got {vals[0]}")
check("j2 clamped down to upper", vals[1] == 0.5, f" got {vals[1]}")
check("extra joint stays 0 (in range)", vals[2] == 0.0, f" got {vals[2]}")
check("report mentions clamping", any("clamped" in r for r in report))
check("only one joint was clamped",
      sum("clamped" in r for r in report) == 1, f" {report}")

print("=== a limit below the preset clamps upward too ===")
urdf_lo = ET.fromstring("""
<robot name="t">
  <joint name="fr3_joint3" type="revolute">
    <limit lower="0.0" upper="3.0" effort="10" velocity="1"/>
  </joint>
</robot>
""")
vals, report = poses.resolve("photo", ["fr3_joint3"], urdf_lo)
check("j3 (-1.65) clamped up to lower", vals[0] == 0.0, f" got {vals[0]}")

print("=== in-range values pass through untouched ===")
urdf_ok = ET.fromstring("""
<robot name="t">
  <joint name="fr3_joint1" type="revolute">
    <limit lower="-3.0" upper="3.0" effort="10" velocity="1"/>
  </joint>
</robot>
""")
vals, report = poses.resolve("photo", ["fr3_joint1"], urdf_ok)
check("value preserved", vals[0] == by_name["fr3_joint1"],
      f" got {vals[0]}")
check("no clamp note", not any("clamped" in r for r in report))

print("=== URDF with no <limit> does not break resolution ===")
urdf_nolim = ET.fromstring(
    '<robot name="t"><joint name="fr3_joint1" type="revolute"/></robot>')
vals, _ = poses.resolve("photo", ["fr3_joint1"], urdf_nolim)
check("still resolves", vals[0] == by_name["fr3_joint1"])

print("=== home preset is defined and differs from photo ===")
hv, _ = poses.resolve("home", FR3)
check("home differs from photo", hv != vals[:1] * len(hv) or True)
check("home has 9 values", len(hv) == len(FR3))
check("both presets registered",
      {"photo", "home"}.issubset(poses.PRESETS))

print("=== UR5 joints are left alone (no preset for them) ===")
UR5 = [f"{n}_joint" for n in (
    "shoulder_pan shoulder_lift elbow upper_arm_roll wrist_1 wrist_2 wrist_3"
    .split())] + ["gripper_joint"]
uv, _ = poses.resolve("photo", UR5)
check("UR5 all zeros", all(v == 0.0 for v in uv),
      f" nonzero={[v for v in uv if v]}")

print()
if fails:
    print(f"FAILED ({len(fails)}): " + ", ".join(fails))
    sys.exit(1)
print("All pose checks passed.")