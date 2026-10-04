#!/usr/bin/env python3
"""Round-trip test: URDF -> glTF joint metadata -> recover the angle.

Mirrors what viewer.html will do in the browser: read each joint_<name> node's
extras (axis, origin_rpy, limits), apply a slider value, build the same
quaternion export_gltf.py would build, and read the angle back out of it.

Pure stdlib: no numpy, no pycollada.
"""
import json
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import export_gltf as ex  # noqa: E402

fails = []


def check(name, cond, extra=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}{extra}")
    if not cond:
        fails.append(name)


def qsub(q, v):
    """Rotate vector v by quaternion q (x,y,z,w) -- conjugate trick."""
    r = ex.quat_mul(list(q), [v[0], v[1], v[2], 0])
    c = [-q[0], -q[1], -q[2], q[3]]
    r = ex.quat_mul(r, c)
    return (r[0], r[1], r[2])


def qangle(q):
    return 2.0 * math.atan2(math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2), q[3])


def qnorm(q):
    return math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)


def qsame(a, b, tol=1e-9):
    """Quaternions are equal up to sign: q and -q are the same rotation."""
    return (all(abs(x - y) <= tol for x, y in zip(a, b))
            or all(abs(x + y) <= tol for x, y in zip(a, b)))


def apply_angle(meta, value):
    """The quaternion the viewer will assign for a given slider value."""
    return ex.quat_mul(ex.rpy_quat(*meta["origin_rpy"]),
                       ex.quat_axis_angle(meta["axis"], value))


URDF = """
<robot name="rt">
  <link name="base"/><link name="l1"/><link name="l2"/><link name="l3"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-2.9" upper="2.9" effort="10" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="0 0 0.3" rpy="0 0.3 0"/><axis xyz="0 1 0"/>
    <limit lower="-1.8" upper="1.8" effort="10" velocity="1"/>
  </joint>
  <joint name="j3" type="revolute">
    <parent link="l2"/><child link="l3"/>
    <origin xyz="0.1 0 0" rpy="0.2 0 0.4"/><axis xyz="1 0 0"/>
    <limit lower="-2.9" upper="2.9" effort="10" velocity="1"/>
  </joint>
  <joint name="fj" type="prismatic">
    <parent link="l3"/><child link="l3"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="0.04" effort="10" velocity="1"/>
  </joint>
</robot>
"""


def build():
    """Build joint nodes the way export_gltf.py now does, with extras."""
    root = ET.fromstring(URDF)
    b = ex.GltfBuilder()
    links = {}
    for l in root.findall("link"):
        links[l.get("name")] = b.add_node(l.get("name"))

    joints = []
    for j in root.findall("joint"):
        pel, cel = j.find("parent"), j.find("child")
        origin, axis_el, lim = j.find("origin"), j.find("axis"), j.find("limit")
        joints.append({
            "name": j.get("name"), "type": j.get("type", "fixed"),
            "parent": pel.get("link"), "child": cel.get("link"),
            "xyz": ex.parse_vec(origin.get("xyz")),
            "rpy": ex.parse_vec(origin.get("rpy")),
            "axis": ex.parse_vec(axis_el.get("xyz")),
            "lower": float(lim.get("lower")) if lim is not None else None,
            "upper": float(lim.get("upper")) if lim is not None else None,
        })

    for j in joints:
        extras = {"armweb_joint": {
            "name": j["name"], "type": j["type"],
            "axis": list(j["axis"]), "origin_rpy": list(j["rpy"]),
            "origin_xyz": list(j["xyz"]),
        }}
        if j.get("lower") is not None:
            extras["armweb_joint"]["lower"] = j["lower"]
        if j.get("upper") is not None:
            extras["armweb_joint"]["upper"] = j["upper"]

        if j["type"] == "prismatic":
            jn = b.add_node("joint_" + j["name"], translation=j["xyz"],
                            extras=extras)
        else:
            jn = b.add_node(
                "joint_" + j["name"], translation=j["xyz"],
                rotation=ex.quat_mul(ex.rpy_quat(*j["rpy"]),
                                     ex.quat_axis_angle(j["axis"], 0.0)),
                extras=extras)
        b.nodes[links[j["parent"]]].setdefault("children", []).append(jn)
        b.nodes[jn].setdefault("children", []).append(links[j["child"]])
    return b, joints
print("=== builder accepts extras and keeps them on the node ===")
b, joints = build()
jn = [n for n in b.nodes if n["name"].startswith("joint_")]
check("4 named joint nodes", len(jn) == 4, f" got {len(jn)}")
check("every joint node has extras",
      all("extras" in n and "armweb_joint" in n["extras"] for n in jn))
check("extras carries axis and limits",
      all("axis" in n["extras"]["armweb_joint"]
          and "upper" in n["extras"]["armweb_joint"] for n in jn))

by_name = {n["extras"]["armweb_joint"]["name"]: n["extras"]["armweb_joint"]
           for n in jn}

def qsigned_angle(q, axis):
    """Signed angle about `axis` from a unit quaternion (x,y,z,w).

    A quaternion and its negation are the same rotation, so the raw angle from
    atan2(|v|, w) is always in [0, 2*pi) and loses the sign. Recover the sign
    from the direction of the vector part relative to the joint axis.
    """
    x, y, z, w = q[0], q[1], q[2], q[3]
    v = math.sqrt(x * x + y * y + z * z)
    if v < 1e-12:
        return 0.0
    # dot of the (normalised) rotation axis with the joint axis gives the sign
    sign = 1.0 if (x * axis[0] + y * axis[1] + z * axis[2]) >= 0 else -1.0
    ang = 2.0 * math.atan2(v, abs(w))
    return sign * ang


def joint_angle(meta, value):
    """The joint's own angle, recovered from the composed node quaternion.

    The node stores origin_rpy * axis_angle(value), so the total rotation is
    not the joint angle. Undo the origin rotation first, then measure.
    """
    total = apply_angle(meta, value)
    origin = list(ex.rpy_quat(*meta["origin_rpy"]))
    inv = [-origin[0], -origin[1], -origin[2], origin[3]]   # conjugate
    return qsigned_angle(ex.quat_mul(inv, total), meta["axis"])


print("=== angle survives the round trip through the quaternion ===")
for jname, value in [("j1", 0.0), ("j1", 1.2), ("j1", -2.5),
                     ("j2", 0.8), ("j2", -1.5),
                     ("j3", 1.1), ("j3", -0.6)]:
    got = joint_angle(by_name[jname], value)
    check(f"{jname} @ {value:+.2f} rad", abs(got - value) < 1e-6,
          f" -> {got:+.6f}")

print("=== joint axis is recoverable from extras ===")
for jname, expect in [("j1", (0, 0, 1)), ("j2", (0, 1, 0)), ("j3", (1, 0, 0))]:
    ax = by_name[jname]["axis"]
    mag = math.sqrt(sum(c * c for c in ax)) or 1.0
    unit = tuple(round(c / mag, 9) for c in ax)
    check(f"{jname} axis", unit == expect, f" {unit}")

print("=== limits arrive in extras for slider ranges ===")
check("j1 limits", by_name["j1"]["lower"] == -2.9
      and by_name["j1"]["upper"] == 2.9)
check("prismatic limits are metres 0..0.04",
      by_name["fj"]["lower"] == 0.0 and by_name["fj"]["upper"] == 0.04)

print("=== origin_rpy round-trips exactly at value 0 ===")
for jname in ("j1", "j2", "j3"):
    meta = by_name[jname]
    q = apply_angle(meta, 0.0)
    check(f"{jname} rest rotation preserved",
          qsame(q, list(ex.rpy_quat(*meta["origin_rpy"]))), f" {q}")

print("=== a 90 deg turn really rotates the vector as expected ===")
# j3 turns about +X, so +Y must map to +Z. Strip the origin rotation first,
# otherwise its own rpy (0.2 0 0.4) would skew the result.
meta = by_name["j3"]
joint_only = ex.quat_axis_angle(meta["axis"], math.pi / 2)
got = qsub(joint_only, (0, 1, 0))
check("Y axis maps to Z", all(abs(c - e) < 1e-6
                             for c, e in zip(got, (0, 0, 1))), f" {got}")

print("=== quaternions stay unit length (no drift when re-exported) ===")
for jname, value in [("j2", 1.7), ("j3", -2.9)]:
    q = apply_angle(by_name[jname], value)
    check(f"{jname} unit quaternion", abs(qnorm(q) - 1.0) < 1e-9,
          f" |q|={qnorm(q):.12f}")

print("=== extras survive GLB serialisation ===")
path = os.path.join(HERE, "_rt_test.glb")
out = ex.GltfBuilder.serialize_glb(b)
with open(path, "wb") as fh:
    fh.write(out)
raw = open(path, "rb").read()
jlen = struct.unpack("<I", raw[12:16])[0]
gj = json.loads(raw[20:20 + jlen].decode("utf-8"))
jn_g = [n for n in gj["nodes"] if n["name"].startswith("joint_")]
check("extras survive serialisation",
      len(jn_g) == 4 and all("extras" in n for n in jn_g),
      f" {len(jn_g)} nodes")
check("limits in serialised extras",
      all("upper" in n["extras"]["armweb_joint"] for n in jn_g))
os.remove(path)

print()
if fails:
    print(f"FAILED ({len(fails)}): " + ", ".join(fails))
    sys.exit(1)
print("All round-trip checks passed.")