#!/usr/bin/env python3
"""
Record /joint_states into an OpenUSD animation (.usda / .usdc).

Builds the kinematic tree from a URDF, then authors one time sample per frame
on each joint's `xformOp:orient` attribute. The result opens in usdview /
Omniverse with working timeline scrubbing.

Each joint is represented by an Xform whose orient op is authored per frame.
Revolute joints get a quaternion about the joint axis (exact for any axis);
prismatic joints get a translate sample along that axis.

Usage (inside the ROS container):
    python3 record_usd_animation.py \
        --urdf /ws/assets/urdf/demo_arm.urdf \
        --out  /ws/data/usd/arm_anim.usda \
        --duration 6 --rate 20
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade


def parse_vec(s, n=3):
    parts = (s or "").replace(",", " ").split()
    vals = [float(p) for p in parts] if parts else []
    if len(vals) < n:
        vals += [0.0] * (n - len(vals))
    return vals[:n]


def quat_about_axis(axis, angle):
    """Gf.Quatf rotating `angle` radians about `axis` (matches orient op)."""
    a = Gf.Vec3d(*axis).GetLength()
    if a < 1e-12:
        return Gf.Quatf(1.0, Gf.Vec3f(0, 0, 0))
    unit = Gf.Vec3f(*axis) / float(a)
    half = angle * 0.5
    return Gf.Quatf(math.cos(half), unit * math.sin(half))


def build_tree(urdf_path):
    """Return (root, links_by_name, joints) from the URDF."""
    root = ET.parse(urdf_path).getroot()
    links = {l.get("name"): l for l in root.findall("link")}
    joints = {}
    for j in root.findall("joint"):
        pel, cel = j.find("parent"), j.find("child")
        if pel is None or cel is None:
            continue
        origin = j.find("origin")
        axis_el = j.find("axis")
        limit_el = j.find("limit")
        joints[j.get("name")] = {
            "name": j.get("name"),
            "type": j.get("type", "fixed"),
            "parent": pel.get("link"),
            "child": cel.get("link"),
            "xyz": parse_vec(origin.get("xyz") if origin is not None else None),
            "rpy": parse_vec(origin.get("rpy") if origin is not None else None),
            "axis": parse_vec(axis_el.get("xyz") if axis_el is not None else None),
            # Needed by the auto-drive to keep the sweep inside the stops.
            "lower": (float(limit_el.get("lower"))
                      if limit_el is not None and limit_el.get("lower") is not None
                      else None),
            "upper": (float(limit_el.get("upper"))
                      if limit_el is not None and limit_el.get("upper") is not None
                      else None),
        }
    return root, links, joints


def find_material_colour(root, name):
    for m in root.findall("material"):
        if m.get("name") == name:
            col = m.find("color")
            if col is not None:
                rgba = parse_vec(col.get("rgba"), 4)
                return rgba[:3], (rgba[3] if len(rgba) > 3 else 1.0)
    return [0.7, 0.7, 0.7], 1.0


def rpy_quatf(r, p, y):
    """Gf.Quatf from roll/pitch/yaw (radians) - matches the float orient op."""
    q = Gf.Quatf(1.0, Gf.Vec3f(0, 0, 0))
    if abs(y) > 1e-12:
        q = q * (Gf.Quatf(1.0, Gf.Vec3f(0, 0, 1)) * math.radians(y))
    if abs(p) > 1e-12:
        q = q * (Gf.Quatf(1.0, Gf.Vec3f(0, 1, 0)) * math.radians(p))
    if abs(r) > 1e-12:
        q = q * (Gf.Quatf(1.0, Gf.Vec3f(1, 0, 0)) * math.radians(r))
    return q


def add_link_geometry(stage, prim_path, link_node, root):
    """Author a link's visual geometry (and material) under prim_path."""
    vis = link_node.find("visual")
    if vis is None:
        return False
    geom = vis.find("geometry")
    if geom is None:
        return False

    origin = vis.find("origin")
    xyz = parse_vec(origin.get("xyz") if origin is not None else None)
    r, p, y = parse_vec(origin.get("rpy") if origin is not None else None)

    x = UsdGeom.Xform.Define(stage, prim_path)
    x.AddTranslateOp().Set(Gf.Vec3d(*xyz))
    x.AddOrientOp().Set(rpy_quatf(r, p, y))

    gp = prim_path + "/geom"
    if geom.find("box") is not None:
        c = UsdGeom.Cube.Define(stage, gp)
        c.CreateSizeAttr(1.0)
        c.AddScaleOp().Set(Gf.Vec3f(*parse_vec(geom.find("box").get("size"))))
    elif geom.find("cylinder") is not None:
        ce = geom.find("cylinder")
        c = UsdGeom.Cylinder.Define(stage, gp)
        c.CreateAxisAttr("Z")
        c.CreateHeightAttr(float(ce.get("length")))
        c.CreateRadiusAttr(float(ce.get("radius")))
    elif geom.find("sphere") is not None:
        s = UsdGeom.Sphere.Define(stage, gp)
        s.CreateRadiusAttr(float(geom.find("sphere").get("radius")))
    else:
        return False

    mat_node = vis.find("material")
    if mat_node is not None:
        rgb, alpha = find_material_colour(root, mat_node.get("name"))
        sh = UsdShade.Shader.Define(stage, gp + "/surface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(alpha)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.45)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        mat = UsdShade.Material.Define(stage, gp + "/material")
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        try:
            UsdShade.MaterialBindingAPI.Apply(
                UsdGeom.Gprim(stage.GetPrimAtPath(gp))).Bind(mat)
        except Exception:
            pass
    return True
def build_stage(stage, root_prim, root, links, joints):
    """Lay out the kinematic tree and return {joint_name: (prim, op_attr)}.

    Topology:  /<root>/<link>/joint_<name>/<child_link>/<child_link>_visual
    The joint Xform holds the animation ops; the child link holds geometry.
    """
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    robot = UsdGeom.Xform.Define(stage, "/" + root_prim)
    stage.SetDefaultPrim(robot.GetPrim())

    anim = {}
    children = set()
    for j in joints.values():
        children.add(j["child"])

    # Root links (not the child of any joint).
    for name, node in links.items():
        if name in children:
            continue
        add_link_geometry(stage, f"/{root_prim}/{name}", node, root)

    for j in joints.values():
        base = f"/{root_prim}/{j['parent']}/joint_{j['name']}"
        jx = UsdGeom.Xform.Define(stage, base)
        r, p, y = j["rpy"]
        origin_q = rpy_quatf(r, p, y)

        if j["type"] == "prismatic":
            trans = jx.AddTranslateOp()
            prim = stage.GetPrimAtPath(base)
            prim.SetMetadata("comment",
                             f"joint={j['name']} type=prismatic axis={j['axis']}")
            anim[j["name"]] = (prim, trans, "prismatic",
                               (tuple(j["xyz"]), tuple(j["axis"])))
        else:
            orient = jx.AddOrientOp()
            orient.Set(origin_q)  # default pose; overwritten per frame
            prim = stage.GetPrimAtPath(base)
            prim.SetMetadata("comment",
                             f"joint={j['name']} type={j['type']} axis={j['axis']}")
            anim[j["name"]] = (prim, orient, "revolute", tuple(j["axis"]))

        child_path = f"{base}/{j['child']}"
        add_link_geometry(stage, child_path, links.get(j["child"]), root)

    return anim


def sample_frame(anim, positions, time_code):
    """Author one frame of joint values onto the USD ops."""
    for jname, (prim, op, kind, extra) in anim.items():
        value = positions.get(jname, 0.0)
        if kind == "prismatic":
            base_xyz, axis = extra
            unit = Gf.Vec3d(*axis)
            if unit.GetLength() > 1e-12:
                unit = unit / unit.GetLength()
            op.Set(Gf.Vec3d(*base_xyz) + unit * value, Usd.TimeCode(time_code))
        else:
            axis = extra
            q = quat_about_axis(axis, value)
            op.Set(q, Usd.TimeCode(time_code))


def main():
    ap = argparse.ArgumentParser(
        description="Record /joint_states into an animated OpenUSD file")
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="arm")
    ap.add_argument("--topic", default="/joint_states")
    ap.add_argument("--duration", type=float, default=5.0,
                    help="seconds of motion to record")
    ap.add_argument("--rate", type=float, default=20.0,
                    help="time samples per second")
    ap.add_argument("--binary", action="store_true",
                    help="write .usdc (binary crate) instead of .usda text")
    ap.add_argument("--timeout", type=float, default=15.0,
                    help="wait this long for the first message")
    ap.add_argument("--auto-drive", action="store_true",
                    help="publish a sinusoidal trajectory to "
                         "~arm/command while recording, so the capture "
                         "always contains motion")
    ap.add_argument("--amp", type=float, default=0.7,
                    help="auto-drive amplitude in radians")
    args = ap.parse_args()

    if not os.path.exists(args.urdf):
        print(f"[record_usd] ERROR: not found: {args.urdf}", file=sys.stderr)
        sys.exit(1)

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    root, links, joints = build_tree(args.urdf)

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    stage = Usd.Stage.CreateNew(args.out)

    # Set timeline metadata BEFORE authoring so time samples land on a stage
    # that already advertises the intended frame rate.
    fps = args.rate
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetStartTimeCode(0.0)
    stage.SetTimeCodesPerSecond(fps)
    stage.SetFramesPerSecond(fps)
    stage.SetMetadata(
        "comment",
        f"Recorded from {args.topic} (URDF: {args.urdf}) at {fps} Hz")

    anim = build_stage(stage, args.name, root, links, joints)

    # Actuated joints, in URDF order, with the limits the auto-drive must
    # respect. Continuous joints get a symmetric range since they have none.
    drive_joints = []
    for jname, j in joints.items():
        if j["type"] not in ("revolute", "prismatic"):
            continue
        lo, hi = j.get("lower"), j.get("upper")
        if lo is None or hi is None:
            lo, hi = -args.amp, args.amp
        drive_joints.append((jname, float(lo), float(hi)))
    print(f"[record_usd] auto-drive targets {len(drive_joints)} actuated "
          f"joint(s): {', '.join(j for j, _, _ in drive_joints)}", flush=True)

    rclpy.init()
    node = Node("usd_animation_recorder")
    samples = []
    got = {"any": False}

    def on_js(msg: JointState):
        got["any"] = True
        samples.append(dict(zip(list(msg.name), [float(v) for v in msg.position])))
        if len(samples) <= 2 or len(samples) % 25 == 0:
            print(f"[record_usd] sample {len(samples)}: "
                  f"{ {k: round(v, 4) for k, v in list(zip(msg.name, msg.position))[:3]} }",
                  flush=True)

    sub = node.create_subscription(JointState, args.topic, on_js, 50)

    # Optional: drive the arm ourselves so the recording has motion.
    cmd_pub = None
    if args.auto_drive:
        from std_msgs.msg import Float64MultiArray
        cmd_pub = node.create_publisher(Float64MultiArray,
                                        "/arm_sim/arm/command", 10)
        print("[record_usd] auto-drive ON: publishing a sinusoidal "
              "trajectory on /arm_sim/arm/command", flush=True)
        node.declare_parameter("demo_amp", args.amp)

    print(f"[record_usd] waiting for {args.topic} ...", flush=True)
    import time as _time
    t0 = _time.time()
    while not got["any"] and (_time.time() - t0) < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not got["any"]:
        print("[record_usd] ERROR: no messages received. Is the sim running?",
              file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    # Collect samples for the requested duration.
    print(f"[record_usd] recording {args.duration}s at {args.rate} Hz "
          f"({int(args.duration * args.rate)} frames)...", flush=True)
    t0 = _time.time()
    last_drive = -1.0
    while (_time.time() - t0) < args.duration:
        rclpy.spin_once(node, timeout_sec=0.02)

        if cmd_pub is not None:
            el = _time.time() - t0
            if el - last_drive >= 0.05:   # ~20 Hz command rate
                last_drive = el
                w = 2 * math.pi * 0.35    # 0.35 Hz sweep
                msg = Float64MultiArray()
                # One phase-shifted sinusoid per actuated joint, amplitude
                # scaled to each joint's own URDF limits. The previous fixed
                # 7-value list was the demo arm's layout: publishing it to
                # the FR3 drove the wrong joints and pinned several against
                # their stops, so the "animation" was a near-static pose.
                msg.data = []
                for k, (jn, lo, hi) in enumerate(drive_joints):
                    span = min(hi - lo, 1.2)          # keep it comfortable
                    mid = 0.5 * (lo + hi)
                    amp = 0.5 * span * (0.6 if k % 2 else 1.0)
                    msg.data.append(mid + amp * math.sin(w * el + 0.7 * k))
                cmd_pub.publish(msg)

    node.destroy_node()
    rclpy.shutdown()

    # Downsample to the requested rate using the wall-clock cadence.
    if not samples:
        print("[record_usd] ERROR: no samples captured", file=sys.stderr)
        sys.exit(3)
    step = max(1, int(round(len(samples) / max(1, args.duration * args.rate))))
    used = samples[::step][: int(args.duration * args.rate)]

    for i, pose in enumerate(used):
        sample_frame(anim, pose, float(i))

    stage.GetRootLayer().Save()
    size = os.path.getsize(args.out)
    print(f"[record_usd] wrote {args.out}")
    print(f"[record_usd] frames={len(used)} rate={args.rate}Hz "
          f"duration={len(used)/args.rate:.2f}s size={size}B")
    print(f"[record_usd] timeCodeRange = [0, {len(used)-1}]")


if __name__ == "__main__":
    main()