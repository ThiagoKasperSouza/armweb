#!/usr/bin/env python3
"""
URDF -> OpenUSD (.usda) converter built directly on the pxr API.

Why native: the pip `usd-core` wheel ships the core pxr modules but NOT
`pxr.UsdUtils.UsdUrdfParser`, so we map the URDF tree onto UsdGeom Xforms
ourselves. Keeps the pipeline fully CPU-only and headless.

Produces a static (single-pose) scene. See record_usd_animation.py for the
time-sampled (animated) variant.
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


def find_material(root, name):
    for m in root.findall("material"):
        if m.get("name") == name:
            col = m.find("color")
            if col is not None:
                return parse_vec(col.get("rgba"), 4)[:3]
    return [0.7, 0.7, 0.7]


def add_visual(prim_path, link_node, root, stage, cache, urdf_dir):
    """Author one link's visual geometry and material under prim_path."""
    vis = link_node.find("visual")
    if vis is None:
        return
    geom = vis.find("geometry")
    if geom is None:
        return

    child_path = prim_path + "/" + link_node.get("name") + "_visual"
    origin = vis.find("origin")
    xyz = parse_vec(origin.get("xyz") if origin is not None else None)
    rpy = parse_vec(origin.get("rpy") if origin is not None else None)

    xform = UsdGeom.Xform.Define(stage, child_path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*xyz))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*[math.degrees(a) for a in rpy]))

    gpath = child_path + "/geom"
    if geom.find("box") is not None:
        size = parse_vec(geom.find("box").get("size"))
        cube = UsdGeom.Cube.Define(stage, gpath)
        cube.CreateSizeAttr(1.0)
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
    elif geom.find("cylinder") is not None:
        cel = geom.find("cylinder")
        cyl = UsdGeom.Cylinder.Define(stage, gpath)
        cyl.CreateAxisAttr("Z")
        cyl.CreateHeightAttr(float(cel.get("length")))
        cyl.CreateRadiusAttr(float(cel.get("radius")))
    elif geom.find("sphere") is not None:
        sph = UsdGeom.Sphere.Define(stage, gpath)
        sph.CreateRadiusAttr(float(geom.find("sphere").get("radius")))
    elif geom.find("mesh") is not None:
        fn = geom.find("mesh").get("filename")
        if fn and os.path.exists(os.path.join(urdf_dir, fn)):
            UsdGeom.Mesh.Define(stage, gpath)
        else:
            return
    else:
        return

    mat_node = vis.find("material")
    if mat_node is None:
        return
    mat_name = mat_node.get("name")
    if mat_name not in cache:
        cache[mat_name] = find_material(root, mat_name)
    rgb = cache[mat_name]

    shader = UsdShade.Shader.Define(stage, gpath + "/surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)

    mat = UsdShade.Material.Define(stage, gpath + "/material")
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    try:
        gprim = UsdGeom.Gprim(stage.GetPrimAtPath(gpath))
        UsdShade.MaterialBindingAPI.Apply(gprim).Bind(mat)
    except Exception:
        pass
def convert(urdf_path, out_path, root_prim_name):
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    root = ET.parse(urdf_path).getroot()
    robot_name = root.get("name", root_prim_name)

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    robot_path = "/" + root_prim_name
    robot = UsdGeom.Xform.Define(stage, robot_path)
    stage.SetDefaultPrim(robot.GetPrim())
    stage.SetMetadata("comment", f"Converted from URDF '{robot_name}' ({urdf_path})")

    cache = {}
    for lnode in root.findall("link"):
        add_visual(robot_path, lnode, root, stage, cache, urdf_dir)

    joint_count = 0
    for jnode in root.findall("joint"):
        jtype = jnode.get("type", "fixed")
        pel = jnode.find("parent")
        if pel is None:
            continue
        parent = pel.get("link")
        origin = jnode.find("origin")
        xyz = parse_vec(origin.get("xyz") if origin is not None else None)
        rpy = parse_vec(origin.get("rpy") if origin is not None else None)

        jname = jnode.get("name")
        jpath = f"{robot_path}/{parent}/joint_{jname}"
        jx = UsdGeom.Xform.Define(stage, jpath)
        jx.AddTranslateOp().Set(Gf.Vec3d(*xyz))
        jx.AddRotateXYZOp().Set(Gf.Vec3f(*[math.degrees(a) for a in rpy]))

        cel = jnode.find("child")
        if cel is not None:
            UsdGeom.Xform.Define(stage, f"{jpath}/{cel.get('link')}")

        axis = jnode.find("axis")
        limit = jnode.find("limit")
        bits = [f"joint={jname}", f"type={jtype}"]
        if axis is not None:
            bits.append("axis=" + axis.get("xyz", ""))
        if limit is not None:
            bits.append(f"lower={limit.get('lower')}")
            bits.append(f"upper={limit.get('upper')}")
        stage.GetPrimAtPath(jpath).SetMetadata("comment", " ".join(bits))
        joint_count += 1

    stage.GetRootLayer().Save()
    print(f"[export_usd] wrote {out_path}")
    print(f"[export_usd] robot={robot_name} joints={joint_count} "
          f"prims={len(list(stage.Traverse()))}")


def main():
    ap = argparse.ArgumentParser(description="Convert a URDF robot to OpenUSD")
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="arm")
    args = ap.parse_args()

    if not os.path.exists(args.urdf):
        print(f"[export_usd] ERROR: not found: {args.urdf}", file=sys.stderr)
        sys.exit(1)
    convert(args.urdf, args.out, args.name)


if __name__ == "__main__":
    main()