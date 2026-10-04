#!/usr/bin/env python3
"""
URDF -> glTF 2.0 (.glb / .gltf) exporter for browser viewers (three.js).

Why hand-written: the pip `usd-core` wheel has no GLTFWriter, and three.js
cannot display USD directly. glTF is the web-native interchange format, so we
emit it ourselves with real tessellated meshes (glTF has no parametric
primitives like USD's Cube/Cylinder).

Options:
  --animated   also bake the current joint pose as node rotations
  --with-animation FILE
               read a recorded .usda and bake every frame as a glTF animation

Everything is CPU-only and dependency-free (stdlib + pxr for the .usda reader).
"""
import argparse
import json
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from load_mesh import weld

# ------------------------------------------------------------------ geometry


def tessellate_box(sx, sy, sz, center=(0, 0, 0)):
    """Axis-aligned box: 24 verts (per-face normals) + 36 indices."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    cx, cy, cz = center
    faces = [
        ((0, 0, 1), [(-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]),
        ((0, 0, -1), [(-hx, hy, -hz), (hx, hy, -hz), (hx, -hy, -hz), (-hx, -hy, -hz)]),
        ((1, 0, 0), [(hx, -hy, hz), (hx, -hy, -hz), (hx, hy, -hz), (hx, hy, hz)]),
        ((-1, 0, 0), [(-hx, hy, hz), (-hx, hy, -hz), (-hx, -hy, -hz), (-hx, -hy, hz)]),
        ((0, 1, 0), [(-hx, hy, hz), (hx, hy, hz), (hx, hy, -hz), (-hx, hy, -hz)]),
        ((0, -1, 0), [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, -hy, hz), (-hx, -hy, hz)]),
    ]
    positions, normals, indices = [], [], []
    for normal, quad in faces:
        base = len(positions)
        for v in quad:
            positions.append((v[0] + cx, v[1] + cy, v[2] + cz))
            normals.append(normal)
        indices += [base, base + 1, base + 2, base, base + 2, base + 3]
    return positions, normals, indices


def tessellate_cylinder(radius, length, segments=24):
    """Cylinder along +Z, centred at the origin."""
    hz = length / 2
    positions, normals, indices = [], [], []
    for i in range(segments):
        a = 2 * math.pi * i / segments
        ca, sa = math.cos(a), math.sin(a)
        positions += [(radius * ca, radius * sa, -hz),
                      (radius * ca, radius * sa, hz)]
        normals += [(ca, sa, 0.0), (ca, sa, 0.0)]
    for i in range(segments):
        a0 = 2 * i
        a1 = 2 * ((i + 1) % segments)
        indices += [a0, a1, a1 + 1, a0, a1 + 1, a0 + 1]

    # Caps
    for sign in (1, -1):
        c = len(positions)
        positions.append((0.0, 0.0, sign * hz))
        normals.append((0.0, 0.0, float(sign)))
        start = len(positions)
        for i in range(segments):
            a = 2 * math.pi * i / segments
            positions.append((radius * math.cos(a), radius * math.sin(a), sign * hz))
            normals.append((0.0, 0.0, float(sign)))
        for i in range(segments):
            i0, i1 = start + i, start + (i + 1) % segments
            if sign > 0:
                indices += [c, i0, i1]
            else:
                indices += [c, i1, i0]
    return positions, normals, indices


def tessellate_sphere(radius, segments=20, rings=12):
    positions, normals, indices = [], [], []
    for r in range(rings + 1):
        phi = math.pi * r / rings
        for s in range(segments + 1):
            theta = 2 * math.pi * s / segments
            x = radius * math.sin(phi) * math.cos(theta)
            y = radius * math.sin(phi) * math.sin(theta)
            z = radius * math.cos(phi)
            positions.append((x, y, z))
            n = math.hypot(x, y) or 1e-12
            normals.append((x / n, y / n, z / n if abs(z) > 1e-12 else 1.0))
    for r in range(rings):
        for s in range(segments):
            a = r * (segments + 1) + s
            b = a + segments + 1
            indices += [a, b, a + 1, a + 1, b, b + 1]
    return positions, normals, indices
# ------------------------------------------------------------------- glTF doc


class GltfBuilder:
    """Accumulates buffers/meshes/nodes and serialises to .glb."""

    COMP_FLOAT = 5126
    COMP_USHORT = 5123
    COMP_UINT = 5125
    TARGET_ARRAY_BUFFER = 34962
    TARGET_ELEMENT_ARRAY_BUFFER = 34963

    def __init__(self):
        self.blob = bytearray()
        self.buffers = []
        self.buffer_views = []
        self.accessors = []
        self.meshes = []
        self.nodes = []
        self.materials = {}
        self.material_order = []
        self.animations = []

    def _align(self):
        while len(self.blob) % 4:
            self.blob.append(0)

    def _add_view(self, data, target, comp_type=None):
        self._align()
        offset = len(self.blob)
        self.blob.extend(data)
        view = {"buffer": 0, "byteOffset": offset, "byteLength": len(data)}
        if target is not None:
            view["target"] = target
        if comp_type is not None:
            view["componentType"] = comp_type
        self.buffer_views.append(view)
        return len(self.buffer_views) - 1

    def add_vec3_accessor(self, values, with_minmax=True):
        data = b"".join(struct.pack("<3f", *v) for v in values)
        view = self._add_view(data, self.TARGET_ARRAY_BUFFER, self.COMP_FLOAT)
        acc = {
            "bufferView": view, "componentType": self.COMP_FLOAT,
            "count": len(values), "type": "VEC3",
        }
        if with_minmax:
            xs = [v[0] for v in values]
            ys = [v[1] for v in values]
            zs = [v[2] for v in values]
            acc["min"] = [min(xs), min(ys), min(zs)]
            acc["max"] = [max(xs), max(ys), max(zs)]
        self.accessors.append(acc)
        return len(self.accessors) - 1

    def add_index_accessor(self, indices):
        # glTF allows UNSIGNED_INT (5125) indices, and every WebGL2 target
        # (three.js in the browser) supports them natively. Forcing 16-bit
        # caps a mesh at 65535 vertices, which is what used to force the
        # decimator to throw away two thirds of the FR3 triangles and left
        # the links looking shredded. Emit 32-bit whenever the mesh needs it.
        if indices and max(indices) > 65535:
            data = b"".join(struct.pack("<I", i) for i in indices)
            comp = self.COMP_UINT
        else:
            data = b"".join(struct.pack("<H", i) for i in indices)
            comp = self.COMP_USHORT
        view = self._add_view(data, self.TARGET_ELEMENT_ARRAY_BUFFER, comp)
        self.accessors.append({
            "bufferView": view, "componentType": comp,
            "count": len(indices), "type": "SCALAR",
        })
        return len(self.accessors) - 1

    def material_index(self, rgb, alpha=1.0, metallic=0.0, roughness=0.45):
        key = (tuple(rgb), round(alpha, 3))
        if key in self.materials:
            return self.materials[key]
        idx = len(self.material_order)
        self.material_order.append({
            "name": "mat_%d" % idx,
            "pbrMetallicRoughness": {
                "baseColorFactor": [rgb[0], rgb[1], rgb[2], alpha],
                "metallicFactor": metallic,
                "roughnessFactor": roughness,
            },
            "doubleSided": True,
        })
        self.materials[key] = idx
        return idx

    def add_mesh(self, positions, normals, indices, material):
        pos_acc = self.add_vec3_accessor(positions)
        nrm_acc = self.add_vec3_accessor(normals)
        idx_acc = self.add_index_accessor(indices)
        self.meshes.append({
            "name": "mesh_%d" % len(self.meshes),
            "primitives": [{
                "attributes": {"POSITION": pos_acc, "NORMAL": nrm_acc},
                "indices": idx_acc,
                "material": material,
                "mode": 4,
            }],
        })
        return len(self.meshes) - 1

    def add_node(self, name, mesh=None, translation=None, rotation=None,
                 scale=None, children=None, extras=None):
        node = {"name": name}
        if mesh is not None:
            node["mesh"] = mesh
        if translation is not None:
            node["translation"] = list(translation)
        if rotation is not None:
            node["rotation"] = list(rotation)
        if scale is not None:
            node["scale"] = list(scale)
        if children:
            node["children"] = list(children)
        if extras is not None:
            node["extras"] = extras
        self.nodes.append(node)
        return len(self.nodes) - 1

    def serialize_glb(self):
        if not self.blob:
            self.blob = bytearray(b"\x00\x00\x00\x00")
        # Only the true hierarchy roots are scene nodes; putting every node in
        # the scene root list flattens the arm and makes it look disjointed.
        roots = getattr(self, "scene_roots", None)
        if roots is None:
            roots = list(range(len(self.nodes)))
        gltf = {
            "asset": {
                "version": "2.0",
                "generator": "armweb export_gltf.py (URDF -> glTF)",
            },
            "scene": 0,
            "scenes": [{"name": "arm", "nodes": list(roots)}],
            "nodes": self.nodes,
            "meshes": self.meshes,
            "materials": self.material_order,
            "accessors": self.accessors,
            "bufferViews": self.buffer_views,
            "buffers": [{"byteLength": len(self.blob)}],
        }
        if self.animations:
            gltf["animations"] = self.animations

        json_bytes = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
        json_bytes += b" " * ((4 - len(json_bytes) % 4) % 4)
        bin_bytes = bytes(self.blob)
        bin_bytes += b"\x00" * ((4 - len(bin_bytes) % 4) % 4)

        total = 12 + 8 + len(json_bytes) + 8 + len(bin_bytes)
        out = bytearray()
        out += struct.pack("<III", 0x46546C67, 2, total)
        out += struct.pack("<II", len(json_bytes), 0x4E4F534A)
        out += json_bytes
        out += struct.pack("<II", len(bin_bytes), 0x004E4942)
        out += bin_bytes
        return bytes(out)

    def write_glb(self, path):
        data = self.serialize_glb()
        with open(path, "wb") as fh:
            fh.write(data)
        return len(data)


# --------------------------------------------------------------- URDF -> glTF


def parse_vec(s, n=3):
    parts = (s or "").replace(",", " ").split()
    vals = [float(p) for p in parts] if parts else []
    if len(vals) < n:
        vals += [0.0] * (n - len(vals))
    return vals[:n]


def rpy_quat(r, p, y):
    """glTF rotation quaternion (x, y, z, w) from roll/pitch/yaw (radians)."""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def quat_axis_angle(axis, angle):
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    s = math.sin(angle / 2)
    return [ax / n * s, ay / n * s, az / n * s, math.cos(angle / 2)]


def material_colour(root, name):
    for m in root.findall("material"):
        if m.get("name") == name:
            col = m.find("color")
            if col is not None:
                rgba = parse_vec(col.get("rgba"), 4)
                return rgba[:3], (rgba[3] if len(rgba) > 3 else 1.0)
    return [0.7, 0.7, 0.7], 1.0


def link_mesh(builder, link_node, root, urdf_dir=".", scale_hint=(1.0, 1.0, 1.0)):
    """Build a glTF mesh for one link's visual, or None.

    Prefers a real <mesh> reference (STL/DAE/OBJ) when the description has
    one, and falls back to tessellating the primitive otherwise.

    scale_hint multiplies the mesh: some descriptions (franka_description,
    for instance) author meshes in millimetres without declaring a <scale>,
    so the caller passes 0.001 to bring them to the URDF metre convention.
    """
    vis = link_node.find("visual") if link_node is not None else None
    if vis is None:
        return None
    geom = vis.find("geometry")
    if geom is None:
        return None

    # <material> may be at the <visual> level or inside <geometry>.
    mat_node = vis.find("material")
    if mat_node is None and geom.find("material") is not None:
        mat_node = geom.find("material")
    if mat_node is None:
        colour, alpha, metallic, roughness = [0.7, 0.7, 0.7], 1.0, 0.0, 0.45
    else:
        rgb, alpha = material_colour(root, mat_node.get("name"))
        reference = mat_node.find("reference")
        metallic, roughness = 0.0, 0.45
        if reference is not None:
            if reference.get("metallic") is not None:
                metallic = float(reference.get("metallic"))
            if reference.get("roughness") is not None:
                roughness = float(reference.get("roughness"))
        colour = rgb
    mat = builder.material_index(colour, alpha, metallic, roughness)

    # 1) Real mesh reference. An explicit <scale> in the URDF wins; otherwise
    # apply scale_hint (e.g. 0.001 for millimetre-authored meshes).
    mesh_el = geom.find("mesh")
    if mesh_el is not None and mesh_el.get("filename"):
        loaded = load_mesh_for(mesh_el, urdf_dir, scale_hint)
        if loaded is not None:
            positions, normals, indices = loaded
            # The DAE readers hand back a triangle soup: one vertex per
            # corner, so the FR3 links arrive with ~3x more vertices than
            # they need. Weld them first, which both shrinks the file and
            # keeps every triangle (the old 16-bit ceiling forced a 2/3
            # decimate and the links rendered shredded).
            positions, normals, indices = weld(positions, normals, indices)
            if positions and indices:
                return builder.add_mesh(positions, normals, indices, mat)

    # 2) Primitive fallback.
    if geom.find("box") is not None:
        sx, sy, sz = parse_vec(geom.find("box").get("size"))
        pos, nrm, idx = tessellate_box(sx, sy, sz)
    elif geom.find("cylinder") is not None:
        ce = geom.find("cylinder")
        pos, nrm, idx = tessellate_cylinder(float(ce.get("radius")),
                                            float(ce.get("length")))
    elif geom.find("sphere") is not None:
        pos, nrm, idx = tessellate_sphere(float(geom.find("sphere").get("radius")))
    else:
        return None

    return builder.add_mesh(pos, nrm, idx, mat)


def load_mesh_for(mesh_el, urdf_dir, scale_hint=(1.0, 1.0, 1.0)):
    """Resolve and load a <mesh> element, applying <scale> or scale_hint."""
    import os
    from load_mesh import load_mesh, resolve_package_uri

    uri = mesh_el.get("filename")
    scale_el = mesh_el.find("scale")
    if scale_el is not None and scale_el.get("xyz"):
        scale = tuple(parse_vec(scale_el.get("xyz")))
    else:
        scale = scale_hint

    path = resolve_package_uri(uri)
    if path is None and uri and not uri.startswith("package://"):
        # Relative path, resolved against the URDF's directory.
        cand = os.path.join(urdf_dir, uri)
        path = cand if os.path.isfile(cand) else None
    return load_mesh(path, scale) if path else None


def detect_mesh_scale(urdf_path, root, links, joints, verbose=False):
    """Infer the scale factor that puts mesh units into metres.

    URDF kinematics are always metres, but mesh files may be authored in
    millimetres (franka_description does) without declaring <scale>. We compare
    the raw extent of a root link's mesh against the robot's own reach and snap
    to the nearest power of ten.

    Returns (1.0, 1.0, 1.0) when there is nothing to infer from.
    """
    import os
    from load_mesh import load_mesh, resolve_package_uri

    # Reach of the kinematic tree, in metres (sum of joint offset lengths).
    reach = 0.0
    for j in joints:
        reach += math.sqrt(sum(c * c for c in j["xyz"]))
    if reach <= 0.05:
        return (1.0, 1.0, 1.0)

    child_links = {j["child"] for j in joints}
    roots = [n for n in links if n not in child_links]

    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    biggest = 0.0
    # Prefer root links, but fall back to any link carrying a mesh: some
    # descriptions model the base as a primitive and only the moving links
    # reference mesh files.
    ordered = roots + [n for n in links if n not in roots]
    for name in ordered:
        lnode = links[name]
        vis = lnode.find("visual")
        geom = vis.find("geometry") if vis is not None else None
        mesh_el = geom.find("mesh") if geom is not None else None
        if mesh_el is None or not mesh_el.get("filename"):
            continue
        # Only trust meshes that do NOT already declare a scale.
        if mesh_el.find("scale") is not None:
            return (1.0, 1.0, 1.0)
        path = resolve_package_uri(mesh_el.get("filename"))
        if path is None:
            cand = os.path.join(urdf_dir, mesh_el.get("filename"))
            path = cand if os.path.isfile(cand) else None
        if path is None:
            continue
        loaded = load_mesh(path)
        if not loaded:
            continue
        positions = loaded[0]
        if not positions:
            continue
        extent = max(
            max(p[0] for p in positions) - min(p[0] for p in positions),
            max(p[1] for p in positions) - min(p[1] for p in positions),
            max(p[2] for p in positions) - min(p[2] for p in positions),
        )
        if extent > biggest:
            biggest = extent

    if biggest <= 0:
        return (1.0, 1.0, 1.0)

    ratio = biggest / reach
    # Use the largest power of ten that keeps the model within a sane range.
    # A real arm is roughly 0.5-3 m across, so target the mesh extent at the
    # same order of magnitude as the kinematic reach. Try every decade and keep
    # the one landing closest to the reach.
    if verbose:
        print(f"[export_gltf] scale check: reach={reach:.3f} m, "
              f"largest mesh extent={biggest:.1f} raw units, "
              f"raw ratio={biggest / reach:.1f}")

    best = None
    # Allow half-decade steps: a millimetre-authored mesh measures ~200x the
    # arm's metre reach, and log10(200) ~= 2.3, so rounding to a whole power
    # of ten would pick the wrong decade.
    for step in range(-13, 14):
        factor = 10.0 ** (-step / 2.0)
        scaled = biggest * factor
        if scaled <= 0:
            continue
        error = abs(math.log10(scaled / reach))
        if best is None or error < best[0] - 1e-12:
            best = (error, factor, scaled)

    if best is None:
        return (1.0, 1.0, 1.0)

    _, factor, scaled = best
    ratio = scaled / reach

    # Trust the inferred scale whenever the raw mesh extent is wildly out of
    # scale with a human-sized arm. A robot is ~0.5-3 m across; anything whose
    # raw extent is more than ~10x that is authored in the wrong unit.
    if biggest > 30.0:
        if verbose:
            print(f"[export_gltf] scale check: reach={reach:.3f} m, raw mesh "
                  f"extent={biggest:.1f} (ratio {biggest / reach:.0f}x) "
                  f"-> applying scale {factor:g} "
                  f"(scaled extent {scaled:.2f} m)")
        return (factor, factor, factor)

    if verbose:
        print(f"[export_gltf] scale check: reach={reach:.3f} m, raw mesh "
              f"extent={biggest:.1f} -> already plausible, no rescale")

    return (1.0, 1.0, 1.0)


def decimate(positions, normals, indices, target):
    """Drop whole triangles until the mesh fits in 16-bit indices.

    Two things this must get right:

    * Only *whole* triangles may be dropped. Slicing the index stream with
      `indices[::step]` starts at a fixed offset and ignores triangle
      boundaries, so it keeps three vertices that were never a face -- which
      renders as a long spike shooting off the part.
    * The result must never be empty. If the step is too coarse, slicing can
      drop everything, and the caller treats an empty mesh as "this link has
      no geometry" and drops the link from the scene entirely.

    The soup here has one position per corner (no index reuse), so dropping a
    triangle means dropping three consecutive positions as well.
    """
    if len(positions) <= target:
        return positions, normals, indices

    n_tris = len(indices) // 3
    if n_tris == 0:
        return positions, normals, indices

    # The soup has one position per corner, so N triangles need 3N positions.
    # target is a *position* budget, hence target // 3 triangles.
    max_tris = max(1, target // 3)
    if 3 * max_tris <= len(positions) and n_tris <= max_tris:
        return positions, normals, indices

    # Uniform stride over *triangles*, never over raw indices.
    step = max(1, -(-n_tris // max_tris))   # ceiling division

    new_indices = []
    kept_positions = []
    kept_normals = []
    for t in range(0, n_tris, step):
        base = t * 3
        tri = indices[base:base + 3]
        if len(tri) != 3:
            continue
        start = len(kept_positions)
        ok = True
        for corner in tri:
            idx = int(corner)
            if idx < 0 or idx >= len(positions):
                ok = False
                break
            kept_positions.append(positions[idx])
            kept_normals.append(normals[idx] if normals else None)
        if not ok:
            del kept_positions[start:]
            if kept_normals:
                del kept_normals[start:]
            continue
        new_indices.extend(range(start, start + 3))

    if not new_indices or not kept_positions:
        # Never hand back an empty mesh: that silently deletes the link.
        return positions, normals, indices

    return kept_positions, kept_normals, new_indices


def convert(urdf_path, out_path, pose=None):
    """Build the glTF scene graph from a URDF and write a .glb.

    URDF/USD are Z-up while glTF mandates Y-up, so the whole robot is baked
    under a root node that rotates +90 deg about X (Z-up -> Y-up). Doing the
    conversion here means every consumer gets a correctly-oriented model
    without having to rotate it in the viewer.
    """
    builder = GltfBuilder()
    _convert_into(urdf_path, builder, pose)
    size = builder.write_glb(out_path)
    return size, len(builder.nodes), len(builder.meshes), builder


def _convert_into(urdf_path, builder, pose):
    """Populate `builder` with the URDF's meshes and node hierarchy.

    Hierarchy is link -> joint -> link, e.g.
        base_link
          └─ joint_shoulder_pan_joint
               └─ shoulder_link
                    └─ joint_elbow_pitch_joint
                         └─ upper_arm_link ...
    Only the root links become scene roots; every other node hangs off its
    parent, which is what makes the model look like a connected arm.
    """
    root = ET.parse(urdf_path).getroot()
    links = {l.get("name"): l for l in root.findall("link")}
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    builder.node_index = {}
    joints = []
    for j in root.findall("joint"):
        pel, cel = j.find("parent"), j.find("child")
        if pel is None or cel is None:
            continue
        origin = j.find("origin")
        axis_el = j.find("axis")
        limit_el = j.find("limit")
        joints.append({
            "name": j.get("name"), "type": j.get("type", "fixed"),
            "parent": pel.get("link"), "child": cel.get("link"),
            "xyz": parse_vec(origin.get("xyz") if origin is not None else None),
            "rpy": parse_vec(origin.get("rpy") if origin is not None else None),
            "axis": parse_vec(axis_el.get("xyz") if axis_el is not None else None),
            "lower": float(limit_el.get("lower"))
            if limit_el is not None and limit_el.get("lower") is not None
            else None,
            "upper": float(limit_el.get("upper"))
            if limit_el is not None and limit_el.get("upper") is not None
            else None,
        })

    # joints_by_parent lets us attach children to the right parent link.
    joints_by_parent = {}
    for j in joints:
        joints_by_parent.setdefault(j["parent"], []).append(j)

    # The <visual><origin> offset places the MESH inside the link frame; it
    # must NOT move the link frame itself, because a joint origin is defined
    # in the parent link's frame. Folding the visual offset into the link
    # node's translation double-counts it and pushes every child joint
    # further away, which is what makes the segments look disconnected.
    def visual_offset(name):
        lnode = links.get(name)
        vis = lnode.find("visual") if lnode is not None else None
        origin = vis.find("origin") if vis is not None else None
        if origin is None or origin.get("xyz") is None:
            return [0.0, 0.0, 0.0]
        return parse_vec(origin.get("xyz"))

    child_links = {j["child"] for j in joints}
    link_node = {}

    # Auto-detect a mesh scale. The kinematic tree's own joint offsets are in
    # metres, so if the meshes are authored in millimetres (franka_description
    # does this and declares no <scale>), they come out ~1000x too big.
    # Compare a root link's mesh extent against the tree's reach.
    scale_hint = detect_mesh_scale(urdf_path, root, links, joints)

    # Create a node per link at its own frame origin (no offset), plus an
    # optional child node that carries the visual mesh at the visual offset.
    for name, lnode in links.items():
        link_node[name] = builder.add_node(name)
        mesh = link_mesh(builder, lnode, root, urdf_dir, scale_hint)
        if mesh is not None:
            off = visual_offset(name)
            mesh_node = builder.add_node(
                name + "_visual", mesh=mesh, translation=off)
            builder.nodes[link_node[name]].setdefault("children", []).append(
                mesh_node)

    # Then wire joint nodes between parent and child links.
    for j in joints:
        value = (pose or {}).get(j["name"], 0.0)
        # Metadata the interactive viewer needs to turn a slider value back
        # into a rotation: the joint axis, the URDF frame offset it is measured
        # from, and the limits. glTF has no concept of "joint angle", so
        # without these the browser cannot edit the pose at all.
        extras = {
            "armweb_joint": {
                "name": j["name"],
                "type": j["type"],
                "axis": list(j["axis"]),
                "origin_rpy": list(j["rpy"]),
                "origin_xyz": list(j["xyz"]),
            }
        }
        if j.get("lower") is not None:
            extras["armweb_joint"]["lower"] = j["lower"]
        if j.get("upper") is not None:
            extras["armweb_joint"]["upper"] = j["upper"]

        if j["type"] == "prismatic":
            unit = j["axis"]
            n = math.sqrt(sum(c * c for c in unit)) or 1.0
            t = [j["xyz"][i] + unit[i] / n * value for i in range(3)]
            # The origin rotation applies to prismatic joints too. FR3's
            # fr3_finger_joint2 is prismatic with rpy=(0,0,pi): dropping the
            # rotation here swings the right finger 180 deg away from its
            # socket, which is exactly how the hand came out broken.
            jn = builder.add_node("joint_" + j["name"], translation=t,
                                  rotation=rpy_quat(*j["rpy"]), extras=extras)
        else:
            qr = rpy_quat(*j["rpy"])
            qa = quat_axis_angle(j["axis"], value)
            jn = builder.add_node("joint_" + j["name"], translation=j["xyz"],
                                  rotation=quat_mul(qr, qa), extras=extras)
        builder.node_index["joint_" + j["name"]] = jn

        # The joint transform belongs under its PARENT link node.
        parent_node = link_node[j["parent"]]
        builder.nodes[parent_node].setdefault("children", []).append(jn)
        builder.nodes[jn].setdefault("children", []).append(link_node[j["child"]])

    # Scene roots are only the links no joint drives into.
    inner_roots = [link_node[n] for n in links if n not in child_links]

    # glTF is Y-up by spec; URDF/USD are Z-up. Wrap the robot in a root node
    # rotated -90 deg about X. That maps the robot's +Z up-axis onto the glTF
    # +Y up-axis: rotating (0,0,1) by -90 about X gives (0,1,0).
    # (Using +90 here would flip the model upside down.)
    axis_node = builder.add_node("zup_to_yup", rotation=rpy_quat(-math.pi / 2, 0, 0))
    builder.nodes[axis_node]["children"] = list(inner_roots)
    builder.scene_roots = [axis_node]
    return joints


def quat_mul(a, b):
    """Multiply two glTF quaternions (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


# --------------------------------------------------- animation from a .usda


def read_animation_from_usda(usda_path):
    """Read joint rotations/translations per time code from a recorded USD.

    Returns {joint_name: track} plus the stage's time codes per second.
    Note: this pxr build exposes GetOrientOp()/GetTranslateOp() (the UsdGeom.Xform
    convenience accessors GetOrientAttr/GetTranslateAttr are absent), so we go
    through the ops, which still expose GetTimeSamples()/Get().
    """
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(usda_path)
    if stage is None:
        raise RuntimeError(f"cannot open {usda_path}")
    fps = stage.GetTimeCodesPerSecond() or 24.0

    def op_of(xform, kind):
        """Return the underlying attribute for a given xformOp type."""
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == kind:
                return op.GetAttr()
        return None

    tracks = {}
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Xform):
            continue
        xf = UsdGeom.Xform(prim)
        name = prim.GetName().replace("joint_", "")

        orient_attr = op_of(xf, UsdGeom.XformOp.TypeOrient)
        if orient_attr is not None:
            samples = orient_attr.GetTimeSamples()
            if samples:
                values = []
                for t in samples:
                    q = orient_attr.Get(t)
                    # USD/GfQuatf exposes real (w) + imaginary (x, y, z).
                    # Store glTF order (x, y, z, w) right here so nothing
                    # downstream has to guess the convention.
                    values.append(list(q.GetImaginary()) + [q.GetReal()])
                tracks[name] = {"kind": "rotation",
                                "times": [float(t) for t in samples],
                                "values": values}

        trans_attr = op_of(xf, UsdGeom.XformOp.TypeTranslate)
        if trans_attr is not None:
            samples = trans_attr.GetTimeSamples()
            if samples:
                values = [list(trans_attr.Get(t)) for t in samples]
                tracks[name] = {"kind": "translation",
                                "times": [float(t) for t in samples],
                                "values": values}
    return tracks, fps


def bake_animation(builder, tracks, fps, joint_rpy=None):
    """Add a glTF animation targeting the joint nodes.

    ``joint_rpy`` maps joint name -> its URDF origin_rpy. The recorder stores
    only the *axis* rotation per frame, while a joint node's rest transform is
    ``rpy_quat(origin_rpy) * axis(angle)`` (see _convert_into). Baking the USD
    rotation verbatim therefore drops the origin frame: every joint carrying a
    90 deg origin_rpy would render rotated by 90 deg from the correct pose.
    Pre-multiplying by the origin quaternion puts both conventions back in
    agreement, and the rest pose (angle=0) reproduces the URDF exactly.
    """
    joint_rpy = joint_rpy or {}
    samplers, channels = [], []
    max_time = 0.0
    unmatched = []

    for jname, track in tracks.items():
        node_idx = builder.node_index.get("joint_" + jname)
        if node_idx is None:
            # Almost always means the recording is for a DIFFERENT robot:
            # arm_anim.usda holds demo-arm joints (shoulder_pan_joint), which
            # do not exist in the FR3 scene, so every track silently missed
            # and the GLB shipped with no animation at all.
            unmatched.append(jname)
            continue
        times, values = track["times"], track["values"]
        if not times:
            continue

        in_view = builder._add_view(
            b"".join(struct.pack("<f", t / fps) for t in times),
            builder.TARGET_ARRAY_BUFFER, builder.COMP_FLOAT)
        builder.accessors.append({
            "bufferView": in_view, "componentType": builder.COMP_FLOAT,
            "count": len(times), "type": "SCALAR",
            "min": [min(times) / fps], "max": [max(times) / fps],
        })
        in_acc = len(builder.accessors) - 1

        if track["kind"] == "rotation":
            # read_animation_from_usda already normalised each sample to
            # (x, y, z, w) -- it appends q.GetReal() *after* the imaginary
            # part. Swapping again here (the old [v1],v[2],v[3],v[0]) turned
            # the identity quaternion (0,0,0,1) into (0,0,1,0): a 180 deg
            # flip about Z on every joint at frame 0, which is why playback
            # started from a pose nothing like the URDF rest pose.
            xyzw = [list(v) for v in values]
            # Re-attach the URDF origin frame the recorder leaves out.
            if jname in joint_rpy:
                oq = rpy_quat(*joint_rpy[jname])
                xyzw = [quat_mul(oq, q) for q in xyzw]
            out_view = builder._add_view(
                b"".join(struct.pack("<4f", *v) for v in xyzw),
                builder.TARGET_ARRAY_BUFFER, builder.COMP_FLOAT)
            builder.accessors.append({
                "bufferView": out_view, "componentType": builder.COMP_FLOAT,
                "count": len(xyzw), "type": "VEC4",
            })
            path = "rotation"
        else:
            out_view = builder._add_view(
                b"".join(struct.pack("<3f", *v) for v in values),
                builder.TARGET_ARRAY_BUFFER, builder.COMP_FLOAT)
            xs = [v[0] for v in values]
            ys = [v[1] for v in values]
            zs = [v[2] for v in values]
            builder.accessors.append({
                "bufferView": out_view, "componentType": builder.COMP_FLOAT,
                "count": len(values), "type": "VEC3",
                "min": [min(xs), min(ys), min(zs)],
                "max": [max(xs), max(ys), max(zs)],
            })
            path = "translation"

        out_acc = len(builder.accessors) - 1
        samplers.append({"input": in_acc, "output": out_acc,
                         "interpolation": "LINEAR"})
        channels.append({"sampler": len(samplers) - 1,
                         "target": {"node": node_idx, "path": path}})
        max_time = max(max_time, max(times) / fps)

    if samplers:
        builder.animations.append({"name": "arm_motion", "samplers": samplers,
                                   "channels": channels})
    if unmatched:
        # Loud, because the silent version of this is an animation-less GLB.
        print(f"[export_gltf] WARNING: {len(unmatched)} recorded joint(s) had no "
              f"matching node in this URDF and were skipped: "
              f"{', '.join(sorted(unmatched)[:6])}"
              + (" ..." if len(unmatched) > 6 else ""), file=sys.stderr)
        print("[export_gltf]          the .usda was probably recorded for a "
              "different robot (e.g. demo arm) -- re-record with "
              "scripts/record_anim.sh", file=sys.stderr)
    elif not samplers:
        print("[export_gltf] WARNING: no animation channels were produced; "
              "the GLB will be a static pose.", file=sys.stderr)
    return len(samplers), max_time


# ------------------------------------------------------------------------ CLI


def parse_pose(urdf_path, pose_str):
    """Map a comma-separated pose string onto URDF joint names."""
    root = ET.parse(urdf_path).getroot()
    jnames = [j.get("name") for j in root.findall("joint")]
    vals = [float(v) for v in pose_str.split(",") if v.strip()]
    return dict(zip(jnames, vals))


def main():
    ap = argparse.ArgumentParser(
        description="Export a URDF robot to glTF 2.0 (.glb) for web viewers")
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--pose", default=None,
                    help="comma-separated joint values for a static pose")
    ap.add_argument("--with-animation", default=None,
                    help="recorded .usda to bake as a glTF animation")
    args = ap.parse_args()

    if not os.path.exists(args.urdf):
        print(f"[export_gltf] ERROR: not found: {args.urdf}", file=sys.stderr)
        sys.exit(1)

    pose = parse_pose(args.urdf, args.pose) if args.pose else None
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)

    builder = GltfBuilder()
    joints = _convert_into(args.urdf, builder, pose)

    anim_info = ""
    if args.with_animation:
        # _convert_into already parsed the URDF, so reuse its joint origins
        # instead of re-reading the file for the same information.
        joint_rpy = {j["name"]: j["rpy"] for j in joints}
        tracks, fps = read_animation_from_usda(args.with_animation)
        n_samplers, max_t = bake_animation(builder, tracks, fps, joint_rpy)
        anim_info = (f", animation: {n_samplers} samplers, "
                     f"duration {max_t:.2f}s @ {fps:g} fps")

    size = builder.write_glb(args.out)
    print(f"[export_gltf] wrote {args.out}")
    print(f"[export_gltf] nodes={len(builder.nodes)} meshes={len(builder.meshes)} "
          f"joints={len(joints)} size={size}B{anim_info}")
    print("[export_gltf] load with three.js GLTFLoader or any glTF viewer")


if __name__ == "__main__":
    main()
# ---TAIL---