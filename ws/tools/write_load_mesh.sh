#!/usr/bin/env bash
# Rewrite load_mesh.py with exact indentation (the editor mangles deep nesting).
# Written in two appended chunks to stay within the editor's size limit.
F=/home/thiag/armweb/ws/tools/load_mesh.py

cat > "$F" <<'PART1'
#!/usr/bin/env python3
"""
Load URDF <mesh> references (STL / DAE / OBJ / PLY) into glTF meshes.

Design notes
------------
* COLLADA (.dae) is read with pycollada directly. trimesh 5.x combined with
  numpy 2.x fails on these files with "only length-1 arrays can be converted
  to Python scalars", so we avoid that path entirely for DAE.
* Geometry is emitted as a flat triangle soup: one position and one normal per
  corner. That keeps positions/normals/indices the same length and avoids index
  remapping. It costs vertex duplication but is simple and robust.
* STL/OBJ go through trimesh, which handles those formats well.

Verified pycollada 0.9 API (against franka_description meshes):
  * geometry lives in doc.geometries (an IndexedList)
  * a primitive is a TriangleSet with .vertex and .index
  * .index rank varies: (N,3), (N,3,2) or (N,2)
  * .normal_index is a list of 3-element arrays, one per triangle
"""
import os
import sys

os.environ.setdefault("TRIMESH_QUIET", "1")

_CACHE = {}


def ros_package_paths():
    """Directories to search when resolving package:// URIs."""
    paths = []
    if os.path.isdir("/opt/ros"):
        for dist in sorted(os.listdir("/opt/ros")):
            share = os.path.join("/opt/ros", dist, "share")
            if os.path.isdir(share):
                paths.append(share)
    for p in os.environ.get("ARMWEB_PACKAGE_PATH", "").split(":"):
        if p:
            paths.append(p)
    return paths


def resolve_package_uri(uri):
    """Resolve package://pkg/rel/path (or a plain path) to a real file."""
    if not uri:
        return None
    if uri.startswith("package://"):
        rest = uri[len("package://"):]
        if "/" not in rest:
            return None
        pkg, rel = rest.split("/", 1)
        for share in ros_package_paths():
            cand = os.path.join(share, pkg, rel)
            if os.path.isfile(cand):
                return cand
        return None
    if uri.startswith("file://"):
        cand = uri[len("file://"):]
        return cand if os.path.isfile(cand) else None
    return uri if os.path.isfile(uri) else None


def _unit(n):
    mag = (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5
    if mag > 1e-12:
        return (n[0] / mag, n[1] / mag, n[2] / mag)
    return (0.0, 0.0, 1.0)


def compute_normals(positions, indices):
    """Area-weighted normals; used when the file supplies none."""
    acc = [[0.0, 0.0, 0.0] for _ in positions]
    for i in range(0, len(indices) - 2, 3):
        a, b, c = indices[i], indices[i + 1], indices[i + 2]
        pa, pb, pc = positions[a], positions[b], positions[c]
        u = (pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2])
        v = (pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2])
        nx = u[1] * v[2] - u[2] * v[1]
        ny = u[2] * v[0] - u[0] * v[2]
        nz = u[0] * v[1] - u[1] * v[0]
        for idx in (a, b, c):
            acc[idx][0] += nx
            acc[idx][1] += ny
            acc[idx][2] += nz
    return [_unit(n) for n in acc]
PART1

cat >> "$F" <<'PART2'


def _emit_primitive(prim, positions, normals, indices):
    """Append one COLLADA primitive's triangles to the soup lists."""
    verts = getattr(prim, "vertex", None)
    if verts is None or len(verts) == 0:
        return

    nrm = getattr(prim, "normal", None)
    nidx = getattr(prim, "normal_index", None)

    def normal_at(k):
        if nrm is None or k < 0 or k >= len(nrm):
            return None
        n = nrm[k]
        return (float(n[0]), float(n[1]), float(n[2]))

    flat = getattr(prim, "index", None)
    if flat is None:
        return

    try:
        shape = tuple(getattr(flat, "shape", ()))
    except Exception:
        shape = ()

    if len(shape) == 3:
        triangles = [[[int(p[0]), int(p[1])] for p in tri] for tri in flat]
    elif len(shape) == 2 and shape[1] == 2:
        triangles = [[[int(p[0]), int(p[1])]] for p in flat]
    else:
        fl = [int(x) for x in flat]
        triangles = [[[fl[i], -1], [fl[i + 1], -1], [fl[i + 2], -1]]
                     for i in range(0, len(fl) - 2, 3)]

    for ti, tri in enumerate(triangles):
        per_tri = None
        if nidx is not None and ti < len(nidx):
            try:
                per_tri = [int(x) for x in nidx[ti]]
            except Exception:
                per_tri = None
        for ci, corner in enumerate(tri):
            vi = corner[0]
            if vi < 0 or vi >= len(verts):
                continue
            v = verts[vi]
            positions.append((float(v[0]), float(v[1]), float(v[2])))
            n = None
            if per_tri is not None and ci < len(per_tri):
                n = normal_at(per_tri[ci])
            if n is None and corner[1] >= 0:
                n = normal_at(corner[1])
            normals.append(n)
            indices.append(len(positions) - 1)


def _load_dae(path):
    """Read a COLLADA file into a flat triangle soup."""
    from collada import Collada

    doc = Collada(path)
    geometries = getattr(doc, "geometries", None)
    if not geometries:
        return None

    positions, normals, indices = [], [], []
    for geo in geometries:
        try:
            for prim in getattr(geo, "primitives", []):
                _emit_primitive(prim, positions, normals, indices)
        except Exception as exc:
            print("[load_mesh] skipped a geometry: %s" % exc, file=sys.stderr)

    if not positions or not indices:
        return None
    if any(n is None for n in normals):
        computed = compute_normals(positions, indices)
        normals = [c if n is None else n for n, c in zip(normals, computed)]
    return positions, [_unit(n) for n in normals], indices


def _load_trimesh(path):
    """Read STL/OBJ/PLY through trimesh, expanded to a triangle soup."""
    import trimesh

    loaded = trimesh.load(path, force="mesh", process=False)
    if loaded is None or not len(getattr(loaded, "faces", [])):
        return None

    verts = [(float(v[0]), float(v[1]), float(v[2])) for v in loaded.vertices]
    faces = [[int(i) for i in f] for f in loaded.faces]
    try:
        vn = [_unit((float(n[0]), float(n[1]), float(n[2])))
              for n in loaded.vertex_normals]
    except Exception:
        vn = None

    positions, normals, indices = [], [], []
    for face in faces:
        if len(face) < 3:
            continue
        for vi in face[:3]:
            if vi >= len(verts):
                return None
            positions.append(verts[vi])
            normals.append(vn[vi] if vn else None)
            indices.append(len(positions) - 1)

    if not positions:
        return None
    if any(n is None for n in normals):
        computed = compute_normals(positions, indices)
        normals = [c if n is None else n for n, c in zip(normals, computed)]
    return positions, normals, indices


def load_mesh(path, scale=(1.0, 1.0, 1.0)):
    """Load a mesh file into (positions, normals, indices), or None."""
    if not path or not os.path.isfile(path):
        return None

    if path not in _CACHE:
        ext = path.rsplit(".", 1)[-1].lower() if "." in path else ""
        try:
            _CACHE[path] = _load_dae(path) if ext == "dae" else _load_trimesh(path)
        except Exception as exc:
            print("[load_mesh] %s: %s" % (os.path.basename(path), exc),
                  file=sys.stderr)
            _CACHE[path] = None

    cached = _CACHE[path]
    if cached is None:
        return None

    positions, normals, indices = cached
    sx, sy, sz = scale
    if (sx, sy, sz) != (1.0, 1.0, 1.0):
        positions = [(p[0] * sx, p[1] * sy, p[2] * sz) for p in positions]
    return positions, normals, indices
PART2

python3 -m py_compile "$F" && echo "load_mesh.py COMPILES OK"