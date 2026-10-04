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
  * geometry lives in ``doc.geometries`` (an IndexedList)
  * a primitive is a ``TriangleSet`` with ``.vertex`` and ``.index``
  * ``.index`` is a 2-D (N, 2) array pairing each corner with a normal index
  * ``.normal_index`` is a list of 3-element arrays, one per triangle
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
    return (n[0] / mag, n[1] / mag, n[2] / mag) if mag > 1e-12 else (0.0, 0.0, 1.0)


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
def _emit_primitive(prim, positions, normals, indices, transform=None,
                   verts_are_bound=False):
    """Append one COLLADA primitive's triangles to the soup lists.

    When ``transform`` is given, it is applied to the normals (and to the
    positions too, unless ``verts_are_bound`` says pycollada already did it
    when binding the primitive to its node).
    """
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
        # (N, 3, C): N triangles of 3 corners, C inputs per corner.
        # C is 2 for [vertex, normal] but only 1 when the primitive has no
        # NORMAL input -- indexing corner[1] blindly would then raise
        # IndexError, so treat a 1-channel array as vertex-only.
        corners_per_tri = shape[1]
        per_corner = shape[2]
        triangles = []
        for tri in flat:
            row = []
            for corner in tri:
                vi = int(corner[0])
                ni = int(corner[1]) if per_corner > 1 else -1
                row.append([vi, ni])
            triangles.append(row)
    elif len(shape) == 2 and shape[1] == 2:
        # (N, 2): 2N corners as [vertex, normal]
        triangles = [[[int(p[0]), int(p[1])]] for p in flat]
    else:
        # (N, 3) or flat: N triangles of three vertex indices
        fl = [int(x) for x in flat.reshape(-1)] if hasattr(flat, "reshape") \
            else [int(x) for x in flat]
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
            pos = (float(v[0]), float(v[1]), float(v[2]))
            n = None
            if per_tri is not None and ci < len(per_tri):
                n = normal_at(per_tri[ci])
            if n is None and corner[1] >= 0:
                n = normal_at(corner[1])
            if transform is not None:
                if not verts_are_bound:
                    pos = _transform_point(transform, pos)
                if n is not None:
                    n = _transform_normal(transform, n)
            positions.append(pos)
            normals.append(n)
            indices.append(len(positions) - 1)


def _identity4():
    return ((1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0))


def _mat_mul(a, b):
    """Compose two 4x4 matrices (apply b first, then a)."""
    return tuple(
        tuple(sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4))
        for r in range(4)
    )


def _mat_to_tuples(m):
    """Normalise whatever pycollada hands back into a 4x4 tuple of tuples."""
    if m is None:
        return _identity4()
    rows = []
    for r in range(4):
        try:
            rows.append(tuple(float(m[r][c]) for c in range(4)))
        except Exception:
            return _identity4()
    return tuple(rows)


def _transform_point(m, p):
    x, y, z = float(p[0]), float(p[1]), float(p[2])
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3],
        m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3],
        m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3],
    )


def _transform_normal(m, n):
    """Rotate a normal by the inverse-transpose of the upper 3x3.

    For column vectors, n_world = (M^-1)^T n = adj(M)^T n / det(M). Expanding
    the adjugate avoids an explicit matrix inverse; the determinant division
    is what makes non-uniform scale behave correctly.
    """
    x, y, z = float(n[0]), float(n[1]), float(n[2])
    a, b, c = m[0][0], m[0][1], m[0][2]
    d, e, f = m[1][0], m[1][1], m[1][2]
    g, h, i = m[2][0], m[2][1], m[2][2]

    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        return _unit((x, y, z))

    # adj[r][c]; result[c] = sum_r adj[r][c] * n[r]
    r0 = (e * i - f * h, c * h - b * i, b * f - c * e)
    r1 = (f * g - d * i, a * i - c * g, c * d - a * f)
    r2 = (d * h - e * g, b * g - a * h, a * e - b * d)

    return _unit((
        (r0[0] * x + r1[0] * y + r2[0] * z) / det,
        (r0[1] * x + r1[1] * y + r2[1] * z) / det,
        (r0[2] * x + r1[2] * y + r2[2] * z) / det,
    ))


def _load_dae(path):
    """Read a COLLADA file into a flat triangle soup.

    Placement matters here. COLLADA stores a part's position in the scene
    graph (a hierarchy of nodes, each with a matrix), not in the vertex data.
    pycollada composes that for us: ``scene.nodes[i].objects('geometry')``
    yields ``BoundGeometry`` objects whose ``.matrix`` is the node's world
    matrix with the whole parent chain already multiplied in. Emitting the
    primitives without applying it stacks every part at its link origin,
    which is what makes a Franka look like a pile of cylinders.

    Verified against pycollada 0.9:
      * the root list is ``scene.nodes`` (there is no ``scene.node``)
      * ``objects()`` and ``primitives()`` are *methods*, not properties
      * ``BoundGeometry.matrix`` holds the composed world matrix
    """
    from collada import Collada

    doc = Collada(path)

    # Prefer the instantiated scene so node transforms are honoured. Fall back
    # to raw geometries when the file exposes no usable scene.
    bound = []
    scene = getattr(doc, "scene", None)
    for root_node in (getattr(scene, "nodes", None) or []):
        try:
            bound.extend(root_node.objects("geometry"))
        except Exception as exc:
            print(f"[load_mesh] scene walk failed for {os.path.basename(path)}: "
                  f"{exc}", file=sys.stderr)

    positions, normals, indices = [], [], []

    if bound:
        for bg in bound:
            try:
                # NB: BoundGeometry.primitives() yields primitives whose
                # vertices are ALREADY bound to the node's world matrix
                # (pycollada's bind() does the transform). Applying
                # bg.matrix again here would double it -- which shows up as
                # geometry at x=20 instead of x=10. So read the vertices
                # as they come, and only carry bg.matrix for normals.
                world = _mat_to_tuples(getattr(bg, "matrix", None))
                for prim in bg.primitives():
                    _emit_primitive(prim, positions, normals, indices,
                                    transform=world, verts_are_bound=True)
            except Exception as exc:
                # One malformed geometry must not discard the whole mesh.
                print(f"[load_mesh] skipped a geometry: {exc}", file=sys.stderr)
    else:
        print(f"[load_mesh] {os.path.basename(path)}: no scene instances, "
              "falling back to raw geometries", file=sys.stderr)
        for geo in getattr(doc, "geometries", None) or []:
            try:
                for prim in getattr(geo, "primitives", []):
                    _emit_primitive(prim, positions, normals, indices)
            except Exception as exc:
                print(f"[load_mesh] skipped a geometry: {exc}", file=sys.stderr)

    if not positions or not indices:
        return None
    if any(n is None for n in normals):
        computed = compute_normals(positions, indices)
        normals = [c if n is None else n for n, c in zip(normals, computed)]
    return positions, [_unit(n) for n in normals], indices


def weld(positions, normals, indices):
    """Collapse a triangle soup into an indexed mesh.

    The COLLADA readers emit one position per *corner*, so a quad's two
    triangles carry four identical corners instead of two shared vertices.
    That is 3x the vertices glTF actually needs, and it is what pushes the
    FR3 links past the 16-bit index limit and into decimation.

    Vertices are keyed on (position, normal) so hard edges survive: two
    corners at the same spot with different normals stay separate, and the
    shading is byte-for-byte what the source described.

    Returns (positions, normals, indices) with indices remapped, or the
    input untouched if it is not a soup to begin with.
    """
    n = len(positions)
    if n == 0 or len(indices) != n:
        # Already indexed (indices shorter than the position list) - nothing
        # to collapse, and remapping would corrupt it.
        return positions, normals, indices

    lookup = {}
    remap = [0] * n
    out_p, out_n = [], []
    for k in range(n):
        p = positions[k]
        nrm = normals[k] if normals and k < len(normals) else None
        # Quantise before hashing: the DAE floats carry more precision than
        # we store in the GLB, so two "identical" corners can differ in the
        # last bits and would otherwise fail to merge.
        key = (round(p[0], 7), round(p[1], 7), round(p[2], 7))
        if nrm is not None:
            key += (round(nrm[0], 5), round(nrm[1], 5), round(nrm[2], 5))
        j = lookup.get(key)
        if j is None:
            j = len(out_p)
            lookup[key] = j
            out_p.append(p)
            out_n.append(nrm)
        remap[k] = j

    out_i = [remap[i] for i in indices]
    return out_p, out_n, out_i


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
            print(f"[load_mesh] {os.path.basename(path)}: {exc}", file=sys.stderr)
            _CACHE[path] = None

    cached = _CACHE[path]
    if cached is None:
        return None

    positions, normals, indices = cached
    sx, sy, sz = scale
    if (sx, sy, sz) != (1.0, 1.0, 1.0):
        positions = [(p[0] * sx, p[1] * sy, p[2] * sz) for p in positions]
    return positions, normals, indices