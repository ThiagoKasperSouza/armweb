#!/usr/bin/env python3
# Diagnose the animated GLB: hierarchy, quaternion order, motion.
import json, struct, sys

p = sys.argv[1] if len(sys.argv) > 1 else 'data/gltf/arm_animated.glb'
d = open(p, 'rb').read()
jlen, _ = struct.unpack('<II', d[12:20])
g = json.loads(d[20:20 + jlen])


def decode(accessor):
    bv = g['bufferViews'][accessor['bufferView']]
    off = 20 + jlen + 8 + bv.get('byteOffset', 0)
    ncomp = {'SCALAR': 1, 'VEC3': 3, 'VEC4': 4}[accessor['type']]
    fmt = {1: '<f', 3: '<3f', 4: '<4f'}[ncomp]
    size = struct.calcsize(fmt)
    out = []
    for i in range(accessor['count']):
        out.append(struct.unpack_from(fmt, d, off + i * size))
    return out


print("=== node hierarchy (scene roots should be ONLY base_link) ===")
roots = g['scenes'][0]['nodes']
print("  scene roots:", roots)
# Walk the tree from each root to find which nodes are actually reachable.
reach = set()


def walk(i, depth=0):
    if i in reach:
        return
    reach.add(i)
    for c in g['nodes'][i].get('children', []):
        walk(c, depth + 1)


for r in roots:
    walk(r)
all_nodes = set(range(len(g['nodes'])))
orphans = sorted(all_nodes - reach)
print(f"  reachable from roots: {len(reach)}/{len(all_nodes)}")
if orphans:
    print(f"  ORPHANED (not in any hierarchy): {orphans}")
    for o in orphans:
        print(f"    [{o}] {g['nodes'][o].get('name')}")

print("\n=== animation targets: are they in the hierarchy? ===")
a = g['animations'][0]
for ch in a['channels']:
    n = ch['target']['node']
    flag = "OK" if n in reach else "ORPHANED (not reachable from roots!)"
    print(f"   node={n:2} {g['nodes'][n].get('name'):26} "
          f"{ch['target']['path']:11} {flag}")

print("\n=== rotation quaternion ordering ===")
for i, s in enumerate(a['samplers']):
    out_acc = g['accessors'][s['output']]
    if out_acc['type'] != 'VEC4':
        continue
    keys = decode(out_acc)
    first, last = keys[0], keys[-1]
    # For a rotation about Z, x and y should be ~0 and |w|<=1.
    print(f"  sampler {i} rotation, {len(keys)} keys")
    print(f"    first={tuple(round(v,4) for v in first)}")
    print(f"    last ={tuple(round(v,4) for v in last)}")
    moved = first != last
    print(f"    values differ over time: {moved}")
    if moved:
        # Which component is the oscillating one?
        diffs = [abs(last[k] - first[k]) for k in range(4)]
        print(f"    per-component delta: "
              f"{[round(x,4) for x in diffs]}")
        print(f"    largest delta at index {diffs.index(max(diffs))}")
    break