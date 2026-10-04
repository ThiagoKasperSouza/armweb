#!/usr/bin/env bash
# Run every exporter/viewer test. The python tests need pycollada + the ROS
# package paths, so they run inside the sim container; the node tests only
# need the viewer source, so they run on the host.
set -uo pipefail
cd "$(dirname "$0")/../.."

PY_TESTS="test_decimate test_gltf_quality test_gltf_roundtrip \
          test_dae_transforms test_node_transforms test_poses"
NODE_TESTS="test_viewer_pose test_viewer_anim check_viewer_syntax"

fail=0

echo "=== python (in container) ==="
for t in $PY_TESTS; do
  printf '%-24s ' "$t"
  if out=$(docker compose exec -T sim python3 "/ws/tools/$t.py" 2>&1); then
    echo "PASS"
  else
    echo "FAIL"
    echo "$out" | tail -20 | sed 's/^/    /'
    fail=1
  fi
done

echo
echo "=== node (host) ==="
for t in $NODE_TESTS; do
  printf '%-24s ' "$t"
  if out=$(node "ws/tools/$t.mjs" 2>&1); then
    echo "PASS"
  else
    echo "FAIL"
    echo "$out" | tail -20 | sed 's/^/    /'
    fail=1
  fi
done

echo
if [ "$fail" -eq 0 ]; then
  echo "All test suites passed."
else
  echo "SOME SUITES FAILED"
fi
exit "$fail"