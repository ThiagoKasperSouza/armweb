#!/usr/bin/env bash
# Resolve a real robot description (xacro) into a plain URDF we can export.
#
# Usage: ./scripts/prepare_arm.sh [demo|fr3|franka|ur5]
set -eo pipefail
cd "$(dirname "$0")/.."

ARM="${1:-${ARM:-fr3}}"
OUT="/ws/assets/urdf/robot.urdf"

case "$ARM" in
  demo)
    echo "demo: procedural arm (no external meshes)"
    python3 ws/tools/gen_urdf.py
    cp ws/assets/urdf/demo_arm.urdf ws/assets/urdf/robot.urdf
    ;;
  fr3|franka)
    SRC="/opt/ros/humble/share/franka_description/robots/fr3/fr3.urdf.xacro"
    echo "franka FR3 (Apache-2.0, DAE visual meshes) -> $SRC"
    ;;
  ur5|ur)
    SRC="/opt/ros/humble/share/ur_description/urdf/ur5/ur5.urdf"
    echo "Universal Robots UR5 -> $SRC"
    ;;
  *)
    echo "unknown arm '$ARM' (use demo|fr3|ur5)"
    exit 1
    ;;
esac

if [ "$ARM" != "demo" ]; then
  docker compose exec -T sim bash -lc "
export AMENT_TRACE_SETUP_FILES=0
source /opt/ros/\$ROS_DISTRO/setup.bash >/dev/null 2>&1
mkdir -p /ws/assets/urdf
if [ ! -f '$SRC' ]; then echo 'ERROR: not found: $SRC'; exit 1; fi
xacro '$SRC' -o $OUT
" 2>&1 | tail -5

  # Report what we got.
  docker compose exec -T sim python3 - <<'PY' 2>&1 | tail -12
import xml.etree.ElementTree as ET
r = ET.parse('/ws/assets/urdf/robot.urdf').getroot()
links = r.findall('link'); joints = r.findall('joint')
moved = [j for j in joints if j.get('type') in ('revolute', 'continuous', 'prismatic')]
meshes = r.findall('.//mesh')
print(f"  robot   : {r.get('name')}")
print(f"  links   : {len(links)}")
print(f"  joints  : {len(joints)} ({len(moved)} actuated)")
print(f"  meshes  : {len(meshes)}")
exts = {}
for m in meshes:
    fn = m.get('filename') or ''
    if '.' in fn:
        exts[fn.rsplit('.', 1)[1]] = exts.get(fn.rsplit('.', 1)[1], 0) + 1
print(f"  formats : {exts}")
PY
fi

echo ""
echo "URDF ready at ws/assets/urdf/robot.urdf"