#!/usr/bin/env bash
# Export the arm to glTF (.glb) for browser viewers.
#
# Uses whichever URDF prepare_arm.sh last produced (default: ws/assets/urdf/
# robot.urdf). If a recorded animation exists it is baked into the GLB too.
set -eo pipefail
cd "$(dirname "$0")/.."

URDF="${URDF:-/ws/assets/urdf/robot.urdf}"
PREFIX="${PREFIX:-}"   # empty -> arm_rest.glb; set to e.g. "fr3_" for fr3_rest.glb
mkdir -p data/gltf

if [ ! -f "ws/assets/urdf/robot.urdf" ]; then
  echo "no URDF found, running prepare_arm.sh (default: fr3)"
  ./scripts/prepare_arm.sh "${ARM:-fr3}"
fi

# Pick the recording that matches the robot we are about to export. Baking a
# .usda recorded for a different arm silently yields zero animation channels,
# because none of its joint names exist in this URDF. Preference order:
#   1. $ANIM_USDA               (explicit override)
#   2. data/usd/<arm>_anim.usda (what record_anim.sh writes for this robot)
#   3. any *.usda whose joint names actually match the URDF
ANIM_USDA="${ANIM_USDA:-}"
ANIM_ARGS=()

pick_anim() {
  local urdf="$1"
  # Preferred name for this robot.
  local arm
  arm=$(python3 - "$urdf" <<'PY'
import sys, xml.etree.ElementTree as ET
r = ET.parse(sys.argv[1]).getroot()
print((r.get('name') or 'arm').split()[0].lower())
PY
)
  local cand="data/usd/${arm}_anim.usda"
  if [ -n "$ANIM_USDA" ]; then
    echo "$ANIM_USDA"; return
  fi
  if [ -f "$cand" ]; then echo "$cand"; return; fi
  # Fall back to any recording that shares joint names with the URDF.
  for f in data/usd/*_anim.usda; do
    [ -f "$f" ] || continue
    if python3 - "$urdf" "$f" <<'PY'
import re, sys, xml.etree.ElementTree as ET
urdf, usda = sys.argv[1], sys.argv[2]
want = {j.get('name') for j in ET.parse(urdf).getroot().findall('joint')}
txt = open(usda, encoding='utf-8', errors='replace').read()
have = {m[len('joint_'):] for m in re.findall(r'def Xform "([^"]+)"', txt)
        if m.startswith('joint_')}
sys.exit(0 if len(want & have) >= max(1, len(want) // 2) else 1)
PY
    then echo "$f"; return; fi
  done
}

ANIM_FILE="$(pick_anim "ws/assets/urdf/robot.urdf" || true)"
if [ -n "$ANIM_FILE" ]; then
  ANIM_ARGS=(--with-animation "/ws/$ANIM_FILE")
  echo "animation source: $ANIM_FILE"
else
  echo "no matching recording found - the *_animated.glb will be a STATIC pose."
  echo "  run ./scripts/record_anim.sh to capture one for this robot."
fi

export_one() {
  local name="$1"; shift
  echo ""
  echo "--- $name.glb ---"
  docker compose exec -T sim python3 /ws/tools/export_gltf.py \
    --urdf "$URDF" \
    "$@" \
    --out "/ws/data/gltf/${PREFIX}_${name}.glb" 2>&1 | tail -4
}

export_one rest
export_one posed --pose "0.0,-0.6,0.0,0.9,0.0,0.4,0.02"
export_one animated "${ANIM_ARGS[@]}"

echo ""
echo "--- validation ---"
for name in rest posed animated; do
  f="data/gltf/${PREFIX}_${name}.glb"
  if [ -f "$f" ]; then
    echo "### $f"
    docker compose exec -T sim python3 /ws/tools/validate_glb.py \
      "/ws/data/gltf/${PREFIX}_${name}.glb" 2>&1 | grep -E 'VALID' | head -1
    python3 ws/tools/verify_gltf_scene.py "$f" 2>&1 | tail -18
    echo ""
  fi
done

echo ""
ls -la data/gltf/