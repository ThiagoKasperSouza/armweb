#!/usr/bin/env bash
# Record an animated OpenUSD file from the live /joint_states stream.
#
# By default the recorder drives the arm itself (sinusoidal sweep) so the
# capture always contains motion. Set AUTO_DRIVE=0 to record passively while
# you command the arm from another terminal.
#
# Usage: ./scripts/record_anim.sh [duration_seconds] [rate_hz]
set -eo pipefail
cd "$(dirname "$0")/.."

DURATION="${1:-5}"
RATE="${2:-20}"
AUTO="${AUTO_DRIVE:-1}"

mkdir -p data/usd

EXTRA=""
[ "$AUTO" = "1" ] && EXTRA="--auto-drive"

# Record from the robot that prepare_arm.sh last resolved (robot.urdf), not a
# hardcoded demo arm. Recording the demo arm is what produced an animation
# whose joint names (shoulder_pan_joint, ...) match nothing in the FR3 URDF,
# so the baked GLB ended up with zero animation channels.
URDF="${URDF:-/ws/assets/urdf/robot.urdf}"
# This script runs on the HOST, so check the host-side mirror, not the
# container path. (Testing /ws/... here always failed, which made the
# recorder look broken.)
HOST_URDF="ws/assets/urdf/robot.urdf"
if [ ! -f "$HOST_URDF" ]; then
  echo "no URDF at $HOST_URDF -- run ./scripts/prepare_arm.sh fr3 first" >&2
  exit 1
fi

# Name the recording after the ROBOT, not the file. robot.urdf is whatever
# prepare_arm.sh last resolved, so keying off its filename always produced
# "arm_anim.usda" -- the same path the demo arm uses, so a stale demo
# recording would silently be baked into the FR3 GLB.
STEM="$(python3 - "$HOST_URDF" <<'PY'
import sys, xml.etree.ElementTree as ET
name = ET.parse(sys.argv[1]).getroot().get('name') or 'arm'
print(name.strip().split()[0].lower())
PY
)"
OUT="${USD_ANIM_OUT:-/ws/data/usd/${STEM}_anim.usda}"
echo "recording robot '$STEM' -> $OUT"

docker compose exec -T sim bash -lc "
export AMENT_TRACE_SETUP_FILES=0
source /opt/ros/\$ROS_DISTRO/setup.bash >/dev/null 2>&1
source /ws/install/setup.bash >/dev/null 2>&1
python3 /ws/tools/record_usd_animation.py \
  --urdf $URDF \
  --out  $OUT \
  --duration $DURATION --rate $RATE $EXTRA
" 2>&1 | grep -vE '^\s*$' | tail -8

echo ""
echo "validating..."
docker compose exec -T sim python3 /ws/tools/verify_anim.py "$OUT" 2>&1 | tail -14

echo ""
ls -la data/usd/