#!/usr/bin/env bash
# Recorder container: waits for the sim to publish, then exports the arm to USD.
set -eo pipefail
export AMENT_TRACE_SETUP_FILES=0
source /opt/ros/${ROS_DISTRO}/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

echo "[recorder] waiting for /joint_states from the sim..."
for i in $(seq 1 60); do
  if ros2 topic list 2>/dev/null | grep -q '^/joint_states$'; then
    echo "[recorder] /joint_states detected after ${i}s"
    break
  fi
  sleep 1
done

# Snapshot the robot description as USD (CPU-only, via OpenUSD/pxr)
echo "[recorder] exporting USD..."
python3 /ws/tools/export_usd.py \
  --urdf /ws/assets/urdf/generated_arm.urdf \
  --out  "${USD_OUT_DIR:-/ws/data/usd}/arm.usda" \
  --name arm

echo "[recorder] done."
sleep infinity