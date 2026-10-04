#!/usr/bin/env bash
# Regenerate the OpenUSD file from the URDF and validate it.
set -eo pipefail
cd "$(dirname "$0")/.."

mkdir -p data/usd

docker compose exec -T sim bash -lc '
export AMENT_TRACE_SETUP_FILES=0
source /opt/ros/$ROS_DISTRO/setup.bash
python3 /ws/tools/gen_urdf.py
python3 /ws/tools/export_usd.py \
  --urdf /ws/assets/urdf/demo_arm.urdf \
  --out  /ws/data/usd/arm.usda \
  --name arm
python3 /ws/tools/reopen_check.py /ws/data/usd/arm.usda
' 2>&1 | tail -15

echo ""
ls -la data/usd/