#!/usr/bin/env bash
# Entrypoint for the sim container.
# NOTE: ROS 2 setup.bash is NOT compatible with `set -u`, so -u is
# intentionally omitted here.
set -eo pipefail

# ament trace hooks read this variable; define it to avoid unbound warnings
export AMENT_TRACE_SETUP_FILES=0

source /opt/ros/${ROS_DISTRO}/setup.bash

# Generate the URDF from its procedural generator (idempotent).
if [ -f /ws/tools/gen_urdf.py ]; then
  python3 /ws/tools/gen_urdf.py
fi

if [ ! -f /ws/install/setup.bash ] || [ "${FORCE_BUILD:-0}" = "1" ]; then
  echo "[entrypoint] building workspace..."
  cd /ws
  colcon build --symlink-install --event-handlers console_direct+
  source /ws/install/setup.bash
else
  echo "[entrypoint] using prebuilt workspace"
  source /ws/install/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

echo "[entrypoint] launching armweb sim (ARM=${ARM:-demo})"
exec ros2 launch armweb_sim sim.launch.py \
  urdf:=/ws/assets/urdf/demo_arm.urdf \
  foxglove_port:=8765 \
  usd_out:=/ws/data/usd/arm.usda \
  "$@"