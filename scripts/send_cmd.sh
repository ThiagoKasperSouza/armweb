#!/usr/bin/env bash
# Send a raw joint target to the running arm (by index, in URDF order).
#
# Usage: ./scripts/send_cmd.sh 0.0 0.5 0.0 1.0 0.0 0.0 0.0
#
# Prefer ./scripts/set_pose.sh <name> for the Franka: it resolves joints by
# NAME, so it is not affected by the joint order in the URDF.
set -eo pipefail
cd "$(dirname "$0")/.."

if [ $# -eq 0 ]; then
  # Default: an easy-to-recognise pose so motion is obvious in Foxglove
  set -- 0.0 0.6 0.0 1.0 0.0 0.0 0.0
fi

DATA=""
for v in "$@"; do
  DATA="$DATA${DATA:+,}$v"
done

docker compose exec -T sim bash -lc "
export AMENT_TRACE_SETUP_FILES=0
source /opt/ros/\$ROS_DISTRO/setup.bash
source /ws/install/setup.bash
ros2 topic pub --once /arm_sim/arm/command \
  std_msgs/msg/Float64MultiArray '{data: [$DATA]}'
" 2>&1 | tail -3

echo "sent: [$DATA]"