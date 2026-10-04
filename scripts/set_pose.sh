#!/usr/bin/env bash
# Switch the running arm to a named pose (resolved by joint NAME).
#
# Usage: ./scripts/set_pose.sh photo
#        ./scripts/set_pose.sh home
#        ./scripts/set_pose.sh --list
#
# The node clamps every value to the joint's own URDF <limit> before applying
# it, and prints what it applied, so a bad pose can never be sent blindly.
set -eo pipefail
cd "$(dirname "$0")/.."

if [ "${1:-}" = "--list" ] || [ $# -eq 0 ]; then
  echo "usage: $0 <pose>"
  echo ""
  echo "available poses:"
  echo "  photo   resembles the FR3 photo: base rotated out, elbow folded, wrist level"
  echo "  home    ready pose, arm up"
  exit 0
fi

POSE="$1"

docker compose exec -T sim bash -lc "
export AMENT_TRACE_SETUP_FILES=0
source /opt/ros/\$ROS_DISTRO/setup.bash
source /ws/install/setup.bash
ros2 topic pub --once /arm_sim/pose std_msgs/msg/String \"{data: '$POSE'}\"
" 2>&1 | tail -3

echo "pose: $POSE"