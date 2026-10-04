#!/usr/bin/env bash
# Print a live summary of the running stack.
set -eo pipefail
cd "$(dirname "$0")/.."

docker compose exec -T sim bash -lc '
# Silence the extremely verbose ament trace that ROS setup.bash emits.
export AMENT_TRACE_SETUP_FILES=0
# Some ament hooks echo their sourcing regardless; drop setup chatter entirely.
source /opt/ros/$ROS_DISTRO/setup.bash >/dev/null 2>&1
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash >/dev/null 2>&1

echo "--- topics ---"
timeout 5 ros2 topic list 2>/dev/null
echo ""
echo "--- nodes ---"
timeout 5 ros2 node list 2>/dev/null
echo ""
echo "--- joint states (1 sample) ---"
timeout 6 ros2 topic echo /joint_states --once 2>/dev/null | head -30
echo ""
echo "--- static TF frames ---"
timeout 5 ros2 topic echo /tf_static --once 2>/dev/null \
  | grep child_frame_id || echo "(no tf_static yet)"
' 2>/dev/null | grep -vE '^\s*$' \
  | grep -vE '^(#|_ament_prefix_sh_source_script|AMENT_CURRENT_PREFIX|export |unset )' \
  | grep -vE '^\s+(\.|if |echo|fi|else)'