#!/usr/bin/env bash
# Container healthcheck: is the simulation actually publishing?
# Kept as a script (not inline CMD-SHELL) because ROS setup.bash needs
# AMENT_TRACE_SETUP_FILES defined and a bash shell, not plain sh.
export AMENT_TRACE_SETUP_FILES=0
source /opt/ros/${ROS_DISTRO}/setup.bash >/dev/null 2>&1
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash >/dev/null 2>&1

if timeout 10 ros2 topic list 2>/dev/null | grep -q '^/joint_states$'; then
  echo "healthy: /joint_states is published"
  exit 0
fi

echo "unhealthy: /joint_states not found"
exit 1