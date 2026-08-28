#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /opt/roarm_vendor_ws/install/setup.bash
if [[ -f /workspace/src/ros2_ws/install/setup.bash ]]; then
    source /workspace/src/ros2_ws/install/setup.bash
fi
exec "$@"
