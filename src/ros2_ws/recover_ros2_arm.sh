#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
if [[ -f /opt/roarm_vendor_ws/install/setup.bash ]]; then
    source /opt/roarm_vendor_ws/install/setup.bash
fi
if [[ -f "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install/setup.bash" ]]; then
    source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install/setup.bash"
fi

echo "Requesting the firmware cooperative stop (not a certified E-stop)..."
ros2 service call /roarm/stop std_srvs/srv/Trigger "{}" || true
echo
echo "Requesting a controlled UART reopen..."
ros2 service call /roarm/reconnect std_srvs/srv/Trigger "{}" || true
echo
echo "Current driver diagnostic:"
ros2 topic echo /diagnostics --once || true
echo
echo "If motion remains unsafe, switch off RoArm power before further diagnosis."
