#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
if [[ -f /opt/roarm_vendor_ws/install/setup.bash ]]; then
    source /opt/roarm_vendor_ws/install/setup.bash
elif [[ -f "${ROARM_VENDOR_WS:-$HOME/ros_vendor/roarm_ws}/install/setup.bash" ]]; then
    source "${ROARM_VENDOR_WS:-$HOME/ros_vendor/roarm_ws}/install/setup.bash"
else
    echo "Error: the pinned Waveshare vendor workspace is not built."
    echo "Use docker/Dockerfile.ros2 or follow src/docs/ROS2_MOVEIT2_SAFE_ARM_ONLY.md."
    exit 1
fi

colcon build --symlink-install \
    --packages-select \
      eee8097_interfaces \
      eee8097_roarm_driver \
      eee8097_moveit_bridge \
      eee8097_moveit_config
