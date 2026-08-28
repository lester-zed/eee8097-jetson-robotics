#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-plan}"
USE_RVIZ="${USE_RVIZ:-true}"
PROJECT_SRC="${PROJECT_SRC:-/workspace/src}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyROARM}"

source /opt/ros/humble/setup.bash
if [[ -f /opt/roarm_vendor_ws/install/setup.bash ]]; then
    source /opt/roarm_vendor_ws/install/setup.bash
elif [[ -f "${ROARM_VENDOR_WS:-$HOME/ros_vendor/roarm_ws}/install/setup.bash" ]]; then
    source "${ROARM_VENDOR_WS:-$HOME/ros_vendor/roarm_ws}/install/setup.bash"
fi
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install/setup.bash"

case "$MODE" in
    plan)
        USE_REAL=false
        ALLOW_MOTION=false
        ALLOW_EXECUTION=false
        ;;
    observe)
        USE_REAL=true
        ALLOW_MOTION=false
        ALLOW_EXECUTION=false
        ;;
    execute)
        if [[ ! -c "$SERIAL_PORT" ]]; then
            echo "Error: serial device not found: $SERIAL_PORT"
            exit 1
        fi
        echo "This mode may move J1-J3 of the physical RoArm."
        echo "The gripper is hard-disabled, but the arm swept volume must be clear."
        read -r -p "Type ENABLE ARM-ONLY MOTION to continue: " CONFIRMATION
        if [[ "$CONFIRMATION" != "ENABLE ARM-ONLY MOTION" ]]; then
            echo "Confirmation did not match; no motion launch started."
            exit 2
        fi
        USE_REAL=true
        ALLOW_MOTION=true
        ALLOW_EXECUTION=true
        ;;
    *)
        echo "Usage: $0 {plan|observe|execute}"
        exit 2
        ;;
esac

exec ros2 launch eee8097_moveit_config safe_moveit.launch.py \
    use_real_hardware:="$USE_REAL" \
    allow_motion:="$ALLOW_MOTION" \
    allow_execution:="$ALLOW_EXECUTION" \
    use_rviz:="$USE_RVIZ" \
    serial_port:="$SERIAL_PORT" \
    project_src:="$PROJECT_SRC"
