#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MODE="${1:-plan}"

if [[ -f "$REPO_ROOT/.env" ]]; then
    set -a
    # shellcheck disable=SC1091
    source "$REPO_ROOT/.env"
    set +a
fi

case "$MODE" in
    plan)
        ROS2_ROARM_DEVICE=/dev/null
        ;;
    observe|execute)
        # Retain the original Jetson hardware ID as a reproducible snapshot;
        # ROARM_DEVICE or .env can override it on another machine.
        ROS2_ROARM_DEVICE="${ROARM_DEVICE:-/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_888387d8d017f011b9526a7db887153e-if00-port0}"
        if [[ ! -c "$ROS2_ROARM_DEVICE" ]]; then
            echo "Error: RoArm serial device not found: $ROS2_ROARM_DEVICE"
            exit 1
        fi
        ;;
    *)
        echo "Usage: $0 {plan|observe|execute}"
        exit 2
        ;;
esac

DIALOUT_GID="$(getent group dialout | cut -d: -f3)"
DIALOUT_GID="${DIALOUT_GID:-20}"
ROS2_XAUTH_FILE="/tmp/eee8097-ros2-xauth-${UID}"
rm -f "$ROS2_XAUTH_FILE"
touch "$ROS2_XAUTH_FILE"
chmod 600 "$ROS2_XAUTH_FILE"
ROS2_DISPLAY="${DISPLAY:-}"
if [[ -n "${USE_RVIZ:-}" ]]; then
    ROS2_USE_RVIZ="$USE_RVIZ"
elif [[ -n "$ROS2_DISPLAY" ]]; then
    ROS2_USE_RVIZ=true
else
    ROS2_USE_RVIZ=false
fi

if [[ -n "$ROS2_DISPLAY" ]] && command -v xauth >/dev/null 2>&1; then
    SOURCE_XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
    if [[ -f "$SOURCE_XAUTHORITY" ]]; then
        XAUTH_DATA="$(xauth -f "$SOURCE_XAUTHORITY" nlist "$ROS2_DISPLAY" 2>/dev/null || true)"
        if [[ -n "$XAUTH_DATA" ]]; then
            printf '%s\n' "$XAUTH_DATA" \
                | sed -e 's/^..../ffff/' \
                | xauth -f "$ROS2_XAUTH_FILE" nmerge -
        fi
    fi
fi

export ROS2_ROARM_DEVICE DIALOUT_GID ROS2_XAUTH_FILE ROS2_DISPLAY
cd "$REPO_ROOT/docker"
docker compose -f docker-compose.ros2.yml up -d --build --force-recreate
docker exec eee8097-ros2 /bin/bash -lc \
    'cd /workspace/src/ros2_ws && ./build_ros2.sh'

if [[ "$MODE" == "execute" ]]; then
    docker exec -it -e USE_RVIZ="$ROS2_USE_RVIZ" eee8097-ros2 /bin/bash -lc \
        'cd /workspace/src/ros2_ws && ./run_safe_moveit.sh execute'
else
    docker exec -it -e USE_RVIZ="$ROS2_USE_RVIZ" eee8097-ros2 /bin/bash -lc \
        "cd /workspace/src/ros2_ws && ./run_safe_moveit.sh $MODE"
fi
