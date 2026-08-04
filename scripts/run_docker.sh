#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$SCRIPT_DIR/../docker"

ROARM_DEVICE="${ROARM_DEVICE:-/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_888387d8d017f011b9526a7db887153e-if00-port0}"

RPLIDAR_DEVICE="${RPLIDAR_DEVICE:-/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_c877520e295df0119ae253401045c30f-if00-port0}"

if [[ ! -c "$ROARM_DEVICE" ]]; then
    echo "Error: RoArm serial device not found: $ROARM_DEVICE"
    echo
    echo "Please check:"
    echo "  1. RoArm is powered on"
    echo "  2. USB data cable is connected"
    echo "  3. Run: ls -l /dev/ttyUSB*"
    exit 1
fi

DIALOUT_GID="$(getent group dialout | cut -d: -f3)"

if [[ -z "$DIALOUT_GID" ]]; then
    echo "Error: dialout group was not found on the Jetson host."
    exit 1
fi

export ROARM_DEVICE
export RPLIDAR_DEVICE
export DIALOUT_GID

cd "$DOCKER_DIR"

echo "RoArm device: $ROARM_DEVICE"
echo "Dialout GID: $DIALOUT_GID"

docker compose up -d --force-recreate

echo
docker compose ps