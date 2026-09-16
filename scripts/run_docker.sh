#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$SCRIPT_DIR/../docker"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ -f "$REPO_ROOT/.env" ]]; then
    set -a
    # shellcheck disable=SC1091
    source "$REPO_ROOT/.env"
    set +a
fi

# Frozen hardware snapshot of the original Jetson test platform.  The
# environment variables (or an optional local .env file) still override these
# machine-specific defaults when the repository is used on different hardware.
ROARM_DEVICE="${ROARM_DEVICE:-/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_888387d8d017f011b9526a7db887153e-if00-port0}"
RPLIDAR_DEVICE="${RPLIDAR_DEVICE:-/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_c877520e295df0119ae253401045c30f-if00-port0}"
CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video0}"

if [[ ! -c "$ROARM_DEVICE" ]]; then
    echo "Error: RoArm serial device not found: $ROARM_DEVICE"
    exit 1
fi

if [[ ! -c "$RPLIDAR_DEVICE" ]]; then
    echo "Error: RPLIDAR serial device not found: $RPLIDAR_DEVICE"
    exit 1
fi

if [[ ! -c "$CAMERA_DEVICE" ]]; then
    echo "Error: Camera device not found: $CAMERA_DEVICE"
    exit 1
fi

DIALOUT_GID="$(getent group dialout | cut -d: -f3)"
if [[ -z "$DIALOUT_GID" ]]; then
    echo "Error: dialout group was not found on the Jetson host."
    exit 1
fi

# ---------------------------------------------------------------------------
# SSH X11 forwarding
# ---------------------------------------------------------------------------
# Expected after connecting with ssh -X/-Y:
#   DISPLAY=localhost:10.0   (or localhost:11.0, etc.)
#
# Do not hard-code :0/:1. SSH allocates the display number per session.
if [[ -z "${DISPLAY:-}" ]]; then
    echo "Error: DISPLAY is not set."
    echo "Reconnect to the Jetson with X11 forwarding, for example:"
    echo "  ssh -Y <JETSON_USER>@<JETSON_IP>"
    exit 1
fi

if ! command -v xauth >/dev/null 2>&1; then
    echo "Error: xauth is not installed on the Jetson host."
    echo "Install it with: sudo apt install xauth"
    exit 1
fi

SOURCE_XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
if [[ ! -f "$SOURCE_XAUTHORITY" ]]; then
    echo "Error: source Xauthority file not found:"
    echo "  $SOURCE_XAUTHORITY"
    exit 1
fi

# A direct ~/.Xauthority bind can fail inside Docker because the Xauthority
# entry contains the host name (e.g. jetson-host/unix:10), while the
# container has a different host name.  Generate a FamilyWild entry instead.
DOCKER_XAUTH_FILE="/tmp/eee8097-docker-xauth-${UID}"
rm -f "$DOCKER_XAUTH_FILE"
touch "$DOCKER_XAUTH_FILE"
chmod 600 "$DOCKER_XAUTH_FILE"

XAUTH_DATA="$(xauth -f "$SOURCE_XAUTHORITY" nlist "$DISPLAY" 2>/dev/null || true)"
if [[ -z "$XAUTH_DATA" ]]; then
    echo "Error: no Xauthority cookie found for DISPLAY=$DISPLAY"
    echo
    echo "Current cookies:"
    xauth -f "$SOURCE_XAUTHORITY" list || true
    exit 1
fi

printf '%s\n' "$XAUTH_DATA" \
    | sed -e 's/^..../ffff/' \
    | xauth -f "$DOCKER_XAUTH_FILE" nmerge -

if [[ ! -s "$DOCKER_XAUTH_FILE" ]]; then
    echo "Error: failed to generate Docker Xauthority file."
    exit 1
fi

export ROARM_DEVICE
export RPLIDAR_DEVICE
export CAMERA_DEVICE
export DIALOUT_GID
export JETSON_DISPLAY="$DISPLAY"
export DOCKER_XAUTH_FILE
EEE8097_GIT_COMMIT="$(git -C "$REPO_ROOT" rev-parse HEAD 2>/dev/null || true)"
EEE8097_GIT_COMMIT="${EEE8097_GIT_COMMIT:-unknown}"
export EEE8097_GIT_COMMIT

cd "$DOCKER_DIR"

echo "RoArm device:   $ROARM_DEVICE"
echo "RPLIDAR device: $RPLIDAR_DEVICE"
echo "Camera device:  $CAMERA_DEVICE"
echo "Dialout GID:    $DIALOUT_GID"
echo "X11 DISPLAY:    $JETSON_DISPLAY"
echo "Docker XAUTH:   $DOCKER_XAUTH_FILE"
echo "Git commit:     $EEE8097_GIT_COMMIT"

docker compose up -d --force-recreate

echo
docker compose ps

echo
echo "Container X11 environment:"
docker exec robot-dev /bin/bash -lc \
    'printf "DISPLAY=%s\nXAUTHORITY=%s\n" "$DISPLAY" "$XAUTHORITY"; ls -l "$XAUTHORITY"'
