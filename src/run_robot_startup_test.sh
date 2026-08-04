#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

run_motion=0
skip_prompt=0
record_directions=0
step_confirm=0
health_args=()

usage() {
    cat <<'EOF'
Usage:
  bash run_robot_startup_test.sh [options]

Default: run the existing read-only RoArm + RPLIDAR + Camera health check.

Options:
  --motion              Then run the low-speed J1-J4 move-and-return test.
  --yes                 Skip the one-time MOVE prompt (guarded bench only).
  --record-directions   Record observed motion directions at each target.
  --step-confirm        Confirm before moving each joint.
  --check-yolo          Include one YOLO inference in the health check.
  -h, --help            Show this help.
EOF
}

while (($#)); do
    case "$1" in
        --motion)
            run_motion=1
            ;;
        --yes)
            skip_prompt=1
            ;;
        --record-directions)
            record_directions=1
            ;;
        --step-confirm)
            step_confirm=1
            ;;
        --check-yolo)
            health_args+=("--check-yolo")
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

echo "[1/2] Read-only device health check"
bash "$SCRIPT_DIR/run_robot_healthcheck.sh" "${health_args[@]}"

if ((run_motion == 0)); then
    echo "[2/2] RoArm motion test skipped (add --motion to enable it)"
    exit 0
fi

motion_args=(
    --port /dev/ttyROARM
    --execute
    --confirm-clearance
)

if ((skip_prompt)); then
    motion_args+=("--yes")
fi
if ((record_directions)); then
    motion_args+=("--record-directions")
fi
if ((step_confirm)); then
    motion_args+=("--step-confirm")
fi

echo "[2/2] Feedback-verified RoArm J1-J4 motion test"
python -m arm_control.roarm_joint_selftest "${motion_args[@]}"
