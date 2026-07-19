#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$SCRIPT_DIR/../docker"

cd "$DOCKER_DIR"

if ! docker compose ps --status running --services | grep -qx "robot-dev"; then
    echo "Error: robot-dev container is not running."
    echo "Start it first with:"
    echo "  ./scripts/start_docker.sh"
    exit 1
fi

docker compose exec robot-dev /bin/bash