#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="robot-dev"

if ! docker inspect \
    --format '{{.State.Running}}' \
    "$CONTAINER_NAME" 2>/dev/null | grep -qx "true"; then
    echo "Error: $CONTAINER_NAME container is not running."
    echo "Start it first with:"
    echo "  ./scripts/run_docker.sh"
    exit 1
fi

# Use docker exec directly instead of `docker compose exec`.
# This avoids reparsing docker-compose.yml in a new shell where temporary
# JETSON_DISPLAY / DOCKER_XAUTH_FILE variables are no longer exported.
exec docker exec -it "$CONTAINER_NAME" /bin/bash
