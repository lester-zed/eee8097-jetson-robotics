#!/usr/bin/env bash
set -euo pipefail

cd /workspace/src
exec python robot_healthcheck.py "$@"
