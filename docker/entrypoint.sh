#!/bin/bash
set -e

echo "=================================="
echo " Robot Development Container"
echo " Python: $(python3 --version)"
echo " Working directory: $(pwd)"
echo "=================================="

exec "$@"
