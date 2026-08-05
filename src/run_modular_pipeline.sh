#!/usr/bin/env bash
set -euo pipefail

# This script is intentionally stored inside src because Docker mounts only
# the host src directory at /workspace/src.
SOURCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PATH="${1:-${SOURCE_ROOT}/configs/modular_pipeline.yaml}"

# A relative custom path is resolved relative to the mounted source root.
if [[ "${CONFIG_PATH}" != /* ]]; then
  CONFIG_PATH="${SOURCE_ROOT}/${CONFIG_PATH}"
fi

cd "${SOURCE_ROOT}"
exec python3 main_modular.py --config "${CONFIG_PATH}"
