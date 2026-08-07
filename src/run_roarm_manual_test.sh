#!/usr/bin/env bash
set -euo pipefail

SOURCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ACTION="${1:-status}"
CONFIG_PATH="${2:-${SOURCE_ROOT}/configs/modular_pipeline.yaml}"

if [[ "${CONFIG_PATH}" != /* ]]; then
  CONFIG_PATH="${SOURCE_ROOT}/${CONFIG_PATH}"
fi

cd "${SOURCE_ROOT}"
exec python3 roarm_manual_test.py "${ACTION}" --config "${CONFIG_PATH}"
