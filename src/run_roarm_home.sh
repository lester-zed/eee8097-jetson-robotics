#!/usr/bin/env bash
set -euo pipefail

SOURCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ACTION="${1:-preview}"
CONFIG="${2:-${SOURCE_ROOT}/configs/roarm_home.yaml}"

if [[ "${CONFIG}" != /* ]]; then
  CONFIG="${SOURCE_ROOT}/${CONFIG}"
fi

cd "${SOURCE_ROOT}"
exec python3 roarm_home.py "${ACTION}" --config "${CONFIG}"
