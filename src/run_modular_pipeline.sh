#!/usr/bin/env bash
set -euo pipefail

SOURCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIPELINE_CONFIG="${1:-${SOURCE_ROOT}/configs/modular_pipeline.yaml}"
HOME_CONFIG="${2:-${SOURCE_ROOT}/configs/roarm_home.yaml}"

if [[ "${PIPELINE_CONFIG}" != /* ]]; then
  PIPELINE_CONFIG="${SOURCE_ROOT}/${PIPELINE_CONFIG}"
fi
if [[ "${HOME_CONFIG}" != /* ]]; then
  HOME_CONFIG="${SOURCE_ROOT}/${HOME_CONFIG}"
fi

cd "${SOURCE_ROOT}"
exec python3 launch_modular_pipeline.py \
  --config "${PIPELINE_CONFIG}" \
  --home-config "${HOME_CONFIG}"
