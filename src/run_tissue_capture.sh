#!/usr/bin/env bash
set -euo pipefail

SOURCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SOURCE_ROOT}"

exec python3 tools/tissue_dataset_capture.py "$@"
