#!/usr/bin/env bash
set -euo pipefail

# One-time setup inside the existing robot-dev container.  The SDK is stored
# under the host-mounted src/ tree, so rebuilding or recreating the container
# does not remove it.
SDK_ROOT="${RPLIDAR_SDK_ROOT:-/workspace/src/third_party/rplidar_sdk}"
SDK_REF="${RPLIDAR_SDK_REF:-99478e5fb90de3b4a6db0080acacd373f8b36869}"
SDK_URL="https://github.com/Slamtec/rplidar_sdk.git"

if [[ ! -d "${SDK_ROOT}/.git" ]]; then
    mkdir -p "${SDK_ROOT}"
    git -C "${SDK_ROOT}" init
    git -C "${SDK_ROOT}" remote add origin "${SDK_URL}"
fi

git -C "${SDK_ROOT}" fetch --depth 1 origin "${SDK_REF}"
git -C "${SDK_ROOT}" checkout --detach FETCH_HEAD

CPU_COUNT="$(getconf _NPROCESSORS_ONLN 2>/dev/null || true)"
if [[ -z "${CPU_COUNT}" || "${CPU_COUNT}" -lt 1 ]]; then
    CPU_COUNT=1
fi

make -C "${SDK_ROOT}" -j"${CPU_COUNT}"

SDK_BIN="${SDK_ROOT}/output/Linux/Release/ultra_simple"
if [[ ! -x "${SDK_BIN}" ]]; then
    echo "ERROR: SDK build finished without ${SDK_BIN}" >&2
    exit 1
fi

echo "RPLIDAR SDK ready: ${SDK_BIN}"
