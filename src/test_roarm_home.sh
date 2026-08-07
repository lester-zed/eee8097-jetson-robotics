#!/usr/bin/env bash
set -euo pipefail

SOURCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SOURCE_ROOT}"

# Ensure repository-local modules and tests take precedence over any third-party
# package also named "tests" in the Docker image.
export PYTHONPATH="${SOURCE_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

printf '\n[1/3] Validate roarm_home.yaml\n'
python3 roarm_home.py validate --config configs/roarm_home.yaml

printf '\n[2/3] Compile V7 home/launcher/test files\n'
python3 -m compileall -q \
  roarm_home.py \
  launch_modular_pipeline.py \
  tests/test_roarm_home.py

printf '\n[3/3] Run repository-local RoArm home unit tests\n'
python3 -m unittest discover \
  -s "${SOURCE_ROOT}/tests" \
  -p 'test_roarm_home.py' \
  -v

printf '\nRoArm custom-home validation completed successfully.\n'
