#!/usr/bin/env bash
set -euo pipefail

SOURCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SOURCE_ROOT}"

python3 main_modular.py \
  --config configs/modular_pipeline.yaml \
  --validate-config

python3 -m compileall -q \
  configuration \
  interfaces \
  adapters \
  pipeline \
  vision/yolo_camera_adapter.py \
  lidar/range_sensor.py \
  localization \
  planning \
  arm_control/arm_adapter.py \
  arm_control/cartesian_roarm_controller.py \
  main_modular.py

python3 -m unittest discover \
  -s tests \
  -p 'test_modular_pipeline*.py' \
  -v
