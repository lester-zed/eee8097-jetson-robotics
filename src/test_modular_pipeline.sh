#!/usr/bin/env bash
set -euo pipefail

SOURCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SOURCE_ROOT}"

python3 main_modular.py \
  --config configs/modular_pipeline.yaml \
  --validate-config

python3 main_modular.py \
  --config configs/calibrated_plan_only.yaml \
  --validate-config

python3 tools/calibration_capture.py \
  --config configs/calibration_capture.yaml \
  --validate-config

python3 roarm_manual_test.py \
  validate \
  --config configs/modular_pipeline.yaml

python3 -m compileall -q \
  configuration \
  experiments \
  interfaces \
  adapters \
  pipeline \
  vision/yolo_camera_adapter.py \
  lidar/range_sensor.py \
  localization \
  planning \
  arm_control/arm_adapter.py \
  arm_control/cartesian_roarm_controller.py \
  main_modular.py \
  launch_modular_pipeline.py \
  roarm_manual_test.py

python3 -m compileall -q \
  tools/calibration_capture.py \
  tools/calibration_report.py \
  tools/annotate_experiment.py

python3 -m unittest discover \
  -s tests \
  -p 'test_*.py' \
  -v
