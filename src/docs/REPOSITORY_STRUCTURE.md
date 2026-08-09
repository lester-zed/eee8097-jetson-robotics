# Repository structure and change boundaries

## Canonical runtime path

```text
main_modular.py
  -> interfaces/
  -> Camera / RPLIDAR / RoArm adapters
  -> pipeline/task_manager.py
  -> localization/
  -> planning/
  -> arm_control/cartesian_roarm_controller.py
```

`run_modular_pipeline.sh` launches the configured custom home and then this
pipeline. There is no second application state machine.

## High-frequency development code

| Area | Authoritative files | Typical changes |
|---|---|---|
| Runtime composition | `main_modular.py`, `configuration/loader.py` | adapter wiring, validated config fields |
| State machine | `pipeline/task_manager.py`, `pipeline/types.py` | task states, recheck, execution results |
| Vision | `vision/yolo_camera.py`, `vision/yolo_camera_adapter.py` | model inference and stable detections |
| Ranging | `lidar/range_sensor.py`, `lidar/rplidar_distance.py` | target-sector selection and scan filtering |
| Localization | `localization/target_localizer.py`, `localization/transforms.py` | Camera/LiDAR/base/arm mapping |
| Planning | `planning/grasp_planner.py` | workspace and grasp waypoints |
| Arm execution | `arm_control/cartesian_roarm_controller.py`, `roarm_uart.py` | T=104/T=105/T=106 behavior |
| Hardware profile | `configs/modular_pipeline.yaml` | measured physical parameters only |
| Regression tests | `tests/` | no-hardware behavior and safety gates |

## Diagnostic and operator tools

- `robot_healthcheck.py`: read-only device and protocol checks.
- `arm_control/roarm_joint_selftest.py`: operator-supervised J1-J4 movement.
- `roarm_home.py`: feedback-verified custom T=122 home.
- `roarm_manual_test.py`: isolated Cartesian/gripper validation.
- `tools/tissue_dataset_capture.py`: dataset acquisition.

These tools are intentionally separate from the autonomous task state machine.
They may test hardware, but they must not become parallel application entry
points.

## Removed legacy chain

The following concepts were superseded and should not be reintroduced:

- `main.py`, `main_roarm.py`, and `main_roarm_reverse.py`;
- the old `task/TaskManager` state machine;
- base-only `ArmController` and `RealRoArmController`;
- `vision/mock_vision.py` and the tracked generic `vision/yolov8n.pt`;
- package-specific ZIP instructions and versioned fix-note documents.

Equivalent maintained functionality now lives in `main_modular.py`,
`pipeline/`, `adapters/mock_devices.py`, and the Cartesian RoArm controller.

## Pull-request boundaries

Use one branch and PR per concern:

1. `chore/...` for cleanup, CI, dependencies, or layout.
2. `fix/...` for one observed defect.
3. `feat/...` for one capability with tests.
4. `exp/...` for calibration or Sim2Real experiments and their results.

Do not mix these categories in one commit:

- code refactoring;
- real-hardware parameter changes;
- new perception or planning behavior.

For every physical parameter change, record the device arrangement, target
position, before/after value, measured error, and test result in the PR.

