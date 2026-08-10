# EEE8097 Jetson Mobile Manipulator

Camera-guided mobile-manipulator project running on a **Jetson Orin Nano** with a
USB camera, **RPLIDAR C1**, and **Waveshare RoArm-M2-S**.

The current real-hardware pipeline is:

```text
YOLO detection
  -> camera-guided RPLIDAR ranging
  -> base_link target localization
  -> measured arm-command calibration
  -> four-waypoint grasp planning
  -> feedback-verified RoArm execution
```

## Current status

- Camera, RPLIDAR, and RoArm adapters are integrated behind stable interfaces.
- The real one-grasp path is implemented with a pre-execution sensor recheck.
- Every terminal pipeline run is recorded as structured JSONL and CSV.
- A Real Camera + Real RPLIDAR + Mock Arm calibration capture path is available.
- A 27-sample affine command calibration maps the raw target to the measured
  RoArm gripper-centre command while preserving both coordinates in run logs.
- A calibrated plan-only profile uses the real sensors with a Mock RoArm before
  the one-grasp real-hardware run.
- RoArm joint self-test, custom home, manual Cartesian test, and boot health checks are available.
- ROS 2, URDF/TF2, MoveIt 2, Gazebo, and Sim2Real evaluation are the next development stage.

> [!CAUTION]
> `src/configs/modular_pipeline.yaml` is the operator-validated **real hardware**
> profile. Starting the pipeline can command physical motion after the typed
> confirmation gate. Keep access to the hardware power switch.

## Authoritative entry points

| Purpose | Entry point |
|---|---|
| Full pipeline | `src/run_modular_pipeline.sh` |
| Pipeline implementation | `src/main_modular.py` |
| Hardware profile | `src/configs/modular_pipeline.yaml` |
| No-motion health check | `src/run_robot_healthcheck.sh` |
| J1-J4 startup self-test | `src/run_robot_startup_test.sh` |
| Manual Cartesian arm test | `src/run_roarm_manual_test.sh` |
| Dataset capture | `src/run_tissue_capture.sh` |
| No-motion calibration capture | `src/run_calibration_capture.sh` |
| Calibration/experiment report | `src/run_calibration_report.sh` |
| Calibrated plan-only profile | `src/configs/calibrated_plan_only.yaml` |

Legacy parallel entry points have been removed. New features should extend the
modular pipeline rather than introduce another top-level controller.

## Quick validation

Inside the existing Docker container:

```bash
cd /workspace/src

# Configuration, compile checks, and all no-hardware unit tests (63 tests)
./test_modular_pipeline.sh

# Validate the committed real-hardware YAML without opening devices
python3 main_modular.py \
  --config configs/modular_pipeline.yaml \
  --validate-config

# Read-only device health check
./run_robot_healthcheck.sh
```

Run the real pipeline only after the health check and workspace inspection:

```bash
cd /workspace/src
./run_modular_pipeline.sh
```

Before the first calibrated real grasp, run the same perception and planning
path with no arm motion:

```bash
./run_modular_pipeline.sh configs/calibrated_plan_only.yaml
```

## Repository layout

```text
docker/                 Jetson container definition
scripts/                Host-side build and container launch helpers
src/configs/            Runtime hardware and home profiles
src/interfaces/         Stable component contracts
src/adapters/           Mock device adapters
src/vision/             YOLO camera implementation
src/lidar/              RPLIDAR parsing and target-sector ranging
src/localization/       Sensor-to-base target localization
src/planning/           Workspace checks and grasp waypoints
src/arm_control/        UART, Cartesian execution, home, and self-test
src/pipeline/           Canonical task state machine
src/experiments/        Structured run records and calibration statistics
src/tests/              No-hardware regression tests
src/docs/               Active project documentation
```

See [`src/docs/README.md`](src/docs/README.md) and
[`src/docs/REPOSITORY_STRUCTURE.md`](src/docs/REPOSITORY_STRUCTURE.md) for the
module ownership and change-management rules.

## Change-management convention

- `feat/<scope>`: one new capability.
- `fix/<scope>`: one defect or measured hardware correction.
- `chore/<scope>`: repository, dependency, or tooling maintenance.
- `exp/<scope>`: experiment code and results that are not production defaults.
- Keep refactors, physical calibration changes, and new features in separate PRs.
- Record hardware parameter changes with test conditions and measured results.
- Use Git history and PRs for revision history; do not add `V5`, `V6`, or
  one-off fix-note documents to the runtime tree.

Model weights are runtime artifacts under `models/` and are intentionally not
tracked in `src/`.
