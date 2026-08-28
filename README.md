# EEE8097 Jetson Mobile Manipulator

Camera-guided mobile-manipulator project running on a **Jetson Orin Nano** with a
USB camera, **RPLIDAR C1**, and **Waveshare RoArm-M2-S**.

The current real-hardware pipeline is:

```text
YOLO detection
  -> camera-guided RPLIDAR ranging
  -> base_link target localization
  -> measured grasp-command calibration
  -> four-waypoint grasp planning
  -> feedback-verified RoArm execution
```

## Current status

- Camera, RPLIDAR, and RoArm adapters are integrated behind stable interfaces.
- The real one-grasp path is implemented with a pre-execution sensor recheck.
- Every terminal pipeline run is recorded as structured JSONL and CSV.
- A Real Camera + Real RPLIDAR + Mock Arm calibration capture path is available.
- The original 27-sample affine command calibration established the first
  repeatable sensor-to-command model and remains preserved in Git history.
- The current production grasp calibration is fitted from **11 manually taught
  gripper-centre poses** captured across the reachable workspace on 19 Aug 2026.
  It uses a global `bilinear_xy` mapping followed by a bounded
  `bilinear_local_xy` residual layer. The local correction uses a Wendland-C2
  basis and decays to zero 20 mm from each taught anchor.
- The global bilinear baseline gives a training planar RMSE of **5.87 mm**. The
  local residual layer reduces the same-anchor fitting RMSE to **0.535 mm**,
  while leave-one-position-out planar RMSE remains **7.96 mm**. The previous
  production model had a **21.37 mm** planar RMSE on the eight newly taught
  points that were inside its calibration domain. These values describe
  command-space calibration error, not end-to-end physical grasp accuracy.
- The existing `+10 mm` X and `-15 mm` Y planner offsets are retained as part of
  the production command convention. Both offsets were accounted for during
  fitting, so they are not double-counted.
- Grasp Z remains the approved fixed task-plane value (`-110 mm`). The manually
  taught T=105 Z values are retained as evidence but are not yet fitted because
  the current RPLIDAR sensing is planar.
- A calibrated plan-only profile uses the real sensors with a Mock RoArm before
  the one-grasp real-hardware run.
- RoArm joint self-test, custom home, manual Cartesian test, and boot health checks are available.
- A separate ROS 2 Humble workspace now provides the official RoArm URDF/TF
  chain, real `/joint_states`, MoveIt 2 IK and collision-aware planning,
  current-TCP pose output, and guarded arm-only trajectory execution.
- The failed gripper is excluded from the MoveIt planning group and controller.
  The real driver hard-rejects every trajectory containing the gripper joint.
- Gazebo and Sim2Real evaluation remain later research stages; they are not
  claimed by this real-arm MoveIt 2 integration.

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
| ROS 2 / MoveIt 2 workspace | `src/ros2_ws/` |
| Safe MoveIt launcher | `src/ros2_ws/run_safe_moveit.sh` |
| ROS 2 container helper | `scripts/run_ros2_docker.sh` |

Legacy parallel entry points have been removed. New features should extend the
modular pipeline rather than introduce another top-level controller.

## Quick validation

Inside the existing Docker container:

```bash
cd /workspace/src

# Configuration, compile checks, and all no-hardware unit tests
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

Before the first real grasp after a calibration change, run the same perception
and planning path with no arm motion:

```bash
./run_modular_pipeline.sh configs/calibrated_plan_only.yaml
```

Validate the MoveIt 2 extension without opening any real device:

```bash
cd /workspace/src/ros2_ws
python3 test_static.py
./build_ros2.sh
./run_safe_moveit.sh plan
```

See [`src/docs/ROS2_MOVEIT2_SAFE_ARM_ONLY.md`](src/docs/ROS2_MOVEIT2_SAFE_ARM_ONLY.md)
before starting the real-state or real-execution mode. The legacy perception
pipeline and the ROS 2 driver must never own `/dev/ttyROARM` at the same time.

## Repository layout

```text
docker/                 Jetson container definition
scripts/                Host-side build and container launch helpers
src/configs/            Runtime hardware and home profiles
src/interfaces/         Stable component contracts
src/adapters/           Mock device adapters
src/vision/             YOLO camera implementation
src/lidar/              RPLIDAR parsing and target-sector ranging
src/localization/       Sensor-to-base target localization and command calibration
src/planning/           Workspace checks and grasp waypoints
src/arm_control/        UART, Cartesian execution, home, and self-test
src/pipeline/           Canonical task state machine
src/experiments/        Structured run records and calibration statistics
src/tests/              No-hardware regression tests and calibration fixtures
src/docs/               Active project documentation
src/ros2_ws/            ROS 2 Humble, MoveIt 2 and guarded real-arm bridge
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
