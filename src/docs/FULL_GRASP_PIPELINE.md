# Full Camera -> RPLIDAR -> RoArm grasp pipeline

## Active data path

```text
YOLO tissue_pack detection
  -> stable single-target selection
  -> camera-pixel bearing
  -> Camera-guided RPLIDAR target sector
  -> filtered planar range
  -> raw target pose in base_link
  -> nominal base_link to arm_base transform
  -> measured arm-command calibration
  -> workspace-checked four-waypoint plan
  -> feedback-verified Cartesian execution
```

The maintained implementation is composed by `main_modular.py`; it is not a
parallel demo.

## Committed hardware geometry

```yaml
camera:
  origin_in_base_mm: [-260.0, 0.0, -70.0]
  yaw_in_base_deg: 180.0

lidar:
  origin_in_base_mm: [-260.0, 0.0, -100.0]
  front_angle_deg: 180.0

arm_mount:
  origin_in_base_mm: [0.0, 0.0, 0.0]
  yaw_in_base_deg: 0.0

localization:
  target_z_mm: -100.0
  calibration_approved: true
  target_z_approved: true

  command_calibration:
    enabled: true
    calibration_id: tissue-command-grid-20260810-v1

planner:
  grasp_y_offset_mm: 0.0
```

These are operator-validated values for the current physical installation, not
portable defaults. Re-measure them after moving a sensor, camera, arm, or target
surface.

The model was fitted from 27 observations at nine commanded gripper-centre
positions. It maps the nominal `arm_base` target to the RoArm command:

```text
x_cmd =  1.27216682*x_raw + 0.01445898*y_raw + 83.15405484
y_cmd = -0.16907799*x_raw + 1.12365137*y_raw - 76.94479474
z_cmd = -100 mm
```

The original sensor target is not overwritten. `target_base_link` remains raw,
`metadata.nominal_target_arm_base` records the pre-model arm coordinate, and
`target_arm_base` is the calibrated command consumed by the planner. The
training planar RMSE is 1.08 mm; grouped leave-one-position-out RMSE is
1.57 mm with 2.87 mm maximum error.

## Sensor and execution safeguards

Before planning, the task manager performs a second Camera + RPLIDAR observation
and rejects excessive image-centre, range, or fused target-position shift. It
also rejects noisy/mixed RPLIDAR sectors through MAD and span thresholds.

The command calibration additionally rejects a target outside the measured
input/output boxes and rejects the bearing-only LiDAR fallback. Its config
locks the sensor geometry used by the dataset. A changed camera, LiDAR, or arm
mount value therefore fails configuration validation before the serial port is
opened. The planner retains the independent `min_x_mm=-430` workspace limit;
the `X=-440 mm` calibration row improves the fit but is not authorized for
real motion.

Real execution additionally requires:

- approved planar calibration and grasp Z;
- the configured Cartesian workspace;
- real-motion and clearance interlocks;
- an exact typed confirmation phrase;
- valid supply voltage and T=105 waypoint feedback;
- one grasp per process.

The controller executes:

```text
open gripper
  -> pregrasp T=104
  -> grasp T=104
  -> close gripper T=106
  -> lift T=104
  -> retreat T=104
```

## Validation commands

```bash
cd /workspace/src

# No device access
./test_modular_pipeline.sh

# Read-only device check
./run_robot_healthcheck.sh

# Real Camera + Real RPLIDAR + calibrated plan + Mock RoArm
./run_modular_pipeline.sh configs/calibrated_plan_only.yaml

# Full real-hardware pipeline
./run_modular_pipeline.sh
```

During a task, use `p` to inspect the latest detection, range statistics, raw
`target_base_link`, calibrated `target_arm_base`, recheck deltas, plan, and
execution feedback. Near P12, the grasp waypoint should be close to
`[-380, -20, -100] mm` before any real motion is authorized.

Every terminal run is also written to structured JSONL/CSV. Use the dedicated
Real Camera + Real RPLIDAR + Mock Arm profile for localization measurements;
see `CALIBRATION_AND_EXPERIMENT_LOGGING.md`.

## Current limitation

The `VERIFYING` state confirms controller execution and feedback, but it does
not yet prove that the object remains inside the gripper. The next perception
milestone should add a post-grasp visual or gripper-feedback success check and
report grasp success separately from trajectory success.
