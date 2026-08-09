# Full Camera -> RPLIDAR -> RoArm grasp pipeline

## Active data path

```text
YOLO tissue_pack detection
  -> stable single-target selection
  -> camera-pixel bearing
  -> Camera-guided RPLIDAR target sector
  -> filtered planar range
  -> target pose in base_link
  -> base_link to arm_base transform
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
  target_z_mm: -110.0
  calibration_approved: true
  target_z_approved: true
```

These are operator-validated values for the current physical installation, not
portable defaults. Re-measure them after moving a sensor, camera, arm, or target
surface.

## Sensor and execution safeguards

Before planning, the task manager performs a second Camera + RPLIDAR observation
and rejects excessive image-centre, range, or fused target-position shift. It
also rejects noisy/mixed RPLIDAR sectors through MAD and span thresholds.

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

# Full real-hardware pipeline
./run_modular_pipeline.sh
```

During a task, use `p` to inspect the latest detection, range statistics,
`target_base_link`, `target_arm_base`, recheck deltas, plan, and execution
feedback.

## Current limitation

The `VERIFYING` state confirms controller execution and feedback, but it does
not yet prove that the object remains inside the gripper. The next perception
milestone should add a post-grasp visual or gripper-feedback success check and
report grasp success separately from trajectory success.
