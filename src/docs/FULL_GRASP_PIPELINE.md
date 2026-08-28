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
  -> measured grasp-command calibration
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

  command_calibration:
    enabled: true
    model: bilinear_local_xy
    calibration_id: tissue-grasp-waypoint-20260819-v3

planner:
  grasp_x_offset_mm: 10.0
  grasp_y_offset_mm: -15.0
```

These are operator-validated values for the current physical installation, not
portable defaults. Re-measure them after moving a sensor, camera, arm, or target
surface.

## 2026-08-19 taught grasp-waypoint model

Eleven physical target positions were sampled from far to near and across the
left/right workspace. At each position, the target was measured by the Camera +
RPLIDAR pipeline and then the RoArm gripper centre was manually taught to the
desired grasp pose. The saved pair is therefore:

```text
nominal Camera/LiDAR arm target -> manually taught RoArm T=105 command pose
```

The manually taught T=105 pose is a **command-space reference**, not external
motion-capture ground truth. The current 2-D LiDAR still provides only planar
X/Y information, so the production grasp Z remains the approved fixed value of
`-110 mm`.

A first-order affine XY fit was compared with a global model that also contains
an `x*y` interaction. The global bilinear model reduced the position-dependent
lateral error, but visible residuals remained around several taught locations.
The current v3 calibration therefore adds a compact local residual layer to the
global mapping. It uses a Wendland-C2 basis with a 20 mm support radius. Each
local term becomes exactly zero outside its measured neighbourhood, so the
global bilinear mapping remains responsible for unsupported parts of the
authorised workspace.

The active model uses centred coordinates:

```text
dx = x_nom - (-377.70475532)
dy = y_nom - 2.01385567

x_pre = 1.11871727*dx
      + 0.03403679*dy
      + 0.00039473493*dx*dy
      - 377.68143017

y_pre = 0.08296279*dx
      + 1.40788329*dy
      + 0.00411045625*dx*dy
      - 8.48493033
```

For each taught anchor, v3 adds the stored X/Y residual multiplied by:

```text
q = distance_to_anchor / 20 mm
w(q) = (1 - q)^4 * (4q + 1),  for 0 <= q < 1
w(q) = 0,                     for q >= 1
```

This local term improves agreement at taught commands without applying a
workspace-wide fixed offset.

The production planner then retains the existing global fine-tune:

```text
x_grasp = x_pre + 10 mm
y_grasp = y_pre - 15 mm
z_grasp = -110 mm
```

Those offsets were explicitly removed from the taught references during fitting,
so they are not double-counted.

The 11-point dataset gives:

- global bilinear training planar RMSE: **5.87 mm**;
- v3 local-residual training planar RMSE: **0.535 mm**;
- leave-one-position-out planar RMSE: **7.96 mm**;
- largest leave-one-position-out error: **13.04 mm**.

The 0.535 mm result is an in-sample command-space fitting error at the taught
anchors. It is not an independent measurement of Cartesian accuracy or
physical grasp success. The leave-one-position-out result is the more cautious
estimate for an unseen position inside the tested workspace.

For comparison, the previous production grasp command had **21.37 mm planar
RMSE** on the eight new taught points that were inside its old calibration
domain. Three newly sampled far positions were outside the previous model's
input bounds entirely.

The original sensor target is not overwritten. `target_base_link` remains raw,
`metadata.nominal_target_arm_base` records the pre-model arm coordinate, and
`target_arm_base` is the calibrated planner input. The command-calibration
metadata records the active model type and origin.

## Sensor and execution safeguards

Before planning, the task manager performs a second Camera + RPLIDAR observation
and rejects excessive image-centre, range, or fused target-position shift. It
also rejects noisy/mixed RPLIDAR sectors through MAD and span thresholds.

The command calibration rejects nominal targets outside its measured input box
and corrected targets outside a conservative output box. The output check is
important because the rectangular input box contains unmeasured corner
combinations. The local residual also has a strict 20 mm support radius, so it
cannot silently modify distant regions. The geometry values used by the
dataset remain locked through `expected_inputs`.

The planner workspace is also independent from the calibration domain. It is
currently expanded only enough to include the manually exercised far/lateral
positions while retaining a bounded radius and Z range.

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

# Validate YAML and active calibration without opening devices
python3 main_modular.py \
  --config configs/modular_pipeline.yaml \
  --validate-config

# Read-only device check
./run_robot_healthcheck.sh

# Real Camera + Real RPLIDAR + calibrated plan + Mock RoArm
./run_modular_pipeline.sh configs/calibrated_plan_only.yaml

# Full real-hardware pipeline
./run_modular_pipeline.sh
```

After a calibration change, use the plan-only profile first and inspect the
planned `grasp` waypoint at several far, middle, close, left and right target
positions before authorising real motion.

Every terminal run is also written to structured JSONL/CSV. Use the dedicated
Real Camera + Real RPLIDAR + Mock Arm profile for sensor-only localisation
measurements; see `CALIBRATION_AND_EXPERIMENT_LOGGING.md`.

## Current limitation

The new model has only one manually taught command reference per sampled
position. It should therefore be treated as a measured prototype update rather
than a production-grade calibration. The next physical validation should repeat
multi-position grasps with the new model and report physical retention success
separately from trajectory completion.

The `VERIFYING` state still confirms controller execution and feedback, but it
does not prove that the object remains inside the gripper. A post-grasp visual
or gripper-feedback success check remains the next perception milestone.
