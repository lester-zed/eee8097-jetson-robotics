# EEE8097 full Camera -> RPLIDAR -> RoArm grasp pipeline

This revision continues the existing modular pipeline instead of adding a parallel demo.

## What changed

The current repository already has the basic path:

```text
YOLO tissue_pack
  -> CameraGuidedRPLidarAdapter
  -> PlanarTargetLocalizer
  -> base_link -> arm_base transform
  -> SimpleTopDownGraspPlanner
  -> CartesianRoArmAdapter
  -> T=104 / T=105 / T=106
```

This revision adds two safeguards needed before real automatic grasping:

1. **Pre-execution Camera + RPLIDAR recheck**
   - detect the target again after the first localization;
   - range it again;
   - reject if image-centre shift or fused target-position shift is too large;
   - plan from the second, most recent fused target.

2. **Independent grasp-Z approval**
   - RPLIDAR C1 is 2-D, so X/Y are sensor-derived but Z is configured;
   - real automatic motion now requires both `calibration_approved` and `target_z_approved`.

## Current calibration preserved from main

```yaml
camera.origin_in_base_mm: [-260.0, 0.0, -70.0]
lidar.origin_in_base_mm:  [-260.0, 0.0, -100.0]
arm_mount.origin_in_base_mm: [0.0, 0.0, 0.0]
arm_mount.yaw_in_base_deg: 180.0
```

The current `target_z_mm: 140.0` remains a configured value and is **not** automatically approved.

## Phase 1 - complete dry run with real Camera + real RPLIDAR

Keep:

```yaml
localization:
  calibration_approved: false
  target_z_approved: false

arm:
  mode: mock
  allow_real_motion: false
  confirm_clearance: false
```

Inside Docker:

```bash
cd /workspace/src
python3 main_modular.py --config configs/modular_pipeline.yaml
```

Then:

```text
robot> s
```

After the task reaches `COMPLETE` or `ERROR`:

```text
robot> p
```

Inspect:

- `detection`
- `range.distance_mm`
- `range.mad_mm`
- `range.minimum_mm` / `range.maximum_mm`
- `range.target_base_link`
- `target.target_base_link`
- `target.target_arm_base`
- `verification.center_shift_px`
- `verification.range_shift_mm`
- `verification.target_shift_mm`
- `plan.waypoints`

## Phase 2 - physical coordinate validation

Place the tissue pack at several known positions: centre / left / right / near / far.

Compare your measured coordinates with:

```text
target.target_base_link
target.target_arm_base
```

Do not approve the calibration if:

- left/right signs are reversed;
- near/far X motion is reversed;
- `front_angle_deg` is not robot-forward;
- `angle_sign` is wrong;
- LiDAR scans the wall/background instead of the tissue pack;
- the LiDAR scan plane does not physically intersect the tissue pack;
- grasp Z has not been physically determined.

## Phase 3 - enable one real Cartesian grasp

Only after the above checks, change:

```yaml
localization:
  target_z_mm: <validated grasp height>
  calibration_approved: true
  target_z_approved: true

arm:
  mode: real
  allow_real_motion: true
  confirm_clearance: true
```

Keep:

```yaml
arm:
  require_typed_confirmation: true
  one_grasp_per_process: true

pipeline:
  pre_execute_recheck: true
```

The existing `CartesianRoArmController` then executes the final plan as:

```text
open clamp
  -> pregrasp T=104
  -> grasp T=104
  -> close clamp T=106
  -> lift T=104
  -> retreat T=104
```

T=105 feedback continues to verify waypoint arrival.

For the first automatic real-grasp test, put the tissue pack in a position already demonstrated to be reachable with the standalone manual Cartesian RoArm test, and keep immediate access to the arm power switch.
