# RoArm manual motion test and coordinate calibration

## Important distinction

- `r` in `main_modular.py` resets only the software task state to `IDLE`.
- `./run_roarm_manual_test.sh reset` sends the physical RoArm `T=100` initial-position command.

## Read-only checks

```bash
cd /workspace/src
./run_roarm_manual_test.sh validate
./run_roarm_manual_test.sh status
./run_roarm_manual_test.sh capture-current
```

`capture-current` prints a conservative four-waypoint YAML template in which all
points equal the current EoAT pose. It produces no Cartesian movement.

## Enable motion deliberately

Edit `/workspace/src/configs/modular_pipeline.yaml`:

```yaml
manual_arm_test:
  enabled: true
  allow_real_motion: true
  confirm_clearance: true
```

Keep `coordinates_approved: false` until the four absolute `arm_base` waypoints
have been reviewed and the plan output is correct.

## Gripper and reset tests

```bash
./run_roarm_manual_test.sh gripper-open
./run_roarm_manual_test.sh gripper-close
./run_roarm_manual_test.sh reset
```

Every motion command requires an exact typed confirmation phrase.

## Fixed-coordinate grasp test

1. Read the current EoAT pose with `status`.
2. Fill `manual_arm_test.waypoints` with a clear, reachable path in `arm_base`.
3. Run `./run_roarm_manual_test.sh plan` and inspect all four points.
4. Set `manual_arm_test.coordinates_approved: true`.
5. Run `./run_roarm_manual_test.sh execute`.
6. Run `./run_roarm_manual_test.sh reset` separately after the grasp test.

The manual path is independent of Camera and RPLIDAR. This isolates T=104,
T=105, gripper control, feedback tolerance and physical clearance before sensor
coordinates are permitted to command the arm.

## Coordinate transform used by the main pipeline

The current V3/V4 transform is:

```text
p_arm = Rz(-arm_mount.yaw_in_base_deg)
        * (p_base - arm_mount.origin_in_base_mm)
```

The main parameters are:

```yaml
camera:
  origin_in_base_mm: [x, y, z]
  yaw_in_base_deg: 0.0
  fx_px: ...
  fy_px: ...
  cx_px: ...
  cy_px: ...

lidar:
  origin_in_base_mm: [x, y, z]
  front_angle_deg: ...
  angle_sign: 1

arm_mount:
  origin_in_base_mm: [x, y, z]
  yaw_in_base_deg: 180.0

localization:
  target_z_mm: ...
  calibration_approved: false
```

`target_z_mm` remains a configured grasp height, not a 3-D RPLIDAR result.
Only set `localization.calibration_approved: true` after testing known target
positions and verifying the resulting `target_arm_base` error.
