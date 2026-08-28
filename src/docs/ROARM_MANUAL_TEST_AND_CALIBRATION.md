# RoArm manual motion test and coordinate calibration

The manual tool isolates RoArm UART, Cartesian waypoints, gripper control, and
feedback from Camera and RPLIDAR behavior.

## Software reset versus physical reset

- `r` in `main_modular.py` resets only the task state to `IDLE`.
- `./run_roarm_manual_test.sh reset` sends the explicit physical T=100 reset
  after its confirmation gate.

## Read-only commands

```bash
cd /workspace/src
./run_roarm_manual_test.sh validate
./run_roarm_manual_test.sh status
./run_roarm_manual_test.sh capture-current
./run_roarm_manual_test.sh plan
```

`capture-current` prints a conservative four-waypoint YAML template; it does
not move the arm.

## Motion commands

```bash
./run_roarm_manual_test.sh gripper-open
./run_roarm_manual_test.sh gripper-close
./run_roarm_manual_test.sh execute
./run_roarm_manual_test.sh reset
```

Every physical command requires the relevant safety flags and exact typed
confirmation phrase from `configs/modular_pipeline.yaml`.

## Fixed-coordinate grasp procedure

1. Read the current end-effector pose with `status`.
2. Put the object in a clear, previously reachable location.
3. Update only `manual_arm_test.waypoints`.
4. Run `plan` and inspect all four absolute `arm_base` points.
5. Confirm physical clearance and keep access to the power switch.
6. Run `execute` once.
7. Record final error and failure type before changing another parameter.

## Transform used by the main pipeline

```text
p_arm = Rz(-arm_mount.yaw_in_base_deg)
        * (p_base - arm_mount.origin_in_base_mm)
```

The committed installation currently uses:

```yaml
arm_mount:
  origin_in_base_mm: [0.0, 0.0, 0.0]
  yaw_in_base_deg: 0.0

localization:
  target_z_mm: -110.0
  calibration_approved: true
  target_z_approved: true
  command_calibration:
    model: bilinear_local_xy
    calibration_id: tissue-grasp-waypoint-20260819-v3

planner:
  grasp_x_offset_mm: 10.0
  grasp_y_offset_mm: -15.0
```

RPLIDAR C1 supplies planar position only. The current `-110 mm` is the approved
tissue-package task-plane command. Before planning, the main pipeline applies
the `tissue-grasp-waypoint-20260819-v3` global bilinear and local-residual X/Y
mapping. The planner then adds the validated `+10 mm` X and `-15 mm` Y offsets
once; both were accounted for during model fitting. Revoke the two approval
flags before experimenting with new geometry.
