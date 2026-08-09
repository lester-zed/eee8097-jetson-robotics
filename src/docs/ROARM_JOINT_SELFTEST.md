# RoArm J1-J4 startup self-test

The startup self-test verifies each joint at low speed and returns it to the
measured starting position. It reuses the canonical `arm_control/roarm_uart.py`
implementation and does not send T=100.

## Commands

```bash
cd /workspace/src

# Device health only; no RoArm movement
./run_robot_startup_test.sh

# Recommended first direction check
./run_robot_startup_test.sh \
  --motion \
  --step-confirm \
  --record-directions

# Later operator-supervised startup check
./run_robot_startup_test.sh --motion
```

The sequence is:

```text
J1 +10 degrees -> return
J2  +8 degrees -> return
J3  +8 degrees -> return
J4  -8 degrees -> return
```

Each step checks target error, return error, and inactive-joint drift through
T=105 feedback. The report is written to:

```text
/workspace/logs/roarm_joint_selftest_latest.json
```

## Current frame convention

- Camera/robot frame: +X forward, +Y left, +Z up.
- J1 positive rotation is toward the Camera-left side.
- `mount_yaw` changes the absolute forward offset, not the J1 sign convention.

J4 firmware feedback up to 0.5 degrees above the 180-degree command limit is
treated as quantization slack and clamped before planning. The command limit
itself remains unchanged; larger feedback overshoot is rejected before motion.

Do not configure unattended `--motion --yes` startup. Automated boot services
should run the no-motion health check only.

