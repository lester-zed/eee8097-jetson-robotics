# RoArm custom machine-forward home V6

## Purpose

The stock `T=100` pose uses base joint `b=0`, which points opposite the mobile
robot's forward direction on the current mechanical installation. V6 adds a
configurable `T=122` home and executes it before the Camera/RPLIDAR pipeline.

## Files

- `configs/roarm_home.yaml`: pose, speed, safety locks and runtime overrides.
- `roarm_home.py`: status, preview and feedback-verified T=122 home.
- `launch_modular_pipeline.py`: home first, then launch `main_modular.py`.
- `run_roarm_home.sh`: manual home utility.
- `run_modular_pipeline.sh`: updated launcher.
- `tests/test_roarm_home.py`: no-hardware fake-UART tests.

## Default pose

```yaml
pose:
  base_deg: 180.0
  shoulder_deg: 0.0
  elbow_deg: 90.0
  hand_deg: null
```

`base_deg: 180` and `base_deg: -180` represent the same heading. Choose the sign
that gives the safer cable route and rotation direction on the actual platform.
`hand_deg: null` preserves the current T=105 clamp/wrist angle.

## Speeds

- T=122 home speed: 15 deg/s, increased from 5 deg/s.
- T=122 acceleration: 20 deg/s^2.
- Pipeline T=104 runtime override: 0.25, increased from 0.15.

The launcher writes a temporary merged YAML and does not modify the existing
`configs/modular_pipeline.yaml`.

## Commands

```bash
cd /workspace/src
./test_roarm_home.sh
./run_roarm_home.sh preview
./run_roarm_home.sh status
./run_roarm_home.sh home
./run_modular_pipeline.sh
```

Before real motion, change both interlocks in `configs/roarm_home.yaml`:

```yaml
allow_real_motion: true
confirm_clearance: true
```

The pipeline will not start if the required custom home fails.
