# RoArm custom startup home

The stock T=100 pose does not match the machine-forward pose of the current
installation. `roarm_home.py` provides a configurable, feedback-verified T=122
home before the Camera/RPLIDAR pipeline starts.

## Configuration

Edit:

```text
/workspace/src/configs/roarm_home.yaml
```

The configured joint pose, speed, tolerance, safety locks, and optional pipeline
overrides are all kept in that file.

## Validation and execution

```bash
cd /workspace/src
./test_roarm_home.sh
./run_roarm_home.sh preview
./run_roarm_home.sh status
./run_roarm_home.sh home
```

Real home motion requires both motion interlocks and the exact confirmation
phrase. Keep the mechanism clear and retain access to the power switch.

`./run_modular_pipeline.sh` executes the custom home first when
`run_before_pipeline` is enabled. The full pipeline will not start if a required
home operation fails.

The launcher applies pipeline overrides through a temporary merged YAML file;
it does not rewrite `configs/modular_pipeline.yaml`.

