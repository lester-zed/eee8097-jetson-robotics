# Modular Camera-RPLIDAR-RoArm pipeline

`main_modular.py` is the canonical application entry point. It composes the
Camera, RPLIDAR, localization, planner, and RoArm components through the
`interfaces/` contracts.

## Committed runtime profile

`configs/modular_pipeline.yaml` currently selects:

```text
Camera:  real
RPLIDAR: real
RoArm:   real
Target:  tissue_pack
```

The current geometry, workspace, distance thresholds, and grasp offsets are
hardware-tuned values. Do not alter them as part of an unrelated code change.

## No-hardware validation

```bash
cd /workspace/src
./test_modular_pipeline.sh
```

This validates the YAML, compiles the active modules, and runs the unit tests.
It does not open the Camera, RPLIDAR, or RoArm serial port.

To validate only the configuration:

```bash
python3 main_modular.py \
  --config configs/modular_pipeline.yaml \
  --validate-config
```

## Hardware startup order

```bash
cd /workspace/src

# 1. Read-only Camera/RPLIDAR/RoArm checks
./run_robot_healthcheck.sh

# 2. Optional operator-supervised J1-J4 movement check
./run_robot_startup_test.sh --motion --step-confirm --record-directions

# 3. Full pipeline; runs configured custom home first
./run_modular_pipeline.sh
```

Real RoArm mode requires the exact typed confirmation phrase before the serial
port is opened. The process remains limited to one grasp.

## Main controls

```text
s  start one detect -> range -> recheck -> localize -> plan -> execute task
x  request abort
p  print complete state
d  latest detection
l  latest LiDAR measurement
t  latest target pose
g  latest grasp plan
a  arm status
r  reset the software state to IDLE
q  abort and quit
```

## Documentation

- `docs/FULL_GRASP_PIPELINE.md`: data path, coordinate frames, and safeguards.
- `docs/ROARM_MANUAL_TEST_AND_CALIBRATION.md`: isolated arm validation.
- `docs/ROARM_HOME.md`: custom T=122 startup home.
- `docs/ROARM_JOINT_SELFTEST.md`: J1-J4 direction and return test.
- `docs/RPLIDAR_AND_HEALTHCHECK.md`: sensor setup and boot checks.
- `docs/REPOSITORY_STRUCTURE.md`: authoritative modules and PR boundaries.
