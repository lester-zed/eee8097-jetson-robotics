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

The current geometry, workspace, distance thresholds, and command calibration
are hardware-tuned values. Do not alter them as part of an unrelated code
change. The former planner-only `-15 mm` Y compensation is now `0 mm`; its
effect is included in the measured two-dimensional command model and must not
be applied a second time.

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

Before the first real run, exercise the calibrated coordinates with real
Camera/RPLIDAR data and a Mock RoArm:

```bash
./run_modular_pipeline.sh configs/calibrated_plan_only.yaml
```

After `s` completes, inspect `t` and `g`. Near the measured P12 anchor, the
corrected target should be close to `[-380, -20, -100] mm`. The plan-only
profile never opens `/dev/ttyROARM` and skips startup home.

## Structured run records

Every terminal real-pipeline run appends JSONL and CSV under
`/workspace/logs/experiments/`. The record includes the merged configuration,
Git commit, sensor observations, recheck deltas, the raw `target_base_link`,
nominal and calibrated arm targets, four waypoints, execution result, duration,
and failure information.

For localization measurements without arm motion, use:

```bash
./run_calibration_capture.sh \
  --point-id P11 \
  --truth-x-mm <measured-X> \
  --truth-y-mm <measured-Y> \
  --repeat 3
```

This selects Real Camera + Real RPLIDAR + Mock Arm, skips startup home, and
never opens the RoArm serial port. See
`docs/CALIBRATION_AND_EXPERIMENT_LOGGING.md` for the 3 x 3 protocol.

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
- `docs/CALIBRATION_AND_EXPERIMENT_LOGGING.md`: no-motion capture, logs, and metrics.
- `docs/REPOSITORY_STRUCTURE.md`: authoritative modules and PR boundaries.
