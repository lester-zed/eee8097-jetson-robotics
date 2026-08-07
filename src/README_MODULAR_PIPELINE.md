# EEE8097 GitHub-baselined modular pipeline V5

This package was regenerated against GitHub `main` commit:

```text
d0536866a0eab21e149173b1ad636ac4d3d0403f
Merge pull request #10: update whole process pipeline with camera rplidar roarm
```

The Docker environment mounts only the host `src/` directory to `/workspace/src`.
All configuration, scripts, documents, tests and runtime code in this package are
therefore located below `src/`.

## Install from the host project root

```bash
unzip -o eee8097_modular_pipeline_v5_github_d053_manual_roarm_extract_into_project_root_2026-08-06.zip
chmod +x src/run_modular_pipeline.sh src/run_roarm_manual_test.sh src/test_modular_pipeline.sh
```

## Test inside Docker

```bash
cd /workspace/src
./test_modular_pipeline.sh
```

## Sensor pipeline

```bash
cd /workspace/src
./run_modular_pipeline.sh
```

The default YAML uses real Camera, real RPLIDAR and mock RoArm.

## Standalone RoArm tests

```bash
cd /workspace/src
./run_roarm_manual_test.sh validate
./run_roarm_manual_test.sh status
./run_roarm_manual_test.sh capture-current
./run_roarm_manual_test.sh plan
./run_roarm_manual_test.sh gripper-open
./run_roarm_manual_test.sh gripper-close
./run_roarm_manual_test.sh reset
./run_roarm_manual_test.sh execute
```

Real motion is disabled by default. Edit only:

```text
/workspace/src/configs/modular_pipeline.yaml
```

Read `docs/ROARM_MANUAL_TEST_AND_CALIBRATION.md` before enabling motion.
