# EEE8097 Docker-mounted modular pipeline V3

This layout assumes Docker mounts only the project `src/` directory:

```text
host project/src  ->  /workspace/src
```

All runtime files are therefore kept inside `src/`:

```text
src/
├── main_modular.py
├── run_modular_pipeline.sh
├── test_modular_pipeline.sh
├── configs/modular_pipeline.yaml
├── docs/
├── configuration/
├── interfaces/
├── adapters/
├── pipeline/
├── vision/
├── lidar/
├── localization/
├── planning/
└── arm_control/
```

## Extract

Run from the host project root:

```bash
unzip -o eee8097_modular_pipeline_v3_docker_src_extract_into_project_root_2026-08-06.zip
chmod +x src/run_modular_pipeline.sh src/test_modular_pipeline.sh
```

The archive contains only a top-level `src/` directory. It does not require a
new Docker bind mount and does not overwrite the existing `src/main.py`.

## Run inside Docker

```bash
cd /workspace/src
./run_modular_pipeline.sh
```

Equivalent command:

```bash
cd /workspace/src
python3 main_modular.py --config configs/modular_pipeline.yaml
```

A custom config located under the mounted source tree can be supplied as:

```bash
./run_modular_pipeline.sh configs/my_pipeline.yaml
```

## Validate and test inside Docker

```bash
cd /workspace/src
./test_modular_pipeline.sh
```

## Configuration

Edit only:

```text
/workspace/src/configs/modular_pipeline.yaml
```

The default configuration keeps RoArm in mock mode. Real motion remains locked
until calibration approval and all real-motion confirmations are enabled.

## Important physical limitations

- The RPLIDAR scan plane must intersect the target.
- `localization.target_z_mm` remains a configured height, not a 3-D LiDAR result.
- Camera intrinsics and Camera/LiDAR/RoArm mounting parameters must be measured.
- T=104 firmware IK does not provide collision checking.
- Software stop is not a certified emergency stop.
