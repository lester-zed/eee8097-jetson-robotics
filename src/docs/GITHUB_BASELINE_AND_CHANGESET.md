# GitHub baseline and V5 changeset

## Baseline inspected

Repository:

```text
lester-zed/eee8097-jetson-robotics
```

Branch and commit:

```text
main
d0536866a0eab21e149173b1ad636ac4d3d0403f
```

Commit message:

```text
Merge pull request #10 from lester-zed/dev
update whole process pipeline with camera rplidar roarm
```

The baseline already contains the modular interfaces, mock adapters, Camera and
RPLIDAR adapters, localization, planner, pipeline state machine and
`main_modular.py`. It remains Mock-RoArm-only and has no YAML runtime loader,
T=104 Cartesian executor or standalone physical reset/grasp test.

## Existing files replaced by this package

1. `src/main_modular.py`
   - Loads runtime parameters from YAML.
   - Selects mock/real Camera, RPLIDAR and RoArm implementations.
   - Requires a typed phrase before opening the real RoArm port.

2. `src/lidar/range_sensor.py`
   - Corrects the temporary 640x480 principal point through YAML (`cx=320`).
   - Adds Camera and LiDAR XY origins in `base_link`.
   - Adds configurable LiDAR front angle and angle direction.
   - Performs translation-aware 2-D Camera-ray/LiDAR-point matching.

3. `src/localization/target_localizer.py`
   - Accepts an already projected `target_base_link` from the range adapter.
   - Retains configured target Z because RPLIDAR C1 is 2-D.

4. `src/localization/transforms.py`
   - Applies RoArm origin translation and mount yaw to convert
     `base_link -> arm_base`.

5. `src/planning/grasp_planner.py`
   - Generates pregrasp/retreat points along the target radial direction.
   - Fixes the previous negative-X approach-direction error.
   - Uses T=104-compatible tool angle in radians and Cartesian speed.

6. `src/pipeline/types.py`
   - Adds projected LiDAR/base points to range data.
   - Replaces `tool_pitch_deg` with explicit `tool_angle_rad`.

7. `src/pipeline/task_manager.py`
   - Blocks provisional coordinates when real execution is selected.

8. `src/arm_control/arm_adapter.py`
   - Keeps the existing mock adapter.
   - Adds an adapter for the new Cartesian RoArm controller.

## Files added by this package

- `src/configs/modular_pipeline.yaml`
- `src/configuration/__init__.py`
- `src/configuration/loader.py`
- `src/arm_control/cartesian_roarm_controller.py`
- `src/roarm_manual_test.py`
- `src/run_modular_pipeline.sh`
- `src/run_roarm_manual_test.sh`
- `src/test_modular_pipeline.sh`
- `src/tests/test_modular_pipeline_configured.py`
- `src/tests/test_roarm_manual_test.py`
- `src/README_MODULAR_PIPELINE.md`
- `src/docs/EEE8097_MODULAR_PIPELINE_ARCHITECTURE.md`
- `src/docs/ROARM_MANUAL_TEST_AND_CALIBRATION.md`
- `src/docs/GITHUB_BASELINE_AND_CHANGESET.md`
- `src/docs/PACKAGE_FILE_LIST.txt`
- `src/docs/V5_VALIDATION.txt`

## Existing low-level files intentionally not replaced

The package reuses the repository's current implementations and does not replace:

- `src/arm_control/roarm_uart.py`
- `src/arm_control/coordinate_frames.py`
- `src/arm_control/arm_controller.py`
- `src/arm_control/real_roarm_controller.py`
- `src/vision/yolo_camera.py`
- `src/vision/types.py`
- `src/lidar/rplidar_distance.py`
- `src/main.py`
- `src/task/task_manager.py`

The Cartesian controller uses the existing public `RoArmUart.send()` method to
send T=104, and existing `get_state()`, `set_gripper_radians()`,
`move_initial_position()` and `stop_continuous_motion()` methods.
