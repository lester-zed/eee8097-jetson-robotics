# Modular Pipeline V2 修改说明

## 配置驱动

所有常用启动参数已集中到 `src/configs/modular_pipeline.yaml`。`main_modular.py`
只保留 `--config` 和 `--validate-config`，后续设备路径、内参、外参、工作空间、
抓取高度、夹爪角度和执行安全开关均通过 YAML 修改。

## 坐标链

```text
YOLO pixel
  → Camera bearing
  → Camera ray in base_link
  → LiDAR points projected to base_link
  → translation-corrected ray selection
  → target_base_link X/Y + configured Z
  → subtract RoArm origin in base_link
  → rotate by -arm yaw
  → target_arm_base
  → radial grasp waypoints
  → RoArm T=104
```

## 真实执行安全门

真实执行需要：

1. `localization.calibration_approved=true`
2. `arm.mode=real`
3. `arm.allow_real_motion=true`
4. `arm.confirm_clearance=true`
5. 启动时输入确认口令
6. `TargetPose.provisional=false`
7. RoArm T=105 电压与反馈检查通过

## 仍需用户标定

- `camera.fx_px/fy_px/cx_px/cy_px`
- `camera.origin_in_base_mm`
- `camera.yaw_in_base_deg`
- `lidar.origin_in_base_mm`
- `lidar.front_angle_deg`
- `lidar.angle_sign`
- `arm_mount.origin_in_base_mm`
- `arm_mount.yaw_in_base_deg`
- `localization.target_z_mm`
- `planner.tool_angle_rad`
- `workspace.*`
