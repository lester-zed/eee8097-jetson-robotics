# EEE8097 模块化抓取流程骨架

本交付包用于把现有仓库逐步整理为：

```text
main.py
  └── TaskManager
      ├── Camera / YOLO
      ├── RPLIDAR
      ├── TargetLocalizer
      ├── GraspPlanner
      └── RoArm
```

## 安全说明

该骨架不会覆盖现有 `src/main.py`、`src/task/task_manager.py`、
`src/vision/yolo_camera.py`、`src/lidar/rplidar_distance.py` 或
`src/arm_control/arm_controller.py`。

新增入口为：

```text
src/main_modular.py
```

默认使用 Mock Camera、Mock LiDAR 和 Mock Arm，可以验证完整状态机，但不会打开
真实硬件，也不会发送机械臂运动命令。

## 推荐安装方式

先解压到临时目录并检查：

```bash
unzip eee8097_modular_pipeline_scaffold_2026-08-05.zip
cd eee8097_modular_pipeline_scaffold_2026-08-05
```

确认后，将 `src/` 中的新增文件复制到仓库现有 `src/`：

```bash
cp -r src/* <repository>/src/
```

因为本包只使用新文件名，正常情况下不会覆盖现有文件。复制前仍建议执行：

```bash
find src -type f | sort
```

## Mock 全流程验证

在仓库容器内：

```bash
cd /workspace/src
python3 main_modular.py
```

然后输入：

```text
s
```

程序将执行：

```text
PRECHECK
→ DETECTING
→ RANGING
→ LOCALIZING
→ PLANNING
→ EXECUTING
→ VERIFYING
→ COMPLETE
```

Mock Arm 只打印规划，不移动机械臂。

## 真实设备接入状态

- Camera：已有 adapter，可复用当前 `vision.yolo_camera.YoloCamera`。
- RPLIDAR：已有 adapter，可复用当前 `lidar.rplidar_distance.RPLidarSdkReader`。
- 坐标转换：已有 adapter，可复用当前 `arm_control.coordinate_frames.MountTransform`。
- RoArm：现有 `ArmController` 仍是区域瞄准控制；笛卡尔抓取尚未接入，真实抓取会被明确拒绝。
- `main_modular.py` 暂不提供真实 Arm 开关，避免未完成标定时误动作。

详细说明见：

```text
docs/EEE8097_MODULAR_PIPELINE_ARCHITECTURE.md
```
