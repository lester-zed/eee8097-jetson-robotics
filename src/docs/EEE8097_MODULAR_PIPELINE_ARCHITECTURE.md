# EEE8097 Camera–RPLIDAR–RoArm 模块化架构

## 1. 目标

将当前仓库整理为一个唯一主入口和一条明确的数据流水线：

```text
键盘命令
  ↓
main.py
  ↓
TaskManager
  ↓
Object Detection
  ↓
RPLIDAR Ranging
  ↓
Target Localization
  ↓
Coordinate Transform
  ↓
Grasp Planning
  ↓
RoArm Execution
```

`main.py` 只负责：

1. 解析命令行参数；
2. 创建各模块实例；
3. 将模块注入 TaskManager；
4. 响应 `s/x/p/r/q` 等键盘命令。

识别、测距、定位、规划和执行不能直接堆叠在 `main.py` 中。

---

## 2. 当前 GitHub 主干与本骨架的对应关系

当前主干已经具备以下实现：

| 当前文件 | 当前职责 | 本骨架如何复用 |
|---|---|---|
| `vision/yolo_camera.py` | YOLO 稳定目标识别 | `vision/yolo_camera_adapter.py` |
| `vision/types.py` | 当前二维检测结果 | Adapter 转成统一 `Detection2D` |
| `lidar/rplidar_distance.py` | 官方 SDK 扫描和扇区距离 | `lidar/range_sensor.py` |
| `arm_control/coordinate_frames.py` | Robot ↔ Arm yaw 转换 | `localization/transforms.py` |
| `arm_control/arm_controller.py` | Mock/Real Arm facade | `arm_control/arm_adapter.py` |
| `task/task_manager.py` | 当前 YOLO → region → mock grasp | 新 `pipeline/task_manager.py` |
| `main.py` | 当前键盘入口 | 先用 `main_modular.py` 验证，最终合并回 `main.py` |

本骨架没有复制 UART、RPLIDAR SDK 或 YOLO 实现，只新增业务接口和适配层。

---

## 3. 推荐最终目录

```text
src/
├── main.py
│
├── interfaces/
│   ├── vision.py
│   ├── range_sensor.py
│   ├── transform.py
│   ├── localizer.py
│   ├── planner.py
│   └── arm.py
│
├── pipeline/
│   ├── types.py
│   └── task_manager.py
│
├── adapters/
│   └── mock_devices.py
│
├── vision/
│   ├── yolo_camera.py
│   ├── types.py
│   └── yolo_camera_adapter.py
│
├── lidar/
│   ├── rplidar_distance.py
│   └── range_sensor.py
│
├── localization/
│   ├── target_localizer.py
│   └── transforms.py
│
├── planning/
│   └── grasp_planner.py
│
├── arm_control/
│   ├── roarm_uart.py
│   ├── coordinate_frames.py
│   ├── arm_controller.py
│   ├── real_roarm_controller.py
│   └── arm_adapter.py
│
└── tests/
    └── test_modular_pipeline_scaffold.py
```

---

## 4. 统一数据契约

### Detection2D
Camera 输出：类别、置信度、bbox、中心像素、图像尺寸和时间戳。

### RangeMeasurement
RPLIDAR 输出：距离、Camera bearing、LiDAR bearing、base bearing、采样质量和离散度。

### TargetPose
定位模块输出 `base_link` 和 `arm_base` 下的目标坐标，并明确是否仍为 provisional。

### GraspPlan
规划模块输出 `pregrasp / grasp / lift / retreat`，每个 waypoint 必须带 frame_id。

### ExecutionResult
机械臂输出成功状态、消息、已完成 waypoint 和反馈。

---

## 5. 状态机

```text
IDLE
  ↓
PRECHECK
  ↓
DETECTING
  ↓
RANGING
  ↓
LOCALIZING
  ↓
PLANNING
  ↓
EXECUTING
  ↓
VERIFYING
  ↓
COMPLETE
```

异常进入 `ERROR`，用户中止进入 `ABORTED`。TaskManager 使用 worker thread，保证主线程仍可响应 `x`。

---

## 6. Camera → RPLIDAR 对齐

V1 使用目标中心像素计算水平视线角：

```text
positive bearing = Camera 左侧
negative bearing = Camera 右侧
camera_bearing = atan2(cx - u, fx)
```

再转换：

```text
base_bearing = camera_yaw_in_base + camera_bearing
lidar_bearing = lidar_front_angle + base_bearing
```

必须标定：`camera_yaw_in_base_deg`、`lidar_front_angle_deg`、`camera_fx_px`、`camera_cx_px`。
V1 暂时忽略 Camera 与 LiDAR 的平移视差，近距离抓取时必须后续升级。

---

## 7. 三维定位

RPLIDAR C1 提供二维平面距离，V1 使用：

```text
X = range × cos(base_bearing)
Y = range × sin(base_bearing)
Z = configured target height
```

必须确认 LiDAR 扫描平面真实经过目标物体。若扫描平面低于桌面物体，LiDAR 不应作为目标深度真值。

随后：

```text
base_link target
  ↓ subtract arm origin
  ↓ MountTransform.robot_to_arm_xyz()
arm_base target
```

本骨架复用当前 `arm_control.coordinate_frames.MountTransform`。

---

## 8. 抓取规划

初版生成：

```text
pregrasp
抓取点 grasp
lift
retreat
```

规划模块只输出 waypoint，不得打开串口。当前只做几何包络检查，不代表已经完成逆运动学、碰撞检查和关节轨迹验证。

---

## 9. RoArm 接入边界

当前真实 `ArmController` 主要是 region 到 J1 的瞄准，不是笛卡尔抓取控制器。
因此本骨架在 Real 模式收到 `GraspPlan` 时会明确拒绝执行。

后续应新增：

```text
arm_control/cartesian_roarm_controller.py
```

负责：T=104、T=105、T=106、abort、安全限位、一次抓取保护和标定批准。

---

## 10. 键盘命令

```text
s  启动一次完整流程
x  中止当前任务
p  打印完整状态
d  打印最近检测
l  打印最近 LiDAR 测量
t  打印最近目标坐标
g  打印最近抓取计划
a  打印 Arm 状态
r  重置到 IDLE
h  显示帮助
q  中止并退出
```

---

## 11. 模块补全顺序

1. `interfaces/*` 与 `pipeline/types.py`：稳定输入输出。
2. `vision/yolo_camera_adapter.py`：统一 Camera 接口。
3. `lidar/range_sensor.py`：Camera-guided RPLIDAR 扇区测距。
4. `localization/*`：标定、坐标融合和转换。
5. `planning/grasp_planner.py`：工作空间、IK、碰撞和抓取参数。
6. `arm_control/arm_adapter.py`：接入新的 Cartesian RoArm backend。
7. 将 `pipeline/task_manager.py` 与 `main_modular.py` 合并回正式 `task/task_manager.py` 和 `main.py`。

---

## 12. 建议 Git 分支

```text
refactor/add-pipeline-interfaces
feat/camera-adapter
feat/camera-guided-rplidar
feat/target-localization
feat/grasp-planner
feat/roarm-cartesian-executor
feat/unified-main-pipeline
```
