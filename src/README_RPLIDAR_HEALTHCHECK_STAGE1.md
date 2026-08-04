# EEE8097：RPLIDAR C1 测距与开机健康检查

本增量包基于仓库 `lester-zed/eee8097-jetson-robotics` 的 `main` 分支，核对基线为：

```text
b7a6f2ecc2ada2eaa53ef0e4909c676dbf647b8e
2026-08-03 22:16 UTC
```

所有代码都是 `src/` 下的新增路径，不覆盖现有 `main.py`、`main_roarm.py`、
`TaskManager`、YOLO 或 RoArm 控制文件。

将 ZIP 放到 Jetson 宿主机项目的 `src/` 目录并使用不覆盖模式解压：

```bash
cd <项目根目录>/src
unzip -n eee8097_rplidar_healthcheck_additive_2026-08-03.zip
```

`-n` 会拒绝覆盖任何同名文件；解压源码不需要重新 build Docker。

## 1. 新增文件

```text
src/
├── lidar/
│   ├── __init__.py
│   └── rplidar_distance.py
├── tests/
│   └── test_rplidar_distance.py
├── robot_healthcheck.py
├── run_robot_healthcheck.sh
├── setup_rplidar_sdk.sh
└── README_RPLIDAR_HEALTHCHECK_STAGE1.md
```

## 2. 当前 Compose 中需要修正的一行

最新仓库中的 RPLIDAR 映射误用了 `ROARM_DEVICE` 变量：

```yaml
- "${ROARM_DEVICE:-/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_c877520e295df0119ae253401045c30f-if00-port0}:/dev/ttyRPLIDAR"
+ "${RPLIDAR_DEVICE:-/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_c877520e295df0119ae253401045c30f-if00-port0}:/dev/ttyRPLIDAR"
```

如果你从未设置 `ROARM_DEVICE`，两个默认 by-id 仍可能暂时映射正确；一旦设置
了它，两个容器别名会指向同一设备。修正后只需重新创建容器，无需重新 build：

```bash
docker compose -f docker/docker-compose.yml up -d --force-recreate robot-dev
```

## 3. 一次性准备官方 RPLIDAR SDK

当前 Dockerfile 已包含 `git`、`make` 和 `g++`，因此不需要 rebuild 镜像。
进入现有容器后执行一次：

```bash
cd /workspace/src
chmod +x setup_rplidar_sdk.sh run_robot_healthcheck.sh
./setup_rplidar_sdk.sh
```

SDK 固定到包含 C1 支持的官方提交，并生成：

```text
/workspace/src/third_party/rplidar_sdk/output/Linux/Release/ultra_simple
```

`third_party/` 位于宿主机挂载的 `src/` 中，容器重建后仍保留。不要把整个
SDK 提交进自己的仓库；建议把 `src/third_party/` 加入 `.gitignore`。

## 4. RPLIDAR C1 测距

C1 必须显式使用 `460800` baud。读取 3 圈，在 LiDAR 坐标系 `0° ± 5°`
内过滤无效距离和低质量点，再取中位数：

```bash
cd /workspace/src
python -m lidar.rplidar_distance \
  --device /dev/ttyRPLIDAR \
  --baudrate 460800 \
  --scans 3 \
  --front-angle 0 \
  --half-width 5
```

JSON 输出：

```bash
python -m lidar.rplidar_distance \
  --device /dev/ttyRPLIDAR \
  --scans 3 \
  --front-angle 0 \
  --half-width 5 \
  --json
```

`front-angle=0` 只是初始值。应在机器人正前方放置纸箱，观察多个角度并记录
纸箱点簇的中心角，将它作为实际 `front-angle`。LiDAR 位于两个 Motor 中间时，
还要确认激光平面高于电机和车轮；固定的近距离回波通常意味着结构遮挡。

## 5. 每次开机的一键测试

容器已经运行时，在 Jetson 宿主机执行：

```bash
docker exec -it robot-dev \
  /workspace/src/run_robot_healthcheck.sh
```

或者在容器中：

```bash
cd /workspace/src
python robot_healthcheck.py
```

默认检查：

1. `/dev/ttyROARM` 与 `/dev/ttyRPLIDAR` 存在且不是同一个内核设备；
2. RoArm 仅发送 `T=105`，必须收到 `T=1051`，不会移动机械臂；
3. RPLIDAR 以 `460800` 读取两圈有效扫描，测试时雷达会短时旋转；
4. `/dev/video0` 必须返回 5 个有效 OpenCV 帧；
5. 报告写入 `/workspace/logs/robot_health_latest.json`。

若还要验证 YOLO 模型能完成一次推理：

```bash
docker exec -it robot-dev \
  /workspace/src/run_robot_healthcheck.sh --check-yolo
```

脚本返回码：`0` 表示全部通过，`1` 表示至少一项失败，因此可以直接用于后续
systemd 或启动脚本的门控条件。

## 6. 无硬件测试

```bash
cd /workspace/src
python -m unittest discover \
  -s tests \
  -p 'test_rplidar_distance.py' \
  -v
```

## 7. YOLO 到真实抓取的第一阶段逻辑

当前仓库已经完成：

```text
Camera → YOLO 稳定识别 → left/centre/right → RoArm Base 低速瞄准
```

这仍然不是完整抓取；现有真实控制器明确只允许 Base 动作。下一阶段建议保持
原文件不变，新增以下模块：

```text
src/
├── vision/                         # 已有
│   ├── yolo_camera.py
│   └── types.py
├── lidar/                          # 本包新增
│   └── rplidar_distance.py
├── fusion/                         # 下一步新增
│   ├── camera_lidar_calibration.py
│   └── target_localizer.py
├── planning/                       # 下一步新增
│   ├── grasp_plan.py
│   └── preset_grasp_planner.py
├── arm_control/                    # 已有；后续新增已标定抓取后端
├── task/
│   ├── task_manager.py             # 先保留
│   └── grasp_task_manager.py       # 下一步新增
├── robot_healthcheck.py            # 本包新增
└── main_grasp_stage1.py            # 下一步新增入口
```

运行顺序应为：

```text
健康检查 PASS
→ 键盘 s
→ YOLO 连续多帧得到唯一 cup bbox
→ 像素横坐标通过相机内参换成 bearing
→ Camera–LiDAR 外参把 bearing 换到 LiDAR 角度
→ C1 在该角度附近读取 3 圈中位距离
→ 生成并检查 pre-grasp / grasp / lift 计划
→ RoArm 低速执行 open → pre-grasp → grasp → close → lift
→ T=105 状态检查与抓取结果验证
```

关键限制：C1 是水平 2D LiDAR，只给平面角度和距离，不能单独给目标高度。
因此真实抓取前必须完成 Camera–LiDAR–RoArm 外参、桌面高度和 RoArm 工作空间
标定。第一版应使用少量人工验证的预设抓取位姿，并保留默认 dry-run；在这些
标定完成前，不应把 YOLO 像素和一条 LiDAR 距离直接发送成 RoArm `x/y/z`。
