# RoArm 反向安装：纯新增代码包

本包专门适配“Camera 朝车头、RoArm 正方向朝车尾”的 180° 反向安装。

它是 **src-only、additive-only** 包：不会包含或覆盖你原有的 `main.py`、
`arm_control/arm_controller.py`、`task/task_manager.py`、YOLO、配置文件或 Docker
文件。原程序仍按原入口运行；只有主动运行新增入口时，才会使用本包。

## 新增文件

```text
src/
├── main_roarm_reverse.py
├── README_ROARM_REVERSE_ADDITIVE.md
├── arm_control/
│   └── roarm_reverse/
│       ├── __init__.py
│       ├── cli.py
│       ├── controller.py
│       ├── transform.py
│       └── uart.py
└── tests/
    └── test_roarm_reverse_addon.py
```

## 在 Jetson 宿主机安装

ZIP 应放在宿主机项目的 `src/` 中。进入该目录后使用 `unzip -n`，其中 `-n`
代表永不覆盖已存在的文件：

```bash
cd <项目根目录>/src
unzip -n eee8097_roarm_src_additive_only_2026-08-03.zip
```

这只是更新被 Docker volume 挂载的源码，不需要重新 build 镜像。

## 在容器内测试

```bash
cd /workspace/src

# 1. 无硬件：检查 180° 映射
python -m arm_control.roarm_reverse.cli map

# 2. 无硬件：运行测试
python -m unittest discover \
  -s tests \
  -p 'test_roarm_reverse_addon.py' \
  -v

# 3. 只读取 T=105 状态，不运动
python -m arm_control.roarm_reverse.cli status \
  --port /dev/ttyROARM
```

如果容器内串口没有重命名，则把 `/dev/ttyROARM` 改为实际的
`/dev/ttyUSB0`。

## 第一次低速真机方向测试

机械臂反向安装后，从 Base 0° 朝 Camera 方向转动可能接近 180°。先整理线缆并
清空完整扫掠区域，再只测试一个区域：

```bash
python -m arm_control.roarm_reverse.cli aim \
  --region left \
  --port /dev/ttyROARM \
  --speed 10 \
  --acceleration 10 \
  --execute \
  --confirm-clearance
```

该命令只发送 `T=121` 控制 Base，不移动 Shoulder、Elbow，也不执行未经标定
的笛卡尔坐标抓取。

## 运行现有 YOLO + 新增 RoArm 后端

真实 Camera + 原有 Mock Arm：

```bash
python main_roarm_reverse.py --ssh
```

真实 Camera + 真实 RoArm Base 瞄准：

```bash
python main_roarm_reverse.py \
  --ssh \
  --real-arm \
  --arm-port /dev/ttyROARM \
  --confirm-motion \
  --confirm-clearance \
  --speed 10 \
  --acceleration 10
```

反向安装默认映射：

```text
Camera left   -> Base -160°
Camera center -> Base +180°
Camera right  -> Base +160°
```

为避免 `-180°/+180°` 边界导致未知长路径，每个进程只允许一次自动 Base
动作，并要求动作前反馈接近 0°。Camera–Arm 标定完成前，本包不会启用
`T=104` 连续 `x/y/z` 控制，也不会声称已经完成真实抓取。
