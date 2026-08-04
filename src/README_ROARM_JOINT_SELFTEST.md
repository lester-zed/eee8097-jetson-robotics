# RoArm J1-J4 开机基础运动自检

本替换包基于仓库 `lester-zed/eee8097-jetson-robotics` 的 `main` 提交
`af0399c5455cac02aecc72121f90c1f6abb9feb4`。它保留 Camera 正前方判断和
J1 方向提示修正，并处理真实硬件上 J4 可能反馈 `180.1°` 的边界量化误差；
不会修改 `robot_healthcheck.py`、`run_robot_healthcheck.sh` 或 RoArm UART
控制层。

仓库同时保留了较早的 `arm_control/roarm_reverse/` 独立兼容包。该包的 UART
只实现 Base 单关节瞄准；本自检必须覆盖 J1～J4，因此统一复用当前
`arm_control/roarm_uart.py` 的 `move_single_joint_degrees()`，不会打开第二套
串口实现。

## 文件

```text
src/
├── arm_control/roarm_joint_selftest.py
├── run_robot_startup_test.sh
├── tests/test_roarm_joint_selftest.py
└── README_ROARM_JOINT_SELFTEST.md
```

## 使用

在 Docker 容器内：

```bash
cd /workspace/src

# 原有三设备健康检查，不移动 RoArm
bash run_robot_startup_test.sh

# 推荐的首次方向标定：逐关节确认并记录 Camera 视角方向
bash run_robot_startup_test.sh \
  --motion \
  --step-confirm \
  --record-directions

# 后续每次开机，在场确认运动范围安全后进行一键运动自检
bash run_robot_startup_test.sh --motion
```

`--motion` 模式先要求原有 RoArm、RPLIDAR、Camera 健康检查全部通过，随后：

```text
J1 +10° -> 返回
J2  +8° -> 返回
J3  +8° -> 返回
J4  -8° -> 返回
```

每一步都会读取 `T=1051` 检查到位误差、回位误差以及非活动关节漂移。默认速度
为 `5°/s`，不会发送 `T=100`。报告写入：

```text
/workspace/logs/roarm_joint_selftest_latest.json
```

默认 `mount_yaw=180°` 时，Camera 正前方对应 J1 `±180°`，而不是 `0°`。
例如当前反馈为 `-178.4°` 时，Camera 正前方误差只有 `1.6°`，自检会正常继续。
J1 正角度始终对应 Camera 左侧；`mount_yaw` 只改变绝对零点偏置，不会反转
J1 的正负方向。

## J4 边界反馈处理

RoArm 上电后可能返回 `J4=180.1°`，比协议上限 `180.0°` 多 `0.1°`。新版仅对
收到的反馈提供 `0.5°` 容差，并将其钳制到合法命令边界：

```text
原始反馈：180.1°
规划起点：180.0°
测试目标：172.0°
回位命令：180.0°
```

关节的正式命令限位仍为 `[45.0°, 180.0°]`，不会放宽到 `181°`。如果反馈高于
`180.5°`（例如 `180.6°`），脚本仍会在发送任何运动命令前报错。计划阶段和
实际逐关节执行阶段均使用相同的边界处理，因此不会出现“计划通过、执行失败”或
“计划阶段直接因 180.1° 中止”的情况。

只有在有物理防护、运动范围长期固定并已完成首次人工方向确认时，才使用：

```bash
bash run_robot_startup_test.sh --motion --yes
```

不要把无人值守的 `--motion --yes` 直接配置成系统开机服务。系统服务适合只运行
默认的无动作健康检查；运动自检应在人员在场时执行。

## 无硬件测试

```bash
cd /workspace/src
python -m unittest discover \
  -s tests \
  -p 'test_roarm_joint_selftest.py' \
  -v
```

当前版本共包含 14 个无硬件测试，其中覆盖：

- `mount_yaw=180°` 的 Camera 正前方与 J1 圆周误差；
- J1 正方向为 Camera 左侧；
- J4 `180.1°` 在计划阶段被安全钳制为 `180.0°`；
- J4 执行 `180.0° -> 172.0° -> 180.0°`；
- J4 `180.6°` 超过反馈容差时仍被拒绝。
