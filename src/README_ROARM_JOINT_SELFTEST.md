# RoArm J1-J4 开机基础运动自检

本增量包基于仓库 `lester-zed/eee8097-jetson-robotics` 的 `main` 提交
`4248c254e1d050555d77526327dea29d758ea70f`。只新增文件，不覆盖现有
`robot_healthcheck.py`、`run_robot_healthcheck.sh` 或 RoArm 控制层。

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
