# EEE8097 ROS 2 workspace

This workspace adds ROS 2 Humble and MoveIt 2 without changing the canonical
YOLO + RPLIDAR + Cartesian RoArm pipeline.

Packages:

- `eee8097_interfaces`: Cartesian planning and TCP-pose services.
- `eee8097_roarm_driver`: real T=105 state publisher and locked J1-J3
  `FollowJointTrajectory` action.
- `eee8097_moveit_bridge`: current-pose topic/service, XYZ planning service and
  support-table collision object.
- `eee8097_moveit_config`: official M2 description/IKFast integration with an
  arm-only SRDF and controller list.

Start with:

```bash
python3 test_static.py
./build_ros2.sh
./run_safe_moveit.sh plan
```

Do not start `observe` or `execute` while any legacy process owns
`/dev/ttyROARM`. Read
[`../docs/ROS2_MOVEIT2_SAFE_ARM_ONLY.md`](../docs/ROS2_MOVEIT2_SAFE_ARM_ONLY.md)
before using real hardware.
