# eee8097_roarm_driver

Read-only ROS 2 Humble bridge for the real RoArm-M2-S used by the EEE8097 project.

## Scope

- Opens `/dev/ttyROARM` on the Jetson host.
- Reuses `src/arm_control/roarm_uart.py` and sends only T=105 state requests.
- Publishes the official RoArm URDF joint names on `/joint_states`.
- Does not subscribe to motion topics and does not send joint, Cartesian, or gripper commands.

## Build

```bash
source /opt/ros/humble/setup.bash
source ~/ros_vendor/roarm_ws_em0/install/setup.bash
cd ~/project/eee8097-jetson-robotics/src/ros2_ws
colcon build --symlink-install --packages-select eee8097_roarm_driver
source install/setup.bash
```

If pyserial is unavailable on the host:

```bash
sudo apt install python3-serial
```

## Read-only smoke test

Stop every process or container that currently owns `/dev/ttyROARM`, then run:

```bash
ros2 run eee8097_roarm_driver roarm_state_node --ros-args \
  -p serial_port:=/dev/ttyROARM \
  -p publish_rate_hz:=5.0
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/project/eee8097-jetson-robotics/src/ros2_ws/install/setup.bash
ros2 topic echo /joint_states --once
```

## Real RoArm -> RViz

Do not run Waveshare `display.launch.py` at the same time because it starts `joint_state_publisher_gui` and would create another `/joint_states` publisher.

Use:

```bash
source /opt/ros/humble/setup.bash
source ~/ros_vendor/roarm_ws_em0/install/setup.bash
source ~/project/eee8097-jetson-robotics/src/ros2_ws/install/setup.bash
ros2 launch eee8097_roarm_driver real_state_rviz.launch.py
```

This launch starts `robot_state_publisher`, the read-only T=105 bridge, and RViz; it does not start Waveshare `roarm_driver` and does not send real-motion commands.
