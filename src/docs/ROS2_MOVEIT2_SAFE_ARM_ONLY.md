# ROS 2 Humble + MoveIt 2: safe arm-only integration

This workspace implements the ROS 2 and MoveIt 2 scope used by the MSc project for **arm-only trajectory planning and execution** on the RoArm-M2-S. It is based on the Waveshare Humble workspace at commit `40dbd84b553695212fab713e8465f817ba95454d`.

## Delivered scope

| Project statement | Implemented evidence |
|---|---|
| RoArm serial control in ROS 2 Humble | Real T=105 feedback node and guarded `FollowJointTrajectory` action |
| URDF and TF | Official Waveshare M2 geometry, `robot_state_publisher`, `world -> base_link -> hand_tcp` |
| Inverse kinematics | Official RoArm M2 IKFast plugin used by MoveIt 2 |
| Collision detection | Vendor self-collision geometry plus a configurable support-table collision box |
| End-effector control | `/roarm/move_cartesian` plans an XYZ target for `hand_tcp` and can execute only when both safety locks are enabled |
| Current pose query | `/roarm/end_effector_pose` topic and `/roarm/get_end_effector_pose` service |
| Reusable startup and recovery | plan, observe and execute modes; diagnostics; cooperative stop; controlled UART reconnect |

This workspace intentionally exposes only the arm joints required for the final MoveIt 2 integration. The arm-only execution boundary is enforced at four levels:

1. the SRDF contains only the three-joint `hand` planning group;
2. `moveit_controllers.yaml` contains only `hand_controller`;
3. the action server rejects trajectories containing joints outside the arm execution set;
4. real execution sends only T=121 commands for hardware joints 1, 2 and 3.

The complete official robot geometry remains available in the URDF for visualization and collision modelling. The arm-only controller does not expose additional end-effector actuation interfaces.

## Why the official launch is not used directly

The Waveshare tutorial is the reference for the M2 URDF, joint names, IKFast plugin and MoveIt configuration. Its generic launch path manages the complete robot interface and also starts components that are not required by this project's arm-only execution scope.

`safe_moveit.launch.py` therefore reuses the official description and IK plugin while keeping the EEE8097 driver as the single owner of the real UART and exposing one guarded arm trajectory action.

## First build on the Jetson

Use the dedicated ROS 2 image. It is separate from the existing PyTorch/YOLO container, so no CUDA or vision dependencies are changed.

```bash
cd ~/project/eee8097-jetson-robotics
./scripts/run_ros2_docker.sh plan
```

The first build installs ROS 2 Humble, MoveIt 2 and RViz, then checks out only the required Waveshare packages at the pinned commit. Later starts reuse the image and rebuild only the local ROS packages.

If the command runs over SSH without a graphical display:

```bash
USE_RVIZ=false ./scripts/run_ros2_docker.sh plan
```

## Validation sequence

Run the following stages in order. Do not skip directly to real execution.

### 1. Static and ROS package tests

Inside `eee8097-ros2`:

```bash
cd /workspace/src/ros2_ws
python3 test_static.py
./build_ros2.sh
source install/setup.bash
colcon test --packages-select eee8097_roarm_driver
colcon test-result --verbose
```

Expected result: Python/XML checks pass, arm mapping tests pass, and trajectories containing joints outside the supported arm execution set are rejected.

### 2. Fake-state planning; no serial device and no motion

```bash
./run_safe_moveit.sh plan
```

In a second terminal:

```bash
docker exec -it eee8097-ros2 bash
cd /workspace/src/ros2_ws
source install/setup.bash

ros2 node list
ros2 topic echo /roarm/end_effector_pose --once
ros2 service call /roarm/get_end_effector_pose \
  eee8097_interfaces/srv/GetEndEffectorPose "{}"

ros2 service call /roarm/move_cartesian \
  eee8097_interfaces/srv/MoveCartesian \
  "{x: 0.20, y: 0.00, z: 0.15, execute: false, velocity_scaling: 0.10, acceleration_scaling: 0.10}"
```

The response should report `planned: true`, `executed: false`, and a non-zero trajectory point count. A target inside or below the support table should fail IK or collision-aware planning:

```bash
ros2 service call /roarm/move_cartesian \
  eee8097_interfaces/srv/MoveCartesian \
  "{x: 0.20, y: 0.00, z: -0.04, execute: false, velocity_scaling: 0.10, acceleration_scaling: 0.10}"
```

The exact reachable pose depends on the current joint state. If the first example is unreachable, select a collision-free target in RViz and retry the same XYZ through the service.

### 3. Real-state observation; UART read only

Stop the legacy pipeline and its container first. Only one process may own `/dev/ttyROARM`.

```bash
cd ~/project/eee8097-jetson-robotics
./scripts/run_ros2_docker.sh observe
```

In a second terminal:

```bash
docker exec -it eee8097-ros2 bash
source /workspace/src/ros2_ws/install/setup.bash
ros2 topic echo /joint_states --once
ros2 topic echo /diagnostics --once
ros2 action list -t
```

Moving the arm gently by hand with torque disabled should change the first three values in `/joint_states` and the RViz pose. No motion goal is accepted in observe mode.

### 4. Real arm-only execution

Clear the full swept volume and keep direct access to the power switch. Start:

```bash
./scripts/run_ros2_docker.sh execute
```

The launcher requires the exact typed phrase `ENABLE ARM-ONLY MOTION`. First send the intended target with `execute: false`. Check the RViz path and table clearance. Then repeat the same reviewed XYZ with `execute: true`.

Use actual reviewed coordinates; do not copy an arbitrary pose into the real execution command. Start near the current `/roarm/end_effector_pose`, with a small displacement, and keep velocity and acceleration scaling at `0.10`.

## Arm-only interface checks

The MoveIt workspace is expected to expose only the arm execution interface. You can verify that no additional end-effector controller or action has been started:

```bash
ros2 action list -t | grep -i gripper && echo "UNEXPECTED" || echo "No extra end-effector action"
ros2 service list | grep -i gripper && echo "UNEXPECTED" || echo "No extra end-effector service"
grep -R "gripper_controller\|/gripper_cmd\|T.*106" \
  src/eee8097_roarm_driver src/eee8097_moveit_config/config/moveit_controllers.yaml
```

The first two commands should find nothing. The source search may find test strings or safety comments, but it must not find an active publisher, subscriber, controller entry, or UART send path for an out-of-scope joint.

## Fault recovery

The driver retries state reads and reopens the UART after repeated failures. For an operator-requested recovery:

```bash
cd /workspace/src/ros2_ws
./recover_ros2_arm.sh
```

`/roarm/stop` sends firmware command T=123. It is only a cooperative software stop. It is not a certified emergency stop. Switch off RoArm power if motion is unexpected or communication is lost.

## Current engineering limits

- MoveIt controls the arm only; mobile-base navigation is not part of this launch.
- The UART bridge converts sampled MoveIt trajectories into closely spaced T=121 commands. It is not a hard real-time controller.
- The default table box is a conservative starting model. Measure the real support surface and update `table.position` and `table.dimensions` before collision claims are used in experiments.
- The fixed end-effector collision posture is a modelling assumption. Inspect the real clearance before execution.
- Gazebo, domain randomisation and Sim2Real comparisons remain separate future work and are not presented as completed by this branch.
