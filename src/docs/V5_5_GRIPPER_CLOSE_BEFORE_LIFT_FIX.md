# V5.5 — Wait for clamp travel before lift

Problem observed on real RoArm-M2-S:

- T=106 close command is sent.
- The old controller waits only `gripper_settle_s` (typically 1–1.5 s).
- Lift starts while the clamp is still physically closing.

Waveshare documents T=106 `spd` in servo steps/second and one servo revolution as 4096 steps.
Therefore the clamp travel time is approximately:

```text
servo_steps = abs(target_rad - start_rad) / (2*pi) * 4096
motion_s    = servo_steps / gripper_speed_steps_s
```

For stock clamp full-open 1.08 rad -> closed 3.14 rad:

- speed 100 steps/s: ~13.43 s
- speed 300 steps/s: ~4.48 s
- speed 400 steps/s: ~3.36 s
- speed 500 steps/s: ~2.69 s

## V5.5 behavior

Production entry points now pass an explicit `gripper_speed_steps_s`. After T=106, the controller waits:

```text
max(gripper_settle_s, estimated_motion_s + gripper_motion_margin_s)
```

After the close wait, it requires one fresh T=1051 response before sending the lift T=104 command. It records:

- post-close `hand_rad`
- raw `torH` if present
- estimated close motion time
- actual close wait time

It does NOT require `hand_rad == 3.14`, because an object held by the clamp can physically prevent the joint from reaching the no-load closed angle.

## Recommended manual test config

Add or adjust these keys under `manual_arm_test`:

```yaml
gripper_open_rad: 1.08
gripper_closed_rad: 3.14

gripper_speed_steps_s: 400
gripper_acceleration: 10

gripper_settle_s: 1.0
gripper_motion_margin_s: 0.75
```

With full travel 1.08 -> 3.14 and speed 400, the estimated motion is ~3.36 s and the controller waits ~4.11 s before requesting post-close feedback and then allowing lift.

The package intentionally does not overwrite `configs/modular_pipeline.yaml` because the hardware calibration and waypoints are user-specific.
