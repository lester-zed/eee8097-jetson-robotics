# RoArm four-waypoint feedback retry V8

## Root cause

The original controller sent T=104 and immediately requested T=105. On this
firmware, T=105 may not respond while T=104 is still executing. The first
2-second `RoArmTimeoutError` escaped the waypoint loop, causing the outer error
handler to send T=123 and terminate before grasp, lift, and retreat.

## Fix

- Treat an individual `RoArmTimeoutError` as transient.
- Retry T=105 until the complete `waypoint_timeout_s` expires.
- Send T=123 only after final failure or explicit abort.
- Print waypoint progress as `1/4`, `2/4`, `3/4`, and `4/4`.
- Record feedback and the number of transient T=105 timeouts for each waypoint.
- Pass `uart_response_timeout_s` to `RoArmUart`; defaults remain compatible.

## Recommended YAML

```yaml
manual_arm_test:
  cartesian_speed: 0.20
  waypoint_timeout_s: 30.0
  feedback_poll_s: 0.50
  gripper_settle_s: 1.0
```

The V8 controller works even when the existing manual launcher does not yet
pass the optional `uart_response_timeout_s` or `initial_feedback_delay_s`
fields. Their controller defaults are 2.0 seconds and 0.75 seconds.
