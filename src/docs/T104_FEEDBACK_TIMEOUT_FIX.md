# RoArm T=104 feedback-timeout fix (V5.2)

## Problem

`CMD_XYZT_GOAL_CTRL` (`T=104`) may keep the RoArm firmware busy while the
interpolated Cartesian move is running.  A `T=105` state request sent
immediately afterwards can therefore receive no `T=1051` reply within the
UART-level response timeout (2 s by default).

The previous controller treated that single `RoArmTimeoutError` as a complete
waypoint failure and aborted the grasp with `T=123`, even though the physical
motion could still be in progress.

## Fix

`CartesianRoArmController` now uses two timeout levels:

- `uart_response_timeout_s`: timeout for one individual `T=105 -> T=1051`
  request.  Default: 2.0 s.
- `waypoint_timeout_s`: overall deadline for a Cartesian waypoint.  Default:
  15.0 s.

After sending `T=104`, the controller waits `feedback_initial_delay_s`
(default 0.75 s) before the first feedback request.  If an individual `T=105`
request times out, the controller retries until the waypoint-level deadline.
A single 2-second feedback miss no longer aborts the entire grasp.

## Optional YAML tuning

No YAML change is required; defaults are built into the code.  To tune the
manual test, add these keys under `manual_arm_test` without replacing your
existing waypoint values:

```yaml
manual_arm_test:
  uart_response_timeout_s: 2.0
  feedback_initial_delay_s: 0.75
  feedback_poll_s: 0.20
  waypoint_timeout_s: 20.0
```

For the full real pipeline, the same optional keys can be placed under `arm`:

```yaml
arm:
  uart_response_timeout_s: 2.0
  feedback_initial_delay_s: 0.75
  feedback_poll_s: 0.20
  waypoint_timeout_s: 20.0
```

For slow Cartesian moves, increase `waypoint_timeout_s` first.  Increasing only
`uart_response_timeout_s` is usually less useful because `T=104` may be busy for
longer than one feedback request.
