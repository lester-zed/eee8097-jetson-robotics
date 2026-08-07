# V5.4 Waypoint feedback diagnostics compatibility fix

This patch fixes the local retry test failure:

```text
KeyError: 'waypoints'
```

The T=104/T=105 retry logic was already succeeding, but `ExecutionResult.feedback`
only exposed the final RoArm state. The controller now preserves those existing
keys and additionally publishes per-waypoint retry diagnostics under:

```python
result.feedback["waypoints"][waypoint_name]
```

Each waypoint contains:

- `target`: x/y/z, EoAT angle, and speed sent with T=104
- `t105_attempts`: total T=105 state-query attempts
- `transient_t105_timeouts`: temporary T=105 timeouts tolerated while T=104 is busy
- `feedback_samples`: successful T=1051 feedback samples
- `final_error_mm`: final Cartesian Euclidean error
- `elapsed_s`: elapsed time for the waypoint

Existing top-level feedback keys (`x_mm`, `y_mm`, `z_mm`, `voltage_v`) are retained
for backward compatibility. No YAML configuration is changed by this patch.
