# V5.3 test compatibility fix

Fixes the three failures observed after enabling the live manual RoArm YAML:

- Accept both `feedback_initial_delay_s` and legacy `initial_feedback_delay_s`.
- Accept either spelling from YAML in manual and modular entry points.
- Unit tests no longer assume the operator's live YAML is still disabled or has null waypoints.

The active YAML is still validated separately by `test_modular_pipeline.sh` before unit tests.
No real hardware motion is performed by these tests.
