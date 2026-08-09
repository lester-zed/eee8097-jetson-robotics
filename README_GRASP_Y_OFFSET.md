# Grasp Y Offset V1

This patch adds a planner-level lateral grasp compensation without altering
Camera/LiDAR calibration or the localized object pose.

Latest real run localized the tissue pack at approximately:

    target_arm_base = [-387.0, -6.34, -100] mm

The physical test showed that the gripper did not move far enough laterally to
fully envelop the tissue package. The new parameter is:

    planner.grasp_y_offset_mm

Recommended next real test:

    grasp_y_offset_mm: -15.0

For the same measured target, the commanded grasp becomes approximately:

    [-387.0, -21.34, -100] mm

The compensation is used to derive pregrasp, grasp, lift and retreat, so the
approach path remains internally consistent. The original measured target is
still preserved in `plan.target`, and the planner metadata records both the
measured and commanded grasp pose.

If your required physical correction is toward +Y rather than -Y, reverse the
sign.
