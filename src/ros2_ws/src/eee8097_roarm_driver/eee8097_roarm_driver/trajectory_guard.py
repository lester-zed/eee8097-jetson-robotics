from __future__ import annotations

import math
from typing import Any, Sequence

from .joint_mapping import (
    ARM_JOINT_NAMES,
    GRIPPER_JOINT_NAME,
    URDF_ARM_LIMITS_RAD,
    reorder_arm_positions,
)


class TrajectoryValidationError(ValueError):
    """Raised when a MoveIt trajectory is unsafe for the arm-only bridge."""


def duration_seconds(duration: Any) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def validate_arm_trajectory(joint_names: Sequence[str], points: Sequence[Any]) -> None:
    """Reject malformed, out-of-range, or gripper-bearing trajectories."""
    names = tuple(str(name) for name in joint_names)
    if len(set(names)) != len(names):
        raise TrajectoryValidationError("trajectory contains duplicate joint names")
    if GRIPPER_JOINT_NAME in names:
        raise TrajectoryValidationError(
            "gripper joint commands are disabled because the gripper servo is unavailable"
        )
    if set(names) != set(ARM_JOINT_NAMES):
        raise TrajectoryValidationError(
            f"expected exactly {ARM_JOINT_NAMES}, received {names}"
        )
    if not points:
        raise TrajectoryValidationError("trajectory contains no points")

    previous_time = -1.0
    for index, point in enumerate(points):
        if len(point.positions) != len(names):
            raise TrajectoryValidationError(
                f"point {index} has {len(point.positions)} positions for {len(names)} joints"
            )
        ordered = reorder_arm_positions(names, point.positions)
        for name, value in zip(ARM_JOINT_NAMES, ordered):
            lower, upper = URDF_ARM_LIMITS_RAD[name]
            if not lower <= value <= upper:
                raise TrajectoryValidationError(
                    f"point {index}: {name}={value:.5f} outside "
                    f"[{lower:.5f}, {upper:.5f}]"
                )
        current_time = duration_seconds(point.time_from_start)
        if not math.isfinite(current_time) or current_time < 0.0:
            raise TrajectoryValidationError(f"point {index} has invalid time_from_start")
        if current_time <= previous_time:
            raise TrajectoryValidationError("time_from_start must increase strictly")
        previous_time = current_time


def command_speed_degrees_s(
    previous_rad: Sequence[float],
    target_rad: Sequence[float],
    delta_time_s: float,
    *,
    maximum: int,
) -> int:
    """Derive a conservative firmware speed from one trajectory segment."""
    if maximum < 1:
        raise ValueError("maximum speed must be positive")
    if len(previous_rad) != 3 or len(target_rad) != 3:
        raise ValueError("speed calculation requires three arm joints")
    if delta_time_s <= 0.0:
        return min(5, maximum)
    peak = max(
        abs(math.degrees(float(target) - float(previous))) / delta_time_s
        for previous, target in zip(previous_rad, target_rad)
    )
    return max(1, min(maximum, int(math.ceil(peak))))
