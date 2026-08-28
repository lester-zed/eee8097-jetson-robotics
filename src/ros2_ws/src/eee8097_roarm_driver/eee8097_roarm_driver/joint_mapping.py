from __future__ import annotations

import math
from typing import Final, Mapping, Sequence

ARM_JOINT_NAMES: Final[tuple[str, str, str]] = (
    "base_link_to_link1",
    "link1_to_link2",
    "link2_to_link3",
)
GRIPPER_JOINT_NAME: Final[str] = "link3_to_gripper_link"
PUBLISHED_JOINT_NAMES: Final[tuple[str, str, str, str]] = (
    *ARM_JOINT_NAMES,
    GRIPPER_JOINT_NAME,
)

URDF_ARM_LIMITS_RAD: Final[Mapping[str, tuple[float, float]]] = {
    "base_link_to_link1": (-math.pi, math.pi),
    "link1_to_link2": (-math.pi / 2.0, math.pi / 2.0),
    # The firmware-side elbow command accepts 0..180 degrees.  The lower
    # bound is therefore stricter than the vendor mesh URDF.
    "link2_to_link3": (0.0, 2.95),
}


def hardware_to_urdf_positions(
    *,
    base_rad: float,
    shoulder_rad: float,
    elbow_rad: float,
    gripper_collision_position_rad: float,
) -> tuple[float, float, float, float]:
    """Map T=1051 feedback to the vendor URDF convention.

    The gripper value is deliberately supplied by configuration.  The failed
    gripper servo is represented by a fixed collision posture and is never
    used as a command target.
    """
    values = (
        base_rad,
        shoulder_rad,
        elbow_rad,
        gripper_collision_position_rad,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("RoArm joint positions must be finite radians")
    gripper = float(gripper_collision_position_rad)
    if not 0.0 <= gripper <= 1.5:
        raise ValueError("gripper_collision_position_rad must be in [0.0, 1.5]")
    return (
        -float(base_rad),
        -float(shoulder_rad),
        float(elbow_rad),
        gripper,
    )


def reorder_arm_positions(
    joint_names: Sequence[str], positions: Sequence[float]
) -> tuple[float, float, float]:
    """Return one trajectory point in canonical arm-joint order."""
    if len(joint_names) != len(positions):
        raise ValueError("joint_names and positions have different lengths")
    lookup = dict(zip(joint_names, positions))
    try:
        ordered = tuple(float(lookup[name]) for name in ARM_JOINT_NAMES)
    except KeyError as exc:
        raise ValueError(f"missing arm joint: {exc.args[0]}") from exc
    if not all(math.isfinite(value) for value in ordered):
        raise ValueError("trajectory positions must be finite")
    return ordered  # type: ignore[return-value]


def urdf_to_hardware_degrees(
    positions_rad: Sequence[float],
) -> tuple[float, float, float]:
    """Convert the three MoveIt arm joints to T=121 hardware degrees."""
    if len(positions_rad) != 3:
        raise ValueError("exactly three arm joint positions are required")
    base, shoulder, elbow = (float(value) for value in positions_rad)
    for name, value in zip(ARM_JOINT_NAMES, (base, shoulder, elbow)):
        if not math.isfinite(value):
            raise ValueError(f"{name} is not finite")
        lower, upper = URDF_ARM_LIMITS_RAD[name]
        if not lower <= value <= upper:
            raise ValueError(f"{name}={value:.5f} outside [{lower:.5f}, {upper:.5f}]")
    return (
        -math.degrees(base),
        -math.degrees(shoulder),
        math.degrees(elbow),
    )
