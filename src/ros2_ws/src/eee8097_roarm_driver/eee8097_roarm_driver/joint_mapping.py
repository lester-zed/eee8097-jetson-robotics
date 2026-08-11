from __future__ import annotations

import math
from typing import Final

JOINT_NAMES: Final[tuple[str, str, str, str]] = (
    "base_link_to_link1",
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_gripper_link",
)


def hardware_to_urdf_positions(
    *,
    base_rad: float,
    shoulder_rad: float,
    elbow_rad: float,
    hand_rad: float,
) -> tuple[float, float, float, float]:
    """Convert RoArm T=1051 radians to the official RoArm URDF convention."""
    values = (base_rad, shoulder_rad, elbow_rad, hand_rad)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("RoArm joint positions must be finite radians")
    return (
        -float(base_rad),
        -float(shoulder_rad),
        float(elbow_rad),
        math.pi - float(hand_rad),
    )
