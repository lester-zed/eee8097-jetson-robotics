from __future__ import annotations

from typing import Protocol

from pipeline.types import Point3D


class BaseToArmTransformPort(Protocol):
    """Convert a point from base_link into arm_base."""

    def base_to_arm(self, point: Point3D) -> Point3D:
        ...
