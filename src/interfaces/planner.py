from __future__ import annotations

from typing import Protocol

from pipeline.types import GraspPlan, TargetPose


class GraspPlannerPort(Protocol):
    """Create a safe grasp plan without controlling hardware."""

    def plan(self, target: TargetPose) -> GraspPlan:
        ...
