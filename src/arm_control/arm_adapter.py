from __future__ import annotations

from threading import Event
from typing import Any

from pipeline.types import ExecutionResult, GraspPlan, HealthResult


class CartesianRoArmAdapter:
    """Expose the feedback-verified Cartesian controller through ``ArmPort``."""

    def __init__(self, **controller_kwargs: Any) -> None:
        from arm_control.cartesian_roarm_controller import CartesianRoArmController
        self._controller = CartesianRoArmController(**controller_kwargs)

    def execute_grasp(self, plan: GraspPlan, abort_event: Event) -> ExecutionResult:
        return self._controller.execute_grasp(plan, abort_event)

    def health_check(self) -> HealthResult:
        return self._controller.health_check()

    def status(self) -> dict[str, Any]:
        return self._controller.status()

    def stop(self) -> None:
        self._controller.stop()

    def close(self) -> None:
        self._controller.close()
