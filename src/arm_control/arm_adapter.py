from __future__ import annotations

from threading import Event
from typing import Any

from pipeline.types import ExecutionResult, GraspPlan, HealthResult


class ExistingArmControllerAdapter:
    """Compatibility adapter for the repository's existing mock controller."""

    def __init__(self, *, mode: str = "mock", **controller_kwargs: Any) -> None:
        from arm_control.arm_controller import ArmController
        self._controller = ArmController(mode=mode, **controller_kwargs)

    def execute_grasp(self, plan: GraspPlan, abort_event: Event) -> ExecutionResult:
        if self._controller.mode != "mock":
            raise NotImplementedError(
                "Use CartesianRoArmAdapter for real GraspPlan execution"
            )
        success = self._controller.execute_grasp(plan, abort_event)
        return ExecutionResult(
            success=bool(success),
            message="existing mock ArmController completed the dry-run",
            completed_waypoints=tuple(item.name for item in plan.waypoints),
            feedback={"mode": self._controller.mode},
        )

    def health_check(self) -> HealthResult:
        return HealthResult("roarm", True, "existing mock ArmController ready")

    def status(self) -> dict[str, Any]:
        return self._controller.status()

    def stop(self) -> None:
        self._controller.stop()

    def close(self) -> None:
        self._controller.close()


class CartesianRoArmAdapter:
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
