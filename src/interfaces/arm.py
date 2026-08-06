from __future__ import annotations

from threading import Event
from typing import Any, Protocol

from pipeline.types import ExecutionResult, GraspPlan, HealthResult


class ArmPort(Protocol):
    """Mechanical-arm execution interface consumed by TaskManager."""

    def execute_grasp(
        self,
        plan: GraspPlan,
        abort_event: Event,
    ) -> ExecutionResult:
        ...

    def health_check(self) -> HealthResult:
        ...

    def status(self) -> dict[str, Any]:
        ...

    def stop(self) -> None:
        ...

    def close(self) -> None:
        ...
