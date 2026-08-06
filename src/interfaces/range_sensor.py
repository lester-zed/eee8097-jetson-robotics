from __future__ import annotations

from threading import Event
from typing import Protocol

from pipeline.types import Detection2D, HealthResult, RangeMeasurement


class RangeSensorPort(Protocol):
    """Target-directed distance interface consumed by TaskManager."""

    def measure_target(
        self,
        detection: Detection2D,
        abort_event: Event,
    ) -> RangeMeasurement:
        ...

    def health_check(self) -> HealthResult:
        ...

    def close(self) -> None:
        ...
