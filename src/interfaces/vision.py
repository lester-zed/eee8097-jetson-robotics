from __future__ import annotations

from threading import Event
from typing import Protocol

from pipeline.types import Detection2D, HealthResult


class VisionPort(Protocol):
    """Object-detection interface consumed by TaskManager."""

    def detect_target(
        self,
        abort_event: Event,
        timeout_seconds: float = 20.0,
    ) -> Detection2D | None:
        ...

    def health_check(self) -> HealthResult:
        ...

    def close(self) -> None:
        ...
