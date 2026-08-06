from __future__ import annotations

from typing import Protocol

from pipeline.types import Detection2D, RangeMeasurement, TargetPose


class TargetLocalizerPort(Protocol):
    """Fuse detection and range data into robot and arm coordinates."""

    def localize(
        self,
        detection: Detection2D,
        measurement: RangeMeasurement,
    ) -> TargetPose:
        ...
