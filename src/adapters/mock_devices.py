from __future__ import annotations

from threading import Event
import time
from typing import Any

from pipeline.types import (
    Detection2D,
    ExecutionResult,
    GraspPlan,
    HealthResult,
    RangeMeasurement,
)


class MockVisionAdapter:
    def __init__(
        self,
        *,
        target_label: str = "cup",
        frame_width: int = 1280,
        frame_height: int = 720,
    ) -> None:
        self.target_label = target_label
        self.frame_width = int(frame_width)
        self.frame_height = int(frame_height)

    def detect_target(
        self,
        abort_event: Event,
        timeout_seconds: float = 20.0,
    ) -> Detection2D | None:
        del timeout_seconds
        if abort_event.is_set():
            return None
        center_x = self.frame_width // 2
        center_y = self.frame_height // 2
        return Detection2D(
            label=self.target_label,
            confidence=0.99,
            center_x=center_x,
            center_y=center_y,
            bbox=(center_x - 60, center_y - 100, center_x + 60, center_y + 100),
            frame_width=self.frame_width,
            frame_height=self.frame_height,
            captured_at_s=time.monotonic(),
        )

    def health_check(self) -> HealthResult:
        return HealthResult("camera", True, "mock camera ready")

    def close(self) -> None:
        return None


class MockRangeSensorAdapter:
    def __init__(self, *, distance_mm: float = 450.0) -> None:
        self.distance_mm = float(distance_mm)

    def measure_target(
        self,
        detection: Detection2D,
        abort_event: Event,
    ) -> RangeMeasurement:
        if abort_event.is_set():
            raise InterruptedError("range measurement aborted")
        bearing = (detection.frame_width / 2.0 - detection.center_x) * 0.05
        return RangeMeasurement(
            distance_mm=self.distance_mm,
            camera_bearing_deg=bearing,
            lidar_bearing_deg=bearing % 360.0,
            base_bearing_deg=bearing,
            sample_count=20,
            median_quality=15.0,
            mad_mm=2.0,
            minimum_mm=self.distance_mm - 3.0,
            maximum_mm=self.distance_mm + 3.0,
            measured_at_s=time.monotonic(),
            metadata={"source": "mock"},
        )

    def health_check(self) -> HealthResult:
        return HealthResult("rplidar", True, "mock RPLIDAR ready")

    def close(self) -> None:
        return None


class MockArmAdapter:
    def __init__(self, *, step_delay_s: float = 0.05) -> None:
        self.step_delay_s = float(step_delay_s)
        self._moving = False
        self._last_plan: GraspPlan | None = None

    def execute_grasp(
        self,
        plan: GraspPlan,
        abort_event: Event,
    ) -> ExecutionResult:
        self._moving = True
        self._last_plan = plan
        completed: list[str] = []
        try:
            for waypoint in plan.waypoints:
                if abort_event.is_set():
                    raise InterruptedError("mock arm execution aborted")
                time.sleep(self.step_delay_s)
                completed.append(waypoint.name)
            return ExecutionResult(
                success=True,
                message="mock grasp completed; no hardware command was sent",
                completed_waypoints=tuple(completed),
                feedback={"mode": "mock"},
            )
        finally:
            self._moving = False

    def health_check(self) -> HealthResult:
        return HealthResult("roarm", True, "mock RoArm ready")

    def status(self) -> dict[str, Any]:
        return {
            "mode": "mock",
            "moving": self._moving,
            "has_last_plan": self._last_plan is not None,
        }

    def stop(self) -> None:
        self._moving = False

    def close(self) -> None:
        self._moving = False
