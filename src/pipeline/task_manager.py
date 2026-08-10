from __future__ import annotations

from datetime import datetime, timezone
from enum import Enum
import math
from threading import Event, Lock, Thread
import time
from typing import Any, Callable
from uuid import uuid4

from interfaces.arm import ArmPort
from interfaces.localizer import TargetLocalizerPort
from interfaces.planner import GraspPlannerPort
from interfaces.range_sensor import RangeSensorPort
from interfaces.vision import VisionPort
from pipeline.types import Detection2D, ExecutionResult, GraspPlan, RangeMeasurement, TargetPose


class TaskState(str, Enum):
    IDLE = "IDLE"
    PRECHECK = "PRECHECK"
    DETECTING = "DETECTING"
    RANGING = "RANGING"
    LOCALIZING = "LOCALIZING"
    RECHECKING = "RECHECKING"
    PLANNING = "PLANNING"
    EXECUTING = "EXECUTING"
    VERIFYING = "VERIFYING"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"
    ERROR = "ERROR"


class ModularTaskManager:
    def __init__(
        self,
        *,
        vision: VisionPort,
        range_sensor: RangeSensorPort,
        localizer: TargetLocalizerPort,
        planner: GraspPlannerPort,
        arm: ArmPort,
        detection_timeout_s: float = 20.0,
        allow_provisional_execution: bool = True,
        pre_execute_recheck: bool = True,
        recheck_timeout_s: float = 8.0,
        max_recheck_center_shift_px: float = 60.0,
        max_recheck_target_shift_mm: float = 60.0,
        max_range_mad_mm: float = 60.0,
        max_range_span_mm: float = 250.0,
        on_run_complete: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self.vision = vision
        self.range_sensor = range_sensor
        self.localizer = localizer
        self.planner = planner
        self.arm = arm
        self.detection_timeout_s = float(detection_timeout_s)
        self.allow_provisional_execution = bool(allow_provisional_execution)
        self.pre_execute_recheck = bool(pre_execute_recheck)
        self.recheck_timeout_s = float(recheck_timeout_s)
        self.max_recheck_center_shift_px = float(max_recheck_center_shift_px)
        self.max_recheck_target_shift_mm = float(max_recheck_target_shift_mm)
        self.max_range_mad_mm = float(max_range_mad_mm)
        self.max_range_span_mm = float(max_range_span_mm)
        self._on_run_complete = on_run_complete

        self._state = TaskState.IDLE
        self._state_lock = Lock()
        self._abort_event = Event()
        self._worker: Thread | None = None
        self._last_detection: Detection2D | None = None
        self._last_range: RangeMeasurement | None = None
        self._last_target: TargetPose | None = None
        self._last_plan: GraspPlan | None = None
        self._last_execution: ExecutionResult | None = None
        self._last_error: str | None = None
        self._health: list[dict[str, Any]] = []
        self._verification: dict[str, Any] | None = None
        self._run_id: str | None = None
        self._run_started_at_utc: str | None = None
        self._run_started_monotonic_s: float | None = None
        self._run_finished_at_utc: str | None = None
        self._run_duration_s: float | None = None
        self._recording_error: str | None = None

    def start(self) -> bool:
        with self._state_lock:
            if self.is_running():
                return False
            if self._state in {TaskState.ERROR, TaskState.ABORTED}:
                return False
            self._clear_last_run()
            now = datetime.now(timezone.utc)
            self._run_id = (
                now.strftime("%Y%m%dT%H%M%S.%fZ")
                + f"-{uuid4().hex[:8]}"
            )
            self._run_started_at_utc = now.isoformat(timespec="milliseconds")
            self._run_started_monotonic_s = time.monotonic()
            self._abort_event.clear()
            self._worker = Thread(
                target=self._run_task,
                name="modular-grasp-worker",
                daemon=True,
            )
            self._worker.start()
            return True

    def abort(self) -> bool:
        if not self.is_running():
            return False
        self._abort_event.set()
        try:
            self.arm.stop()
        except Exception:
            pass
        return True

    def reset(self) -> bool:
        with self._state_lock:
            if self.is_running():
                return False
            self._abort_event.clear()
            self._clear_last_run()
            self._state = TaskState.IDLE
            return True

    def is_running(self) -> bool:
        return self._worker is not None and self._worker.is_alive()

    def status(self) -> dict[str, Any]:
        return {
            "run_id": self._run_id,
            "state": self._state.value,
            "running": self.is_running(),
            "timing": {
                "started_at_utc": self._run_started_at_utc,
                "finished_at_utc": self._run_finished_at_utc,
                "duration_s": self._run_duration_s,
            },
            "health": list(self._health),
            "detection": self._last_detection.to_dict() if self._last_detection else None,
            "range": self._last_range.to_dict() if self._last_range else None,
            "target": self._last_target.to_dict() if self._last_target else None,
            "verification": self._verification,
            "plan": self._last_plan.to_dict() if self._last_plan else None,
            "execution": self._last_execution.to_dict() if self._last_execution else None,
            "error": self._last_error,
            "recording_error": self._recording_error,
        }

    def shutdown(self) -> None:
        self.abort()
        if self._worker is not None:
            self._worker.join(timeout=5.0)
        for module in (self.vision, self.range_sensor, self.arm):
            try:
                module.close()
            except Exception:
                pass

    def _validate_range_quality(
        self,
        measurement: RangeMeasurement,
        *,
        stage: str,
    ) -> None:
        if measurement.distance_mm <= 0.0:
            raise RuntimeError(f"{stage} produced a non-positive range")

        if measurement.mad_mm > self.max_range_mad_mm:
            raise RuntimeError(
                f"{stage} range MAD too large: "
                f"{measurement.mad_mm:.1f} mm > {self.max_range_mad_mm:.1f} mm"
            )

        span = measurement.maximum_mm - measurement.minimum_mm
        if span > self.max_range_span_mm:
            raise RuntimeError(
                f"{stage} range span too large: "
                f"{span:.1f} mm > {self.max_range_span_mm:.1f} mm"
            )

    @staticmethod
    def _detection_shift_px(first: Detection2D, second: Detection2D) -> float:
        return math.hypot(
            float(second.center_x - first.center_x),
            float(second.center_y - first.center_y),
        )

    @staticmethod
    def _target_shift_mm(first: TargetPose, second: TargetPose) -> float:
        a = first.target_base_link
        b = second.target_base_link
        return math.sqrt(
            (b.x_mm - a.x_mm) ** 2
            + (b.y_mm - a.y_mm) ** 2
            + (b.z_mm - a.z_mm) ** 2
        )

    def _run_task(self) -> None:
        try:
            self._set_state(TaskState.PRECHECK)
            self._health = [
                self.vision.health_check().to_dict(),
                self.range_sensor.health_check().to_dict(),
                self.arm.health_check().to_dict(),
            ]
            failed = [item for item in self._health if not item["ok"]]
            if failed:
                raise RuntimeError(f"precheck failed: {failed}")
            self._raise_if_aborted()

            # Pass 1: Camera -> RPLIDAR -> localization.
            self._set_state(TaskState.DETECTING)
            detection = self.vision.detect_target(
                self._abort_event,
                timeout_seconds=self.detection_timeout_s,
            )
            if detection is None:
                self._raise_if_aborted()
                raise RuntimeError("target was not detected before timeout")
            self._last_detection = detection
            self._raise_if_aborted()

            self._set_state(TaskState.RANGING)
            measurement = self.range_sensor.measure_target(
                detection,
                self._abort_event,
            )
            self._validate_range_quality(
                measurement,
                stage="initial RPLIDAR measurement",
            )
            self._last_range = measurement
            self._raise_if_aborted()

            self._set_state(TaskState.LOCALIZING)
            target = self.localizer.localize(detection, measurement)
            self._last_target = target
            self._raise_if_aborted()

            # Pass 2: repeat Camera + RPLIDAR immediately before final planning.
            if self.pre_execute_recheck:
                initial_detection = detection
                initial_measurement = measurement
                initial_target = target

                self._set_state(TaskState.RECHECKING)
                verified_detection = self.vision.detect_target(
                    self._abort_event,
                    timeout_seconds=self.recheck_timeout_s,
                )
                if verified_detection is None:
                    self._raise_if_aborted()
                    raise RuntimeError(
                        "pre-execution Camera recheck did not find the target"
                    )
                if verified_detection.label != initial_detection.label:
                    raise RuntimeError(
                        "pre-execution target label changed: "
                        f"{initial_detection.label!r} -> {verified_detection.label!r}"
                    )

                center_shift = self._detection_shift_px(
                    initial_detection,
                    verified_detection,
                )
                if center_shift > self.max_recheck_center_shift_px:
                    raise RuntimeError(
                        "pre-execution Camera target moved too far: "
                        f"{center_shift:.1f}px > "
                        f"{self.max_recheck_center_shift_px:.1f}px"
                    )

                verified_measurement = self.range_sensor.measure_target(
                    verified_detection,
                    self._abort_event,
                )
                self._validate_range_quality(
                    verified_measurement,
                    stage="verification RPLIDAR measurement",
                )

                verified_target = self.localizer.localize(
                    verified_detection,
                    verified_measurement,
                )
                target_shift = self._target_shift_mm(
                    initial_target,
                    verified_target,
                )
                if target_shift > self.max_recheck_target_shift_mm:
                    raise RuntimeError(
                        "pre-execution fused target moved too far: "
                        f"{target_shift:.1f}mm > "
                        f"{self.max_recheck_target_shift_mm:.1f}mm"
                    )

                self._verification = {
                    "passed": True,
                    "center_shift_px": center_shift,
                    "range_shift_mm": abs(
                        verified_measurement.distance_mm
                        - initial_measurement.distance_mm
                    ),
                    "target_shift_mm": target_shift,
                    "initial_target_base_link": (
                        initial_target.target_base_link.to_dict()
                    ),
                    "verified_target_base_link": (
                        verified_target.target_base_link.to_dict()
                    ),
                }

                # Use the observation closest to execution as the authoritative target.
                detection = verified_detection
                measurement = verified_measurement
                target = verified_target
                self._last_detection = detection
                self._last_range = measurement
                self._last_target = target
                self._raise_if_aborted()

            self._set_state(TaskState.PLANNING)
            self._last_plan = self.planner.plan(self._last_target)
            self._raise_if_aborted()

            if self._last_target.provisional and not self.allow_provisional_execution:
                raise PermissionError(
                    "provisional target coordinates cannot be executed in this mode"
                )

            self._set_state(TaskState.EXECUTING)
            self._last_execution = self.arm.execute_grasp(
                self._last_plan,
                self._abort_event,
            )
            self._raise_if_aborted()

            self._set_state(TaskState.VERIFYING)
            if not self._last_execution.success:
                raise RuntimeError(self._last_execution.message)
            self._raise_if_aborted()
            self._set_state(TaskState.COMPLETE)

        except InterruptedError:
            self._set_state(TaskState.ABORTED)
        except Exception as exc:
            self._last_error = f"{type(exc).__name__}: {exc}"
            self._set_state(TaskState.ERROR)
        finally:
            self._run_finished_at_utc = datetime.now(timezone.utc).isoformat(
                timespec="milliseconds"
            )
            if self._run_started_monotonic_s is not None:
                self._run_duration_s = max(
                    0.0,
                    time.monotonic() - self._run_started_monotonic_s,
                )
            if self._on_run_complete is not None:
                snapshot = self.status()
                # The callback runs at the end of this worker, so the durable
                # record should describe the terminal state rather than the
                # still-unwinding Python thread.
                snapshot["running"] = False
                try:
                    self._on_run_complete(snapshot)
                except Exception as exc:
                    self._recording_error = f"{type(exc).__name__}: {exc}"

    def _clear_last_run(self) -> None:
        self._last_detection = None
        self._last_range = None
        self._last_target = None
        self._last_plan = None
        self._last_execution = None
        self._last_error = None
        self._health = []
        self._verification = None
        self._run_id = None
        self._run_started_at_utc = None
        self._run_started_monotonic_s = None
        self._run_finished_at_utc = None
        self._run_duration_s = None
        self._recording_error = None

    def _set_state(self, state: TaskState) -> None:
        with self._state_lock:
            self._state = state

    def _raise_if_aborted(self) -> None:
        if self._abort_event.is_set():
            raise InterruptedError("task aborted by user")
