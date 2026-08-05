from __future__ import annotations

from enum import Enum
from threading import Event, Lock, Thread
from typing import Any

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
    ) -> None:
        self.vision = vision
        self.range_sensor = range_sensor
        self.localizer = localizer
        self.planner = planner
        self.arm = arm
        self.detection_timeout_s = float(detection_timeout_s)
        self.allow_provisional_execution = bool(allow_provisional_execution)
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

    def start(self) -> bool:
        with self._state_lock:
            if self.is_running():
                return False
            if self._state in {TaskState.ERROR, TaskState.ABORTED}:
                return False
            self._clear_last_run()
            self._abort_event.clear()
            self._worker = Thread(target=self._run_task, name="modular-grasp-worker", daemon=True)
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
            "state": self._state.value,
            "running": self.is_running(),
            "health": list(self._health),
            "detection": self._last_detection.to_dict() if self._last_detection else None,
            "range": self._last_range.to_dict() if self._last_range else None,
            "target": self._last_target.to_dict() if self._last_target else None,
            "plan": self._last_plan.to_dict() if self._last_plan else None,
            "execution": self._last_execution.to_dict() if self._last_execution else None,
            "error": self._last_error,
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

            self._set_state(TaskState.DETECTING)
            detection = self.vision.detect_target(
                self._abort_event, timeout_seconds=self.detection_timeout_s
            )
            if detection is None:
                self._raise_if_aborted()
                raise RuntimeError("target was not detected before timeout")
            self._last_detection = detection
            self._raise_if_aborted()

            self._set_state(TaskState.RANGING)
            self._last_range = self.range_sensor.measure_target(detection, self._abort_event)
            self._raise_if_aborted()

            self._set_state(TaskState.LOCALIZING)
            self._last_target = self.localizer.localize(detection, self._last_range)
            self._raise_if_aborted()

            self._set_state(TaskState.PLANNING)
            self._last_plan = self.planner.plan(self._last_target)
            self._raise_if_aborted()

            if self._last_target.provisional and not self.allow_provisional_execution:
                raise PermissionError(
                    "provisional target coordinates cannot be executed in this mode"
                )

            self._set_state(TaskState.EXECUTING)
            self._last_execution = self.arm.execute_grasp(self._last_plan, self._abort_event)
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

    def _clear_last_run(self) -> None:
        self._last_detection = None
        self._last_range = None
        self._last_target = None
        self._last_plan = None
        self._last_execution = None
        self._last_error = None
        self._health = []

    def _set_state(self, state: TaskState) -> None:
        with self._state_lock:
            self._state = state

    def _raise_if_aborted(self) -> None:
        if self._abort_event.is_set():
            raise InterruptedError("task aborted by user")
