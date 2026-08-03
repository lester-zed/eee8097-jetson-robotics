from enum import Enum
from threading import Event, Lock, Thread
from typing import Optional

from arm_control.arm_controller import ArmController
from utils.logger import get_logger
from vision.types import DetectionResult
from vision.yolo_camera import YoloCamera


class TaskState(str, Enum):
    IDLE = "IDLE"
    SEARCHING = "SEARCHING"
    PLANNING = "PLANNING"
    EXECUTING = "EXECUTING"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"
    ERROR = "ERROR"


class TaskManager:
    def __init__(
        self,
        vision: YoloCamera,
        arm: ArmController,
    ) -> None:
        self.logger = get_logger(self.__class__.__name__)
        self.vision = vision
        self.arm = arm

        self._state = TaskState.IDLE
        self._last_detection: Optional[DetectionResult] = None
        self._last_error: Optional[str] = None

        self._abort_event = Event()
        self._state_lock = Lock()
        self._worker: Optional[Thread] = None

    def start(self) -> bool:
        with self._state_lock:
            if self._worker is not None and self._worker.is_alive():
                self.logger.warning(
                    "A task is already running."
                )
                return False

            if self._state in {
                TaskState.ERROR,
                TaskState.ABORTED,
            }:
                self.logger.warning(
                    "Reset the task before starting again."
                )
                return False

            self._abort_event.clear()
            self._last_detection = None
            self._last_error = None
            self._state = TaskState.SEARCHING

            self._worker = Thread(
                target=self._run_task,
                name="grasp-worker",
                daemon=True,
            )
            self._worker.start()

        return True

    def abort(self) -> None:
        if not self.is_running():
            self.logger.info("No active task to abort.")
            return

        self.logger.warning("Abort requested.")
        self._abort_event.set()

    def reset(self) -> bool:
        with self._state_lock:
            if self._worker is not None and self._worker.is_alive():
                self.logger.warning(
                    "Cannot reset while a task is running."
                )
                return False

            self._abort_event.clear()
            self._last_detection = None
            self._last_error = None
            self._state = TaskState.IDLE

        self.logger.info("Task manager reset to IDLE.")
        return True

    def is_running(self) -> bool:
        return (
            self._worker is not None
            and self._worker.is_alive()
        )

    def status(self) -> dict:
        detection = None

        if self._last_detection is not None:
            detection = {
                "label": self._last_detection.label,
                "confidence": self._last_detection.confidence,
                "center": (
                    self._last_detection.center_x,
                    self._last_detection.center_y,
                ),
                "region": self._last_detection.region,
            }

        return {
            "state": self._state.value,
            "running": self.is_running(),
            "detection": detection,
            "error": self._last_error,
        }

    def shutdown(self) -> None:
        self.abort()

        if self._worker is not None:
            self._worker.join(timeout=5.0)

    def _set_state(self, state: TaskState) -> None:
        with self._state_lock:
            self._state = state

        self.logger.info("State changed to %s", state.value)

    def _run_task(self) -> None:
        try:
            detection = self.vision.detect_target(
                abort_event=self._abort_event,
                timeout_seconds=20.0,
            )

            if self._abort_event.is_set():
                self._set_state(TaskState.ABORTED)
                return

            if detection is None:
                raise RuntimeError(
                    "Target was not detected before timeout"
                )

            self._last_detection = detection
            self._set_state(TaskState.PLANNING)

            self.logger.info(
                "Selected preset grasp region: %s",
                detection.region,
            )

            if self._abort_event.is_set():
                self._set_state(TaskState.ABORTED)
                return

            self._set_state(TaskState.EXECUTING)

            success = self.arm.execute_mock_grasp(
                region=detection.region,
                abort_event=self._abort_event,
            )

            if not success or self._abort_event.is_set():
                self._set_state(TaskState.ABORTED)
                return

            self._set_state(TaskState.COMPLETE)
            self._set_state(TaskState.IDLE)

        except Exception as exc:
            self._last_error = str(exc)
            self.logger.exception("Task failed: %s", exc)
            self._set_state(TaskState.ERROR)