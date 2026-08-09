from __future__ import annotations

from pathlib import Path
from threading import Condition, Event, Thread, current_thread
from time import monotonic
from typing import Optional

import cv2
from ultralytics import YOLO

from utils.logger import get_logger
from vision.types import DetectionResult


class YoloCamera:
    """Persistent YOLO camera stream.

    The camera device, YOLO inference loop, and OpenCV preview are owned by one
    long-lived background thread for the lifetime of the pipeline process.

    ``detect_target()`` no longer opens/releases ``cv2.VideoCapture``.  It waits
    for a fresh stable detection produced by the persistent stream.  This lets
    the Camera remain live during RPLIDAR, planning, RoArm execution, reset, and
    subsequent pipeline runs.
    """

    def __init__(
        self,
        *,
        model_path: str | Path,
        camera_index: int = 0,
        target_label: str = "cup",
        confidence_threshold: float = 0.55,
        stable_frames: int = 3,
        stability_tolerance_px: int = 40,
        inference_imgsz: int = 640,
        device: int | str | None = None,
        show_preview: bool = True,
    ) -> None:
        self.logger = get_logger(self.__class__.__name__)
        resolved_model_path = Path(model_path).expanduser()
        if not resolved_model_path.is_absolute():
            resolved_model_path = resolved_model_path.resolve()
        if not resolved_model_path.is_file():
            raise FileNotFoundError(f"YOLO model was not found: {resolved_model_path}")

        self.logger.info("Loading YOLO model: %s", resolved_model_path)
        self.model_path = resolved_model_path
        self.model = YOLO(str(resolved_model_path))

        model_names = self.model.names
        if isinstance(model_names, dict):
            available_labels = {str(name) for name in model_names.values()}
        else:
            available_labels = {str(name) for name in model_names}
        if target_label not in available_labels:
            raise ValueError(
                f"Configured target label {target_label!r} is not present in "
                f"YOLO model {resolved_model_path}. "
                f"Available labels: {sorted(available_labels)}"
            )

        self.camera_index = int(camera_index)
        self.target_label = str(target_label)
        self.confidence_threshold = float(confidence_threshold)
        self.stable_frames = int(stable_frames)
        self.stability_tolerance_px = int(stability_tolerance_px)
        self.inference_imgsz = int(inference_imgsz)
        self.device = device
        self.show_preview = bool(show_preview)

        if self.stable_frames < 1:
            raise ValueError("stable_frames must be >= 1")
        if self.stability_tolerance_px < 1:
            raise ValueError("stability_tolerance_px must be >= 1")

        self._condition = Condition()
        self._stop_event = Event()
        self._thread: Thread | None = None

        self._ready = False
        self._last_error: str | None = None
        self._frame_seq = 0
        self._tracking_detection: DetectionResult | None = None
        self._stable_detection: DetectionResult | None = None
        self._stable_detection_seq = -1
        self._consecutive_detections = 0

        self.start()

    def start(self) -> None:
        """Start the persistent camera worker once."""
        with self._condition:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._ready = False
            self._last_error = None
            self._thread = Thread(
                target=self._stream_loop,
                name="persistent-yolo-camera",
                daemon=True,
            )
            self._thread.start()

    def _set_error(self, message: str) -> None:
        with self._condition:
            self._last_error = str(message)
            self._condition.notify_all()

    def _stream_loop(self) -> None:
        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            self._set_error(f"Could not open camera index {self.camera_index}")
            with self._condition:
                self._ready = False
                self._condition.notify_all()
            return

        self.logger.info(
            "Persistent Camera stream opened: index=%d target=%r model=%s",
            self.camera_index,
            self.target_label,
            self.model_path,
        )

        with self._condition:
            self._ready = True
            self._last_error = None
            self._condition.notify_all()

        try:
            while not self._stop_event.is_set():
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError("Camera returned an empty frame")

                kwargs = {
                    "source": frame,
                    "conf": self.confidence_threshold,
                    "imgsz": self.inference_imgsz,
                    "verbose": False,
                }
                if self.device is not None:
                    kwargs["device"] = self.device

                result = self.model.predict(**kwargs)[0]
                matching_detections: list[DetectionResult] = []
                height, width = frame.shape[:2]

                for box in result.boxes:
                    class_id = int(box.cls.item())
                    label = str(result.names[class_id])
                    confidence = float(box.conf.item())
                    if label != self.target_label:
                        continue

                    x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
                    matching_detections.append(
                        DetectionResult(
                            label=label,
                            confidence=confidence,
                            center_x=(x1 + x2) // 2,
                            center_y=(y1 + y2) // 2,
                            bbox=(x1, y1, x2, y2),
                            frame_width=width,
                            frame_height=height,
                        )
                    )

                with self._condition:
                    self._frame_seq += 1
                    frame_seq = self._frame_seq

                    if len(matching_detections) == 1:
                        current = matching_detections[0]
                        previous = self._tracking_detection
                        if (
                            previous is not None
                            and abs(current.center_x - previous.center_x)
                            < self.stability_tolerance_px
                            and abs(current.center_y - previous.center_y)
                            < self.stability_tolerance_px
                        ):
                            self._consecutive_detections += 1
                        else:
                            self._consecutive_detections = 1

                        self._tracking_detection = current
                        if self._consecutive_detections >= self.stable_frames:
                            self._stable_detection = current
                            self._stable_detection_seq = frame_seq
                    else:
                        if len(matching_detections) > 1:
                            self.logger.warning(
                                "Multiple '%s' targets detected; waiting for an "
                                "unambiguous target.",
                                self.target_label,
                            )
                        self._tracking_detection = None
                        self._consecutive_detections = 0

                    stable_count = self._consecutive_detections
                    self._condition.notify_all()

                if self.show_preview:
                    preview = result.plot()
                    cv2.putText(
                        preview,
                        (
                            f"Persistent Camera | target: {self.target_label} | "
                            f"stable: {stable_count}/{self.stable_frames}"
                        ),
                        (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.75,
                        (0, 255, 0),
                        2,
                    )
                    cv2.imshow("YOLO persistent grasp camera", preview)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        self.logger.info(
                            "Preview 'q' requested; stopping persistent Camera stream."
                        )
                        self._stop_event.set()
                        break

        except Exception as exc:
            self.logger.exception("Persistent Camera stream failed: %s", exc)
            self._set_error(f"{type(exc).__name__}: {exc}")
        finally:
            cap.release()
            if self.show_preview:
                try:
                    cv2.destroyWindow("YOLO persistent grasp camera")
                    cv2.waitKey(1)
                except cv2.error:
                    pass
            with self._condition:
                self._ready = False
                self._condition.notify_all()
            self.logger.info("Persistent Camera stream closed.")

    def detect_target(
        self,
        abort_event: Event,
        timeout_seconds: float = 20.0,
    ) -> Optional[DetectionResult]:
        """Wait for a fresh stable detection from the already-running stream."""
        deadline = monotonic() + float(timeout_seconds)

        with self._condition:
            request_after_seq = self._frame_seq

        self.logger.info(
            "Waiting for fresh stable target '%s' from persistent Camera stream...",
            self.target_label,
        )

        while monotonic() < deadline:
            if abort_event.is_set():
                self.logger.info("Target detection aborted.")
                return None

            with self._condition:
                if self._last_error is not None:
                    raise RuntimeError(
                        f"Persistent Camera stream error: {self._last_error}"
                    )

                if (
                    self._stable_detection is not None
                    and self._stable_detection_seq > request_after_seq
                ):
                    detection = self._stable_detection
                    self.logger.info(
                        "Stable target detected: label=%s confidence=%.2f "
                        "centre=(%d, %d) region=%s",
                        detection.label,
                        detection.confidence,
                        detection.center_x,
                        detection.center_y,
                        detection.region,
                    )
                    return detection

                if self._thread is None or not self._thread.is_alive():
                    raise RuntimeError("Persistent Camera stream is not running")

                remaining = max(0.0, deadline - monotonic())
                self._condition.wait(timeout=min(0.10, remaining))

        self.logger.warning(
            "Target detection timed out after %.1f seconds.", timeout_seconds
        )
        return None

    def health_snapshot(self, wait_seconds: float = 3.0) -> dict[str, object]:
        """Return a lightweight status snapshot; optionally wait for startup."""
        deadline = monotonic() + max(0.0, float(wait_seconds))
        with self._condition:
            while (
                not self._ready
                and self._last_error is None
                and self._thread is not None
                and self._thread.is_alive()
                and monotonic() < deadline
            ):
                self._condition.wait(timeout=min(0.10, deadline - monotonic()))

            return {
                "ready": self._ready,
                "running": bool(self._thread and self._thread.is_alive()),
                "frame_seq": self._frame_seq,
                "stable_detection_available": self._stable_detection is not None,
                "last_error": self._last_error,
                "camera_index": self.camera_index,
                "show_preview": self.show_preview,
            }

    def close(self) -> None:
        """Stop the persistent stream and release the Camera on its owner thread."""
        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()

        thread = self._thread
        if (
            thread is not None
            and thread.is_alive()
            and thread is not current_thread()
        ):
            thread.join(timeout=5.0)

        if thread is not None and thread.is_alive():
            self.logger.warning(
                "Persistent Camera worker did not stop within the join timeout."
            )
