from pathlib import Path
from threading import Event
from time import monotonic
from typing import Optional

import cv2
from ultralytics import YOLO

from utils.logger import get_logger
from vision.types import DetectionResult


class YoloCamera:
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

        self.camera_index = camera_index
        self.target_label = target_label
        self.confidence_threshold = confidence_threshold
        self.stable_frames = stable_frames
        self.stability_tolerance_px = stability_tolerance_px
        self.inference_imgsz = inference_imgsz
        self.device = device
        self.show_preview = show_preview

    def detect_target(
        self,
        abort_event: Event,
        timeout_seconds: float = 20.0,
    ) -> Optional[DetectionResult]:
        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            raise RuntimeError(f"Could not open camera index {self.camera_index}")

        deadline = monotonic() + timeout_seconds
        consecutive_detections = 0
        latest_detection: Optional[DetectionResult] = None

        self.logger.info(
            "Searching for target '%s' using model '%s'...",
            self.target_label,
            self.model_path,
        )

        try:
            while monotonic() < deadline:
                if abort_event.is_set():
                    self.logger.info("Target detection aborted.")
                    return None

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
                matching_detections = []

                for box in result.boxes:
                    class_id = int(box.cls.item())
                    label = result.names[class_id]
                    confidence = float(box.conf.item())
                    if label != self.target_label:
                        continue

                    x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
                    height, width = frame.shape[:2]
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

                if len(matching_detections) == 1:
                    current = matching_detections[0]
                    if (
                        latest_detection is not None
                        and abs(current.center_x - latest_detection.center_x) < self.stability_tolerance_px
                        and abs(current.center_y - latest_detection.center_y) < self.stability_tolerance_px
                    ):
                        consecutive_detections += 1
                    else:
                        consecutive_detections = 1
                    latest_detection = current
                elif len(matching_detections) > 1:
                    consecutive_detections = 0
                    latest_detection = None
                    self.logger.warning(
                        "Multiple '%s' targets detected; waiting for an unambiguous target.",
                        self.target_label,
                    )
                else:
                    consecutive_detections = 0
                    latest_detection = None

                if self.show_preview:
                    preview = result.plot()
                    cv2.putText(
                        preview,
                        f"Target: {self.target_label} | stable: {consecutive_detections}/{self.stable_frames}",
                        (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0),
                        2,
                    )
                    cv2.imshow("YOLO grasp detection", preview)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        abort_event.set()
                        return None

                if latest_detection is not None and consecutive_detections >= self.stable_frames:
                    self.logger.info(
                        "Stable target detected: label=%s confidence=%.2f centre=(%d, %d) region=%s",
                        latest_detection.label,
                        latest_detection.confidence,
                        latest_detection.center_x,
                        latest_detection.center_y,
                        latest_detection.region,
                    )
                    return latest_detection

            self.logger.warning("Target detection timed out after %.1f seconds.", timeout_seconds)
            return None
        finally:
            cap.release()
            if self.show_preview:
                cv2.destroyAllWindows()
