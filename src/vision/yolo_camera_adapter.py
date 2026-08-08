from __future__ import annotations

from pathlib import Path
from threading import Event
import time
from typing import Any

from pipeline.types import Detection2D, HealthResult


class ExistingYoloCameraAdapter:
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
        from vision.yolo_camera import YoloCamera

        self._model_path = str(model_path)
        self._camera: Any = YoloCamera(
            model_path=model_path,
            camera_index=camera_index,
            target_label=target_label,
            confidence_threshold=confidence_threshold,
            stable_frames=stable_frames,
            stability_tolerance_px=stability_tolerance_px,
            inference_imgsz=inference_imgsz,
            device=device,
            show_preview=show_preview,
        )

    def detect_target(self, abort_event: Event, timeout_seconds: float = 20.0) -> Detection2D | None:
        result = self._camera.detect_target(abort_event=abort_event, timeout_seconds=timeout_seconds)
        if result is None:
            return None
        return Detection2D(
            label=result.label,
            confidence=float(result.confidence),
            center_x=int(result.center_x),
            center_y=int(result.center_y),
            bbox=tuple(int(value) for value in result.bbox),
            frame_width=int(result.frame_width),
            frame_height=int(result.frame_height),
            captured_at_s=time.monotonic(),
        )

    def health_check(self) -> HealthResult:
        return HealthResult(
            "camera",
            True,
            f"YoloCamera adapter initialized with model={self._model_path}; Camera opens when a task starts",
        )

    def close(self) -> None:
        return None
