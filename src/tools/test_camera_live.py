#!/usr/bin/env python3
"""Standalone persistent Camera + YOLO preview test.

Run from /workspace/src:
    python3 tools/test_camera_live.py --config configs/modular_pipeline.yaml

The Camera remains open until Ctrl+C or until 'q' is pressed in the preview.
This script does not open RPLIDAR or RoArm.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from configuration.loader import load_runtime_config
from vision.yolo_camera import YoloCamera


SOURCE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = SOURCE_ROOT / "configs/modular_pipeline.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standalone persistent YOLO Camera test"
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument(
        "--status-every-s",
        type=float,
        default=2.0,
        help="Print Camera status every N seconds; use 0 to disable.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_runtime_config(args.config)
    app = config.section("app")
    camera_cfg = config.section("camera")

    camera = YoloCamera(
        model_path=str(camera_cfg["model_path"]),
        camera_index=int(camera_cfg.get("index", 0)),
        target_label=str(app.get("target", "cup")),
        confidence_threshold=float(camera_cfg.get("confidence", 0.55)),
        stable_frames=int(camera_cfg.get("stable_frames", 3)),
        stability_tolerance_px=int(
            camera_cfg.get("stability_tolerance_px", 40)
        ),
        inference_imgsz=int(camera_cfg.get("imgsz", 640)),
        device=camera_cfg.get("device"),
        show_preview=True,
    )

    print("=" * 72)
    print("Standalone persistent Camera test")
    print(f"Config: {config.path}")
    print("RPLIDAR: NOT OPENED")
    print("RoArm:   NOT OPENED")
    print("Press Ctrl+C in terminal or 'q' in preview to stop.")
    print("=" * 72)

    try:
        while True:
            status = camera.health_snapshot(wait_seconds=1.0)
            if status["last_error"]:
                print(json.dumps(status, indent=2))
                return 1
            if not status["running"]:
                print(json.dumps(status, indent=2))
                return 0

            if args.status_every_s > 0:
                print(json.dumps(status, indent=2))
                time.sleep(args.status_every_s)
            else:
                time.sleep(0.25)
    except KeyboardInterrupt:
        print("\nStopping Camera test...")
        return 0
    finally:
        camera.close()


if __name__ == "__main__":
    raise SystemExit(main())
