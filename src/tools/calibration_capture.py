#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time


SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from configuration.loader import load_runtime_config
from experiments.calibration import validate_no_motion_calibration_profile
from experiments.recorder import (
    CalibrationGroundTruth,
    ExperimentRecorder,
    RunContext,
)
from main_modular import build_manager
from pipeline.task_manager import TaskState


DEFAULT_CONFIG = SOURCE_ROOT / "configs/calibration_capture.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture Camera + RPLIDAR target estimates with a Mock RoArm; "
            "no RoArm serial port or startup home is opened"
        )
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--validate-config", action="store_true")
    parser.add_argument("--point-id")
    parser.add_argument("--truth-x-mm", type=float)
    parser.add_argument("--truth-y-mm", type=float)
    parser.add_argument("--truth-z-mm", type=float)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--timeout-s", type=float, default=60.0)
    parser.add_argument("--notes", default="")
    return parser.parse_args()


def _ground_truth(args: argparse.Namespace) -> CalibrationGroundTruth:
    missing = [
        name
        for name, value in (
            ("--point-id", args.point_id),
            ("--truth-x-mm", args.truth_x_mm),
            ("--truth-y-mm", args.truth_y_mm),
        )
        if value is None or (isinstance(value, str) and not value.strip())
    ]
    if missing:
        raise ValueError("Missing calibration arguments: " + ", ".join(missing))
    return CalibrationGroundTruth(
        point_id=str(args.point_id),
        x_mm=float(args.truth_x_mm),
        y_mm=float(args.truth_y_mm),
        z_mm=args.truth_z_mm,
    )


def _wait_for_terminal_state(manager, timeout_s: float) -> dict:
    deadline = time.monotonic() + timeout_s
    while manager.is_running() and time.monotonic() < deadline:
        time.sleep(0.05)
    if manager.is_running():
        manager.abort()
        abort_deadline = time.monotonic() + 5.0
        while manager.is_running() and time.monotonic() < abort_deadline:
            time.sleep(0.05)
        if manager.is_running():
            raise TimeoutError("Calibration task did not stop after abort")
    return manager.status()


def main() -> int:
    args = parse_args()
    if args.repeat < 1:
        raise ValueError("--repeat must be >= 1")
    if args.timeout_s <= 0.0:
        raise ValueError("--timeout-s must be positive")

    config = load_runtime_config(args.config)
    validate_no_motion_calibration_profile(config)
    if args.validate_config:
        print(f"No-motion calibration configuration valid: {config.path}")
        print("Camera=REAL, RPLIDAR=REAL, RoArm=MOCK, startup home=SKIPPED")
        return 0

    truth = _ground_truth(args)
    recorder = ExperimentRecorder.from_runtime_config(config)
    manager = build_manager(config, on_run_complete=recorder.record)

    print("=" * 72)
    print("EEE8097 no-motion calibration capture")
    print(f"Point: {truth.point_id}")
    print(
        "Ground truth base_link: "
        f"X={truth.x_mm:.1f} mm, Y={truth.y_mm:.1f} mm, "
        f"Z={truth.z_mm if truth.z_mm is not None else 'not supplied'}"
    )
    print(f"Samples requested: {args.repeat}")
    print("RoArm: MOCK (no serial port, no home, no motion command)")
    print(f"JSONL: {recorder.jsonl_path}")
    print(f"CSV:   {recorder.csv_path}")
    print("=" * 72)

    try:
        for repeat_index in range(1, args.repeat + 1):
            if repeat_index > 1 and not manager.reset():
                raise RuntimeError("Could not reset completed calibration task")
            recorder.set_run_context(
                RunContext(
                    ground_truth=truth,
                    repeat_index=repeat_index,
                    notes=args.notes,
                )
            )
            if not manager.start():
                raise RuntimeError("Calibration task start was rejected")
            status = _wait_for_terminal_state(manager, args.timeout_s)
            summary = {
                "run_id": status.get("run_id"),
                "state": status.get("state"),
                "duration_s": status.get("timing", {}).get("duration_s"),
                "target_base_link": (
                    status.get("target", {}).get("target_base_link")
                    if isinstance(status.get("target"), dict)
                    else None
                ),
                "error": status.get("error"),
                "recording_error": status.get("recording_error"),
            }
            print(json.dumps(summary, indent=2, ensure_ascii=False))
            if status.get("recording_error"):
                raise RuntimeError(status["recording_error"])
            if status.get("state") != TaskState.COMPLETE.value:
                print(
                    "Capture stopped after the first failed/aborted sample; "
                    "no automatic retry was attempted."
                )
                return 2
    finally:
        manager.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
