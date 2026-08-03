from __future__ import annotations

import argparse
import inspect
import json
from pathlib import Path
import sys
from typing import Any

from arm_control.arm_controller import ArmController
from task.task_manager import TaskManager


REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
HELP = (
    "s + Enter: start | x + Enter: abort | p + Enter: task status | "
    "a + Enter: arm status | r + Enter: reset | q + Enter: quit"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="YOLO + reverse-mounted RoArm base-only controller"
    )
    parser.add_argument("--camera", default="0")
    parser.add_argument("--model", default="src/vision/yolov8n.pt")
    parser.add_argument("--target", default="cup")
    parser.add_argument("--confidence", type=float, default=0.55)
    parser.add_argument("--stable-frames", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument(
        "--mock-vision",
        action="store_true",
        help="Use the repository MockVision instead of Camera/YOLO.",
    )
    parser.add_argument(
        "--ssh",
        action="store_true",
        help="Disable cv2.imshow(); local execution previews by default.",
    )
    parser.add_argument(
        "--real-arm",
        action="store_true",
        help="Open the RoArm UART; without this flag the arm is mock-only.",
    )
    parser.add_argument("--arm-port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--mount-yaw", type=float, default=180.0)
    parser.add_argument("--region-yaw", type=float, default=20.0)
    parser.add_argument("--speed", type=int, default=10)
    parser.add_argument("--acceleration", type=int, default=10)
    parser.add_argument("--confirm-motion", action="store_true")
    parser.add_argument("--confirm-clearance", action="store_true")
    return parser.parse_args()


def _camera_value(value: str) -> int | str:
    if value.isdigit():
        return int(value)
    if value.startswith("/dev/video") and value[10:].isdigit():
        return int(value[10:])
    return value


def build_vision(args: argparse.Namespace) -> Any:
    if args.mock_vision:
        from vision.mock_vision import MockVision

        return MockVision()

    from vision.yolo_camera import YoloCamera

    parameters = inspect.signature(YoloCamera).parameters
    model_path = Path(args.model)
    if not model_path.is_absolute():
        model_path = REPOSITORY_ROOT / model_path

    # Supports the simple constructor currently shown in the user's local
    # main.py and the more modular constructor generated for the repository.
    candidates: dict[str, Any] = {
        "camera_index": _camera_value(args.camera),
        "target_label": args.target,
        "camera_device": _camera_value(args.camera),
        "model_path": model_path,
        "target_labels": [args.target],
        "confidence_threshold": args.confidence,
        "timeout_seconds": args.timeout,
        "stable_frames": args.stable_frames,
        "stable_pixel_tolerance": 60,
        "require_single_target": True,
        "image_size": 640,
        "width": 1920,
        "height": 1080,
        "frame_rate": 30,
        "pixel_format": "MJPG",
        "disable_dynamic_framerate": True,
        "show_preview": not args.ssh,
        "snapshot_path": REPOSITORY_ROOT / "logs" / "last_detection.jpg",
    }
    kwargs = {
        name: value
        for name, value in candidates.items()
        if name in parameters
    }
    if "model_path" in parameters and not model_path.is_file():
        raise FileNotFoundError(f"YOLO model not found: {model_path}")
    return YoloCamera(**kwargs)


def build_manager(args: argparse.Namespace) -> TaskManager:
    vision = build_vision(args)
    arm = ArmController(
        mode="real" if args.real_arm else "mock",
        port=args.arm_port,
        baudrate=args.baudrate,
        mount_yaw_degrees=args.mount_yaw,
        region_yaw_degrees=args.region_yaw,
        speed_degrees_s=args.speed,
        acceleration_degrees_s2=args.acceleration,
        confirm_motion=args.confirm_motion,
        confirm_clearance=args.confirm_clearance,
    )

    parameters = inspect.signature(TaskManager).parameters
    kwargs: dict[str, Any] = {"vision": vision, "arm": arm}
    if "planner" in parameters:
        from planning.preset_planner import PresetPlanner

        kwargs["planner"] = PresetPlanner()
    return TaskManager(**kwargs)


def run_cli(manager: TaskManager) -> int:
    print("EEE8097 YOLO + RoArm controller")
    if manager.arm.mode == "real":
        print("Arm mode: REAL, reverse-mounted, base-only, one motion/process")
    else:
        print("Arm mode: MOCK; no serial port will be opened")
    print(HELP)

    try:
        while True:
            try:
                command = input("robot> ").strip().lower()
            except EOFError:
                command = "q"

            if command == "s":
                print(
                    "Task started."
                    if manager.start()
                    else "Start rejected; use p to inspect the state."
                )
            elif command == "x":
                print(
                    "Abort requested."
                    if manager.abort()
                    else "No running task."
                )
            elif command == "p":
                print(json.dumps(manager.status(), indent=2, ensure_ascii=False))
            elif command == "a":
                try:
                    print(
                        json.dumps(
                            manager.arm.status(),
                            indent=2,
                            ensure_ascii=False,
                        )
                    )
                except Exception as exc:
                    print(f"Arm status failed: {type(exc).__name__}: {exc}")
            elif command == "r":
                print(
                    "State reset to IDLE."
                    if manager.reset()
                    else "Reset rejected; use p to inspect the state."
                )
            elif command == "q":
                return 0
            elif command in {"h", "help", "?"}:
                print(HELP)
            elif command:
                print(f"Unknown command: {command!r}")
                print(HELP)
    except KeyboardInterrupt:
        print("\nStopping...")
        return 130
    finally:
        manager.shutdown()


def main() -> int:
    args = parse_args()
    if args.real_arm and not (
        args.confirm_motion and args.confirm_clearance
    ):
        print(
            "ERROR: --real-arm requires --confirm-motion and "
            "--confirm-clearance.",
            file=sys.stderr,
        )
        return 2
    return run_cli(build_manager(args))


if __name__ == "__main__":
    raise SystemExit(main())
