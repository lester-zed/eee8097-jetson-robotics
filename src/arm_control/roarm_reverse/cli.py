"""Safe diagnostics runnable from a Docker-mounted src directory."""

from __future__ import annotations

import argparse
import json
import logging
import os
from threading import Event
from typing import Any

from .controller import RealRoArmController
from .transform import ReverseMountTransform
from .uart import RoArmState, RoArmUart


DEFAULT_PORT = os.environ.get("ROARM_PORT", "/dev/ttyROARM")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Safe RoArm-M2-S status and reverse-mount base test"
    )
    parser.add_argument(
        "command",
        nargs="?",
        default="status",
        choices=("map", "status", "aim", "gripper"),
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--mount-yaw", type=float, default=180.0)
    parser.add_argument("--region-yaw", type=float, default=20.0)
    parser.add_argument(
        "--region",
        choices=("left", "center", "centre", "right"),
        default="center",
    )
    parser.add_argument("--speed", type=int, default=10)
    parser.add_argument("--acceleration", type=int, default=10)
    parser.add_argument(
        "--gripper-position",
        choices=("partial-open", "closed"),
        default="partial-open",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Required for commands that cause physical motion.",
    )
    parser.add_argument(
        "--confirm-clearance",
        action="store_true",
        help="Confirm that the full reverse-mounted base sweep is clear.",
    )
    return parser.parse_args()


def state_payload(state: RoArmState) -> dict[str, Any]:
    return {
        "x_mm": state.x_mm,
        "y_mm": state.y_mm,
        "z_mm": state.z_mm,
        "base_rad": state.base_rad,
        "shoulder_rad": state.shoulder_rad,
        "elbow_rad": state.elbow_rad,
        "hand_rad": state.hand_rad,
        "voltage_v": state.voltage_v,
        "raw": state.raw,
    }


def require_execution(args: argparse.Namespace, *, clearance: bool) -> None:
    if not args.execute:
        raise SystemExit(
            "Refusing real motion: add --execute only after checking the command."
        )
    if clearance and not args.confirm_clearance:
        raise SystemExit(
            "Refusing base motion: add --confirm-clearance only after clearing "
            "the full sweep area."
        )


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )
    args = parse_args()
    transform = ReverseMountTransform(
        mount_yaw_degrees=args.mount_yaw,
        region_yaw_degrees=args.region_yaw,
    )

    if args.command == "map":
        output = {
            "mount_yaw_degrees": transform.mount_yaw_degrees,
            "region_yaw_degrees": transform.region_yaw_degrees,
            "camera_to_arm_base_degrees": {
                region: transform.camera_region_to_arm_base(region)
                for region in ("left", "center", "right")
            },
            "example_robot_xyz_mm": [300.0, 50.0, 100.0],
            "example_arm_xyz_mm": list(
                transform.robot_to_arm_xyz(300.0, 50.0, 100.0)
            ),
        }
        print(json.dumps(output, indent=2))
        return 0

    if args.command == "status":
        with RoArmUart(args.port, args.baudrate) as arm:
            print(json.dumps(state_payload(arm.get_state()), indent=2))
        return 0

    if args.command == "aim":
        require_execution(args, clearance=True)
        controller = RealRoArmController(
            port=args.port,
            baudrate=args.baudrate,
            mount_yaw_degrees=args.mount_yaw,
            region_yaw_degrees=args.region_yaw,
            speed_degrees_s=args.speed,
            acceleration_degrees_s2=args.acceleration,
            confirm_motion=True,
            confirm_clearance=True,
        )
        try:
            controller.execute_region(args.region, Event())
            print(json.dumps(controller.status(), indent=2))
        finally:
            controller.close()
        return 0

    if args.command == "gripper":
        require_execution(args, clearance=False)
        angle = 2.70 if args.gripper_position == "partial-open" else 3.14
        with RoArmUart(args.port, args.baudrate) as arm:
            # Identify the serial device as RoArm before sending T=106.
            print(json.dumps(state_payload(arm.get_state()), indent=2))
            arm.set_gripper_radians(
                angle,
                speed_steps_s=100,
                acceleration=10,
            )
        return 0

    raise AssertionError(f"Unhandled command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
