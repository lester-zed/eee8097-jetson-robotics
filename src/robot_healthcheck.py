#!/usr/bin/env python3
"""One-command, non-arm-motion health check for the EEE8097 robot.

Checks are sequential so two processes never compete for one serial port:

1. Docker aliases exist and RoArm/RPLIDAR are different character devices.
2. RoArm answers the read-only T=105 request with T=1051.
3. RPLIDAR C1 returns complete scans through Slamtec's official SDK.
4. The USB camera returns non-empty OpenCV frames.

The RPLIDAR motor rotates during its scan.  No RoArm movement command is sent.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import stat
import sys
import time
from typing import Any, Callable, Sequence

from lidar.rplidar_distance import (
    RPLidarSdkReader,
    SectorMeasurementError,
    measure_sector,
)


@dataclass(frozen=True)
class CheckResult:
    name: str
    status: str
    summary: str
    details: dict[str, Any]
    elapsed_s: float

    @property
    def passed(self) -> bool:
        return self.status == "PASS"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _timed_check(
    name: str,
    function: Callable[[], tuple[str, dict[str, Any]]],
) -> CheckResult:
    started = time.monotonic()
    try:
        summary, details = function()
        return CheckResult(
            name=name,
            status="PASS",
            summary=summary,
            details=details,
            elapsed_s=round(time.monotonic() - started, 3),
        )
    except Exception as exc:  # A health report should collect, not crash.
        return CheckResult(
            name=name,
            status="FAIL",
            summary=f"{type(exc).__name__}: {exc}",
            details={},
            elapsed_s=round(time.monotonic() - started, 3),
        )


def _skipped(name: str, reason: str) -> CheckResult:
    return CheckResult(
        name=name,
        status="SKIP",
        summary=reason,
        details={},
        elapsed_s=0.0,
    )


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8", errors="replace").strip()
    except (OSError, UnicodeError):
        return None


def device_identity(device: str) -> dict[str, Any]:
    """Resolve a container device alias through major/minor and sysfs."""
    path = Path(device)
    if not path.exists():
        raise FileNotFoundError(f"device node does not exist: {device}")
    device_stat = path.stat()
    if not stat.S_ISCHR(device_stat.st_mode):
        raise RuntimeError(f"not a character device: {device}")

    major = os.major(device_stat.st_rdev)
    minor = os.minor(device_stat.st_rdev)
    identity: dict[str, Any] = {
        "path": device,
        "major": major,
        "minor": minor,
        "major_minor": f"{major}:{minor}",
    }

    sysfs_link = Path(f"/sys/dev/char/{major}:{minor}")
    if not sysfs_link.exists():
        return identity
    try:
        resolved = sysfs_link.resolve()
    except OSError:
        return identity
    identity["sysfs_path"] = str(resolved)

    attribute_names = {
        "serial": "usb_serial",
        "manufacturer": "usb_manufacturer",
        "product": "usb_product",
        "idVendor": "usb_vid",
        "idProduct": "usb_pid",
    }
    for directory in (resolved, *resolved.parents):
        for filename, output_name in attribute_names.items():
            if output_name in identity:
                continue
            value = _read_text(directory / filename)
            if value:
                identity[output_name] = value
    return identity


def check_serial_mapping(
    roarm_device: str,
    lidar_device: str,
    *,
    expected_roarm_serial: str | None = None,
    expected_lidar_serial: str | None = None,
) -> tuple[str, dict[str, Any]]:
    roarm = device_identity(roarm_device)
    lidar = device_identity(lidar_device)
    if roarm["major_minor"] == lidar["major_minor"]:
        raise RuntimeError(
            "RoArm and RPLIDAR aliases refer to the same kernel device "
            f"({roarm['major_minor']}); check docker-compose variables"
        )

    if expected_roarm_serial:
        observed = roarm.get("usb_serial")
        if observed and observed != expected_roarm_serial:
            raise RuntimeError(
                "RoArm USB serial mismatch: "
                f"expected {expected_roarm_serial}, observed {observed}"
            )
    if expected_lidar_serial:
        observed = lidar.get("usb_serial")
        if observed and observed != expected_lidar_serial:
            raise RuntimeError(
                "RPLIDAR USB serial mismatch: "
                f"expected {expected_lidar_serial}, observed {observed}"
            )
    return (
        "serial aliases are present and distinct",
        {"roarm": roarm, "rplidar": lidar},
    )


def check_roarm(device: str) -> tuple[str, dict[str, Any]]:
    # Import lazily so Camera/LiDAR diagnostics can still be inspected when a
    # local test environment does not contain the user's RoArm package.
    from arm_control.roarm_uart import RoArmUart

    with RoArmUart(
        port=device,
        baudrate=115200,
        response_timeout_seconds=3.0,
    ) as arm:
        state = arm.get_state()

    if not 7.0 <= state.voltage_v <= 13.0:
        raise RuntimeError(
            f"T=1051 received but voltage is implausible: {state.voltage_v:.2f} V"
        )
    details = {
        "feedback_type": state.raw.get("T"),
        "voltage_v": state.voltage_v,
        "base_rad": state.base_rad,
        "shoulder_rad": state.shoulder_rad,
        "elbow_rad": state.elbow_rad,
        "hand_rad": state.hand_rad,
        "x_mm": state.x_mm,
        "y_mm": state.y_mm,
        "z_mm": state.z_mm,
    }
    return (
        f"read-only T=1051 received, voltage={state.voltage_v:.2f} V",
        details,
    )


def check_rplidar(
    device: str,
    *,
    sdk_binary: str | None,
    scans: int,
    timeout_seconds: float,
    front_angle_deg: float,
    half_width_deg: float,
) -> tuple[str, dict[str, Any]]:
    reader = RPLidarSdkReader(
        device=device,
        baudrate=460800,
        sdk_binary=sdk_binary,
    )
    capture = reader.capture_scans(
        scan_count=scans,
        timeout_seconds=timeout_seconds,
    )
    details = capture.summary_dict()
    try:
        front = measure_sector(
            capture.points,
            centre_angle_deg=front_angle_deg,
            half_width_deg=half_width_deg,
            min_range_mm=120.0,
            max_range_mm=12000.0,
            min_quality=1,
            min_points=3,
            outlier_mm=150.0,
        )
        details["front_sector"] = front.to_dict()
        front_text = f", front={front.distance_mm:.0f} mm"
    except SectorMeasurementError as exc:
        # Complete valid scans prove the device works even when the selected
        # sector has no reflecting object.
        details["front_sector"] = None
        details["front_sector_warning"] = str(exc)
        front_text = ", front sector has no valid target"

    return (
        f"{capture.completed_scans} scans, "
        f"{len(capture.valid_points)} valid points{front_text}",
        details,
    )


def _camera_source(value: str) -> int | str:
    stripped = str(value).strip()
    if stripped.isdigit():
        return int(stripped)
    return stripped


def check_camera(
    device: str,
    *,
    frames: int,
    check_yolo: bool,
    yolo_model: str,
) -> tuple[str, dict[str, Any]]:
    import cv2

    source = _camera_source(device)
    capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if not capture.isOpened():
        capture.release()
        # Some OpenCV builds choose the backend correctly only when CAP_ANY is
        # used.  Retry once without hiding an actual failure.
        capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError(f"OpenCV could not open camera: {device}")

    successful = 0
    last_frame = None
    means: list[float] = []
    deviations: list[float] = []
    try:
        # Discard a few startup frames so auto-exposure can settle.
        for _ in range(3):
            capture.read()
        for _ in range(max(1, int(frames))):
            ok, frame = capture.read()
            if not ok or frame is None or getattr(frame, "size", 0) == 0:
                continue
            successful += 1
            last_frame = frame
            means.append(float(frame.mean()))
            deviations.append(float(frame.std()))
    finally:
        capture.release()

    required = max(1, min(3, int(frames)))
    if successful < required or last_frame is None:
        raise RuntimeError(
            f"camera returned only {successful}/{int(frames)} valid frames"
        )

    height, width = last_frame.shape[:2]
    details: dict[str, Any] = {
        "frames_requested": int(frames),
        "frames_valid": successful,
        "width": int(width),
        "height": int(height),
        "mean_intensity": round(sum(means) / len(means), 3),
        "mean_stddev": round(sum(deviations) / len(deviations), 3),
        "yolo_checked": bool(check_yolo),
    }

    yolo_text = ""
    if check_yolo:
        from ultralytics import YOLO

        model_path = Path(yolo_model)
        if not model_path.is_file():
            raise FileNotFoundError(f"YOLO model not found: {yolo_model}")
        model = YOLO(str(model_path))
        result = model.predict(source=last_frame, verbose=False)[0]
        detection_count = len(result.boxes) if result.boxes is not None else 0
        details["yolo_detection_count"] = detection_count
        yolo_text = f", YOLO inference OK ({detection_count} detections)"

    return (
        f"{successful}/{int(frames)} frames at {width}x{height}{yolo_text}",
        details,
    )


def _default_report_path() -> Path:
    workspace_logs = Path("/workspace/logs")
    if workspace_logs.exists():
        return workspace_logs / "robot_health_latest.json"
    return Path("/tmp/robot_health_latest.json")


def _write_report(path: Path, payload: dict[str, Any]) -> Path:
    path = path.expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    return path


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only RoArm + RPLIDAR C1 + Camera boot health check"
    )
    parser.add_argument("--roarm", default="/dev/ttyROARM")
    parser.add_argument("--rplidar", default="/dev/ttyRPLIDAR")
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--sdk-bin")
    parser.add_argument("--lidar-scans", type=int, default=2)
    parser.add_argument("--lidar-timeout", type=float, default=12.0)
    parser.add_argument("--front-angle", type=float, default=0.0)
    parser.add_argument("--front-half-width", type=float, default=5.0)
    parser.add_argument("--camera-frames", type=int, default=5)
    parser.add_argument("--check-yolo", action="store_true")
    parser.add_argument(
        "--yolo-model",
        default="/workspace/src/vision/yolov8n.pt",
    )
    parser.add_argument(
        "--expected-roarm-serial",
        default=os.environ.get("ROARM_USB_SERIAL"),
    )
    parser.add_argument(
        "--expected-lidar-serial",
        default=os.environ.get("RPLIDAR_USB_SERIAL"),
    )
    parser.add_argument("--skip-roarm", action="store_true")
    parser.add_argument("--skip-rplidar", action="store_true")
    parser.add_argument("--skip-camera", action="store_true")
    parser.add_argument(
        "--continue-after-roarm-failure",
        action="store_true",
        help=(
            "Normally the RPLIDAR test is skipped after a RoArm protocol "
            "failure, because swapped serial aliases have not been excluded."
        ),
    )
    parser.add_argument("--report", default=str(_default_report_path()))
    parser.add_argument("--no-report", action="store_true")
    parser.add_argument("--json", action="store_true")
    return parser


def run_checks(args: argparse.Namespace) -> tuple[list[CheckResult], dict[str, Any]]:
    results: list[CheckResult] = []

    mapping = _timed_check(
        "device-map",
        lambda: check_serial_mapping(
            args.roarm,
            args.rplidar,
            expected_roarm_serial=args.expected_roarm_serial,
            expected_lidar_serial=args.expected_lidar_serial,
        ),
    )
    results.append(mapping)

    serial_mapping_safe = mapping.passed
    if args.skip_roarm:
        roarm = _skipped("roarm", "skipped by command line")
    elif not serial_mapping_safe:
        roarm = _skipped("roarm", "unsafe serial mapping; test not opened")
    else:
        roarm = _timed_check("roarm", lambda: check_roarm(args.roarm))
    results.append(roarm)

    if args.skip_rplidar:
        rplidar = _skipped("rplidar", "skipped by command line")
    elif not serial_mapping_safe:
        rplidar = _skipped("rplidar", "unsafe serial mapping; test not opened")
    elif (
        not args.skip_roarm
        and not roarm.passed
        and not args.continue_after_roarm_failure
    ):
        rplidar = _skipped(
            "rplidar",
            "RoArm protocol check failed; verify aliases before sending SDK "
            "traffic to the second serial device",
        )
    else:
        rplidar = _timed_check(
            "rplidar",
            lambda: check_rplidar(
                args.rplidar,
                sdk_binary=args.sdk_bin,
                scans=args.lidar_scans,
                timeout_seconds=args.lidar_timeout,
                front_angle_deg=args.front_angle,
                half_width_deg=args.front_half_width,
            ),
        )
    results.append(rplidar)

    if args.skip_camera:
        camera = _skipped("camera", "skipped by command line")
    else:
        camera = _timed_check(
            "camera",
            lambda: check_camera(
                args.camera,
                frames=args.camera_frames,
                check_yolo=args.check_yolo,
                yolo_model=args.yolo_model,
            ),
        )
    results.append(camera)

    required = [result for result in results if result.status != "SKIP"]
    overall_pass = bool(required) and all(result.passed for result in required)
    # A skipped hardware test means the complete three-device boot check did
    # not pass, unless the user explicitly requested that test be skipped.
    explicit_skips = {
        "roarm": args.skip_roarm,
        "rplidar": args.skip_rplidar,
        "camera": args.skip_camera,
    }
    unexpected_skip = any(
        result.status == "SKIP" and not explicit_skips.get(result.name, False)
        for result in results
    )
    overall_pass = overall_pass and not unexpected_skip

    payload = {
        "schema_version": 1,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "overall": "PASS" if overall_pass else "FAIL",
        "safety": {
            "roarm_motion_commands_sent": False,
            "roarm_command": "T=105 only",
            "rplidar_motor_rotates_during_test": not args.skip_rplidar,
        },
        "checks": [result.to_dict() for result in results],
    }
    return results, payload


def _print_human(results: Sequence[CheckResult], payload: dict[str, Any]) -> None:
    print("=" * 72)
    print("EEE8097 robot boot health check")
    print("RoArm: read-only T=105; RPLIDAR: motor rotates briefly")
    print("=" * 72)
    for result in results:
        print(
            f"[{result.status:4}] {result.name:<12} "
            f"{result.summary} ({result.elapsed_s:.2f}s)"
        )
    print("-" * 72)
    print(f"OVERALL: {payload['overall']}")


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    results, payload = run_checks(args)

    report_error: str | None = None
    if not args.no_report:
        try:
            report_path = _write_report(Path(args.report), payload)
            payload["report_path"] = str(report_path)
        except OSError as exc:
            report_error = f"Could not write report: {exc}"
            payload["report_error"] = report_error

    if args.json:
        print(json.dumps(payload, indent=2, ensure_ascii=False))
    else:
        _print_human(results, payload)
        if payload.get("report_path"):
            print(f"Report: {payload['report_path']}")
        if report_error:
            print(report_error, file=sys.stderr)

    return 0 if payload["overall"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
