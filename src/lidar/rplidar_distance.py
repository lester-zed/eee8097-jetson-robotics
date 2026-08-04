#!/usr/bin/env python3
"""Measure a sector with a Slamtec RPLIDAR C1 via the official C++ SDK.

The official ``ultra_simple`` application owns the serial protocol.  This
module starts it with the C1-specific 460800 baud rate, parses its scan output,
and always asks it to stop cleanly with SIGINT so the SDK can stop scanning and
the motor before the process exits.

No third-party Python RPLIDAR package is used.  Build the official SDK once
with ``setup_rplidar_sdk.sh`` or provide ``--sdk-bin``.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import re
import selectors
import shutil
import signal
import statistics
import subprocess
import sys
import time
from typing import Iterable, Sequence


C1_BAUDRATE = 460800

_POINT_PATTERN = re.compile(
    r"^\s*(?:(?P<sync>S)\s+)?"
    r"theta:\s*(?P<angle>[+-]?\d+(?:\.\d+)?)\s+"
    r"Dist:\s*(?P<distance>[+-]?\d+(?:\.\d+)?)\s+"
    r"Q:\s*(?P<quality>\d+)\s*$",
    re.IGNORECASE,
)


class RPLidarSdkError(RuntimeError):
    """The SDK binary could not be started or did not return valid scans."""


class SectorMeasurementError(RuntimeError):
    """Not enough valid points were present in the requested angular sector."""


@dataclass(frozen=True)
class ScanPoint:
    angle_deg: float
    distance_mm: float
    quality: int
    sync: bool = False


@dataclass(frozen=True)
class ScanCapture:
    device: str
    baudrate: int
    requested_scans: int
    completed_scans: int
    elapsed_s: float
    points: tuple[ScanPoint, ...]
    serial_number: str | None = None
    firmware_version: str | None = None
    hardware_revision: str | None = None
    health_status: int | None = None
    sdk_version: str | None = None
    diagnostics: tuple[str, ...] = ()

    @property
    def valid_points(self) -> tuple[ScanPoint, ...]:
        return tuple(
            point
            for point in self.points
            if point.distance_mm > 0.0 and point.quality > 0
        )

    def summary_dict(self) -> dict[str, object]:
        return {
            "device": self.device,
            "baudrate": self.baudrate,
            "requested_scans": self.requested_scans,
            "completed_scans": self.completed_scans,
            "elapsed_s": round(self.elapsed_s, 3),
            "point_count": len(self.points),
            "valid_point_count": len(self.valid_points),
            "serial_number": self.serial_number,
            "firmware_version": self.firmware_version,
            "hardware_revision": self.hardware_revision,
            "health_status": self.health_status,
            "sdk_version": self.sdk_version,
            "diagnostics": list(self.diagnostics),
        }


@dataclass(frozen=True)
class DistanceMeasurement:
    centre_angle_deg: float
    half_width_deg: float
    distance_mm: float
    sample_count: int
    median_quality: float
    mad_mm: float
    minimum_mm: float
    maximum_mm: float

    @property
    def distance_m(self) -> float:
        return self.distance_mm / 1000.0

    def to_dict(self) -> dict[str, object]:
        payload = asdict(self)
        payload["distance_m"] = self.distance_m
        return payload


def parse_ultra_simple_line(line: str) -> ScanPoint | None:
    """Parse one point printed by Slamtec's ``ultra_simple`` demo."""
    match = _POINT_PATTERN.match(str(line).strip())
    if match is None:
        return None
    return ScanPoint(
        angle_deg=float(match.group("angle")) % 360.0,
        distance_mm=float(match.group("distance")),
        quality=int(match.group("quality")),
        sync=bool(match.group("sync")),
    )


def circular_delta_degrees(angle_deg: float, centre_deg: float) -> float:
    """Signed shortest angular difference in the interval [-180, 180)."""
    return (float(angle_deg) - float(centre_deg) + 180.0) % 360.0 - 180.0


def measure_sector(
    points: Iterable[ScanPoint],
    *,
    centre_angle_deg: float = 0.0,
    half_width_deg: float = 5.0,
    min_range_mm: float = 120.0,
    max_range_mm: float = 12000.0,
    min_quality: int = 1,
    min_points: int = 3,
    outlier_mm: float | None = 150.0,
) -> DistanceMeasurement:
    """Return a robust multi-scan median for one angular sector.

    ``centre_angle_deg`` is in the LiDAR's own frame.  It must be calibrated;
    the physical robot-forward direction is not necessarily 0 degrees.
    """
    if not 0.0 < float(half_width_deg) <= 90.0:
        raise ValueError("half_width_deg must be in (0, 90]")
    if not 0.0 <= float(min_range_mm) < float(max_range_mm):
        raise ValueError("range limits are invalid")
    if int(min_points) < 1:
        raise ValueError("min_points must be positive")
    if outlier_mm is not None and float(outlier_mm) <= 0.0:
        raise ValueError("outlier_mm must be positive or None")

    selected = [
        point
        for point in points
        if abs(circular_delta_degrees(point.angle_deg, centre_angle_deg))
        <= float(half_width_deg)
        and float(min_range_mm) <= point.distance_mm <= float(max_range_mm)
        and point.quality >= int(min_quality)
    ]
    if len(selected) < int(min_points):
        raise SectorMeasurementError(
            "Not enough valid RPLIDAR points in sector: "
            f"received {len(selected)}, need {int(min_points)}"
        )

    distances = [point.distance_mm for point in selected]
    first_median = statistics.median(distances)
    if outlier_mm is not None:
        inlier_points = [
            point
            for point in selected
            if abs(point.distance_mm - first_median) <= float(outlier_mm)
        ]
        if len(inlier_points) >= int(min_points):
            selected = inlier_points
            distances = [point.distance_mm for point in selected]

    distance_mm = float(statistics.median(distances))
    absolute_deviations = [abs(value - distance_mm) for value in distances]
    return DistanceMeasurement(
        centre_angle_deg=float(centre_angle_deg) % 360.0,
        half_width_deg=float(half_width_deg),
        distance_mm=distance_mm,
        sample_count=len(selected),
        median_quality=float(
            statistics.median(point.quality for point in selected)
        ),
        mad_mm=float(statistics.median(absolute_deviations)),
        minimum_mm=float(min(distances)),
        maximum_mm=float(max(distances)),
    )


def resolve_sdk_binary(requested: str | os.PathLike[str] | None = None) -> Path:
    """Find the official ``ultra_simple`` binary without downloading anything."""
    if requested:
        candidates = [Path(requested).expanduser()]
    else:
        source_root = Path(__file__).resolve().parents[1]
        candidates = []
        environment_value = os.environ.get("RPLIDAR_SDK_BIN")
        if environment_value:
            candidates.append(Path(environment_value).expanduser())
        candidates.extend(
            [
                source_root
                / "third_party/rplidar_sdk/output/Linux/Release/ultra_simple",
                Path(
                    "/workspace/rplidar_sdk/output/Linux/Release/ultra_simple"
                ),
                Path(
                    "/opt/rplidar_sdk/output/Linux/Release/ultra_simple"
                ),
                Path("/usr/local/bin/ultra_simple"),
            ]
        )
        path_value = shutil.which("ultra_simple")
        if path_value:
            candidates.append(Path(path_value))

    for candidate in candidates:
        try:
            resolved = candidate.resolve()
        except OSError:
            continue
        if resolved.is_file() and os.access(resolved, os.X_OK):
            return resolved

    rendered = "\n  - ".join(str(path) for path in candidates)
    raise FileNotFoundError(
        "Cannot find the official rplidar_sdk ultra_simple binary. "
        "Run /workspace/src/setup_rplidar_sdk.sh once, set "
        "RPLIDAR_SDK_BIN, or pass --sdk-bin. Checked:\n  - "
        + rendered
    )


def _extract_metadata(line: str, metadata: dict[str, object]) -> None:
    stripped = line.strip()
    if stripped.startswith("Version:"):
        metadata["sdk_version"] = stripped.split(":", 1)[1].strip()
    elif stripped.startswith("SLAMTEC LIDAR S/N:"):
        metadata["serial_number"] = stripped.split(":", 1)[1].strip()
    elif stripped.startswith("Firmware Ver:"):
        metadata["firmware_version"] = stripped.split(":", 1)[1].strip()
    elif stripped.startswith("Hardware Rev:"):
        metadata["hardware_revision"] = stripped.split(":", 1)[1].strip()
    elif "Lidar health status" in stripped:
        try:
            metadata["health_status"] = int(stripped.rsplit(":", 1)[1])
        except (ValueError, IndexError):
            pass


def _stop_process(process: subprocess.Popen[str], grace_seconds: float = 4.0) -> None:
    """Give the SDK a chance to call stop() and setMotorSpeed(0)."""
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except (ProcessLookupError, PermissionError):
        process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=grace_seconds)
        return
    except subprocess.TimeoutExpired:
        process.terminate()
    try:
        process.wait(timeout=2.0)
        return
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2.0)


class RPLidarSdkReader:
    """Short-lived reader backed by Slamtec's official SDK demo binary."""

    def __init__(
        self,
        *,
        device: str = "/dev/ttyRPLIDAR",
        baudrate: int = C1_BAUDRATE,
        sdk_binary: str | os.PathLike[str] | None = None,
    ) -> None:
        if int(baudrate) <= 0:
            raise ValueError("baudrate must be positive")
        self.device = str(device)
        self.baudrate = int(baudrate)
        self.sdk_binary = resolve_sdk_binary(sdk_binary)

    def capture_scans(
        self,
        *,
        scan_count: int = 3,
        timeout_seconds: float = 12.0,
    ) -> ScanCapture:
        if int(scan_count) < 1:
            raise ValueError("scan_count must be positive")
        if float(timeout_seconds) <= 0.0:
            raise ValueError("timeout_seconds must be positive")
        if not Path(self.device).exists():
            raise FileNotFoundError(f"RPLIDAR device does not exist: {self.device}")

        command: list[str] = []
        stdbuf = shutil.which("stdbuf")
        if stdbuf:
            command.extend([stdbuf, "-oL", "-eL"])
        command.extend(
            [
                str(self.sdk_binary),
                "--channel",
                "--serial",
                self.device,
                str(self.baudrate),
            ]
        )

        started = time.monotonic()
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        if process.stdout is None:
            _stop_process(process)
            raise RPLidarSdkError("Failed to capture rplidar_sdk stdout")

        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ)
        points: list[ScanPoint] = []
        diagnostics: list[str] = []
        metadata: dict[str, object] = {}
        sync_count = 0
        deadline = started + float(timeout_seconds)

        try:
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    remainder = process.stdout.read()
                    for line in remainder.splitlines():
                        point = parse_ultra_simple_line(line)
                        if point is not None:
                            if point.sync:
                                sync_count += 1
                            points.append(point)
                        elif line.strip() and len(diagnostics) < 80:
                            diagnostics.append(line.strip())
                            _extract_metadata(line, metadata)
                    break

                events = selector.select(
                    timeout=min(0.25, max(0.0, deadline - time.monotonic()))
                )
                if not events:
                    continue

                line = process.stdout.readline()
                if not line:
                    continue
                point = parse_ultra_simple_line(line)
                if point is None:
                    stripped = line.strip()
                    if stripped and len(diagnostics) < 80:
                        diagnostics.append(stripped)
                    _extract_metadata(line, metadata)
                    continue

                if point.sync:
                    sync_count += 1
                    if sync_count > int(scan_count):
                        break
                if sync_count >= 1:
                    points.append(point)
        finally:
            selector.close()
            _stop_process(process)

        elapsed = time.monotonic() - started
        completed_scans = min(sync_count, int(scan_count))
        capture = ScanCapture(
            device=self.device,
            baudrate=self.baudrate,
            requested_scans=int(scan_count),
            completed_scans=completed_scans,
            elapsed_s=elapsed,
            points=tuple(points),
            serial_number=metadata.get("serial_number"),
            firmware_version=metadata.get("firmware_version"),
            hardware_revision=metadata.get("hardware_revision"),
            health_status=metadata.get("health_status"),
            sdk_version=metadata.get("sdk_version"),
            diagnostics=tuple(diagnostics),
        )
        if completed_scans < int(scan_count) or not capture.valid_points:
            tail = " | ".join(diagnostics[-8:]) or "no diagnostic output"
            raise RPLidarSdkError(
                "RPLIDAR did not return the requested valid scans: "
                f"completed={completed_scans}/{int(scan_count)}, "
                f"valid_points={len(capture.valid_points)}; {tail}"
            )
        if capture.health_status not in (None, 0):
            raise RPLidarSdkError(
                f"RPLIDAR SDK reported non-zero health status: "
                f"{capture.health_status}"
            )
        return capture

    def measure(
        self,
        *,
        scan_count: int = 3,
        timeout_seconds: float = 12.0,
        centre_angle_deg: float = 0.0,
        half_width_deg: float = 5.0,
        min_range_mm: float = 120.0,
        max_range_mm: float = 12000.0,
        min_quality: int = 1,
        min_points: int = 3,
        outlier_mm: float | None = 150.0,
    ) -> tuple[ScanCapture, DistanceMeasurement]:
        capture = self.capture_scans(
            scan_count=scan_count,
            timeout_seconds=timeout_seconds,
        )
        measurement = measure_sector(
            capture.points,
            centre_angle_deg=centre_angle_deg,
            half_width_deg=half_width_deg,
            min_range_mm=min_range_mm,
            max_range_mm=max_range_mm,
            min_quality=min_quality,
            min_points=min_points,
            outlier_mm=outlier_mm,
        )
        return capture, measurement


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Measure a RPLIDAR C1 angular sector with rplidar_sdk"
    )
    parser.add_argument("--device", default="/dev/ttyRPLIDAR")
    parser.add_argument("--baudrate", type=int, default=C1_BAUDRATE)
    parser.add_argument("--sdk-bin")
    parser.add_argument("--scans", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--front-angle", type=float, default=0.0)
    parser.add_argument("--half-width", type=float, default=5.0)
    parser.add_argument("--min-range-mm", type=float, default=120.0)
    parser.add_argument("--max-range-mm", type=float, default=12000.0)
    parser.add_argument("--min-quality", type=int, default=1)
    parser.add_argument("--min-points", type=int, default=3)
    parser.add_argument("--outlier-mm", type=float, default=150.0)
    parser.add_argument(
        "--health-only",
        action="store_true",
        help="Pass when complete scans contain valid points anywhere.",
    )
    parser.add_argument("--json", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        reader = RPLidarSdkReader(
            device=args.device,
            baudrate=args.baudrate,
            sdk_binary=args.sdk_bin,
        )
        capture = reader.capture_scans(
            scan_count=args.scans,
            timeout_seconds=args.timeout,
        )
        measurement: DistanceMeasurement | None = None
        measurement_error: str | None = None
        try:
            measurement = measure_sector(
                capture.points,
                centre_angle_deg=args.front_angle,
                half_width_deg=args.half_width,
                min_range_mm=args.min_range_mm,
                max_range_mm=args.max_range_mm,
                min_quality=args.min_quality,
                min_points=args.min_points,
                outlier_mm=args.outlier_mm,
            )
        except SectorMeasurementError as exc:
            measurement_error = str(exc)
            if not args.health_only:
                raise

        payload = {
            "status": "PASS",
            "capture": capture.summary_dict(),
            "measurement": measurement.to_dict() if measurement else None,
            "measurement_warning": measurement_error,
        }
        if args.json:
            print(json.dumps(payload, indent=2, ensure_ascii=False))
        else:
            print(
                "RPLIDAR PASS: "
                f"scans={capture.completed_scans}, "
                f"valid_points={len(capture.valid_points)}, "
                f"health={capture.health_status}"
            )
            if measurement is not None:
                print(
                    "Sector distance: "
                    f"{measurement.distance_mm:.1f} mm "
                    f"({measurement.distance_m:.3f} m), "
                    f"samples={measurement.sample_count}, "
                    f"MAD={measurement.mad_mm:.1f} mm"
                )
            elif measurement_error:
                print(f"Sector warning: {measurement_error}")
        return 0
    except (FileNotFoundError, RPLidarSdkError, SectorMeasurementError) as exc:
        payload = {"status": "FAIL", "error": f"{type(exc).__name__}: {exc}"}
        if args.json:
            print(json.dumps(payload, indent=2, ensure_ascii=False))
        else:
            print(f"RPLIDAR FAIL: {payload['error']}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("RPLIDAR test interrupted.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
