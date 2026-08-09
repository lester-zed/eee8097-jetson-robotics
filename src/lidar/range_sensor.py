from __future__ import annotations

from dataclasses import dataclass
from threading import Event
import math
import statistics
import time
from typing import Any, Iterable

from pipeline.types import Detection2D, HealthResult, Point3D, RangeMeasurement


def wrap_360(angle_deg: float) -> float:
    return float(angle_deg) % 360.0


def circular_delta_degrees(angle_deg: float, centre_deg: float) -> float:
    return (float(angle_deg) - float(centre_deg) + 180.0) % 360.0 - 180.0


def camera_pixel_to_bearing_deg(*, pixel_x: float, fx_px: float, cx_px: float) -> float:
    """Horizontal target bearing; positive means Camera-left."""
    if float(fx_px) <= 0.0:
        raise ValueError("fx_px must be positive")
    return math.degrees(math.atan2(float(cx_px) - float(pixel_x), float(fx_px)))


@dataclass(frozen=True)
class _ProjectedScanPoint:
    raw_angle_deg: float
    lidar_distance_mm: float
    quality: int
    base_bearing_deg: float
    camera_bearing_in_base_deg: float
    x_base_mm: float
    y_base_mm: float


class CameraGuidedRPLidarAdapter:
    """Translation-aware 2-D Camera/LiDAR target-sector matcher.

    Each valid LiDAR point is converted into base_link using the calibrated
    LiDAR origin and angle convention. The point is then viewed from the Camera
    origin and retained when its bearing agrees with the YOLO-selected ray.
    This corrects the main horizontal parallax ignored by the original V1.

    It still assumes the 2-D LiDAR scan plane intersects the physical target.
    """

    def __init__(
        self,
        *,
        device: str = "/dev/ttyRPLIDAR",
        sdk_binary: str | None = None,
        baudrate: int = 460800,
        camera_fx_px: float,
        camera_cx_px: float,
        camera_yaw_in_base_deg: float = 0.0,
        camera_origin_in_base_mm: tuple[float, float, float] = (0.0, 0.0, 0.0),
        lidar_origin_in_base_mm: tuple[float, float, float] = (0.0, 0.0, 0.0),
        lidar_front_angle_deg: float = 0.0,
        lidar_angle_sign: int = 1,
        half_width_deg: float = 5.0,
        scan_count: int = 3,
        timeout_seconds: float = 12.0,
        min_range_mm: float = 50.0,
        max_range_mm: float = 12000.0,
        min_quality: int = 1,
        min_points: int = 3,
        outlier_mm: float = 150.0,
    ) -> None:
        from lidar.rplidar_distance import RPLidarSdkReader

        if int(lidar_angle_sign) not in {-1, 1}:
            raise ValueError("lidar_angle_sign must be +1 or -1")
        self.device = str(device)
        self.camera_fx_px = float(camera_fx_px)
        self.camera_cx_px = float(camera_cx_px)
        self.camera_yaw_in_base_deg = float(camera_yaw_in_base_deg)
        self.camera_origin = tuple(float(v) for v in camera_origin_in_base_mm)
        self.lidar_origin = tuple(float(v) for v in lidar_origin_in_base_mm)
        self.lidar_front_angle_deg = float(lidar_front_angle_deg)
        self.lidar_angle_sign = int(lidar_angle_sign)
        self.half_width_deg = float(half_width_deg)
        self.scan_count = int(scan_count)
        self.timeout_seconds = float(timeout_seconds)
        self.min_range_mm = float(min_range_mm)
        self.max_range_mm = float(max_range_mm)
        self.min_quality = int(min_quality)
        self.min_points = int(min_points)
        self.outlier_mm = float(outlier_mm)
        self._reader: Any = RPLidarSdkReader(
            device=self.device,
            baudrate=int(baudrate),
            sdk_binary=sdk_binary,
        )

    def raw_lidar_angle_to_base_bearing(self, raw_angle_deg: float) -> float:
        delta = circular_delta_degrees(raw_angle_deg, self.lidar_front_angle_deg)
        return self.lidar_angle_sign * delta

    def expected_raw_lidar_angle(self, base_bearing_deg: float) -> float:
        return wrap_360(
            self.lidar_front_angle_deg
            + self.lidar_angle_sign * float(base_bearing_deg)
        )

    def _project_points(self, points: Iterable[Any]) -> list[_ProjectedScanPoint]:
        projected: list[_ProjectedScanPoint] = []
        camera_x, camera_y, _ = self.camera_origin
        lidar_x, lidar_y, _ = self.lidar_origin
        for point in points:
            distance = float(point.distance_mm)
            quality = int(point.quality)
            if not self.min_range_mm <= distance <= self.max_range_mm:
                continue
            if quality < self.min_quality:
                continue
            base_bearing = self.raw_lidar_angle_to_base_bearing(point.angle_deg)
            theta = math.radians(base_bearing)
            x_base = lidar_x + distance * math.cos(theta)
            y_base = lidar_y + distance * math.sin(theta)
            camera_view_bearing = math.degrees(
                math.atan2(y_base - camera_y, x_base - camera_x)
            )
            projected.append(
                _ProjectedScanPoint(
                    raw_angle_deg=float(point.angle_deg),
                    lidar_distance_mm=distance,
                    quality=quality,
                    base_bearing_deg=base_bearing,
                    camera_bearing_in_base_deg=camera_view_bearing,
                    x_base_mm=x_base,
                    y_base_mm=y_base,
                )
            )
        return projected

    def measure_target(self, detection: Detection2D, abort_event: Event) -> RangeMeasurement:
        if abort_event.is_set():
            raise InterruptedError("RPLIDAR measurement aborted before scan")

        camera_bearing = camera_pixel_to_bearing_deg(
            pixel_x=detection.center_x,
            fx_px=self.camera_fx_px,
            cx_px=self.camera_cx_px,
        )
        desired_base_bearing = self.camera_yaw_in_base_deg + camera_bearing
        expected_raw_angle = self.expected_raw_lidar_angle(desired_base_bearing)

        capture = self._reader.capture_scans(
            scan_count=self.scan_count,
            timeout_seconds=self.timeout_seconds,
        )
        if abort_event.is_set():
            raise InterruptedError("RPLIDAR measurement aborted after scan")

        candidates = [
            item for item in self._project_points(capture.valid_points)
            if abs(circular_delta_degrees(
                item.camera_bearing_in_base_deg,
                desired_base_bearing,
            )) <= self.half_width_deg
        ]
        if len(candidates) < self.min_points:
            raise RuntimeError(
                "Not enough translation-corrected RPLIDAR points on the Camera ray: "
                f"received {len(candidates)}, need {self.min_points}; "
                f"camera_bearing={desired_base_bearing:.2f} deg"
            )

        median_distance = float(statistics.median(
            item.lidar_distance_mm for item in candidates
        ))
        inliers = [
            item for item in candidates
            if abs(item.lidar_distance_mm - median_distance) <= self.outlier_mm
        ]
        if len(inliers) >= self.min_points:
            candidates = inliers

        distances = [item.lidar_distance_mm for item in candidates]
        qualities = [item.quality for item in candidates]
        x_base = float(statistics.median(item.x_base_mm for item in candidates))
        y_base = float(statistics.median(item.y_base_mm for item in candidates))
        lidar_x, lidar_y, lidar_z = self.lidar_origin
        target_lidar = Point3D(
            x_mm=x_base - lidar_x,
            y_mm=y_base - lidar_y,
            z_mm=0.0,
            frame_id="lidar_frame_aligned",
        )
        target_base = Point3D(x_base, y_base, lidar_z, "base_link")
        selected_raw_angle = float(statistics.median(
            item.raw_angle_deg for item in candidates
        ))
        selected_base_bearing = math.degrees(math.atan2(y_base, x_base))
        absolute_deviations = [abs(value - statistics.median(distances)) for value in distances]

        return RangeMeasurement(
            distance_mm=float(statistics.median(distances)),
            camera_bearing_deg=camera_bearing,
            lidar_bearing_deg=selected_raw_angle,
            base_bearing_deg=selected_base_bearing,
            sample_count=len(candidates),
            median_quality=float(statistics.median(qualities)),
            mad_mm=float(statistics.median(absolute_deviations)),
            minimum_mm=float(min(distances)),
            maximum_mm=float(max(distances)),
            measured_at_s=time.monotonic(),
            target_lidar=target_lidar,
            target_base_link=target_base,
            metadata={
                "device": capture.device,
                "completed_scans": capture.completed_scans,
                "valid_points": len(capture.valid_points),
                "health_status": capture.health_status,
                "expected_raw_lidar_angle_deg": expected_raw_angle,
                "translation_parallax_corrected_2d": True,
                "scan_plane_must_intersect_target": True,
            },
        )

    def health_check(self) -> HealthResult:
        return HealthResult(
            "rplidar",
            True,
            "RPLidarSdkReader configured; motor starts only during measurement",
            {
                "device": self.device,
                "front_angle_deg": self.lidar_front_angle_deg,
                "angle_sign": self.lidar_angle_sign,
            },
        )

    def close(self) -> None:
        return None
