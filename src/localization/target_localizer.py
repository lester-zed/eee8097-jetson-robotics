from __future__ import annotations

import math

from interfaces.transform import BaseToArmTransformPort
from pipeline.types import Detection2D, Point3D, RangeMeasurement, TargetPose


class PlanarTargetLocalizer:
    """2-D LiDAR X/Y plus configured grasp-point height."""

    def __init__(
        self,
        *,
        base_to_arm: BaseToArmTransformPort,
        target_z_mm: float,
        calibration_approved: bool = False,
    ) -> None:
        self.base_to_arm = base_to_arm
        self.target_z_mm = float(target_z_mm)
        self.calibration_approved = bool(calibration_approved)

    def localize(self, detection: Detection2D, measurement: RangeMeasurement) -> TargetPose:
        if measurement.distance_mm <= 0.0:
            raise ValueError("range distance must be positive")

        if measurement.target_base_link is not None:
            source = measurement.target_base_link
            target_base = Point3D(
                x_mm=source.x_mm,
                y_mm=source.y_mm,
                z_mm=self.target_z_mm,
                frame_id="base_link",
            )
            method = "translation_corrected_lidar_xy_plus_fixed_height_v2"
        else:
            bearing_rad = math.radians(measurement.base_bearing_deg)
            target_base = Point3D(
                x_mm=measurement.distance_mm * math.cos(bearing_rad),
                y_mm=measurement.distance_mm * math.sin(bearing_rad),
                z_mm=self.target_z_mm,
                frame_id="base_link",
            )
            method = "planar_lidar_plus_fixed_height_v1"

        target_arm = self.base_to_arm.base_to_arm(target_base)
        return TargetPose(
            label=detection.label,
            confidence=detection.confidence,
            target_base_link=target_base,
            target_arm_base=target_arm,
            provisional=not self.calibration_approved,
            metadata={
                "localization_method": method,
                "camera_bearing_deg": measurement.camera_bearing_deg,
                "base_bearing_deg": measurement.base_bearing_deg,
                "lidar_bearing_deg": measurement.lidar_bearing_deg,
                "lidar_scan_plane_must_intersect_target": True,
                "target_z_is_configured_not_measured": True,
            },
        )
