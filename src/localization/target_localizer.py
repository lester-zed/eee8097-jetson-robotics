from __future__ import annotations

import math

from interfaces.transform import BaseToArmTransformPort
from localization.command_calibration import AffineXYCommandCalibration
from pipeline.types import Detection2D, Point3D, RangeMeasurement, TargetPose


class PlanarTargetLocalizer:
    """2-D LiDAR X/Y plus configured grasp-point height."""

    def __init__(
        self,
        *,
        base_to_arm: BaseToArmTransformPort,
        target_z_mm: float,
        calibration_approved: bool = False,
        target_z_approved: bool = False,
        arm_command_calibration: AffineXYCommandCalibration | None = None,
    ) -> None:
        self.base_to_arm = base_to_arm
        self.target_z_mm = float(target_z_mm)
        self.calibration_approved = bool(calibration_approved)
        self.target_z_approved = bool(target_z_approved)
        self.arm_command_calibration = arm_command_calibration

    @property
    def execution_calibration_ready(self) -> bool:
        return self.calibration_approved and self.target_z_approved

    def localize(
        self,
        detection: Detection2D,
        measurement: RangeMeasurement,
    ) -> TargetPose:
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
            if (
                self.arm_command_calibration is not None
                and self.arm_command_calibration.require_translation_corrected_lidar
            ):
                raise ValueError(
                    "measured command calibration requires a "
                    "translation-corrected LiDAR target"
                )
            bearing_rad = math.radians(measurement.base_bearing_deg)
            target_base = Point3D(
                x_mm=measurement.distance_mm * math.cos(bearing_rad),
                y_mm=measurement.distance_mm * math.sin(bearing_rad),
                z_mm=self.target_z_mm,
                frame_id="base_link",
            )
            method = "planar_lidar_plus_fixed_height_v1"

        nominal_target_arm = self.base_to_arm.base_to_arm(target_base)
        if self.arm_command_calibration is not None:
            target_arm = self.arm_command_calibration.apply(nominal_target_arm)
            command_calibration = self.arm_command_calibration.to_metadata()
            method += f"_plus_{self.arm_command_calibration.model}_arm_command_v2"
        else:
            target_arm = nominal_target_arm
            command_calibration = {"applied": False}
        return TargetPose(
            label=detection.label,
            confidence=detection.confidence,
            target_base_link=target_base,
            target_arm_base=target_arm,
            provisional=not self.execution_calibration_ready,
            metadata={
                "localization_method": method,
                "camera_bearing_deg": measurement.camera_bearing_deg,
                "base_bearing_deg": measurement.base_bearing_deg,
                "lidar_bearing_deg": measurement.lidar_bearing_deg,
                "lidar_scan_plane_must_intersect_target": True,
                "target_z_is_configured_not_measured": True,
                "xy_calibration_approved": self.calibration_approved,
                "target_z_approved": self.target_z_approved,
                "raw_target_base_link": target_base.to_dict(),
                "nominal_target_arm_base": nominal_target_arm.to_dict(),
                "commanded_target_arm_base": target_arm.to_dict(),
                "arm_command_calibration": command_calibration,
            },
        )
