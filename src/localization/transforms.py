from __future__ import annotations

import math

from pipeline.types import Point3D


class ExistingArmMountTransformAdapter:
    """base_link → arm_base translation plus yaw rotation.

    In the real repository this reuses ``arm_control.coordinate_frames.MountTransform``.
    A mathematically equivalent local fallback keeps the standalone package testable.
    """

    def __init__(
        self,
        *,
        arm_origin_x_mm: float = 0.0,
        arm_origin_y_mm: float = 0.0,
        arm_origin_z_mm: float = 0.0,
        mount_yaw_degrees: float = 180.0,
    ) -> None:
        self.arm_origin_x_mm = float(arm_origin_x_mm)
        self.arm_origin_y_mm = float(arm_origin_y_mm)
        self.arm_origin_z_mm = float(arm_origin_z_mm)
        self.mount_yaw_degrees = float(mount_yaw_degrees)
        try:
            from arm_control.coordinate_frames import MountTransform

            self._transform = MountTransform(
                mount_yaw_degrees=self.mount_yaw_degrees,
            )
        except ModuleNotFoundError:
            self._transform = None

    def base_to_arm(self, point: Point3D) -> Point3D:
        if point.frame_id != "base_link":
            raise ValueError(f"expected base_link point, received {point.frame_id!r}")

        relative_x = point.x_mm - self.arm_origin_x_mm
        relative_y = point.y_mm - self.arm_origin_y_mm
        relative_z = point.z_mm - self.arm_origin_z_mm
        if self._transform is not None:
            x_arm, y_arm, z_arm = self._transform.robot_to_arm_xyz(
                relative_x,
                relative_y,
                relative_z,
            )
        else:
            theta = math.radians(-self.mount_yaw_degrees)
            cosine = math.cos(theta)
            sine = math.sin(theta)
            x_arm = cosine * relative_x - sine * relative_y
            y_arm = sine * relative_x + cosine * relative_y
            z_arm = relative_z
        return Point3D(x_arm, y_arm, z_arm, "arm_base")


class IdentityBaseToArmTransform:
    """Test transform used only when physical mounting data is not required."""

    def base_to_arm(self, point: Point3D) -> Point3D:
        if point.frame_id != "base_link":
            raise ValueError("Identity transform expects base_link")
        return Point3D(point.x_mm, point.y_mm, point.z_mm, "arm_base")
