from __future__ import annotations

from dataclasses import dataclass
import math


CAMERA_REGIONS = {"left", "center", "centre", "right"}


def wrap_degrees(
    angle_degrees: float,
    *,
    prefer_positive_180: bool = True,
) -> float:
    """Wrap an angle to the RoArm base range [-180, 180]."""
    wrapped = (float(angle_degrees) + 180.0) % 360.0 - 180.0
    if prefer_positive_180 and math.isclose(wrapped, -180.0, abs_tol=1e-9):
        return 180.0
    return wrapped


@dataclass(frozen=True)
class MountTransform:
    """Transform robot/camera-frame targets into the RoArm base frame.

    Robot frame:
      +X points in the camera/robot forward direction.
      +Y points to the camera image's left side.
      +Z points upward.

    ``mount_yaw_degrees=180`` means the RoArm's own +X direction points
    toward the rear of the robot, opposite to the camera.
    """

    mount_yaw_degrees: float = 180.0
    region_yaw_degrees: float = 20.0
    prefer_positive_180: bool = True

    def __post_init__(self) -> None:
        if not 0.0 < self.region_yaw_degrees < 90.0:
            raise ValueError("region_yaw_degrees must be between 0 and 90")

    def robot_to_arm_xyz(
        self,
        x_robot: float,
        y_robot: float,
        z_robot: float,
    ) -> tuple[float, float, float]:
        """Rotate a point from robot coordinates into arm coordinates."""
        theta = math.radians(-self.mount_yaw_degrees)
        cosine = math.cos(theta)
        sine = math.sin(theta)
        x_arm = cosine * x_robot - sine * y_robot
        y_arm = sine * x_robot + cosine * y_robot
        return x_arm, y_arm, z_robot

    def arm_to_robot_xyz(
        self,
        x_arm: float,
        y_arm: float,
        z_arm: float,
    ) -> tuple[float, float, float]:
        """Rotate a point from arm coordinates into robot coordinates."""
        theta = math.radians(self.mount_yaw_degrees)
        cosine = math.cos(theta)
        sine = math.sin(theta)
        x_robot = cosine * x_arm - sine * y_arm
        y_robot = sine * x_arm + cosine * y_arm
        return x_robot, y_robot, z_arm

    def camera_region_to_robot_yaw(self, region: str) -> float:
        """Return target yaw in the robot/camera frame.

        Positive yaw is to camera-left, matching the right-hand rule.
        """
        normalized = self.normalize_region(region)
        if normalized == "left":
            return self.region_yaw_degrees
        if normalized == "right":
            return -self.region_yaw_degrees
        return 0.0

    def camera_region_to_arm_base(self, region: str) -> float:
        """Return the absolute RoArm base angle for a camera region.

        For a 180-degree reverse mount with a 20-degree region offset:
          camera left   -> arm base -160 degrees
          camera center -> arm base +180 degrees
          camera right  -> arm base +160 degrees
        """
        robot_yaw = self.camera_region_to_robot_yaw(region)
        return wrap_degrees(
            robot_yaw - self.mount_yaw_degrees,
            prefer_positive_180=self.prefer_positive_180,
        )

    @staticmethod
    def normalize_region(region: str) -> str:
        normalized = str(region).strip().lower()
        if normalized == "centre":
            normalized = "center"
        if normalized not in {"left", "center", "right"}:
            raise ValueError(
                f"Unsupported camera region {region!r}; "
                "expected left, center/centre, or right"
            )
        return normalized
