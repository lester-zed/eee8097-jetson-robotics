"""Coordinate conversion for a camera-forward, arm-rearward installation."""

from __future__ import annotations

from dataclasses import dataclass
import math


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
class ReverseMountTransform:
    """Transform camera/robot-frame targets into the RoArm frame.

    Robot frame:
      +X is the camera-facing forward direction.
      +Y is camera-left.
      +Z is upward.

    The default mount yaw of 180 degrees means that the arm's own +X axis
    points toward the rear of the robot.
    """

    mount_yaw_degrees: float = 180.0
    region_yaw_degrees: float = 20.0
    prefer_positive_180: bool = True

    def __post_init__(self) -> None:
        if not 0.0 < float(self.region_yaw_degrees) < 90.0:
            raise ValueError("region_yaw_degrees must be between 0 and 90")

    def robot_to_arm_xyz(
        self,
        x_robot: float,
        y_robot: float,
        z_robot: float,
    ) -> tuple[float, float, float]:
        """Rotate a point from the robot frame into the arm frame."""
        theta = math.radians(-float(self.mount_yaw_degrees))
        cosine = math.cos(theta)
        sine = math.sin(theta)
        x_arm = cosine * float(x_robot) - sine * float(y_robot)
        y_arm = sine * float(x_robot) + cosine * float(y_robot)
        return x_arm, y_arm, float(z_robot)

    def camera_region_to_arm_base(self, region: str) -> float:
        """Return an absolute base target for left/center/right detection.

        With the default 180-degree mount and 20-degree region offset:
          camera left   -> arm base -160 degrees
          camera center -> arm base +180 degrees
          camera right  -> arm base +160 degrees
        """
        normalized = self.normalize_region(region)
        if normalized == "left":
            camera_yaw = float(self.region_yaw_degrees)
        elif normalized == "right":
            camera_yaw = -float(self.region_yaw_degrees)
        else:
            camera_yaw = 0.0

        return wrap_degrees(
            camera_yaw - float(self.mount_yaw_degrees),
            prefer_positive_180=self.prefer_positive_180,
        )

    @staticmethod
    def normalize_region(region: str) -> str:
        normalized = str(region).strip().lower()
        if normalized == "centre":
            normalized = "center"
        if normalized not in {"left", "center", "right"}:
            raise ValueError(
                f"Unsupported camera region {region!r}; expected "
                "left, center/centre, or right"
            )
        return normalized
