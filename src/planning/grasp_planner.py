from __future__ import annotations

from dataclasses import dataclass
import math

from pipeline.types import CartesianWaypoint, GraspPlan, Point3D, TargetPose


@dataclass(frozen=True)
class WorkspaceLimits:
    min_x_mm: float = -500.0
    max_x_mm: float = 500.0
    min_y_mm: float = -500.0
    max_y_mm: float = 500.0
    min_z_mm: float = 0.0
    max_z_mm: float = 500.0
    max_radius_mm: float = 600.0


class SimpleTopDownGraspPlanner:
    """Radial pregrasp planner; still requires physical collision validation."""

    def __init__(
        self,
        *,
        approach_distance_mm: float = 80.0,
        pregrasp_height_mm: float = 60.0,
        lift_height_mm: float = 100.0,
        tool_angle_rad: float = 3.14,
        speed: float = 0.15,
        gripper_open_rad: float = 2.70,
        gripper_closed_rad: float = 3.14,
        limits: WorkspaceLimits | None = None,
    ) -> None:
        if approach_distance_mm <= 0.0:
            raise ValueError("approach_distance_mm must be positive")
        self.approach_distance_mm = float(approach_distance_mm)
        self.pregrasp_height_mm = float(pregrasp_height_mm)
        self.lift_height_mm = float(lift_height_mm)
        self.tool_angle_rad = float(tool_angle_rad)
        self.speed = float(speed)
        self.gripper_open_rad = float(gripper_open_rad)
        self.gripper_closed_rad = float(gripper_closed_rad)
        self.limits = limits or WorkspaceLimits()

    def plan(self, target: TargetPose) -> GraspPlan:
        grasp = target.target_arm_base
        if grasp.frame_id != "arm_base":
            raise ValueError("grasp planning requires an arm_base target")

        horizontal_radius = math.hypot(grasp.x_mm, grasp.y_mm)
        if horizontal_radius <= self.approach_distance_mm:
            raise ValueError(
                "target is too close to arm origin for configured approach distance"
            )
        unit_x = grasp.x_mm / horizontal_radius
        unit_y = grasp.y_mm / horizontal_radius
        pregrasp = Point3D(
            grasp.x_mm - self.approach_distance_mm * unit_x,
            grasp.y_mm - self.approach_distance_mm * unit_y,
            grasp.z_mm + self.pregrasp_height_mm,
            "arm_base",
        )
        lift = Point3D(
            grasp.x_mm,
            grasp.y_mm,
            grasp.z_mm + self.lift_height_mm,
            "arm_base",
        )
        retreat = Point3D(
            pregrasp.x_mm,
            pregrasp.y_mm,
            lift.z_mm,
            "arm_base",
        )

        points = (
            ("pregrasp", pregrasp),
            ("grasp", grasp),
            ("lift", lift),
            ("retreat", retreat),
        )
        for name, point in points:
            self._validate_point(name, point)

        return GraspPlan(
            target=target,
            waypoints=tuple(
                CartesianWaypoint(
                    name=name,
                    point=point,
                    tool_angle_rad=self.tool_angle_rad,
                    speed=self.speed,
                )
                for name, point in points
            ),
            gripper_open_rad=self.gripper_open_rad,
            gripper_closed_rad=self.gripper_closed_rad,
            metadata={
                "planner": "radial_top_down_v2",
                "requires_collision_validation": True,
                "firmware_t104_performs_inverse_kinematics": True,
            },
        )

    def _validate_point(self, name: str, point: Point3D) -> None:
        limits = self.limits
        if not limits.min_x_mm <= point.x_mm <= limits.max_x_mm:
            raise ValueError(f"{name} X outside workspace")
        if not limits.min_y_mm <= point.y_mm <= limits.max_y_mm:
            raise ValueError(f"{name} Y outside workspace")
        if not limits.min_z_mm <= point.z_mm <= limits.max_z_mm:
            raise ValueError(f"{name} Z outside workspace")
        radius = math.sqrt(point.x_mm**2 + point.y_mm**2 + point.z_mm**2)
        if radius > limits.max_radius_mm:
            raise ValueError(f"{name} radius outside workspace")
