from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(frozen=True)
class Point3D:
    x_mm: float
    y_mm: float
    z_mm: float
    frame_id: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class Detection2D:
    label: str
    confidence: float
    center_x: int
    center_y: int
    bbox: tuple[int, int, int, int]
    frame_width: int
    frame_height: int
    captured_at_s: float

    @property
    def width_px(self) -> int:
        return max(0, int(self.bbox[2]) - int(self.bbox[0]))

    @property
    def height_px(self) -> int:
        return max(0, int(self.bbox[3]) - int(self.bbox[1]))

    @property
    def region(self) -> str:
        left_boundary = self.frame_width / 3.0
        right_boundary = self.frame_width * 2.0 / 3.0
        if self.center_x < left_boundary:
            return "left"
        if self.center_x > right_boundary:
            return "right"
        return "center"

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["region"] = self.region
        payload["width_px"] = self.width_px
        payload["height_px"] = self.height_px
        return payload


@dataclass(frozen=True)
class RangeMeasurement:
    distance_mm: float
    camera_bearing_deg: float
    lidar_bearing_deg: float
    base_bearing_deg: float
    sample_count: int
    median_quality: float
    mad_mm: float
    minimum_mm: float
    maximum_mm: float
    measured_at_s: float
    target_lidar: Point3D | None = None
    target_base_link: Point3D | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class TargetPose:
    label: str
    confidence: float
    target_base_link: Point3D
    target_arm_base: Point3D
    provisional: bool = True
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class CartesianWaypoint:
    name: str
    point: Point3D
    tool_angle_rad: float = 3.14
    speed: float = 0.15

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class GraspPlan:
    target: TargetPose
    waypoints: tuple[CartesianWaypoint, ...]
    gripper_open_rad: float = 2.70
    gripper_closed_rad: float = 3.14
    metadata: dict[str, Any] = field(default_factory=dict)

    def waypoint(self, name: str) -> CartesianWaypoint:
        for item in self.waypoints:
            if item.name == name:
                return item
        raise KeyError(f"waypoint not found: {name}")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ExecutionResult:
    success: bool
    message: str
    completed_waypoints: tuple[str, ...] = ()
    feedback: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class HealthResult:
    name: str
    ok: bool
    summary: str
    details: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
