from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Mapping, Sequence

from pipeline.types import Point3D


class CalibrationDomainError(ValueError):
    """Raised when a target is outside the measured calibration domain."""


def _finite(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _coefficients(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must contain exactly three numbers")
    if len(value) != 3:
        raise ValueError(f"{name} must contain exactly three numbers")
    return tuple(_finite(item, name) for item in value)  # type: ignore[return-value]


@dataclass(frozen=True)
class XYBounds:
    min_x_mm: float
    max_x_mm: float
    min_y_mm: float
    max_y_mm: float

    def __post_init__(self) -> None:
        for name in ("min_x_mm", "max_x_mm", "min_y_mm", "max_y_mm"):
            object.__setattr__(self, name, _finite(getattr(self, name), name))
        if self.min_x_mm >= self.max_x_mm:
            raise ValueError("min_x_mm must be smaller than max_x_mm")
        if self.min_y_mm >= self.max_y_mm:
            raise ValueError("min_y_mm must be smaller than max_y_mm")

    @classmethod
    def from_mapping(cls, value: Any, name: str) -> "XYBounds":
        if not isinstance(value, Mapping):
            raise ValueError(f"{name} must be a mapping")
        try:
            return cls(
                min_x_mm=value["min_x_mm"],
                max_x_mm=value["max_x_mm"],
                min_y_mm=value["min_y_mm"],
                max_y_mm=value["max_y_mm"],
            )
        except KeyError as exc:
            raise ValueError(f"{name} is missing {exc.args[0]}") from exc

    def contains(self, point: Point3D) -> bool:
        return (
            self.min_x_mm <= point.x_mm <= self.max_x_mm
            and self.min_y_mm <= point.y_mm <= self.max_y_mm
        )

    def describe(self) -> str:
        return (
            f"X=[{self.min_x_mm:.1f}, {self.max_x_mm:.1f}] mm, "
            f"Y=[{self.min_y_mm:.1f}, {self.max_y_mm:.1f}] mm"
        )


@dataclass(frozen=True)
class AffineXYCommandCalibration:
    """Map nominal arm coordinates to measured RoArm gripper commands.

    This is deliberately an arm-command calibration, not a replacement for the
    raw Camera/LiDAR pose. The raw ``base_link`` estimate remains available for
    recheck logic and logging, while only the planner-facing ``arm_base`` point
    is corrected.
    """

    calibration_id: str
    x_coefficients: tuple[float, float, float]
    y_coefficients: tuple[float, float, float]
    input_bounds: XYBounds
    output_bounds: XYBounds
    require_translation_corrected_lidar: bool = True
    source: Mapping[str, Any] | None = None

    def __post_init__(self) -> None:
        if not str(self.calibration_id).strip():
            raise ValueError("calibration_id must be non-empty")
        object.__setattr__(
            self,
            "x_coefficients",
            _coefficients(self.x_coefficients, "x_coefficients"),
        )
        object.__setattr__(
            self,
            "y_coefficients",
            _coefficients(self.y_coefficients, "y_coefficients"),
        )

    @classmethod
    def from_mapping(cls, value: Any) -> "AffineXYCommandCalibration":
        if not isinstance(value, Mapping):
            raise ValueError("command_calibration must be a mapping")
        model = str(value.get("model", "")).strip().lower()
        if model != "affine_xy":
            raise ValueError("command_calibration.model must be 'affine_xy'")
        try:
            return cls(
                calibration_id=str(value["calibration_id"]),
                x_coefficients=_coefficients(
                    value["x_coefficients"],
                    "command_calibration.x_coefficients",
                ),
                y_coefficients=_coefficients(
                    value["y_coefficients"],
                    "command_calibration.y_coefficients",
                ),
                input_bounds=XYBounds.from_mapping(
                    value["input_bounds_mm"],
                    "command_calibration.input_bounds_mm",
                ),
                output_bounds=XYBounds.from_mapping(
                    value["output_bounds_mm"],
                    "command_calibration.output_bounds_mm",
                ),
                require_translation_corrected_lidar=bool(
                    value.get("require_translation_corrected_lidar", True)
                ),
                source=(
                    dict(value["source"])
                    if isinstance(value.get("source"), Mapping)
                    else None
                ),
            )
        except KeyError as exc:
            raise ValueError(
                f"command_calibration is missing {exc.args[0]}"
            ) from exc

    def apply(self, nominal_arm: Point3D) -> Point3D:
        if nominal_arm.frame_id != "arm_base":
            raise ValueError("command calibration requires an arm_base point")
        if not self.input_bounds.contains(nominal_arm):
            raise CalibrationDomainError(
                "nominal arm target is outside measured command-calibration "
                f"input bounds ({self.input_bounds.describe()}): "
                f"({nominal_arm.x_mm:.1f}, {nominal_arm.y_mm:.1f}) mm"
            )

        ax, ay, bias_x = self.x_coefficients
        bx, by, bias_y = self.y_coefficients
        corrected = Point3D(
            x_mm=ax * nominal_arm.x_mm + ay * nominal_arm.y_mm + bias_x,
            y_mm=bx * nominal_arm.x_mm + by * nominal_arm.y_mm + bias_y,
            z_mm=nominal_arm.z_mm,
            frame_id="arm_base",
        )
        if not self.output_bounds.contains(corrected):
            raise CalibrationDomainError(
                "corrected RoArm target is outside measured command bounds "
                f"({self.output_bounds.describe()}): "
                f"({corrected.x_mm:.1f}, {corrected.y_mm:.1f}) mm"
            )
        return corrected

    def to_metadata(self) -> dict[str, Any]:
        return {
            "applied": True,
            "calibration_id": self.calibration_id,
            "model": "affine_xy",
            "x_coefficients": list(self.x_coefficients),
            "y_coefficients": list(self.y_coefficients),
            "input_bounds_mm": asdict(self.input_bounds),
            "output_bounds_mm": asdict(self.output_bounds),
            "require_translation_corrected_lidar": (
                self.require_translation_corrected_lidar
            ),
            "source": dict(self.source or {}),
        }
