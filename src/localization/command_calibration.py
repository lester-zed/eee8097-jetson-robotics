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


def _coefficients(
    value: Any,
    name: str,
    *,
    expected_length: int,
) -> tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must contain exactly {expected_length} numbers")
    if len(value) != expected_length:
        raise ValueError(f"{name} must contain exactly {expected_length} numbers")
    return tuple(_finite(item, name) for item in value)


def _origin_xy(value: Any, name: str) -> tuple[float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must contain exactly two numbers")
    if len(value) != 2:
        raise ValueError(f"{name} must contain exactly two numbers")
    return _finite(value[0], name), _finite(value[1], name)


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

    The historical ``affine_xy`` model uses ``[x, y, 1]``. The newer
    ``bilinear_xy`` model adds a bounded ``dx*dy`` interaction term around a
    recorded origin. The bilinear term captures the spatially varying lateral
    residual observed in manually taught grasp-waypoint measurements, while
    preserving the existing fail-closed input/output domain checks.

    This remains an arm-command calibration, not a replacement for the raw
    Camera/LiDAR pose. The raw ``base_link`` estimate remains available for
    recheck logic and logging, while only the planner-facing ``arm_base`` point
    is corrected.
    """

    calibration_id: str
    x_coefficients: tuple[float, ...]
    y_coefficients: tuple[float, ...]
    input_bounds: XYBounds
    output_bounds: XYBounds
    require_translation_corrected_lidar: bool = True
    source: Mapping[str, Any] | None = None
    model: str = "affine_xy"
    origin_x_mm: float = 0.0
    origin_y_mm: float = 0.0

    def __post_init__(self) -> None:
        if not str(self.calibration_id).strip():
            raise ValueError("calibration_id must be non-empty")
        model = str(self.model).strip().lower()
        if model not in {"affine_xy", "bilinear_xy"}:
            raise ValueError(
                "command_calibration.model must be 'affine_xy' or 'bilinear_xy'"
            )
        object.__setattr__(self, "model", model)
        expected_length = 3 if model == "affine_xy" else 4
        object.__setattr__(
            self,
            "x_coefficients",
            _coefficients(
                self.x_coefficients,
                "x_coefficients",
                expected_length=expected_length,
            ),
        )
        object.__setattr__(
            self,
            "y_coefficients",
            _coefficients(
                self.y_coefficients,
                "y_coefficients",
                expected_length=expected_length,
            ),
        )
        object.__setattr__(
            self,
            "origin_x_mm",
            _finite(self.origin_x_mm, "origin_x_mm"),
        )
        object.__setattr__(
            self,
            "origin_y_mm",
            _finite(self.origin_y_mm, "origin_y_mm"),
        )

    @classmethod
    def from_mapping(cls, value: Any) -> "AffineXYCommandCalibration":
        if not isinstance(value, Mapping):
            raise ValueError("command_calibration must be a mapping")
        model = str(value.get("model", "")).strip().lower()
        if model not in {"affine_xy", "bilinear_xy"}:
            raise ValueError(
                "command_calibration.model must be 'affine_xy' or 'bilinear_xy'"
            )
        expected_length = 3 if model == "affine_xy" else 4
        origin_x_mm = 0.0
        origin_y_mm = 0.0
        if model == "bilinear_xy":
            origin_x_mm, origin_y_mm = _origin_xy(
                value.get("origin_mm"),
                "command_calibration.origin_mm",
            )
        try:
            return cls(
                calibration_id=str(value["calibration_id"]),
                x_coefficients=_coefficients(
                    value["x_coefficients"],
                    "command_calibration.x_coefficients",
                    expected_length=expected_length,
                ),
                y_coefficients=_coefficients(
                    value["y_coefficients"],
                    "command_calibration.y_coefficients",
                    expected_length=expected_length,
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
                model=model,
                origin_x_mm=origin_x_mm,
                origin_y_mm=origin_y_mm,
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

        if self.model == "affine_xy":
            ax, ay, bias_x = self.x_coefficients
            bx, by, bias_y = self.y_coefficients
            corrected_x = ax * nominal_arm.x_mm + ay * nominal_arm.y_mm + bias_x
            corrected_y = bx * nominal_arm.x_mm + by * nominal_arm.y_mm + bias_y
        else:
            dx = nominal_arm.x_mm - self.origin_x_mm
            dy = nominal_arm.y_mm - self.origin_y_mm
            interaction = dx * dy
            ax, ay, axy, bias_x = self.x_coefficients
            bx, by, bxy, bias_y = self.y_coefficients
            corrected_x = ax * dx + ay * dy + axy * interaction + bias_x
            corrected_y = bx * dx + by * dy + bxy * interaction + bias_y

        corrected = Point3D(
            x_mm=corrected_x,
            y_mm=corrected_y,
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
        metadata = {
            "applied": True,
            "calibration_id": self.calibration_id,
            "model": self.model,
            "x_coefficients": list(self.x_coefficients),
            "y_coefficients": list(self.y_coefficients),
            "input_bounds_mm": asdict(self.input_bounds),
            "output_bounds_mm": asdict(self.output_bounds),
            "require_translation_corrected_lidar": (
                self.require_translation_corrected_lidar
            ),
            "source": dict(self.source or {}),
        }
        if self.model == "bilinear_xy":
            metadata["origin_mm"] = [self.origin_x_mm, self.origin_y_mm]
        return metadata
