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


def _local_anchor_rows(
    value: Any,
    name: str,
) -> tuple[tuple[float, float, float, float], ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must be a sequence of [x, y, coeff_x, coeff_y]")
    rows: list[tuple[float, float, float, float]] = []
    for index, row in enumerate(value):
        if not isinstance(row, Sequence) or isinstance(row, (str, bytes)):
            raise ValueError(
                f"{name}[{index}] must contain [x, y, coeff_x, coeff_y]"
            )
        if len(row) != 4:
            raise ValueError(
                f"{name}[{index}] must contain exactly four numbers"
            )
        rows.append(
            tuple(
                _finite(item, f"{name}[{index}]")
                for item in row
            )  # type: ignore[arg-type]
        )
    if not rows:
        raise ValueError(f"{name} must contain at least one anchor")
    return tuple(rows)


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

    Supported models:

    ``affine_xy``
        Historical first-order map using ``[x, y, 1]``.

    ``bilinear_xy``
        Global map using ``[dx, dy, dx*dy, 1]`` around a recorded origin.

    ``bilinear_local_xy``
        The same global bilinear map plus a compact, bounded local residual
        correction learned from manually taught grasp-waypoint anchors. The
        compact Wendland-C2 basis decays to zero outside its support radius, so
        measured anchors can be matched closely without replacing the global
        model in unsupported regions.

    This remains a command-space calibration. The raw Camera/LiDAR target is
    preserved separately for recheck logic and logging.
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
    local_support_radius_mm: float = 0.0
    local_residual_anchors: tuple[tuple[float, float, float, float], ...] = ()

    def __post_init__(self) -> None:
        if not str(self.calibration_id).strip():
            raise ValueError("calibration_id must be non-empty")
        model = str(self.model).strip().lower()
        allowed_models = {"affine_xy", "bilinear_xy", "bilinear_local_xy"}
        if model not in allowed_models:
            raise ValueError(
                "command_calibration.model must be 'affine_xy', "
                "'bilinear_xy', or 'bilinear_local_xy'"
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

        if model == "bilinear_local_xy":
            radius = _finite(
                self.local_support_radius_mm,
                "local_support_radius_mm",
            )
            if radius <= 0.0:
                raise ValueError(
                    "bilinear_local_xy requires local_support_radius_mm > 0"
                )
            anchors = _local_anchor_rows(
                self.local_residual_anchors,
                "local_residual_anchors",
            )
            object.__setattr__(self, "local_support_radius_mm", radius)
            object.__setattr__(self, "local_residual_anchors", anchors)
        else:
            object.__setattr__(self, "local_support_radius_mm", 0.0)
            object.__setattr__(self, "local_residual_anchors", ())

    @classmethod
    def from_mapping(cls, value: Any) -> "AffineXYCommandCalibration":
        if not isinstance(value, Mapping):
            raise ValueError("command_calibration must be a mapping")
        model = str(value.get("model", "")).strip().lower()
        allowed_models = {"affine_xy", "bilinear_xy", "bilinear_local_xy"}
        if model not in allowed_models:
            raise ValueError(
                "command_calibration.model must be 'affine_xy', "
                "'bilinear_xy', or 'bilinear_local_xy'"
            )

        expected_length = 3 if model == "affine_xy" else 4
        origin_x_mm = 0.0
        origin_y_mm = 0.0
        if model in {"bilinear_xy", "bilinear_local_xy"}:
            origin_x_mm, origin_y_mm = _origin_xy(
                value.get("origin_mm"),
                "command_calibration.origin_mm",
            )

        local_support_radius_mm = 0.0
        local_residual_anchors: tuple[
            tuple[float, float, float, float], ...
        ] = ()
        if model == "bilinear_local_xy":
            local = value.get("local_residual")
            if not isinstance(local, Mapping):
                raise ValueError(
                    "bilinear_local_xy requires command_calibration.local_residual"
                )
            basis = str(local.get("basis", "")).strip().lower()
            if basis != "wendland_c2":
                raise ValueError(
                    "command_calibration.local_residual.basis must be "
                    "'wendland_c2'"
                )
            local_support_radius_mm = _finite(
                local.get("support_radius_mm"),
                "command_calibration.local_residual.support_radius_mm",
            )
            local_residual_anchors = _local_anchor_rows(
                local.get("anchors"),
                "command_calibration.local_residual.anchors",
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
                local_support_radius_mm=local_support_radius_mm,
                local_residual_anchors=local_residual_anchors,
            )
        except KeyError as exc:
            raise ValueError(
                f"command_calibration is missing {exc.args[0]}"
            ) from exc

    @staticmethod
    def _wendland_c2(distance_mm: float, support_radius_mm: float) -> float:
        q = float(distance_mm) / float(support_radius_mm)
        if q >= 1.0:
            return 0.0
        one_minus_q = 1.0 - q
        return one_minus_q**4 * (4.0 * q + 1.0)

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
            corrected_x = (
                ax * nominal_arm.x_mm
                + ay * nominal_arm.y_mm
                + bias_x
            )
            corrected_y = (
                bx * nominal_arm.x_mm
                + by * nominal_arm.y_mm
                + bias_y
            )
        else:
            dx = nominal_arm.x_mm - self.origin_x_mm
            dy = nominal_arm.y_mm - self.origin_y_mm
            interaction = dx * dy
            ax, ay, axy, bias_x = self.x_coefficients
            bx, by, bxy, bias_y = self.y_coefficients
            corrected_x = (
                ax * dx + ay * dy + axy * interaction + bias_x
            )
            corrected_y = (
                bx * dx + by * dy + bxy * interaction + bias_y
            )

            if self.model == "bilinear_local_xy":
                for (
                    anchor_x,
                    anchor_y,
                    coefficient_x,
                    coefficient_y,
                ) in self.local_residual_anchors:
                    distance = math.hypot(
                        nominal_arm.x_mm - anchor_x,
                        nominal_arm.y_mm - anchor_y,
                    )
                    weight = self._wendland_c2(
                        distance,
                        self.local_support_radius_mm,
                    )
                    corrected_x += weight * coefficient_x
                    corrected_y += weight * coefficient_y

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
        if self.model in {"bilinear_xy", "bilinear_local_xy"}:
            metadata["origin_mm"] = [self.origin_x_mm, self.origin_y_mm]
        if self.model == "bilinear_local_xy":
            metadata["local_residual"] = {
                "basis": "wendland_c2",
                "support_radius_mm": self.local_support_radius_mm,
                "anchor_count": len(self.local_residual_anchors),
                "anchors": [list(row) for row in self.local_residual_anchors],
            }
        return metadata
