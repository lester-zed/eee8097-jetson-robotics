from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any
import yaml


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class RuntimeConfig:
    path: Path
    data: dict[str, Any]

    def section(self, name: str) -> dict[str, Any]:
        value = self.data.get(name)
        if not isinstance(value, dict):
            raise ConfigError(f"Missing or invalid config section: {name}")
        return value

    def get(self, section: str, key: str, default: Any = None) -> Any:
        return self.section(section).get(key, default)

    def vector3(self, section: str, key: str) -> tuple[float, float, float]:
        value = self.get(section, key)
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ConfigError(f"{section}.{key} must contain exactly 3 numbers")
        try:
            return float(value[0]), float(value[1]), float(value[2])
        except (TypeError, ValueError) as exc:
            raise ConfigError(f"{section}.{key} must contain numeric values") from exc

    def validate(self) -> None:
        required_sections = (
            "app", "camera", "lidar", "localization", "arm_mount",
            "planner", "workspace", "arm", "pipeline",
        )
        for name in required_sections:
            self.section(name)

        for section in ("camera", "lidar", "arm"):
            mode = str(self.get(section, "mode", "")).strip().lower()
            if mode not in {"mock", "real"}:
                raise ConfigError(f"{section}.mode must be 'mock' or 'real'")

        if not str(self.get("app", "target", "")).strip():
            raise ConfigError("app.target must be a non-empty class label")

        confidence = float(self.get("camera", "confidence", 0.55))
        if not 0.0 < confidence <= 1.0:
            raise ConfigError("camera.confidence must be in (0, 1]")

        if int(self.get("camera", "stable_frames", 3)) < 1:
            raise ConfigError("camera.stable_frames must be >= 1")

        if int(self.get("camera", "stability_tolerance_px", 40)) < 1:
            raise ConfigError("camera.stability_tolerance_px must be >= 1")

        if int(self.get("camera", "imgsz", 640)) <= 0:
            raise ConfigError("camera.imgsz must be positive")

        if str(self.get("camera", "mode")).lower() == "real":
            if not str(self.get("camera", "model_path", "")).strip():
                raise ConfigError("camera.mode=real requires camera.model_path")

        fx = float(self.get("camera", "fx_px"))
        fy = float(self.get("camera", "fy_px"))
        if fx <= 0.0 or fy <= 0.0:
            raise ConfigError("camera.fx_px and camera.fy_px must be positive")

        sign = int(self.get("lidar", "angle_sign", 1))
        if sign not in {-1, 1}:
            raise ConfigError("lidar.angle_sign must be +1 or -1")

        self.vector3("camera", "origin_in_base_mm")
        self.vector3("lidar", "origin_in_base_mm")
        self.vector3("arm_mount", "origin_in_base_mm")

        if str(self.get("arm", "mode")).lower() == "real":
            if not bool(self.get("arm", "allow_real_motion", False)):
                raise ConfigError("arm.mode=real requires arm.allow_real_motion=true")
            if not bool(self.get("arm", "confirm_clearance", False)):
                raise ConfigError("arm.mode=real requires arm.confirm_clearance=true")
            if not bool(self.get("localization", "calibration_approved", False)):
                raise ConfigError("arm.mode=real requires localization.calibration_approved=true")


def load_runtime_config(path: str | Path) -> RuntimeConfig:
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"Configuration file not found: {config_path}")
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ConfigError("Configuration root must be a YAML mapping")
    config = RuntimeConfig(path=config_path, data=payload)
    config.validate()
    return config
