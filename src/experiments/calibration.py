from __future__ import annotations

from configuration.loader import ConfigError, RuntimeConfig


def validate_no_motion_calibration_profile(config: RuntimeConfig) -> None:
    modes = {
        name: str(config.get(name, "mode")).lower()
        for name in ("camera", "lidar", "arm")
    }
    expected = {"camera": "real", "lidar": "real", "arm": "mock"}
    if modes != expected:
        raise ConfigError(
            "Calibration capture requires REAL Camera + REAL RPLIDAR + "
            f"MOCK RoArm; received {modes}"
        )

    arm = config.section("arm")
    forbidden = {
        "allow_real_motion": bool(arm.get("allow_real_motion", False)),
        "confirm_clearance": bool(arm.get("confirm_clearance", False)),
        "require_typed_confirmation": bool(
            arm.get("require_typed_confirmation", False)
        ),
    }
    enabled = [name for name, value in forbidden.items() if value]
    if enabled:
        raise ConfigError(
            "Calibration capture must disable RoArm motion gates: "
            + ", ".join(enabled)
        )

    experiment = config.data.get("experiment")
    if not isinstance(experiment, dict):
        raise ConfigError("Calibration capture requires experiment settings")
    if not bool(experiment.get("enabled", False)):
        raise ConfigError("Calibration capture requires experiment.enabled=true")
    if str(experiment.get("profile", "")) != "calibration_capture":
        raise ConfigError(
            "Calibration capture requires experiment.profile=calibration_capture"
        )
    if not bool(experiment.get("require_mock_arm", False)):
        raise ConfigError(
            "Calibration capture requires experiment.require_mock_arm=true"
        )

    manual = config.data.get("manual_arm_test", {})
    if isinstance(manual, dict) and (
        bool(manual.get("enabled", False))
        or bool(manual.get("allow_real_motion", False))
    ):
        raise ConfigError(
            "Calibration capture profile must disable manual_arm_test"
        )
