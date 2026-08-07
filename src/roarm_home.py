from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time
from typing import Any, Callable

import yaml


SOURCE_ROOT = Path(__file__).resolve().parent
DEFAULT_HOME_CONFIG = SOURCE_ROOT / "configs/roarm_home.yaml"


class HomeConfigError(ValueError):
    pass


@dataclass(frozen=True)
class HomeResult:
    success: bool
    target_deg: dict[str, float]
    final_deg: dict[str, float]
    error_deg: dict[str, float]
    elapsed_s: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def load_home_document(path: str | Path) -> dict[str, Any]:
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"RoArm home config not found: {config_path}")
    document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise HomeConfigError("RoArm home YAML root must be a mapping")
    section = document.get("roarm_home")
    if not isinstance(section, dict):
        raise HomeConfigError("Missing roarm_home mapping")
    return document


def _finite_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise HomeConfigError(f"{name} must be numeric") from exc
    if not math.isfinite(result):
        raise HomeConfigError(f"{name} must be finite")
    return result


def _positive_float(value: Any, name: str) -> float:
    result = _finite_float(value, name)
    if result <= 0.0:
        raise HomeConfigError(f"{name} must be positive")
    return result


def _positive_int(value: Any, name: str, maximum: int) -> int:
    integer = int(value)
    if not 1 <= integer <= maximum:
        raise HomeConfigError(f"{name} must be in [1, {maximum}]")
    return integer


def _state_degrees(state: Any) -> dict[str, float]:
    return {
        "base": math.degrees(float(state.base_rad)),
        "shoulder": math.degrees(float(state.shoulder_rad)),
        "elbow": math.degrees(float(state.elbow_rad)),
        "hand": math.degrees(float(state.hand_rad)),
    }


def _angular_error_deg(actual: float, target: float) -> float:
    return (float(actual) - float(target) + 180.0) % 360.0 - 180.0


def resolve_home_target(section: dict[str, Any], state: Any) -> dict[str, float]:
    pose = section.get("pose")
    if not isinstance(pose, dict):
        raise HomeConfigError("roarm_home.pose must be a mapping")
    current = _state_degrees(state)
    hand_value = pose.get("hand_deg")
    target = {
        "base": _finite_float(pose.get("base_deg"), "pose.base_deg"),
        "shoulder": _finite_float(
            pose.get("shoulder_deg"), "pose.shoulder_deg"
        ),
        "elbow": _finite_float(pose.get("elbow_deg"), "pose.elbow_deg"),
        "hand": current["hand"] if hand_value is None else _finite_float(
            hand_value, "pose.hand_deg"
        ),
    }
    limits = {
        "base": (-180.0, 180.0),
        "shoulder": (-90.0, 90.0),
        "elbow": (0.0, 180.0),
        "hand": (45.0 - 1e-3, 180.0 + 1e-3),
    }
    for name, value in target.items():
        lower, upper = limits[name]
        if not lower <= value <= upper:
            raise HomeConfigError(
                f"Home {name}={value:.2f} outside [{lower}, {upper}] degrees"
            )
    return target


def preview_home(document: dict[str, Any], state: Any | None = None) -> dict[str, Any]:
    section = document["roarm_home"]
    pose = dict(section.get("pose", {}))
    if pose.get("hand_deg") is None:
        pose["hand_deg"] = (
            "preserve current T=105 hand angle"
            if state is None
            else math.degrees(float(state.hand_rad))
        )
    return {
        "enabled": bool(section.get("enabled", False)),
        "run_before_pipeline": bool(section.get("run_before_pipeline", False)),
        "device": section.get("device", "/dev/ttyROARM"),
        "pose": pose,
        "motion": section.get("motion", {}),
    }


def execute_custom_home(
    document: dict[str, Any],
    *,
    require_confirmation: bool = True,
    input_fn: Callable[[str], str] = input,
    uart_factory: Callable[..., Any] | None = None,
) -> HomeResult:
    section = document["roarm_home"]
    if not bool(section.get("enabled", False)):
        raise PermissionError("roarm_home.enabled must be true")
    if not bool(section.get("allow_real_motion", False)):
        raise PermissionError("roarm_home.allow_real_motion must be true")
    if not bool(section.get("confirm_clearance", False)):
        raise PermissionError("roarm_home.confirm_clearance must be true")

    phrase = str(section.get("confirmation_phrase", "HOME ROARM FOR PIPELINE"))
    if require_confirmation:
        entered = input_fn(
            f"Type exactly {phrase!r} to execute the T=122 custom home: "
        )
        if entered != phrase:
            raise PermissionError("Custom-home confirmation phrase did not match")

    motion = section.get("motion")
    if not isinstance(motion, dict):
        raise HomeConfigError("roarm_home.motion must be a mapping")
    speed = _positive_int(motion.get("speed_deg_s", 15), "speed_deg_s", 60)
    acceleration = _positive_int(
        motion.get("acceleration_deg_s2", 20),
        "acceleration_deg_s2",
        254,
    )
    timeout_s = _positive_float(motion.get("timeout_s", 20.0), "timeout_s")
    poll_s = _positive_float(motion.get("poll_s", 0.25), "poll_s")
    tolerance_deg = _positive_float(
        motion.get("tolerance_deg", 5.0), "tolerance_deg"
    )
    settle_s = max(0.0, _finite_float(motion.get("settle_s", 0.8), "settle_s"))

    if uart_factory is None:
        from arm_control.roarm_uart import RoArmUart
        uart_factory = RoArmUart

    started = time.monotonic()
    with uart_factory(
        port=str(section.get("device", "/dev/ttyROARM")),
        baudrate=int(section.get("baudrate", 115200)),
        response_timeout_seconds=float(section.get("response_timeout_s", 3.0)),
    ) as arm:
        before = arm.get_state()
        target = resolve_home_target(section, before)

        if bool(section.get("stop_continuous_first", True)):
            arm.stop_continuous_motion()
            time.sleep(0.25)
        if bool(section.get("enable_torque_first", True)):
            arm.send({"T": 210, "cmd": 1})
            time.sleep(0.25)

        arm.move_all_joints_degrees(
            base=target["base"],
            shoulder=target["shoulder"],
            elbow=target["elbow"],
            hand=target["hand"],
            speed_degrees_s=speed,
            acceleration_degrees_s2=acceleration,
        )

        deadline = time.monotonic() + timeout_s
        last_actual: dict[str, float] | None = None
        last_error: dict[str, float] | None = None
        while time.monotonic() < deadline:
            time.sleep(poll_s)
            try:
                state = arm.get_state()
            except Exception:
                # T=122 or firmware motion can temporarily delay feedback.
                continue
            actual = _state_degrees(state)
            error = {
                "base": _angular_error_deg(actual["base"], target["base"]),
                "shoulder": actual["shoulder"] - target["shoulder"],
                "elbow": actual["elbow"] - target["elbow"],
                "hand": actual["hand"] - target["hand"],
            }
            last_actual = actual
            last_error = error
            if max(abs(value) for value in error.values()) <= tolerance_deg:
                if settle_s:
                    time.sleep(settle_s)
                return HomeResult(
                    success=True,
                    target_deg=target,
                    final_deg=actual,
                    error_deg=error,
                    elapsed_s=time.monotonic() - started,
                )

        raise RuntimeError(
            "T=122 custom home feedback timeout; "
            f"target={target}, actual={last_actual}, error={last_error}"
        )


def read_status(document: dict[str, Any], uart_factory: Callable[..., Any] | None = None) -> dict[str, Any]:
    section = document["roarm_home"]
    if uart_factory is None:
        from arm_control.roarm_uart import RoArmUart
        uart_factory = RoArmUart
    with uart_factory(
        port=str(section.get("device", "/dev/ttyROARM")),
        baudrate=int(section.get("baudrate", 115200)),
        response_timeout_seconds=float(section.get("response_timeout_s", 3.0)),
    ) as arm:
        state = arm.get_state()
    payload = dict(state.raw)
    payload["joint_degrees"] = _state_degrees(state)
    return payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RoArm T=122 custom-home utility")
    parser.add_argument(
        "action",
        choices=("validate", "preview", "status", "home"),
    )
    parser.add_argument("--config", default=str(DEFAULT_HOME_CONFIG))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    document = load_home_document(args.config)
    if args.action == "validate":
        print(f"RoArm home configuration valid: {Path(args.config).resolve()}")
        print(json.dumps(preview_home(document), indent=2, ensure_ascii=False))
        return 0
    if args.action == "preview":
        print(json.dumps(preview_home(document), indent=2, ensure_ascii=False))
        return 0
    if args.action == "status":
        print(json.dumps(read_status(document), indent=2, ensure_ascii=False))
        return 0
    result = execute_custom_home(document)
    print(json.dumps(result.to_dict(), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
