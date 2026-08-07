from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from threading import Event
from typing import Any

from configuration.loader import RuntimeConfig, load_runtime_config
from pipeline.types import CartesianWaypoint, GraspPlan, Point3D, TargetPose


SOURCE_ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = SOURCE_ROOT / "configs/modular_pipeline.yaml"
WAYPOINT_NAMES = ("pregrasp", "grasp", "lift", "retreat")
MOTION_ACTIONS = {"reset", "gripper-open", "gripper-close", "execute"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Configuration-driven manual RoArm motion test"
    )
    parser.add_argument(
        "action",
        choices=(
            "validate",
            "status",
            "capture-current",
            "plan",
            "reset",
            "gripper-open",
            "gripper-close",
            "execute",
        ),
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    return parser.parse_args()


def _manual_section(config: RuntimeConfig) -> dict[str, Any]:
    value = config.data.get("manual_arm_test")
    if not isinstance(value, dict):
        raise ValueError("Missing or invalid manual_arm_test configuration section")
    return value


def _vector3(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"manual_arm_test.waypoints.{name} must contain 3 values")
    if any(item is None for item in value):
        raise ValueError(
            f"manual_arm_test.waypoints.{name} is not configured; "
            "replace null values with arm_base coordinates"
        )
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"manual_arm_test.waypoints.{name} must contain numeric values"
        ) from exc
    if not all(math.isfinite(item) for item in result):
        raise ValueError(
            f"manual_arm_test.waypoints.{name} contains a non-finite value"
        )
    return result  # type: ignore[return-value]


def build_manual_plan(config: RuntimeConfig) -> GraspPlan:
    manual = _manual_section(config)
    waypoints_cfg = manual.get("waypoints")
    if not isinstance(waypoints_cfg, dict):
        raise ValueError("manual_arm_test.waypoints must be a mapping")

    tool_angle = float(manual.get("tool_angle_rad", 3.14))
    speed = float(manual.get("cartesian_speed", 0.10))
    if not math.isfinite(tool_angle):
        raise ValueError("manual_arm_test.tool_angle_rad must be finite")
    if not 0.0 < speed <= 1.0:
        raise ValueError("manual_arm_test.cartesian_speed must be in (0, 1]")

    waypoints: list[CartesianWaypoint] = []
    for name in WAYPOINT_NAMES:
        x_mm, y_mm, z_mm = _vector3(waypoints_cfg.get(name), name)
        waypoints.append(
            CartesianWaypoint(
                name=name,
                point=Point3D(x_mm, y_mm, z_mm, "arm_base"),
                tool_angle_rad=tool_angle,
                speed=speed,
            )
        )

    grasp_point = waypoints[1].point
    target = TargetPose(
        label="manual_test_target",
        confidence=1.0,
        target_base_link=Point3D(
            grasp_point.x_mm,
            grasp_point.y_mm,
            grasp_point.z_mm,
            "base_link_placeholder",
        ),
        target_arm_base=grasp_point,
        provisional=False,
        metadata={"source": "manual_arm_test.yaml"},
    )
    return GraspPlan(
        target=target,
        waypoints=tuple(waypoints),
        gripper_open_rad=float(manual.get("gripper_open_rad", 2.70)),
        gripper_closed_rad=float(manual.get("gripper_closed_rad", 3.14)),
        metadata={"planner": "manual_fixed_waypoints_v1"},
    )


def validate_manual_config(config: RuntimeConfig, *, require_waypoints: bool) -> None:
    manual = _manual_section(config)
    if not isinstance(manual.get("enabled", False), bool):
        raise ValueError("manual_arm_test.enabled must be true or false")
    if require_waypoints:
        build_manual_plan(config)


def _confirm_motion(config: RuntimeConfig, action: str) -> None:
    manual = _manual_section(config)
    if not bool(manual.get("enabled", False)):
        raise PermissionError("manual_arm_test.enabled must be true")
    if not bool(manual.get("allow_real_motion", False)):
        raise PermissionError("manual_arm_test.allow_real_motion must be true")
    if not bool(manual.get("confirm_clearance", False)):
        raise PermissionError("manual_arm_test.confirm_clearance must be true")
    if action == "execute" and not bool(manual.get("coordinates_approved", False)):
        raise PermissionError(
            "manual_arm_test.coordinates_approved must be true before execute"
        )

    phrase_key = {
        "reset": "reset_confirmation_phrase",
        "gripper-open": "gripper_confirmation_phrase",
        "gripper-close": "gripper_confirmation_phrase",
        "execute": "execute_confirmation_phrase",
    }[action]
    default_phrase = {
        "reset": "RESET ROARM TO INITIAL POSITION",
        "gripper-open": "MOVE ROARM GRIPPER",
        "gripper-close": "MOVE ROARM GRIPPER",
        "execute": "EXECUTE MANUAL ROARM GRASP",
    }[action]
    phrase = str(manual.get(phrase_key, default_phrase))
    print("\nREAL ROARM MOTION REQUESTED")
    print("Clear the full sweep volume and keep access to the hardware power switch.")
    entered = input(f"Type exactly {phrase!r}: ")
    if entered != phrase:
        raise PermissionError("Typed confirmation phrase did not match")


def _make_uart(config: RuntimeConfig):
    from arm_control.roarm_uart import RoArmUart

    arm = config.section("arm")
    manual = _manual_section(config)
    return RoArmUart(
        port=str(arm.get("device", "/dev/ttyROARM")),
        baudrate=int(arm.get("baudrate", 115200)),
        response_timeout_seconds=float(manual.get("uart_response_timeout_s", 2.0)),
    )


def _state_payload(state: Any) -> dict[str, float]:
    return {
        "x_mm": float(state.x_mm),
        "y_mm": float(state.y_mm),
        "z_mm": float(state.z_mm),
        "base_rad": float(state.base_rad),
        "shoulder_rad": float(state.shoulder_rad),
        "elbow_rad": float(state.elbow_rad),
        "hand_rad": float(state.hand_rad),
        "voltage_v": float(state.voltage_v),
    }


def _print_state(state: Any) -> None:
    print(json.dumps(_state_payload(state), indent=2, ensure_ascii=False))


def _read_status(config: RuntimeConfig) -> int:
    uart = _make_uart(config)
    try:
        _print_state(uart.get_state())
        return 0
    finally:
        uart.close()


def _capture_current(config: RuntimeConfig) -> int:
    uart = _make_uart(config)
    try:
        state = uart.get_state()
    finally:
        uart.close()
    point = [round(float(state.x_mm), 3), round(float(state.y_mm), 3), round(float(state.z_mm), 3)]
    print("# Copy this conservative no-translation template into manual_arm_test.waypoints")
    print("waypoints:")
    for name in WAYPOINT_NAMES:
        print(f"  {name}: {point}")
    print("# This template tests gripper sequencing but intentionally commands no Cartesian displacement.")
    return 0


def _reset(config: RuntimeConfig) -> int:
    manual = _manual_section(config)
    uart = _make_uart(config)
    try:
        before = uart.get_state()
        print("Before reset:")
        _print_state(before)
        uart.move_initial_position()
        wait_s = float(manual.get("reset_wait_s", 8.0))
        timeout_s = float(manual.get("reset_timeout_s", 25.0))
        time.sleep(max(0.0, wait_s))
        deadline = time.monotonic() + max(1.0, timeout_s)
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                after = uart.get_state()
                print("After reset:")
                _print_state(after)
                return 0
            except Exception as exc:  # Firmware may be busy while T=100 blocks.
                last_error = exc
                time.sleep(0.5)
        raise RuntimeError(f"No T=105 feedback after reset: {last_error}")
    finally:
        uart.close()


def _move_gripper(config: RuntimeConfig, *, opening: bool) -> int:
    manual = _manual_section(config)
    angle = float(
        manual.get("gripper_open_rad" if opening else "gripper_closed_rad", 2.70 if opening else 3.14)
    )
    settle_s = float(manual.get("gripper_settle_s", 1.0))
    speed_steps_s = int(manual.get("gripper_speed_steps_s", 100))
    acceleration = int(manual.get("gripper_acceleration", 10))
    margin_s = float(manual.get("gripper_motion_margin_s", 0.75))
    if not 1 <= speed_steps_s <= 1000:
        raise ValueError("gripper_speed_steps_s must be in [1, 1000]")
    if not 1 <= acceleration <= 254:
        raise ValueError("gripper_acceleration must be in [1, 254]")
    uart = _make_uart(config)
    try:
        before = uart.get_state()
        uart.set_gripper_radians(
            angle,
            speed_steps_s=speed_steps_s,
            acceleration=acceleration,
        )
        delta_rad = abs(float(angle) - float(before.hand_rad))
        estimated_s = delta_rad / (2.0 * math.pi) * 4096.0 / float(speed_steps_s)
        wait_s = max(settle_s, estimated_s + margin_s)
        time.sleep(max(0.0, wait_s))
        _print_state(uart.get_state())
        return 0
    finally:
        uart.close()


def _execute(config: RuntimeConfig) -> int:
    from arm_control.cartesian_roarm_controller import CartesianRoArmController

    manual = _manual_section(config)
    arm = config.section("arm")
    plan = build_manual_plan(config)
    controller = CartesianRoArmController(
        port=str(arm.get("device", "/dev/ttyROARM")),
        baudrate=int(arm.get("baudrate", 115200)),
        allow_real_motion=True,
        confirm_clearance=True,
        calibration_approved=bool(manual.get("coordinates_approved", False)),
        one_grasp_per_process=True,
        feedback_tolerance_mm=float(manual.get("feedback_tolerance_mm", 12.0)),
        waypoint_timeout_s=float(manual.get("waypoint_timeout_s", 15.0)),
        feedback_poll_s=float(manual.get("feedback_poll_s", 0.20)),
        feedback_initial_delay_s=float(manual.get("feedback_initial_delay_s", manual.get("initial_feedback_delay_s", 0.75))),
        uart_response_timeout_s=float(manual.get("uart_response_timeout_s", 2.0)),
        gripper_settle_s=float(manual.get("gripper_settle_s", 1.0)),
        gripper_speed_steps_s=int(manual.get("gripper_speed_steps_s", 100)),
        gripper_acceleration=int(manual.get("gripper_acceleration", 10)),
        gripper_motion_margin_s=float(manual.get("gripper_motion_margin_s", 0.75)),
        min_voltage_v=float(arm.get("min_voltage_v", 7.0)),
        max_voltage_v=float(arm.get("max_voltage_v", 13.0)),
    )
    try:
        result = controller.execute_grasp(plan, Event())
        print(json.dumps(result.to_dict(), indent=2, ensure_ascii=False))
        return 0
    finally:
        controller.close()


def main() -> int:
    args = parse_args()
    config = load_runtime_config(args.config)
    require_waypoints = args.action in {"plan", "execute"}
    validate_manual_config(config, require_waypoints=require_waypoints)

    if args.action == "validate":
        print(f"PASS: {Path(args.config).resolve()}")
        return 0
    if args.action == "plan":
        print(json.dumps(build_manual_plan(config).to_dict(), indent=2, ensure_ascii=False))
        return 0
    if args.action == "status":
        return _read_status(config)
    if args.action == "capture-current":
        return _capture_current(config)

    if args.action in MOTION_ACTIONS:
        _confirm_motion(config, args.action)
    if args.action == "reset":
        return _reset(config)
    if args.action == "gripper-open":
        return _move_gripper(config, opening=True)
    if args.action == "gripper-close":
        return _move_gripper(config, opening=False)
    if args.action == "execute":
        return _execute(config)
    raise AssertionError(f"Unhandled action: {args.action}")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("Interrupted by user.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"FAIL: {type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(2)
