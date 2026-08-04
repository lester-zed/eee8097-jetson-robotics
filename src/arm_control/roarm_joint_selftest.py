#!/usr/bin/env python3
"""Safe, feedback-verified J1-J4 motion self-test for RoArm-M2-S.

This module uses the repository's canonical ``arm_control.roarm_uart`` layer.
It never sends T=100.  Real movement requires both ``--execute`` and
``--confirm-clearance``; without ``--execute`` it performs a read-only plan.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Callable, Sequence

from arm_control.roarm_uart import RoArmState, RoArmTimeoutError, RoArmUart


STATE_ATTRIBUTES = {
    1: "base_rad",
    2: "shoulder_rad",
    3: "elbow_rad",
    4: "hand_rad",
}

JOINT_NAMES = {
    1: "J1 Base",
    2: "J2 Shoulder",
    3: "J3 Elbow",
    4: "J4 Hand/Wrist",
}

DEFAULT_DELTAS_DEG = {
    1: 10.0,
    2: 8.0,
    3: 8.0,
    # The normal power-on hand feedback is close to +180 degrees, so the
    # first safe test direction must be negative rather than beyond the limit.
    4: -8.0,
}


@dataclass(frozen=True)
class JointSpec:
    joint: int
    name: str
    state_attribute: str
    preferred_delta_deg: float


@dataclass(frozen=True)
class MotionSettings:
    speed_deg_s: int = 5
    acceleration_deg_s2: int = 10
    tolerance_deg: float = 4.0
    inactive_tolerance_deg: float = 4.0
    limit_margin_deg: float = 1.0
    settle_margin_s: float = 0.8
    feedback_timeout_s: float = 3.0
    feedback_poll_s: float = 0.25
    hold_s: float = 1.5


@dataclass
class JointResult:
    joint: int
    name: str
    status: str
    start_deg: float
    target_deg: float
    commanded_delta_deg: float
    reached_deg: float | None = None
    returned_deg: float | None = None
    target_error_deg: float | None = None
    return_error_deg: float | None = None
    inactive_drift_at_target_deg: float | None = None
    inactive_drift_after_return_deg: float | None = None
    observation: str = ""
    error: str = ""
    elapsed_s: float = 0.0

    @property
    def passed(self) -> bool:
        return self.status == "PASS"


def _finite(value: float, label: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def state_angle_degrees(state: RoArmState, joint: int) -> float:
    attribute = STATE_ATTRIBUTES[joint]
    return math.degrees(_finite(getattr(state, attribute), attribute))


def state_angles_degrees(state: RoArmState) -> dict[int, float]:
    return {joint: state_angle_degrees(state, joint) for joint in range(1, 5)}


def joint_error_degrees(joint: int, actual: float, expected: float) -> float:
    """Return absolute joint error; J1 is circular at -180/+180."""
    if joint == 1:
        return abs((float(actual) - float(expected) + 180.0) % 360.0 - 180.0)
    return abs(float(actual) - float(expected))


def choose_target_degrees(
    start_deg: float,
    preferred_delta_deg: float,
    limits_deg: tuple[float, float],
    *,
    margin_deg: float,
) -> tuple[float, float]:
    """Choose the preferred small target, reversing only near a joint limit."""
    start = _finite(start_deg, "start_deg")
    preferred = _finite(preferred_delta_deg, "preferred_delta_deg")
    margin = _finite(margin_deg, "margin_deg")
    lower, upper = map(float, limits_deg)
    if preferred == 0.0:
        raise ValueError("preferred joint delta cannot be zero")
    if margin < 0.0 or lower + margin >= upper - margin:
        raise ValueError("invalid joint limit margin")
    if not lower <= start <= upper:
        raise ValueError(
            f"start angle {start:.1f} outside joint limits [{lower:.1f}, {upper:.1f}]"
        )

    for delta in (preferred, -preferred):
        target = start + delta
        if lower + margin <= target <= upper - margin:
            return target, delta
    raise ValueError(
        f"no safe +/-{abs(preferred):.1f} degree target from {start:.1f} "
        f"within [{lower:.1f}, {upper:.1f}]"
    )


def validate_state(state: RoArmState) -> None:
    if int(state.raw.get("T", -1)) != 1051:
        raise RuntimeError(f"unexpected feedback type: {state.raw.get('T')!r}")
    if not 7.0 <= float(state.voltage_v) <= 13.0:
        raise RuntimeError(
            f"RoArm voltage feedback is implausible: {state.voltage_v:.2f} V"
        )
    for joint, angle in state_angles_degrees(state).items():
        if not math.isfinite(angle):
            raise RuntimeError(f"J{joint} feedback is not finite")


def inactive_joint_drift_degrees(
    reference: RoArmState,
    observed: RoArmState,
    *,
    active_joint: int,
) -> float:
    drifts = [
        joint_error_degrees(
            joint,
            state_angle_degrees(observed, joint),
            state_angle_degrees(reference, joint),
        )
        for joint in range(1, 5)
        if joint != active_joint
    ]
    return max(drifts, default=0.0)


def wait_for_target(
    arm: RoArmUart,
    spec: JointSpec,
    *,
    start_deg: float,
    target_deg: float,
    settings: MotionSettings,
) -> tuple[RoArmState, float, bool]:
    """Wait conservatively, then poll T=105 until target or timeout."""
    travel = abs(float(target_deg) - float(start_deg))
    estimated_motion_s = travel / float(settings.speed_deg_s)
    time.sleep(estimated_motion_s + settings.settle_margin_s)

    deadline = time.monotonic() + settings.feedback_timeout_s
    last_state: RoArmState | None = None
    last_error = math.inf
    last_timeout: Exception | None = None

    while True:
        try:
            state = arm.get_state()
            validate_state(state)
            last_state = state
            actual = state_angle_degrees(state, spec.joint)
            last_error = joint_error_degrees(spec.joint, actual, target_deg)
            if last_error <= settings.tolerance_deg:
                return state, last_error, True
        except RoArmTimeoutError as exc:
            last_timeout = exc

        if time.monotonic() >= deadline:
            if last_state is not None:
                return last_state, last_error, False
            if last_timeout is not None:
                raise last_timeout
            raise RuntimeError(f"no valid J{spec.joint} feedback after motion")
        time.sleep(settings.feedback_poll_s)


def direction_hint(joint: int, commanded_delta_deg: float, mount_yaw_deg: float) -> str:
    sign = "+" if commanded_delta_deg > 0 else "-"
    normalized_yaw = (float(mount_yaw_deg) + 180.0) % 360.0 - 180.0
    if joint == 1 and math.isclose(abs(normalized_yaw), 180.0, abs_tol=1.0):
        expected = "Camera 右侧" if commanded_delta_deg > 0 else "Camera 左侧"
        return (
            f"反向安装假设下，J1 {sign} 方向应朝 {expected}；"
            "若相反，需要复核 mount_yaw 或 J1 方向符号"
        )
    if joint == 1 and math.isclose(normalized_yaw, 0.0, abs_tol=1.0):
        expected = "Camera 左侧" if commanded_delta_deg > 0 else "Camera 右侧"
        return f"同向安装假设下，J1 {sign} 方向应朝 {expected}"
    if joint == 1:
        return "记录末端从 Camera 视角向左还是向右，用于安装偏航标定"
    if joint == 2:
        return f"记录 J2 {sign} 方向使机械臂抬高还是降低"
    if joint == 3:
        return f"记录 J3 {sign} 方向使前臂展开还是收拢"
    return f"记录 J4 {sign} 方向使末端向上还是向下俯仰"


def _safe_stop(arm: RoArmUart) -> None:
    try:
        arm.stop_continuous_motion()
    except Exception:
        pass


def run_joint_sequence(
    arm: RoArmUart,
    specs: Sequence[JointSpec],
    settings: MotionSettings,
    *,
    mount_yaw_deg: float = 180.0,
    before_motion: Callable[[JointSpec, float, float, str], None] | None = None,
    observe_direction: Callable[[JointSpec, str], str] | None = None,
) -> list[JointResult]:
    """Move each requested joint to a small target and back to its start."""
    results: list[JointResult] = []

    for spec in specs:
        started = time.monotonic()
        start_state = arm.get_state()
        validate_state(start_state)
        start_deg = state_angle_degrees(start_state, spec.joint)
        limits = arm.JOINT_LIMITS_DEG[spec.joint]
        target_deg, delta_deg = choose_target_degrees(
            start_deg,
            spec.preferred_delta_deg,
            limits,
            margin_deg=settings.limit_margin_deg,
        )
        hint = direction_hint(spec.joint, delta_deg, mount_yaw_deg)
        result = JointResult(
            joint=spec.joint,
            name=spec.name,
            status="FAIL",
            start_deg=start_deg,
            target_deg=target_deg,
            commanded_delta_deg=delta_deg,
        )

        try:
            if before_motion is not None:
                before_motion(spec, start_deg, target_deg, hint)

            print(
                f"\n{spec.name}: {start_deg:.1f}° -> {target_deg:.1f}° "
                f"-> {start_deg:.1f}°"
            )
            print(f"观察提示：{hint}")

            arm.move_single_joint_degrees(
                spec.joint,
                target_deg,
                speed_degrees_s=settings.speed_deg_s,
                acceleration_degrees_s2=settings.acceleration_deg_s2,
            )
            reached_state, target_error, target_ok = wait_for_target(
                arm,
                spec,
                start_deg=start_deg,
                target_deg=target_deg,
                settings=settings,
            )
            result.reached_deg = state_angle_degrees(reached_state, spec.joint)
            result.target_error_deg = target_error
            result.inactive_drift_at_target_deg = inactive_joint_drift_degrees(
                start_state,
                reached_state,
                active_joint=spec.joint,
            )

            if observe_direction is not None:
                result.observation = observe_direction(spec, hint).strip()
            elif settings.hold_s > 0.0:
                time.sleep(settings.hold_s)

            # A valid T=1051 response was received, so return to the recorded
            # per-joint start even when target tolerance was missed.
            arm.move_single_joint_degrees(
                spec.joint,
                start_deg,
                speed_degrees_s=settings.speed_deg_s,
                acceleration_degrees_s2=settings.acceleration_deg_s2,
            )
            returned_state, return_error, return_ok = wait_for_target(
                arm,
                spec,
                start_deg=target_deg,
                target_deg=start_deg,
                settings=settings,
            )
            result.returned_deg = state_angle_degrees(returned_state, spec.joint)
            result.return_error_deg = return_error
            result.inactive_drift_after_return_deg = inactive_joint_drift_degrees(
                start_state,
                returned_state,
                active_joint=spec.joint,
            )

            inactive_ok = (
                result.inactive_drift_at_target_deg
                <= settings.inactive_tolerance_deg
                and result.inactive_drift_after_return_deg
                <= settings.inactive_tolerance_deg
            )
            if target_ok and return_ok and inactive_ok:
                result.status = "PASS"
                print(
                    f"PASS: 到位误差 {target_error:.1f}°，"
                    f"回位误差 {return_error:.1f}°"
                )
            else:
                reasons: list[str] = []
                if not target_ok:
                    reasons.append(f"target error {target_error:.1f}°")
                if not return_ok:
                    reasons.append(f"return error {return_error:.1f}°")
                if not inactive_ok:
                    reasons.append(
                        "inactive-joint drift "
                        f"{max(result.inactive_drift_at_target_deg, result.inactive_drift_after_return_deg):.1f}°"
                    )
                result.error = "; ".join(reasons)
                print(f"FAIL: {result.error}")
        except KeyboardInterrupt:
            _safe_stop(arm)
            raise
        except Exception as exc:
            result.error = f"{type(exc).__name__}: {exc}"
            _safe_stop(arm)
            print(f"FAIL: {result.error}", file=sys.stderr)
        finally:
            result.elapsed_s = round(time.monotonic() - started, 3)
            results.append(result)

        if not result.passed:
            print("已停止后续关节测试。")
            break

    return results


def _parse_joint_numbers(value: str) -> list[int]:
    try:
        joints = [int(item.strip()) for item in value.split(",") if item.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("joints must be comma-separated integers") from exc
    if not joints or any(joint not in STATE_ATTRIBUTES for joint in joints):
        raise argparse.ArgumentTypeError("joints must contain only 1,2,3,4")
    if len(set(joints)) != len(joints):
        raise argparse.ArgumentTypeError("joints cannot contain duplicates")
    return joints


def _default_report_path() -> Path:
    workspace_logs = Path("/workspace/logs")
    if workspace_logs.exists():
        return workspace_logs / "roarm_joint_selftest_latest.json"
    return Path("/tmp/roarm_joint_selftest_latest.json")


def _write_report(path: Path, payload: dict[str, Any]) -> Path:
    path = path.expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    return path


def _state_payload(state: RoArmState) -> dict[str, Any]:
    return {
        "voltage_v": state.voltage_v,
        "joint_degrees": {
            f"J{joint}": round(angle, 3)
            for joint, angle in state_angles_degrees(state).items()
        },
        "xyz_mm": [state.x_mm, state.y_mm, state.z_mm],
    }


def _build_specs(args: argparse.Namespace) -> list[JointSpec]:
    deltas = {
        1: args.j1_delta,
        2: args.j2_delta,
        3: args.j3_delta,
        4: args.j4_delta,
    }
    return [
        JointSpec(
            joint=joint,
            name=JOINT_NAMES[joint],
            state_attribute=STATE_ATTRIBUTES[joint],
            preferred_delta_deg=deltas[joint],
        )
        for joint in args.joints
    ]


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Feedback-verified RoArm J1-J4 motion and return self-test"
    )
    parser.add_argument("--port", default="/dev/ttyROARM")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--joints", type=_parse_joint_numbers, default=[1, 2, 3, 4])
    parser.add_argument("--j1-delta", type=float, default=DEFAULT_DELTAS_DEG[1])
    parser.add_argument("--j2-delta", type=float, default=DEFAULT_DELTAS_DEG[2])
    parser.add_argument("--j3-delta", type=float, default=DEFAULT_DELTAS_DEG[3])
    parser.add_argument("--j4-delta", type=float, default=DEFAULT_DELTAS_DEG[4])
    parser.add_argument("--speed", type=int, default=5)
    parser.add_argument("--acceleration", type=int, default=10)
    parser.add_argument("--tolerance", type=float, default=4.0)
    parser.add_argument("--inactive-tolerance", type=float, default=4.0)
    parser.add_argument("--hold", type=float, default=1.5)
    parser.add_argument("--mount-yaw", type=float, default=180.0)
    parser.add_argument("--base-zero-tolerance", type=float, default=15.0)
    parser.add_argument("--allow-nonzero-base", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm-clearance", action="store_true")
    parser.add_argument("--yes", action="store_true", help="Skip the one-time MOVE prompt")
    parser.add_argument("--step-confirm", action="store_true")
    parser.add_argument("--record-directions", action="store_true")
    parser.add_argument("--report", default=str(_default_report_path()))
    parser.add_argument("--no-report", action="store_true")
    parser.add_argument("--json", action="store_true")
    return parser


def _validate_settings(settings: MotionSettings) -> None:
    if not 1 <= settings.speed_deg_s <= 30:
        raise ValueError("speed must be in [1, 30] deg/s")
    if not 1 <= settings.acceleration_deg_s2 <= 50:
        raise ValueError("acceleration must be in [1, 50]")
    for value, label in (
        (settings.tolerance_deg, "tolerance"),
        (settings.inactive_tolerance_deg, "inactive tolerance"),
        (settings.feedback_timeout_s, "feedback timeout"),
        (settings.feedback_poll_s, "feedback poll"),
    ):
        if value <= 0.0:
            raise ValueError(f"{label} must be positive")
    if settings.hold_s < 0.0:
        raise ValueError("hold must be non-negative")


def _print_plan(
    initial: RoArmState,
    specs: Sequence[JointSpec],
    settings: MotionSettings,
    limits: dict[int, tuple[float, float]],
    mount_yaw_deg: float,
) -> list[dict[str, Any]]:
    plan: list[dict[str, Any]] = []
    print("\n=== RoArm J1-J4 self-test plan ===")
    print(f"Voltage: {initial.voltage_v:.2f} V")
    for spec in specs:
        start = state_angle_degrees(initial, spec.joint)
        target, delta = choose_target_degrees(
            start,
            spec.preferred_delta_deg,
            limits[spec.joint],
            margin_deg=settings.limit_margin_deg,
        )
        hint = direction_hint(spec.joint, delta, mount_yaw_deg)
        print(f"{spec.name:<16} {start:7.1f}° -> {target:7.1f}° -> {start:7.1f}°")
        print(f"  {hint}")
        plan.append(
            {
                "joint": spec.joint,
                "name": spec.name,
                "start_deg": start,
                "target_deg": target,
                "commanded_delta_deg": delta,
                "direction_hint": hint,
            }
        )
    return plan


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.execute and not args.confirm_clearance:
        print(
            "ERROR: real motion requires --confirm-clearance after clearing the full sweep area.",
            file=sys.stderr,
        )
        return 2
    if args.record_directions and not args.execute:
        print("ERROR: --record-directions requires --execute.", file=sys.stderr)
        return 2

    settings = MotionSettings(
        speed_deg_s=args.speed,
        acceleration_deg_s2=args.acceleration,
        tolerance_deg=args.tolerance,
        inactive_tolerance_deg=args.inactive_tolerance,
        hold_s=args.hold,
    )
    try:
        _validate_settings(settings)
        specs = _build_specs(args)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if args.execute and not args.yes:
        print("\nWARNING: J1-J4 will move one at a time at low speed.")
        print("Clear the full arm sweep, remove objects, and arrange cables.")
        confirmation = input("Type MOVE to continue (anything else cancels): ").strip()
        if confirmation != "MOVE":
            print("Cancelled before opening the RoArm serial port.")
            return 2

    timestamp = datetime.now(timezone.utc).isoformat()
    results: list[JointResult] = []
    payload: dict[str, Any] = {
        "schema_version": 1,
        "timestamp_utc": timestamp,
        "device": args.port,
        "mount_yaw_degrees": args.mount_yaw,
        "motion_requested": bool(args.execute),
        "t100_sent": False,
        "overall": "FAIL",
        "plan": [],
        "checks": [],
    }

    try:
        with RoArmUart(
            port=args.port,
            baudrate=args.baudrate,
            response_timeout_seconds=3.0,
        ) as arm:
            initial = arm.get_state()
            validate_state(initial)
            payload["initial_state"] = _state_payload(initial)

            if 1 in args.joints and not args.allow_nonzero_base:
                base_error = joint_error_degrees(
                    1,
                    state_angle_degrees(initial, 1),
                    0.0,
                )
                if base_error > args.base_zero_tolerance:
                    raise RuntimeError(
                        "Base is not near 0 degrees, so Camera direction cannot be "
                        f"judged reliably (error={base_error:.1f} degrees)."
                    )

            plan = _print_plan(
                initial,
                specs,
                settings,
                arm.JOINT_LIMITS_DEG,
                args.mount_yaw,
            )
            payload["plan"] = plan

            if not args.execute:
                payload["overall"] = "PLAN_ONLY"
                print("\nPLAN_ONLY: only T=105 feedback was requested; no joint moved.")
            else:
                def before_motion(
                    spec: JointSpec,
                    start: float,
                    target: float,
                    hint: str,
                ) -> None:
                    if not args.step_confirm:
                        return
                    answer = input(
                        f"\n{spec.name} {start:.1f}° -> {target:.1f}°. "
                        "Press Enter to run, or q to stop: "
                    ).strip().lower()
                    if answer == "q":
                        raise KeyboardInterrupt

                def observe(spec: JointSpec, hint: str) -> str:
                    print(f"到达目标。{hint}")
                    return input(
                        "记录实际方向（例如 camera-left / camera-right / up / down）: "
                    )

                results = run_joint_sequence(
                    arm,
                    specs,
                    settings,
                    mount_yaw_deg=args.mount_yaw,
                    before_motion=before_motion,
                    observe_direction=observe if args.record_directions else None,
                )
                payload["checks"] = [asdict(result) for result in results]
                payload["overall"] = (
                    "PASS"
                    if len(results) == len(specs) and all(item.passed for item in results)
                    else "FAIL"
                )
    except KeyboardInterrupt:
        payload["overall"] = "ABORTED"
        payload["error"] = "Interrupted by user; hardware power-off is the emergency stop."
        print("\nABORTED. If motion is unsafe, switch off RoArm power.", file=sys.stderr)
    except Exception as exc:
        payload["overall"] = "FAIL"
        payload["error"] = f"{type(exc).__name__}: {exc}"
        print(f"\nFAIL: {payload['error']}", file=sys.stderr)

    if not args.no_report:
        try:
            report = _write_report(Path(args.report), payload)
            payload["report_path"] = str(report)
        except OSError as exc:
            payload["report_error"] = str(exc)

    if args.json:
        print(json.dumps(payload, indent=2, ensure_ascii=False))
    else:
        print(f"\nOVERALL: {payload['overall']}")
        if payload.get("report_path"):
            print(f"Report: {payload['report_path']}")

    return 0 if payload["overall"] in {"PASS", "PLAN_ONLY"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
