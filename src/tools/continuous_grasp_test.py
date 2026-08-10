#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
import time
from typing import Any
from uuid import uuid4

import yaml


SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from configuration.loader import RuntimeConfig, load_runtime_config
from experiments.recorder import (
    ExperimentRecorder,
    RunContext,
    annotate_run,
    load_jsonl_records,
)
from experiments.statistics import summarize_records
from launch_modular_pipeline import should_execute_startup_home
from main_modular import build_manager
from roarm_home import execute_custom_home, load_home_document, resolve_home_target


DEFAULT_CONFIG = SOURCE_ROOT / "configs/modular_pipeline.yaml"
DEFAULT_HOME_CONFIG = SOURCE_ROOT / "configs/roarm_home.yaml"
DEFAULT_OUTPUT_ROOT = Path("/workspace/logs/baseline_grasp")
SESSION_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
CONTINUOUS_CONFIRMATION_PHRASE = "EXECUTE CONTINUOUS GRASP TEST"

# The currently committed affine calibration was fitted from
# tests/fixtures/command_grid_20260810.json. Its operator-established command
# Y values are -35, -20 and -5 mm. Positive command Y is therefore an
# extrapolation and should be called out explicitly in continuous-test logs.
MEASURED_COMMAND_Y_MAX_BY_CALIBRATION = {
    "tissue-command-grid-20260810-v1": -5.0,
}

TRIAL_FIELDS = (
    "trial_number",
    "run_id",
    "position_label",
    "operator_outcome",
    "pipeline_state",
    "pipeline_success",
    "execution_success",
    "started_at_utc",
    "finished_at_utc",
    "duration_s",
    "notes_before",
    "notes_after",
)


@dataclass(frozen=True)
class SessionPaths:
    root: Path
    config: Path
    manifest: Path
    trials_csv: Path
    events_jsonl: Path
    summary: Path


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _new_session_id() -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"baseline-{stamp}-{uuid4().hex[:8]}"


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def prepare_effective_data(
    base_data: dict[str, Any],
    home_document: dict[str, Any],
    *,
    output_dir: Path,
) -> dict[str, Any]:
    """Build a session-only config without mutating the committed profile."""
    override_section = home_document.get("pipeline_overrides", {})
    values: dict[str, Any] = {}
    if isinstance(override_section, dict) and bool(
        override_section.get("enabled", False)
    ):
        candidate = override_section.get("values", {})
        if not isinstance(candidate, dict):
            raise ValueError("pipeline_overrides.values must be a mapping")
        values = candidate

    data = _deep_merge(base_data, values)
    arm = data.setdefault("arm", {})
    if not isinstance(arm, dict):
        raise ValueError("arm configuration must be a mapping")
    # This runner supplies a RUN gate before every trial and never overlaps runs.
    # The committed modular_pipeline.yaml remains one-grasp-per-process.
    arm["one_grasp_per_process"] = False

    experiment = data.setdefault("experiment", {})
    if not isinstance(experiment, dict):
        raise ValueError("experiment configuration must be a mapping")
    experiment.update(
        {
            "enabled": True,
            "profile": "continuous_grasp_baseline",
            "output_dir": str(output_dir),
            "jsonl_filename": "runs.jsonl",
            "csv_filename": "runs.csv",
            "require_mock_arm": False,
        }
    )
    return data


def parse_operator_outcome(value: str) -> tuple[str, bool | None]:
    normalized = value.strip().lower()
    aliases = {
        "y": ("success", True),
        "yes": ("success", True),
        "s": ("success", True),
        "n": ("failure", False),
        "no": ("failure", False),
        "f": ("failure", False),
        "u": ("uncertain", None),
        "uncertain": ("uncertain", None),
    }
    if normalized not in aliases:
        raise ValueError("请输入 y（成功）、n（失败）或 u（不确定）")
    return aliases[normalized]


def _atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            delete=False,
        ) as handle:
            temporary = Path(handle.name)
            json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temporary, path)
        temporary = None
    finally:
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass


def _append_event(path: Path, event: dict[str, Any]) -> None:
    payload = {"timestamp_utc": _utc_now(), **event}
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")


def append_trial_row(path: Path, row: dict[str, Any]) -> None:
    new_file = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=TRIAL_FIELDS)
        if new_file:
            writer.writeheader()
        writer.writerow({field: row.get(field) for field in TRIAL_FIELDS})


def _write_summary(recorder: ExperimentRecorder, path: Path) -> dict[str, Any]:
    records = (
        load_jsonl_records(recorder.jsonl_path)
        if recorder.jsonl_path.exists()
        else []
    )
    summary = summarize_records(records)
    summary["updated_at_utc"] = _utc_now()
    _atomic_write_json(path, summary)
    return summary


def _print_run_result(status: dict[str, Any]) -> None:
    detection = status.get("detection") or {}
    measurement = status.get("range") or {}
    target = status.get("target") or {}
    target_base = target.get("target_base_link") or {}
    target_arm = target.get("target_arm_base") or {}
    target_metadata = target.get("metadata") or {}
    nominal_arm = target_metadata.get("nominal_target_arm_base") or {}
    calibration_metadata = target_metadata.get("arm_command_calibration") or {}
    plan = status.get("plan") or {}
    plan_metadata = plan.get("metadata") or {}
    grasp = plan_metadata.get("commanded_grasp_arm_base") or {}
    execution = status.get("execution") or {}
    timing = status.get("timing") or {}

    print("\n本轮结果")
    print(f"  run_id:      {status.get('run_id')}")
    print(f"  state:       {status.get('state')}")
    print(f"  duration_s:  {timing.get('duration_s')}")
    if detection:
        print(
            "  detection:   "
            f"{detection.get('label')} conf={detection.get('confidence')} "
            f"center=({detection.get('center_x')}, {detection.get('center_y')})"
        )
    if measurement:
        print(
            "  lidar:       "
            f"{measurement.get('distance_mm')} mm, "
            f"MAD={measurement.get('mad_mm')} mm, "
            f"samples={measurement.get('sample_count')}"
        )
    if target_base:
        print(
            "  target_base: "
            f"({target_base.get('x_mm')}, {target_base.get('y_mm')}, "
            f"{target_base.get('z_mm')}) mm"
        )
    if nominal_arm:
        print(
            "  nominal_arm: "
            f"({nominal_arm.get('x_mm')}, {nominal_arm.get('y_mm')}, "
            f"{nominal_arm.get('z_mm')}) mm"
        )
    if target_arm:
        print(
            "  target_arm:  "
            f"({target_arm.get('x_mm')}, {target_arm.get('y_mm')}, "
            f"{target_arm.get('z_mm')}) mm"
        )
    if grasp:
        print(
            "  grasp_cmd:   "
            f"({grasp.get('x_mm')}, {grasp.get('y_mm')}, "
            f"{grasp.get('z_mm')}) mm"
        )

    calibration_id = str(calibration_metadata.get("calibration_id") or "")
    support_max_y = MEASURED_COMMAND_Y_MAX_BY_CALIBRATION.get(calibration_id)
    command_y = target_arm.get("y_mm") if target_arm else None
    if support_max_y is not None and command_y is not None:
        try:
            command_y_value = float(command_y)
        except (TypeError, ValueError):
            command_y_value = float("nan")
        if math.isfinite(command_y_value) and command_y_value > support_max_y:
            print(
                "  Y WARNING:   "
                f"commanded target Y={command_y_value:.1f} mm exceeds the "
                f"measured command-Y support ({support_max_y:.1f} mm max) "
                f"for calibration {calibration_id}; this point is extrapolated."
            )

    if execution:
        print(
            "  execution:   "
            f"success={execution.get('success')} "
            f"message={execution.get('message')}"
        )
    if status.get("error"):
        print(f"  error:       {status['error']}")
    if status.get("recording_error"):
        print(f"  log_error:   {status['recording_error']}")


def _read_outcome() -> tuple[str, bool | None]:
    while True:
        try:
            return parse_operator_outcome(
                input("抓取是否成功？[y=成功 / n=失败 / u=不确定]: ")
            )
        except ValueError as exc:
            print(exc)


def _wait_for_run(
    manager: Any,
    paths: SessionPaths,
    trial_number: int,
) -> dict[str, Any]:
    last_state: str | None = None
    try:
        while manager.is_running():
            status = manager.status()
            state = str(status.get("state"))
            if state != last_state:
                print(f"  [{_utc_now()}] {state}")
                _append_event(
                    paths.events_jsonl,
                    {
                        "event": "state_change",
                        "trial_number": trial_number,
                        "run_id": status.get("run_id"),
                        "state": state,
                    },
                )
                last_state = state
            time.sleep(0.10)
    except KeyboardInterrupt:
        print("\n正在请求中止当前任务……")
        manager.abort()
        deadline = time.monotonic() + 10.0
        while manager.is_running() and time.monotonic() < deadline:
            time.sleep(0.10)
    return manager.status()


def _real_controller(manager: Any) -> Any:
    """Return the already-open Cartesian controller used by this manager."""
    arm = getattr(manager, "arm", None)
    controller = getattr(arm, "_controller", None)
    if controller is None or not hasattr(controller, "uart"):
        raise RuntimeError(
            "Continuous real-arm recovery requires CartesianRoArmAdapter "
            "with its existing controller/UART"
        )
    return controller


def _gripper_wait_seconds(
    controller: Any,
    start_rad: float,
    target_rad: float,
) -> float:
    speed_steps_s = int(getattr(controller, "gripper_speed_steps_s", 250))
    settle_s = max(0.0, float(getattr(controller, "gripper_settle_s", 1.0)))
    margin_s = max(
        0.0,
        float(getattr(controller, "gripper_motion_margin_s", 0.75)),
    )
    if speed_steps_s <= 0:
        raise ValueError("gripper_speed_steps_s must be positive")
    servo_steps = (
        abs(float(target_rad) - float(start_rad))
        / (2.0 * math.pi)
        * 4096.0
    )
    return max(settle_s, servo_steps / speed_steps_s + margin_s)


def _open_gripper_after_grasp(
    controller: Any,
    config: RuntimeConfig,
) -> dict[str, Any]:
    """Open the stock clamp through the UART already owned by the controller."""
    open_rad = float(config.get("planner", "gripper_open_rad", 2.20))
    if not 1.08 - 1e-6 <= open_rad <= 3.14 + 1e-6:
        raise ValueError(
            f"planner.gripper_open_rad={open_rad} outside stock clamp range "
            "[1.08, 3.14]"
        )

    before = controller.uart.get_state()
    controller._validate_state(before)
    start_rad = float(before.hand_rad)
    speed_steps_s = int(getattr(controller, "gripper_speed_steps_s", 250))
    acceleration = int(getattr(controller, "gripper_acceleration", 10))
    controller.uart.set_gripper_radians(
        open_rad,
        speed_steps_s=speed_steps_s,
        acceleration=acceleration,
    )
    wait_s = _gripper_wait_seconds(controller, start_rad, open_rad)
    time.sleep(wait_s)

    after = controller.uart.get_state()
    controller._validate_state(after)
    controller._last_state = after
    return {
        "target_open_rad": open_rad,
        "before_hand_rad": start_rad,
        "after_hand_rad": float(after.hand_rad),
        "wait_s": wait_s,
        "speed_steps_s": speed_steps_s,
        "acceleration": acceleration,
    }


def _state_degrees(state: Any) -> dict[str, float]:
    return {
        "base": math.degrees(float(state.base_rad)),
        "shoulder": math.degrees(float(state.shoulder_rad)),
        "elbow": math.degrees(float(state.elbow_rad)),
        "hand": math.degrees(float(state.hand_rad)),
    }


def _home_existing_uart(
    controller: Any,
    home_document: dict[str, Any],
) -> dict[str, Any]:
    """Run the configured T=122 Home without opening a second serial port."""
    section = home_document.get("roarm_home")
    if not isinstance(section, dict):
        raise ValueError("Missing roarm_home mapping")
    if not bool(section.get("enabled", False)):
        raise PermissionError("roarm_home.enabled must be true for per-trial recovery")
    if not bool(section.get("allow_real_motion", False)):
        raise PermissionError(
            "roarm_home.allow_real_motion must be true for per-trial recovery"
        )
    if not bool(section.get("confirm_clearance", False)):
        raise PermissionError(
            "roarm_home.confirm_clearance must be true for per-trial recovery"
        )

    motion = section.get("motion")
    if not isinstance(motion, dict):
        raise ValueError("roarm_home.motion must be a mapping")
    speed = int(motion.get("speed_deg_s", 15))
    acceleration = int(motion.get("acceleration_deg_s2", 20))
    timeout_s = float(motion.get("timeout_s", 20.0))
    poll_s = float(motion.get("poll_s", 0.25))
    tolerance_deg = float(motion.get("tolerance_deg", 5.0))
    settle_s = max(0.0, float(motion.get("settle_s", 0.8)))
    if not 1 <= speed <= 60:
        raise ValueError("home speed_deg_s must be in [1, 60]")
    if not 1 <= acceleration <= 254:
        raise ValueError("home acceleration_deg_s2 must be in [1, 254]")
    if timeout_s <= 0.0 or poll_s <= 0.0 or tolerance_deg <= 0.0:
        raise ValueError("home timeout/poll/tolerance must be positive")

    before = controller.uart.get_state()
    controller._validate_state(before)
    target = resolve_home_target(section, before)

    if bool(section.get("stop_continuous_first", True)):
        controller.uart.stop_continuous_motion()
        time.sleep(0.25)
    if bool(section.get("enable_torque_first", True)):
        controller.uart.send({"T": 210, "cmd": 1})
        time.sleep(0.25)

    controller.uart.move_all_joints_degrees(
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
            state = controller.uart.get_state()
        except Exception:
            # T=122 can temporarily occupy the firmware while the move runs.
            continue
        controller._validate_state(state)
        actual = _state_degrees(state)
        error = {
            "base": (actual["base"] - target["base"] + 180.0) % 360.0 - 180.0,
            "shoulder": actual["shoulder"] - target["shoulder"],
            "elbow": actual["elbow"] - target["elbow"],
            "hand": actual["hand"] - target["hand"],
        }
        last_actual = actual
        last_error = error
        controller._last_state = state
        if max(abs(value) for value in error.values()) <= tolerance_deg:
            if settle_s:
                time.sleep(settle_s)
            return {
                "target_deg": target,
                "final_deg": actual,
                "error_deg": error,
                "speed_deg_s": speed,
                "acceleration_deg_s2": acceleration,
            }

    raise RuntimeError(
        "Per-trial T=122 Home feedback timeout; "
        f"target={target}, actual={last_actual}, error={last_error}"
    )


def _post_grasp_recovery(
    manager: Any,
    config: RuntimeConfig,
    home_document: dict[str, Any],
    paths: SessionPaths,
    trial_number: int,
    run_id: str,
) -> dict[str, Any]:
    """Release the object, Home the arm, and record the physical recovery."""
    if str(config.get("arm", "mode")).lower() != "real":
        return {"skipped": True, "reason": "arm mode is not real"}

    controller = _real_controller(manager)
    print("\n本轮 Pipeline 已完成：先打开夹爪，再返回自定义 Home……")
    _append_event(
        paths.events_jsonl,
        {
            "event": "post_grasp_recovery_started",
            "trial_number": trial_number,
            "run_id": run_id,
        },
    )
    try:
        gripper = _open_gripper_after_grasp(controller, config)
        home = _home_existing_uart(controller, home_document)
    except Exception as exc:
        _append_event(
            paths.events_jsonl,
            {
                "event": "post_grasp_recovery_failed",
                "trial_number": trial_number,
                "run_id": run_id,
                "error": f"{type(exc).__name__}: {exc}",
            },
        )
        raise

    result = {
        "success": True,
        "gripper": gripper,
        "home": home,
    }
    _append_event(
        paths.events_jsonl,
        {
            "event": "post_grasp_recovery_complete",
            "trial_number": trial_number,
            "run_id": run_id,
            "recovery": result,
        },
    )
    print(
        "物理复位完成："
        f"gripper={gripper['after_hand_rad']:.3f} rad, "
        f"home_error_deg={home['error_deg']}"
    )
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Continuous, operator-labelled EEE8097 grasp baseline test"
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--home-config", default=str(DEFAULT_HOME_CONFIG))
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--session-id")
    parser.add_argument("--session-label", default="workspace-baseline")
    parser.add_argument(
        "--skip-home",
        action="store_true",
        help="Skip T=122 startup home for this session",
    )
    parser.add_argument(
        "--validate-config",
        action="store_true",
        help="Build and validate the session config without opening devices",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    session_id = args.session_id or _new_session_id()
    if not SESSION_ID_PATTERN.fullmatch(session_id):
        raise ValueError(
            "session-id 只能包含字母、数字、点、下划线和连字符，最长64字符"
        )

    output_root = Path(args.output_root).expanduser().resolve()
    session_root = output_root / session_id
    if session_root.exists() and any(session_root.iterdir()):
        raise FileExistsError(
            f"Session directory is not empty; choose a new --session-id: {session_root}"
        )
    session_root.mkdir(parents=True, exist_ok=True)
    paths = SessionPaths(
        root=session_root,
        config=session_root / "effective_config.yaml",
        manifest=session_root / "session.json",
        trials_csv=session_root / "trials.csv",
        events_jsonl=session_root / "events.jsonl",
        summary=session_root / "summary.json",
    )

    base_config = load_runtime_config(args.config)
    home_document = load_home_document(args.home_config)
    effective_data = prepare_effective_data(
        base_config.data,
        home_document,
        output_dir=session_root,
    )
    paths.config.write_text(
        yaml.safe_dump(effective_data, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    config = RuntimeConfig(
        path=paths.config,
        data=effective_data,
        source_paths=(*base_config.source_paths, paths.config),
    )
    config.validate()

    manifest: dict[str, Any] = {
        "session_id": session_id,
        "session_label": args.session_label,
        "started_at_utc": _utc_now(),
        "finished_at_utc": None,
        "source_config": str(Path(args.config).expanduser().resolve()),
        "effective_config": str(paths.config),
        "output_dir": str(session_root),
        "safety": {
            "startup_home_skipped": bool(args.skip_home),
            "per_trial_gate": "RUN",
            "continuous_confirmation_phrase": CONTINUOUS_CONFIRMATION_PHRASE,
            "committed_one_grasp_per_process_unchanged": True,
            "session_one_grasp_per_process": False,
            "post_complete_recovery": (
                "open_gripper_then_T122_home_then_manager_reset"
            ),
            "automatic_recovery_on_error_or_abort": False,
        },
    }
    _atomic_write_json(paths.manifest, manifest)

    print(f"Session: {session_id}")
    print(f"Output:  {session_root}")
    print(f"Config:  {paths.config}")
    print("正式 modular_pipeline.yaml 未被修改。")
    if args.validate_config:
        print("Continuous grasp test configuration valid; no device was opened.")
        return 0

    home_section = home_document["roarm_home"]
    if should_execute_startup_home(
        base_config,
        home_section,
        skip_home=bool(args.skip_home),
    ):
        print("\n执行本 session 启动 T=122 自定义 Home。")
        execute_custom_home(home_document)

    arm_mode = str(config.get("arm", "mode")).lower()
    if arm_mode == "real":
        print("\n连续真实抓取测试模式：每轮仍必须由操作者输入 RUN。")
        print("每轮 COMPLETE 后将自动打开夹爪并执行 T=122 Home。")
        print("ERROR/ABORTED 后不会自动继续运动；需要人工检查。")
        print("保持机械臂完整运动范围无障碍，并确保电源开关随时可达。")
        entered = input(
            f"输入 {CONTINUOUS_CONFIRMATION_PHRASE!r} 打开设备: "
        )
        if entered != CONTINUOUS_CONFIRMATION_PHRASE:
            raise PermissionError("连续测试确认口令不匹配")

    recorder = ExperimentRecorder.from_runtime_config(config)
    manager = build_manager(config, on_run_complete=recorder.record)
    trial_number = 0
    try:
        while True:
            trial_number += 1
            print("\n" + "=" * 72)
            print(f"准备第 {trial_number} 轮。输入 q 可结束 session。")
            position_label = input(
                f"位置标签 [默认 T{trial_number:03d}，可用 P11/P12/...]: "
            ).strip()
            if position_label.lower() == "q":
                break
            if not position_label:
                position_label = f"T{trial_number:03d}"
            notes_before = input("运行前备注（可留空）: ").strip()

            gate = input(
                "确认目标已摆好、运动范围已清空；输入 RUN 开始，q 结束: "
            ).strip()
            if gate.lower() == "q":
                break
            if gate != "RUN":
                print("未输入 RUN，本轮未执行。")
                trial_number -= 1
                continue

            context_note = (
                f"position_label={position_label}; pre_notes={notes_before}"
            )
            recorder.set_run_context(
                RunContext(repeat_index=trial_number, notes=context_note)
            )
            if not manager.start():
                raise RuntimeError(
                    f"Pipeline rejected trial start from state "
                    f"{manager.status()['state']}"
                )
            _append_event(
                paths.events_jsonl,
                {
                    "event": "trial_started",
                    "trial_number": trial_number,
                    "position_label": position_label,
                    "run_id": manager.status().get("run_id"),
                },
            )

            status = _wait_for_run(manager, paths, trial_number)
            _print_run_result(status)
            run_id = str(status.get("run_id") or "")
            if not run_id:
                raise RuntimeError("Terminal pipeline status is missing run_id")
            if status.get("recording_error"):
                raise RuntimeError(
                    "Run completed but durable recording failed: "
                    f"{status['recording_error']}"
                )

            outcome_name, outcome_bool = _read_outcome()
            notes_after = input("运行后备注/失败原因（可留空）: ").strip()
            combined_notes = (
                f"position_label={position_label}; outcome={outcome_name}; "
                f"pre_notes={notes_before}; post_notes={notes_after}"
            )
            annotate_run(
                jsonl_path=recorder.jsonl_path,
                csv_path=recorder.csv_path,
                run_id=run_id,
                operator_grasp_success=outcome_bool,
                notes=combined_notes,
            )

            timing = status.get("timing") or {}
            execution = status.get("execution") or {}
            pipeline_success = (
                status.get("state") == "COMPLETE"
                and execution.get("success") is not False
            )
            append_trial_row(
                paths.trials_csv,
                {
                    "trial_number": trial_number,
                    "run_id": run_id,
                    "position_label": position_label,
                    "operator_outcome": outcome_name,
                    "pipeline_state": status.get("state"),
                    "pipeline_success": pipeline_success,
                    "execution_success": execution.get("success"),
                    "started_at_utc": timing.get("started_at_utc"),
                    "finished_at_utc": timing.get("finished_at_utc"),
                    "duration_s": timing.get("duration_s"),
                    "notes_before": notes_before,
                    "notes_after": notes_after,
                },
            )
            _append_event(
                paths.events_jsonl,
                {
                    "event": "trial_annotated",
                    "trial_number": trial_number,
                    "run_id": run_id,
                    "position_label": position_label,
                    "operator_outcome": outcome_name,
                },
            )
            summary = _write_summary(recorder, paths.summary)
            outcomes = summary["grasp_outcomes"]
            print(
                "已保存："
                f"累计 {summary['run_count']} 轮，"
                f"人工确认成功 {outcomes['success_count']} 轮，"
                f"失败 {outcomes['failure_count']} 轮，"
                f"成功率 {outcomes['success_rate']}"
            )

            # Physical recovery is intentionally limited to a clean COMPLETE.
            # On ERROR/ABORTED, automatic motion could worsen a collision or jam.
            if not pipeline_success:
                raise RuntimeError(
                    "Pipeline did not complete successfully; automatic gripper/Home "
                    "recovery was skipped. Inspect the arm before continuing."
                )

            _post_grasp_recovery(
                manager,
                config,
                home_document,
                paths,
                trial_number,
                run_id,
            )

            if not manager.reset():
                raise RuntimeError(
                    "Pipeline could not reset after physical recovery"
                )

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在安全结束 session……")
        if manager.is_running():
            manager.abort()
    finally:
        manager.shutdown()
        summary = _write_summary(recorder, paths.summary)
        manifest["finished_at_utc"] = _utc_now()
        manifest["run_count"] = summary["run_count"]
        manifest["complete_count"] = summary["complete_count"]
        manifest["failure_count"] = summary["failure_count"]
        manifest["grasp_outcomes"] = summary["grasp_outcomes"]
        _atomic_write_json(paths.manifest, manifest)

    print("\nSession 已结束，数据均已保留：")
    print(f"  {recorder.jsonl_path}")
    print(f"  {recorder.csv_path}")
    print(f"  {paths.trials_csv}")
    print(f"  {paths.events_jsonl}")
    print(f"  {paths.summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
