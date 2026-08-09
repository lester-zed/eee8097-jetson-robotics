from __future__ import annotations

import copy
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
from threading import Lock
from typing import Any, Iterable

from configuration.loader import ConfigError, RuntimeConfig


SCHEMA_VERSION = 1

CSV_FIELDS = (
    "schema_version",
    "run_id",
    "profile",
    "point_id",
    "repeat_index",
    "started_at_utc",
    "finished_at_utc",
    "duration_s",
    "git_commit",
    "config_sha256",
    "camera_mode",
    "lidar_mode",
    "arm_mode",
    "state",
    "pipeline_success",
    "failure_type",
    "error",
    "detection_label",
    "detection_confidence",
    "center_x_px",
    "center_y_px",
    "bbox_json",
    "range_mm",
    "camera_bearing_deg",
    "lidar_bearing_deg",
    "base_bearing_deg",
    "range_sample_count",
    "range_mad_mm",
    "range_minimum_mm",
    "range_maximum_mm",
    "target_base_x_mm",
    "target_base_y_mm",
    "target_base_z_mm",
    "target_arm_x_mm",
    "target_arm_y_mm",
    "target_arm_z_mm",
    "recheck_center_shift_px",
    "recheck_range_shift_mm",
    "recheck_target_shift_mm",
    "grasp_y_offset_mm",
    "waypoints_json",
    "execution_success",
    "execution_message",
    "ground_truth_x_mm",
    "ground_truth_y_mm",
    "ground_truth_z_mm",
    "error_x_mm",
    "error_y_mm",
    "error_z_mm",
    "planar_error_mm",
    "operator_grasp_success",
    "notes",
    "config_json",
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True)
class CalibrationGroundTruth:
    point_id: str
    x_mm: float
    y_mm: float
    z_mm: float | None = None
    frame_id: str = "base_link"

    def __post_init__(self) -> None:
        if not self.point_id.strip():
            raise ValueError("point_id must be non-empty")
        if self.frame_id != "base_link":
            raise ValueError("ground truth must use frame_id='base_link'")
        object.__setattr__(self, "x_mm", _finite(self.x_mm, "x_mm"))
        object.__setattr__(self, "y_mm", _finite(self.y_mm, "y_mm"))
        if self.z_mm is not None:
            object.__setattr__(self, "z_mm", _finite(self.z_mm, "z_mm"))

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class RunContext:
    ground_truth: CalibrationGroundTruth | None = None
    repeat_index: int | None = None
    operator_grasp_success: bool | None = None
    notes: str = ""

    def __post_init__(self) -> None:
        if self.repeat_index is not None and int(self.repeat_index) < 1:
            raise ValueError("repeat_index must be >= 1")

    def to_dict(self) -> dict[str, Any]:
        return {
            "ground_truth": (
                self.ground_truth.to_dict() if self.ground_truth else None
            ),
            "repeat_index": self.repeat_index,
            "operator_grasp_success": self.operator_grasp_success,
            "notes": self.notes,
        }


def _config_digest(data: dict[str, Any]) -> str:
    payload = json.dumps(
        data,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _resolve_git_commit(config_path: Path) -> str:
    configured = os.environ.get("EEE8097_GIT_COMMIT", "").strip()
    if configured and configured.lower() != "unknown":
        return configured

    for candidate in config_path.parents:
        if not (candidate / ".git").exists():
            continue
        try:
            completed = subprocess.run(
                ["git", "-C", str(candidate), "rev-parse", "HEAD"],
                check=True,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except (OSError, subprocess.SubprocessError):
            break
        commit = completed.stdout.strip()
        if commit:
            return commit
    return "unknown"


def _mapping(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _calculate_metrics(record: dict[str, Any]) -> dict[str, Any]:
    operator = _mapping(record.get("operator"))
    truth = _mapping(operator.get("ground_truth"))
    pipeline = _mapping(record.get("pipeline"))
    target = _mapping(pipeline.get("target"))
    estimate = _mapping(target.get("target_base_link"))
    if not truth or not estimate:
        return {}

    error_x = float(estimate["x_mm"]) - float(truth["x_mm"])
    error_y = float(estimate["y_mm"]) - float(truth["y_mm"])
    metrics: dict[str, Any] = {
        "error_x_mm": error_x,
        "error_y_mm": error_y,
        "planar_error_mm": math.hypot(error_x, error_y),
    }
    if truth.get("z_mm") is not None and estimate.get("z_mm") is not None:
        metrics["error_z_mm"] = (
            float(estimate["z_mm"]) - float(truth["z_mm"])
        )
    return metrics


def _failure_type(pipeline: dict[str, Any]) -> str | None:
    state = str(pipeline.get("state", ""))
    error = pipeline.get("error")
    if error:
        return str(error).split(":", 1)[0]
    if state == "ABORTED":
        return "ABORTED"
    if state and state != "COMPLETE":
        return state
    return None


def _json_cell(value: Any) -> str:
    if value is None:
        return ""
    return json.dumps(value, ensure_ascii=False, sort_keys=True)


def _flatten_record(record: dict[str, Any]) -> dict[str, Any]:
    pipeline = _mapping(record.get("pipeline"))
    detection = _mapping(pipeline.get("detection"))
    measurement = _mapping(pipeline.get("range"))
    target = _mapping(pipeline.get("target"))
    base = _mapping(target.get("target_base_link"))
    arm = _mapping(target.get("target_arm_base"))
    verification = _mapping(pipeline.get("verification"))
    plan = _mapping(pipeline.get("plan"))
    plan_metadata = _mapping(plan.get("metadata"))
    execution = _mapping(pipeline.get("execution"))
    operator = _mapping(record.get("operator"))
    truth = _mapping(operator.get("ground_truth"))
    metrics = _mapping(record.get("metrics"))
    timing = _mapping(pipeline.get("timing"))
    modes = _mapping(record.get("modes"))
    config = _mapping(record.get("config"))

    pipeline_success = (
        pipeline.get("state") == "COMPLETE"
        and execution.get("success") is not False
    )
    return {
        "schema_version": record.get("schema_version"),
        "run_id": record.get("run_id"),
        "profile": record.get("profile"),
        "point_id": truth.get("point_id"),
        "repeat_index": operator.get("repeat_index"),
        "started_at_utc": timing.get("started_at_utc"),
        "finished_at_utc": timing.get("finished_at_utc"),
        "duration_s": timing.get("duration_s"),
        "git_commit": record.get("git_commit"),
        "config_sha256": record.get("config_sha256"),
        "camera_mode": modes.get("camera"),
        "lidar_mode": modes.get("lidar"),
        "arm_mode": modes.get("arm"),
        "state": pipeline.get("state"),
        "pipeline_success": pipeline_success,
        "failure_type": _failure_type(pipeline),
        "error": pipeline.get("error"),
        "detection_label": detection.get("label"),
        "detection_confidence": detection.get("confidence"),
        "center_x_px": detection.get("center_x"),
        "center_y_px": detection.get("center_y"),
        "bbox_json": _json_cell(detection.get("bbox")),
        "range_mm": measurement.get("distance_mm"),
        "camera_bearing_deg": measurement.get("camera_bearing_deg"),
        "lidar_bearing_deg": measurement.get("lidar_bearing_deg"),
        "base_bearing_deg": measurement.get("base_bearing_deg"),
        "range_sample_count": measurement.get("sample_count"),
        "range_mad_mm": measurement.get("mad_mm"),
        "range_minimum_mm": measurement.get("minimum_mm"),
        "range_maximum_mm": measurement.get("maximum_mm"),
        "target_base_x_mm": base.get("x_mm"),
        "target_base_y_mm": base.get("y_mm"),
        "target_base_z_mm": base.get("z_mm"),
        "target_arm_x_mm": arm.get("x_mm"),
        "target_arm_y_mm": arm.get("y_mm"),
        "target_arm_z_mm": arm.get("z_mm"),
        "recheck_center_shift_px": verification.get("center_shift_px"),
        "recheck_range_shift_mm": verification.get("range_shift_mm"),
        "recheck_target_shift_mm": verification.get("target_shift_mm"),
        "grasp_y_offset_mm": plan_metadata.get("grasp_y_offset_mm"),
        "waypoints_json": _json_cell(plan.get("waypoints")),
        "execution_success": execution.get("success"),
        "execution_message": execution.get("message"),
        "ground_truth_x_mm": truth.get("x_mm"),
        "ground_truth_y_mm": truth.get("y_mm"),
        "ground_truth_z_mm": truth.get("z_mm"),
        "error_x_mm": metrics.get("error_x_mm"),
        "error_y_mm": metrics.get("error_y_mm"),
        "error_z_mm": metrics.get("error_z_mm"),
        "planar_error_mm": metrics.get("planar_error_mm"),
        "operator_grasp_success": operator.get("operator_grasp_success"),
        "notes": operator.get("notes"),
        "config_json": _json_cell(config),
    }


def _write_csv_header_and_rows(
    path: Path,
    records: Iterable[dict[str, Any]],
) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for record in records:
            writer.writerow(_flatten_record(record))


class ExperimentRecorder:
    def __init__(
        self,
        *,
        config: RuntimeConfig,
        output_dir: str | Path,
        profile: str,
        jsonl_filename: str = "runs.jsonl",
        csv_filename: str = "runs.csv",
        git_commit: str | None = None,
    ) -> None:
        if Path(jsonl_filename).name != jsonl_filename:
            raise ValueError("jsonl_filename must not contain directories")
        if Path(csv_filename).name != csv_filename:
            raise ValueError("csv_filename must not contain directories")
        if not profile.strip():
            raise ValueError("profile must be non-empty")

        self.config = config
        self.output_dir = Path(output_dir).expanduser()
        self.profile = profile.strip()
        self.jsonl_path = self.output_dir / jsonl_filename
        self.csv_path = self.output_dir / csv_filename
        self.git_commit = git_commit or _resolve_git_commit(config.path)
        self._config_sha256 = _config_digest(config.data)
        self._lock = Lock()
        self._next_context = RunContext()
        self.last_run_id: str | None = None

    @classmethod
    def from_runtime_config(cls, config: RuntimeConfig) -> "ExperimentRecorder":
        section = config.data.get("experiment")
        if not isinstance(section, dict) or not bool(section.get("enabled", False)):
            raise ConfigError("experiment logging is not enabled")
        return cls(
            config=config,
            output_dir=str(section["output_dir"]),
            profile=str(section["profile"]),
            jsonl_filename=str(section.get("jsonl_filename", "runs.jsonl")),
            csv_filename=str(section.get("csv_filename", "runs.csv")),
        )

    def set_run_context(self, context: RunContext) -> None:
        with self._lock:
            self._next_context = context

    def record(self, pipeline_status: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            context = self._next_context
            self._next_context = RunContext()
            record = self._build_record(pipeline_status, context)
            self.output_dir.mkdir(parents=True, exist_ok=True)

            with self.jsonl_path.open("a", encoding="utf-8") as handle:
                handle.write(
                    json.dumps(record, ensure_ascii=False, sort_keys=True)
                    + "\n"
                )

            new_csv = not self.csv_path.exists() or self.csv_path.stat().st_size == 0
            with self.csv_path.open("a", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
                if new_csv:
                    writer.writeheader()
                writer.writerow(_flatten_record(record))

            self.last_run_id = str(record["run_id"])
            return copy.deepcopy(record)

    def _build_record(
        self,
        pipeline_status: dict[str, Any],
        context: RunContext,
    ) -> dict[str, Any]:
        pipeline = copy.deepcopy(pipeline_status)
        run_id = str(pipeline.get("run_id") or "").strip()
        if not run_id:
            raise ValueError("pipeline status is missing run_id")
        record: dict[str, Any] = {
            "schema_version": SCHEMA_VERSION,
            "recorded_at_utc": _utc_now(),
            "run_id": run_id,
            "profile": self.profile,
            "git_commit": self.git_commit,
            "config_path": str(self.config.path),
            "config_sources": [
                str(item)
                for item in (self.config.source_paths or (self.config.path,))
            ],
            "config_sha256": self._config_sha256,
            "config": copy.deepcopy(self.config.data),
            "modes": {
                name: str(self.config.get(name, "mode")).lower()
                for name in ("camera", "lidar", "arm")
            },
            "pipeline": pipeline,
            "operator": context.to_dict(),
            "outcome": {
                "state": pipeline.get("state"),
                "success": (
                    pipeline.get("state") == "COMPLETE"
                    and _mapping(pipeline.get("execution")).get("success")
                    is not False
                ),
                "failure_type": _failure_type(pipeline),
                "error": pipeline.get("error"),
            },
        }
        record["metrics"] = _calculate_metrics(record)
        return record


def load_jsonl_records(path: str | Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    log_path = Path(path).expanduser()
    with log_path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            payload = json.loads(stripped)
            if not isinstance(payload, dict):
                raise ValueError(f"JSONL line {line_number} is not an object")
            records.append(payload)
    return records


def annotate_run(
    *,
    jsonl_path: str | Path,
    csv_path: str | Path,
    run_id: str,
    ground_truth: CalibrationGroundTruth | None = None,
    operator_grasp_success: bool | None = None,
    notes: str | None = None,
) -> dict[str, Any]:
    jsonl = Path(jsonl_path).expanduser()
    csv_output = Path(csv_path).expanduser()
    records = load_jsonl_records(jsonl)
    matches = [record for record in records if record.get("run_id") == run_id]
    if len(matches) != 1:
        raise KeyError(
            f"Expected exactly one run_id={run_id!r}, found {len(matches)}"
        )
    record = matches[0]
    operator = _mapping(record.setdefault("operator", {}))
    if ground_truth is not None:
        operator["ground_truth"] = ground_truth.to_dict()
    if operator_grasp_success is not None:
        operator["operator_grasp_success"] = operator_grasp_success
    if notes is not None:
        operator["notes"] = notes
    record["operator"] = operator
    record["metrics"] = _calculate_metrics(record)
    record["annotated_at_utc"] = _utc_now()

    jsonl.parent.mkdir(parents=True, exist_ok=True)
    csv_output.parent.mkdir(parents=True, exist_ok=True)
    json_temp: Path | None = None
    csv_temp: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=jsonl.parent,
            prefix=f".{jsonl.name}.",
            delete=False,
        ) as handle:
            json_temp = Path(handle.name)
            for item in records:
                handle.write(
                    json.dumps(item, ensure_ascii=False, sort_keys=True) + "\n"
                )

        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=csv_output.parent,
            prefix=f".{csv_output.name}.",
            delete=False,
        ) as handle:
            csv_temp = Path(handle.name)
        _write_csv_header_and_rows(csv_temp, records)

        os.replace(json_temp, jsonl)
        json_temp = None
        os.replace(csv_temp, csv_output)
        csv_temp = None
    finally:
        for temporary in (json_temp, csv_temp):
            if temporary is not None:
                try:
                    temporary.unlink()
                except FileNotFoundError:
                    pass
    return copy.deepcopy(record)
