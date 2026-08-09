#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import uuid


SOURCE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = SOURCE_ROOT / "configs/calibration_capture.yaml"
DEFAULT_OUTPUT_ROOT = Path("/workspace/logs/calibration/sessions")
CAPTURE_SCRIPT = SOURCE_ROOT / "run_calibration_capture.sh"
REPORT_SCRIPT = SOURCE_ROOT / "run_calibration_report.sh"


@dataclass(frozen=True)
class GridPoint:
    point_id: str
    x_mm: float
    y_mm: float
    z_mm: float | None


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def build_grid_points(
    *,
    anchor_x_mm: float,
    anchor_y_mm: float,
    x_step_mm: float,
    y_step_mm: float,
    truth_z_mm: float | None,
) -> list[GridPoint]:
    """Build the asymmetric 3 x 3 grid whose near-row centre is P12."""
    anchor_x = _finite(anchor_x_mm, "anchor_x_mm")
    anchor_y = _finite(anchor_y_mm, "anchor_y_mm")
    x_step = _finite(x_step_mm, "x_step_mm")
    y_step = _finite(y_step_mm, "y_step_mm")
    z_value = (
        None if truth_z_mm is None else _finite(truth_z_mm, "truth_z_mm")
    )
    if x_step <= 0.0:
        raise ValueError("x_step_mm must be positive")
    if y_step <= 0.0:
        raise ValueError("y_step_mm must be positive")

    x_values = (anchor_x, anchor_x - x_step, anchor_x - 2.0 * x_step)
    y_values = (anchor_y + y_step, anchor_y, anchor_y - y_step)
    return [
        GridPoint(
            point_id=f"P{row_index}{column_index}",
            x_mm=x_value,
            y_mm=y_value,
            z_mm=z_value,
        )
        for row_index, x_value in enumerate(x_values, start=1)
        for column_index, y_value in enumerate(y_values, start=1)
    ]


def validate_summary(
    summary: dict,
    *,
    points: list[GridPoint],
    repeat: int,
) -> list[str]:
    expected = len(points) * repeat
    errors: list[str] = []
    if summary.get("run_count") != expected:
        errors.append(
            f"run_count={summary.get('run_count')!r}; expected {expected}"
        )
    if summary.get("complete_count") != expected:
        errors.append(
            f"complete_count={summary.get('complete_count')!r}; expected {expected}"
        )
    if summary.get("failure_count") != 0:
        errors.append(
            f"failure_count={summary.get('failure_count')!r}; expected 0"
        )

    by_point = summary.get("by_point")
    if not isinstance(by_point, dict):
        errors.append("by_point is missing or invalid")
        return errors
    for point in points:
        item = by_point.get(point.point_id)
        if not isinstance(item, dict):
            errors.append(f"{point.point_id} is missing from by_point")
        elif item.get("sample_count") != repeat:
            errors.append(
                f"{point.point_id}.sample_count={item.get('sample_count')!r}; "
                f"expected {repeat}"
            )
    return errors


def _default_session_id() -> str:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"grid-{timestamp}-{uuid.uuid4().hex[:6]}"


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _validate_session_id(value: str) -> str:
    session_id = value.strip()
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,79}", session_id):
        raise ValueError(
            "session-id must be 1-80 characters using only letters, numbers, "
            "dot, underscore, or hyphen"
        )
    return session_id


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _write_plan_csv(path: Path, points: list[GridPoint], repeat: int) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=(
                "point_id",
                "truth_x_mm",
                "truth_y_mm",
                "truth_z_mm",
                "repeat",
            ),
        )
        writer.writeheader()
        for point in points:
            writer.writerow(
                {
                    "point_id": point.point_id,
                    "truth_x_mm": point.x_mm,
                    "truth_y_mm": point.y_mm,
                    "truth_z_mm": "" if point.z_mm is None else point.z_mm,
                    "repeat": repeat,
                }
            )


def _format_coordinate(value: float | None) -> str:
    return "not supplied" if value is None else f"{value:.1f}"


def _print_plan(points: list[GridPoint], repeat: int) -> None:
    print("\n3 x 3 calibration plan (base_link, millimetres)")
    print("Point       X        Y             Z  Samples")
    print("-----  -------  -------  ------------  -------")
    for point in points:
        print(
            f"{point.point_id:<5}  {point.x_mm:>7.1f}  {point.y_mm:>7.1f}  "
            f"{_format_coordinate(point.z_mm):>12}  {repeat:>7}"
        )
    print(f"Total requested samples: {len(points) * repeat}")


def _run_report(session_dir: Path) -> bool:
    jsonl_path = session_dir / "runs.jsonl"
    if not jsonl_path.is_file() or jsonl_path.stat().st_size == 0:
        print("No recorded samples are available for a report.")
        return False
    completed = subprocess.run(
        [
            "bash",
            str(REPORT_SCRIPT),
            "--input",
            str(jsonl_path),
            "--output",
            str(session_dir / "summary.json"),
        ],
        check=False,
    )
    return completed.returncode == 0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Interactively capture a no-motion 3 x 3 Camera + RPLIDAR grid. "
            "P12 is the measured near-row anchor; the grid expands in -X."
        )
    )
    parser.add_argument("--anchor-x-mm", type=float, required=True)
    parser.add_argument("--anchor-y-mm", type=float, required=True)
    parser.add_argument("--truth-z-mm", type=float)
    parser.add_argument("--x-step-mm", type=float, default=30.0)
    parser.add_argument("--y-step-mm", type=float, default=15.0)
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--timeout-s", type=float, default=60.0)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--session-id", default=None)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the generated grid without creating files or opening devices",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.repeat < 1:
        raise ValueError("--repeat must be >= 1")
    if args.timeout_s <= 0.0 or not math.isfinite(args.timeout_s):
        raise ValueError("--timeout-s must be finite and positive")

    points = build_grid_points(
        anchor_x_mm=args.anchor_x_mm,
        anchor_y_mm=args.anchor_y_mm,
        x_step_mm=args.x_step_mm,
        y_step_mm=args.y_step_mm,
        truth_z_mm=args.truth_z_mm,
    )
    _print_plan(points, args.repeat)
    print(
        "\nP12 is the measured anchor. Confirm every generated coordinate "
        "against the physical marks before collecting data."
    )
    if args.dry_run:
        print("Dry run only: no files, Camera, RPLIDAR, or RoArm were opened.")
        return 0

    session_id = _validate_session_id(args.session_id or _default_session_id())
    output_root = Path(args.output_root).expanduser().resolve()
    session_dir = output_root / session_id
    if session_dir.exists():
        raise FileExistsError(
            f"Session directory already exists; choose another --session-id: "
            f"{session_dir}"
        )

    print("\nSafety boundary: Camera=REAL, RPLIDAR=REAL, RoArm=MOCK.")
    print("The RPLIDAR rotates during each capture; keep hands and cables clear.")
    print("The target must remain still during the three repeats at each point.")
    if input("Type START after checking the complete plan: ").strip() != "START":
        print("Cancelled before creating the session or opening devices.")
        return 1

    session_dir.mkdir(parents=True, exist_ok=False)
    session_config = session_dir / "calibration_capture.session.yaml"
    source_config = Path(args.config).expanduser().resolve()
    _write_json(
        session_config,
        {
            "extends": str(source_config),
            "experiment": {"output_dir": str(session_dir)},
        },
    )
    _write_plan_csv(session_dir / "plan.csv", points, args.repeat)

    manifest = {
        "schema_version": 1,
        "session_id": session_id,
        "created_at_utc": _utc_now(),
        "status": "running",
        "source_config": str(source_config),
        "session_config": str(session_config),
        "output_dir": str(session_dir),
        "repeat_per_point": args.repeat,
        "requested_sample_count": len(points) * args.repeat,
        "points": [asdict(point) for point in points],
    }
    manifest_path = session_dir / "session.json"
    _write_json(manifest_path, manifest)

    validation = subprocess.run(
        [
            "bash",
            str(CAPTURE_SCRIPT),
            "--config",
            str(session_config),
            "--validate-config",
        ],
        check=False,
    )
    if validation.returncode != 0:
        manifest.update(
            status="config_validation_failed",
            finished_at_utc=_utc_now(),
        )
        _write_json(manifest_path, manifest)
        return validation.returncode

    try:
        for point_number, point in enumerate(points, start=1):
            print("\n" + "=" * 72)
            z_description = (
                "not supplied"
                if point.z_mm is None
                else f"{point.z_mm:.1f} mm"
            )
            print(
                f"Point {point_number}/9: {point.point_id}  "
                f"X={point.x_mm:.1f}, Y={point.y_mm:.1f}, "
                f"Z={z_description}"
            )
            print(
                f"Place the target centre on {point.point_id}; then leave it "
                f"still for {args.repeat} measurements."
            )
            reply = input("Press Enter when ready, or type q to stop: ").strip()
            if reply.lower() in {"q", "quit"}:
                manifest.update(
                    status="aborted_by_operator",
                    stopped_before_point=point.point_id,
                    finished_at_utc=_utc_now(),
                )
                _write_json(manifest_path, manifest)
                _run_report(session_dir)
                print(f"Partial session preserved: {session_dir}")
                return 130

            command = [
                "bash",
                str(CAPTURE_SCRIPT),
                "--config",
                str(session_config),
                "--point-id",
                point.point_id,
                "--truth-x-mm",
                str(point.x_mm),
                "--truth-y-mm",
                str(point.y_mm),
                "--repeat",
                str(args.repeat),
                "--timeout-s",
                str(args.timeout_s),
                "--notes",
                f"session={session_id}; grid point {point_number}/9",
            ]
            if point.z_mm is not None:
                command.extend(("--truth-z-mm", str(point.z_mm)))
            completed = subprocess.run(command, check=False)
            if completed.returncode != 0:
                manifest.update(
                    status="stopped_after_capture_failure",
                    failed_point=point.point_id,
                    finished_at_utc=_utc_now(),
                )
                _write_json(manifest_path, manifest)
                _run_report(session_dir)
                print("No automatic retry was attempted.")
                print(f"Partial session preserved: {session_dir}")
                return completed.returncode
    except KeyboardInterrupt:
        print("\nStopped by operator; no automatic retry was attempted.")
        manifest.update(
            status="aborted_by_operator",
            finished_at_utc=_utc_now(),
        )
        _write_json(manifest_path, manifest)
        _run_report(session_dir)
        print(f"Partial session preserved: {session_dir}")
        return 130

    if not _run_report(session_dir):
        manifest.update(status="report_failed", finished_at_utc=_utc_now())
        _write_json(manifest_path, manifest)
        return 3

    summary_path = session_dir / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    validation_errors = validate_summary(
        summary,
        points=points,
        repeat=args.repeat,
    )
    if validation_errors:
        manifest.update(
            status="result_validation_failed",
            validation_errors=validation_errors,
            finished_at_utc=_utc_now(),
        )
        _write_json(manifest_path, manifest)
        print("Session result validation failed:")
        for error in validation_errors:
            print(f"- {error}")
        return 4

    manifest.update(status="complete", finished_at_utc=_utc_now())
    _write_json(manifest_path, manifest)
    expected = len(points) * args.repeat
    print("\n" + "=" * 72)
    print(f"Calibration grid complete: {expected}/{expected} successful samples")
    print(f"Session directory: {session_dir}")
    print(f"Report: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
