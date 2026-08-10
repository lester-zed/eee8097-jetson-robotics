#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from experiments.recorder import CalibrationGroundTruth, annotate_run


DEFAULT_LOG_DIR = Path("/workspace/logs/experiments")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Add measured ground truth or grasp outcome to one run"
    )
    parser.add_argument("--run-id", required=True)
    parser.add_argument(
        "--jsonl", default=str(DEFAULT_LOG_DIR / "runs.jsonl")
    )
    parser.add_argument("--csv", default=str(DEFAULT_LOG_DIR / "runs.csv"))
    parser.add_argument("--point-id")
    parser.add_argument("--truth-x-mm", type=float)
    parser.add_argument("--truth-y-mm", type=float)
    parser.add_argument("--truth-z-mm", type=float)
    parser.add_argument("--grasp-success", choices=("yes", "no"))
    parser.add_argument("--notes")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    truth_values = (args.truth_x_mm, args.truth_y_mm)
    if any(value is not None for value in truth_values):
        if any(value is None for value in truth_values) or not args.point_id:
            raise ValueError(
                "Ground-truth annotation requires --point-id, "
                "--truth-x-mm, and --truth-y-mm together"
            )
        ground_truth = CalibrationGroundTruth(
            point_id=args.point_id,
            x_mm=args.truth_x_mm,
            y_mm=args.truth_y_mm,
            z_mm=args.truth_z_mm,
        )
    else:
        if args.point_id or args.truth_z_mm is not None:
            raise ValueError(
                "--point-id/--truth-z-mm require X and Y ground truth"
            )
        ground_truth = None

    if ground_truth is None and args.grasp_success is None and args.notes is None:
        raise ValueError("No annotation was supplied")

    success = (
        None if args.grasp_success is None else args.grasp_success == "yes"
    )
    record = annotate_run(
        jsonl_path=args.jsonl,
        csv_path=args.csv,
        run_id=args.run_id,
        ground_truth=ground_truth,
        operator_grasp_success=success,
        notes=args.notes,
    )
    print(
        json.dumps(
            {
                "run_id": record["run_id"],
                "operator": record.get("operator"),
                "metrics": record.get("metrics"),
            },
            indent=2,
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
