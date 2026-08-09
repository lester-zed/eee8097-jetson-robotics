from __future__ import annotations

from collections import Counter, defaultdict
import math
from statistics import fmean, pstdev
from typing import Any, Iterable


def _mapping(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _error_summary(records: list[dict[str, Any]]) -> dict[str, Any]:
    errors = [
        _mapping(record.get("metrics"))
        for record in records
        if _mapping(record.get("pipeline")).get("state") == "COMPLETE"
        and _mapping(record.get("metrics")).get("error_x_mm") is not None
        and _mapping(record.get("metrics")).get("error_y_mm") is not None
    ]
    if not errors:
        return {
            "sample_count": 0,
            "bias_x_mm": None,
            "bias_y_mm": None,
            "rmse_x_mm": None,
            "rmse_y_mm": None,
            "planar_rmse_mm": None,
            "std_x_mm": None,
            "std_y_mm": None,
        }

    error_x = [float(item["error_x_mm"]) for item in errors]
    error_y = [float(item["error_y_mm"]) for item in errors]
    return {
        "sample_count": len(errors),
        "bias_x_mm": fmean(error_x),
        "bias_y_mm": fmean(error_y),
        "rmse_x_mm": math.sqrt(fmean(value * value for value in error_x)),
        "rmse_y_mm": math.sqrt(fmean(value * value for value in error_y)),
        "planar_rmse_mm": math.sqrt(
            fmean(
                x_value * x_value + y_value * y_value
                for x_value, y_value in zip(error_x, error_y)
            )
        ),
        "std_x_mm": pstdev(error_x),
        "std_y_mm": pstdev(error_y),
    }


def summarize_records(records: Iterable[dict[str, Any]]) -> dict[str, Any]:
    items = list(records)
    states = Counter(
        str(_mapping(record.get("pipeline")).get("state", "UNKNOWN"))
        for record in items
    )
    failures = Counter()
    for record in items:
        pipeline = _mapping(record.get("pipeline"))
        state = str(pipeline.get("state", "UNKNOWN"))
        error = pipeline.get("error")
        if error:
            failures[str(error).split(":", 1)[0]] += 1
        elif state != "COMPLETE":
            failures[state] += 1

    annotated = [
        record
        for record in items
        if _mapping(record.get("operator")).get("operator_grasp_success")
        is not None
    ]
    grasp_successes = sum(
        bool(
            _mapping(record.get("operator")).get("operator_grasp_success")
        )
        for record in annotated
    )

    by_point: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in items:
        truth = _mapping(
            _mapping(record.get("operator")).get("ground_truth")
        )
        point_id = truth.get("point_id")
        if point_id:
            by_point[str(point_id)].append(record)

    return {
        "run_count": len(items),
        "complete_count": states.get("COMPLETE", 0),
        "failure_count": sum(failures.values()),
        "states": dict(sorted(states.items())),
        "failure_types": dict(sorted(failures.items())),
        "calibration": _error_summary(items),
        "by_point": {
            point_id: _error_summary(point_records)
            for point_id, point_records in sorted(by_point.items())
        },
        "grasp_outcomes": {
            "annotated_count": len(annotated),
            "success_count": grasp_successes,
            "failure_count": len(annotated) - grasp_successes,
            "success_rate": (
                grasp_successes / len(annotated) if annotated else None
            ),
        },
    }
