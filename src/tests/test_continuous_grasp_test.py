from __future__ import annotations

import copy
import csv
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

MODULE_PATH = SOURCE_ROOT / "tools/continuous_grasp_test.py"
SPEC = importlib.util.spec_from_file_location(
    "continuous_grasp_test_under_test",
    MODULE_PATH,
)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"Could not load test target: {MODULE_PATH}")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

append_trial_row = MODULE.append_trial_row
parse_operator_outcome = MODULE.parse_operator_outcome
prepare_effective_data = MODULE.prepare_effective_data


class ContinuousGraspTestHelpers(unittest.TestCase):
    def test_effective_config_is_session_only_and_applies_home_overrides(self) -> None:
        base = {
            "arm": {"one_grasp_per_process": True},
            "planner": {"cartesian_speed": 0.15},
            "experiment": {"enabled": True, "profile": "real_grasp"},
        }
        original = copy.deepcopy(base)
        home = {
            "pipeline_overrides": {
                "enabled": True,
                "values": {"planner": {"cartesian_speed": 0.25}},
            }
        }

        effective = prepare_effective_data(
            base,
            home,
            output_dir=Path("/tmp/baseline-session"),
        )

        self.assertEqual(base, original)
        self.assertFalse(effective["arm"]["one_grasp_per_process"])
        self.assertEqual(effective["planner"]["cartesian_speed"], 0.25)
        self.assertEqual(
            effective["experiment"]["profile"],
            "continuous_grasp_baseline",
        )
        self.assertEqual(
            effective["experiment"]["output_dir"],
            "/tmp/baseline-session",
        )

    def test_operator_outcome_aliases(self) -> None:
        self.assertEqual(parse_operator_outcome("y"), ("success", True))
        self.assertEqual(parse_operator_outcome("N"), ("failure", False))
        self.assertEqual(parse_operator_outcome("u"), ("uncertain", None))
        with self.assertRaises(ValueError):
            parse_operator_outcome("maybe")

    def test_trial_csv_is_append_only_with_one_header(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "trials.csv"
            append_trial_row(
                output,
                {
                    "trial_number": 1,
                    "run_id": "run-1",
                    "position_label": "P11",
                    "operator_outcome": "success",
                },
            )
            append_trial_row(
                output,
                {
                    "trial_number": 2,
                    "run_id": "run-2",
                    "position_label": "P12",
                    "operator_outcome": "failure",
                },
            )

            with output.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["position_label"], "P11")
            self.assertEqual(rows[1]["operator_outcome"], "failure")


if __name__ == "__main__":
    unittest.main()
