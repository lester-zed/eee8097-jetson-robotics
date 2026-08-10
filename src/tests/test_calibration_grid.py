from __future__ import annotations

from pathlib import Path
import sys
import unittest


SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from tools.calibration_grid import build_grid_points, validate_summary


class CalibrationGridTests(unittest.TestCase):
    def test_grid_expands_from_p12_anchor_in_negative_x(self) -> None:
        points = build_grid_points(
            anchor_x_mm=-350.0,
            anchor_y_mm=0.0,
            x_step_mm=30.0,
            y_step_mm=15.0,
            truth_z_mm=-110.0,
        )

        self.assertEqual(
            [point.point_id for point in points],
            [
                "P11",
                "P12",
                "P13",
                "P21",
                "P22",
                "P23",
                "P31",
                "P32",
                "P33",
            ],
        )
        self.assertEqual(
            [(point.x_mm, point.y_mm) for point in points],
            [
                (-350.0, 15.0),
                (-350.0, 0.0),
                (-350.0, -15.0),
                (-380.0, 15.0),
                (-380.0, 0.0),
                (-380.0, -15.0),
                (-410.0, 15.0),
                (-410.0, 0.0),
                (-410.0, -15.0),
            ],
        )
        self.assertTrue(all(point.z_mm == -110.0 for point in points))

    def test_grid_rejects_non_positive_steps(self) -> None:
        with self.assertRaises(ValueError):
            build_grid_points(
                anchor_x_mm=-350.0,
                anchor_y_mm=0.0,
                x_step_mm=0.0,
                y_step_mm=15.0,
                truth_z_mm=None,
            )

    def test_summary_requires_three_complete_samples_per_point(self) -> None:
        points = build_grid_points(
            anchor_x_mm=-350.0,
            anchor_y_mm=0.0,
            x_step_mm=30.0,
            y_step_mm=15.0,
            truth_z_mm=None,
        )
        summary = {
            "run_count": 27,
            "complete_count": 27,
            "failure_count": 0,
            "by_point": {
                point.point_id: {"sample_count": 3} for point in points
            },
        }
        self.assertEqual(
            validate_summary(summary, points=points, repeat=3),
            [],
        )

        summary["by_point"]["P33"]["sample_count"] = 2
        errors = validate_summary(summary, points=points, repeat=3)
        self.assertIn("P33.sample_count=2; expected 3", errors)


if __name__ == "__main__":
    unittest.main()
