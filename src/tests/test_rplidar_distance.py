"""No-hardware tests for the additive RPLIDAR parser and sector filter."""

from pathlib import Path
import sys
import unittest


SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from lidar.rplidar_distance import (
    ScanPoint,
    SectorMeasurementError,
    circular_delta_degrees,
    measure_sector,
    parse_ultra_simple_line,
)


class UltraSimpleParserTests(unittest.TestCase):
    def test_parses_sync_point_from_official_output(self) -> None:
        point = parse_ultra_simple_line(
            "S  theta: 003.52 Dist: 00824.00 Q: 47"
        )
        self.assertIsNotNone(point)
        assert point is not None
        self.assertTrue(point.sync)
        self.assertAlmostEqual(point.angle_deg, 3.52)
        self.assertAlmostEqual(point.distance_mm, 824.0)
        self.assertEqual(point.quality, 47)

    def test_parses_non_sync_point_and_ignores_metadata(self) -> None:
        point = parse_ultra_simple_line(
            "   theta: 359.90 Dist: 00123.25 Q: 7"
        )
        self.assertIsNotNone(point)
        assert point is not None
        self.assertFalse(point.sync)
        self.assertAlmostEqual(point.angle_deg, 359.9)
        self.assertIsNone(
            parse_ultra_simple_line("SLAMTEC Lidar health status : 0")
        )


class SectorMeasurementTests(unittest.TestCase):
    def test_wraparound_sector_includes_359_and_1_degrees(self) -> None:
        self.assertAlmostEqual(circular_delta_degrees(359.0, 0.0), -1.0)
        self.assertAlmostEqual(circular_delta_degrees(1.0, 0.0), 1.0)
        points = [
            ScanPoint(359.0, 1000.0, 20),
            ScanPoint(0.0, 1010.0, 22),
            ScanPoint(1.0, 1020.0, 24),
            ScanPoint(20.0, 500.0, 30),
        ]
        result = measure_sector(
            points,
            centre_angle_deg=0.0,
            half_width_deg=2.0,
            min_range_mm=100.0,
            max_range_mm=2000.0,
            outlier_mm=None,
        )
        self.assertEqual(result.sample_count, 3)
        self.assertAlmostEqual(result.distance_mm, 1010.0)

    def test_zero_range_low_quality_and_outlier_are_filtered(self) -> None:
        points = [
            ScanPoint(0.0, 0.0, 30),
            ScanPoint(0.1, 1000.0, 0),
            ScanPoint(0.2, 990.0, 20),
            ScanPoint(0.3, 1000.0, 22),
            ScanPoint(0.4, 1010.0, 24),
            ScanPoint(0.5, 4000.0, 25),
        ]
        result = measure_sector(
            points,
            centre_angle_deg=0.0,
            half_width_deg=1.0,
            min_range_mm=120.0,
            max_range_mm=5000.0,
            min_quality=1,
            min_points=3,
            outlier_mm=150.0,
        )
        self.assertEqual(result.sample_count, 3)
        self.assertAlmostEqual(result.distance_mm, 1000.0)
        self.assertAlmostEqual(result.mad_mm, 10.0)

    def test_not_enough_points_raises_clear_error(self) -> None:
        with self.assertRaises(SectorMeasurementError):
            measure_sector(
                [ScanPoint(90.0, 1000.0, 20)],
                centre_angle_deg=0.0,
                half_width_deg=5.0,
            )


if __name__ == "__main__":
    unittest.main()
