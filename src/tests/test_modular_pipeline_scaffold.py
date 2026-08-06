from __future__ import annotations

from pathlib import Path
import sys
import time
import unittest


SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from adapters.mock_devices import MockArmAdapter, MockRangeSensorAdapter, MockVisionAdapter
from lidar.range_sensor import camera_pixel_to_bearing_deg
from localization.target_localizer import PlanarTargetLocalizer
from localization.transforms import IdentityBaseToArmTransform
from pipeline.task_manager import ModularTaskManager, TaskState
from pipeline.types import Detection2D, RangeMeasurement
from planning.grasp_planner import SimpleTopDownGraspPlanner


class BearingTests(unittest.TestCase):
    def test_center_pixel_is_zero_bearing(self) -> None:
        self.assertAlmostEqual(
            camera_pixel_to_bearing_deg(pixel_x=640.0, fx_px=900.0, cx_px=640.0),
            0.0,
        )

    def test_left_pixel_is_positive_bearing(self) -> None:
        self.assertGreater(
            camera_pixel_to_bearing_deg(pixel_x=500.0, fx_px=900.0, cx_px=640.0),
            0.0,
        )


class LocalizationTests(unittest.TestCase):
    def test_planar_localization(self) -> None:
        detection = Detection2D(
            label="cup",
            confidence=0.9,
            center_x=640,
            center_y=360,
            bbox=(600, 300, 680, 420),
            frame_width=1280,
            frame_height=720,
            captured_at_s=0.0,
        )
        measurement = RangeMeasurement(
            distance_mm=500.0,
            camera_bearing_deg=0.0,
            lidar_bearing_deg=0.0,
            base_bearing_deg=0.0,
            sample_count=10,
            median_quality=10.0,
            mad_mm=1.0,
            minimum_mm=498.0,
            maximum_mm=502.0,
            measured_at_s=0.0,
        )
        localizer = PlanarTargetLocalizer(
            base_to_arm=IdentityBaseToArmTransform(),
            target_z_mm=140.0,
        )
        target = localizer.localize(detection, measurement)
        self.assertAlmostEqual(target.target_base_link.x_mm, 500.0)
        self.assertAlmostEqual(target.target_base_link.y_mm, 0.0)
        self.assertEqual(target.target_arm_base.frame_id, "arm_base")
        self.assertTrue(target.provisional)


class PipelineTests(unittest.TestCase):
    def test_complete_mock_pipeline(self) -> None:
        manager = ModularTaskManager(
            vision=MockVisionAdapter(),
            range_sensor=MockRangeSensorAdapter(distance_mm=350.0),
            localizer=PlanarTargetLocalizer(
                base_to_arm=IdentityBaseToArmTransform(),
                target_z_mm=100.0,
            ),
            planner=SimpleTopDownGraspPlanner(),
            arm=MockArmAdapter(step_delay_s=0.001),
        )
        try:
            self.assertTrue(manager.start())
            deadline = time.monotonic() + 2.0
            while manager.is_running() and time.monotonic() < deadline:
                time.sleep(0.005)
            status = manager.status()
            self.assertEqual(status["state"], TaskState.COMPLETE.value)
            self.assertIsNotNone(status["detection"])
            self.assertIsNotNone(status["range"])
            self.assertIsNotNone(status["target"])
            self.assertIsNotNone(status["plan"])
            self.assertTrue(status["execution"]["success"])
        finally:
            manager.shutdown()


if __name__ == "__main__":
    unittest.main()
