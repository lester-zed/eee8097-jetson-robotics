from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import sys
import unittest


SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from configuration.loader import ConfigError, RuntimeConfig, load_runtime_config
from localization.command_calibration import (
    AffineXYCommandCalibration,
    CalibrationDomainError,
)
from localization.target_localizer import PlanarTargetLocalizer
from localization.transforms import IdentityBaseToArmTransform
from pipeline.types import Detection2D, Point3D, RangeMeasurement
from planning.grasp_planner import SimpleTopDownGraspPlanner, WorkspaceLimits


CONFIG_PATH = SOURCE_ROOT / "configs/modular_pipeline.yaml"
PLAN_ONLY_PATH = SOURCE_ROOT / "configs/calibrated_plan_only.yaml"
CAPTURE_PATH = SOURCE_ROOT / "configs/calibration_capture.yaml"
FIXTURE_PATH = SOURCE_ROOT / "tests/fixtures/command_grid_20260810.json"


def _detection() -> Detection2D:
    return Detection2D(
        label="tissue_pack",
        confidence=0.95,
        center_x=320,
        center_y=240,
        bbox=(280, 180, 360, 300),
        frame_width=640,
        frame_height=480,
        captured_at_s=0.0,
    )


def _measurement(
    x_mm: float | None = None,
    y_mm: float | None = None,
) -> RangeMeasurement:
    target = None
    if x_mm is not None and y_mm is not None:
        target = Point3D(x_mm, y_mm, 0.0, "base_link")
    return RangeMeasurement(
        distance_mm=365.0,
        camera_bearing_deg=0.0,
        lidar_bearing_deg=180.0,
        base_bearing_deg=180.0,
        sample_count=8,
        median_quality=10.0,
        mad_mm=1.0,
        minimum_mm=363.0,
        maximum_mm=367.0,
        measured_at_s=0.0,
        target_base_link=target,
    )


class MeasuredCommandCalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = load_runtime_config(CONFIG_PATH)
        cls.section = cls.config.get(
            "localization", "command_calibration"
        )
        cls.calibration = AffineXYCommandCalibration.from_mapping(cls.section)

    def test_all_27_measured_samples_replay_with_expected_error(self) -> None:
        fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
        errors: list[float] = []
        for sample in fixture["samples"]:
            corrected = self.calibration.apply(
                Point3D(
                    sample["raw_arm_x_mm"],
                    sample["raw_arm_y_mm"],
                    -100.0,
                    "arm_base",
                )
            )
            errors.append(
                math.hypot(
                    corrected.x_mm - sample["command_x_mm"],
                    corrected.y_mm - sample["command_y_mm"],
                )
            )

        planar_rmse = math.sqrt(sum(value * value for value in errors) / len(errors))
        self.assertLessEqual(planar_rmse, 1.080)
        self.assertLessEqual(max(errors), 1.943)

    def test_p12_mean_maps_to_operator_command_anchor(self) -> None:
        fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
        p12 = [item for item in fixture["samples"] if item["point_id"] == "P12"]
        raw_x = sum(item["raw_arm_x_mm"] for item in p12) / len(p12)
        raw_y = sum(item["raw_arm_y_mm"] for item in p12) / len(p12)
        corrected = self.calibration.apply(
            Point3D(raw_x, raw_y, -100.0, "arm_base")
        )
        self.assertAlmostEqual(corrected.x_mm, -380.0391, places=3)
        self.assertAlmostEqual(corrected.y_mm, -20.1200, places=3)
        self.assertEqual(corrected.z_mm, -100.0)

    def test_target_outside_measured_domain_is_rejected(self) -> None:
        with self.assertRaisesRegex(
            CalibrationDomainError,
            "outside measured command-calibration input bounds",
        ):
            self.calibration.apply(
                Point3D(-350.0, -20.0, -100.0, "arm_base")
            )

    def test_calibration_rejects_wrong_frame(self) -> None:
        with self.assertRaisesRegex(ValueError, "requires an arm_base point"):
            self.calibration.apply(
                Point3D(-364.0, -4.2, -100.0, "base_link")
            )


class CalibratedLocalizerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        config = load_runtime_config(CONFIG_PATH)
        cls.calibration = AffineXYCommandCalibration.from_mapping(
            config.get("localization", "command_calibration")
        )

    def _localizer(self) -> PlanarTargetLocalizer:
        return PlanarTargetLocalizer(
            base_to_arm=IdentityBaseToArmTransform(),
            target_z_mm=-100.0,
            calibration_approved=True,
            target_z_approved=True,
            arm_command_calibration=self.calibration,
        )

    @staticmethod
    def _runtime_planner() -> SimpleTopDownGraspPlanner:
        return SimpleTopDownGraspPlanner(
            approach_distance_mm=80.0,
            pregrasp_height_mm=300.0,
            lift_height_mm=200.0,
            grasp_y_offset_mm=0.0,
            limits=WorkspaceLimits(
                min_x_mm=-430.0,
                max_x_mm=500.0,
                min_y_mm=-100.0,
                max_y_mm=100.0,
                min_z_mm=-120.0,
                max_z_mm=280.0,
                max_radius_mm=460.0,
            ),
        )

    def test_raw_sensor_target_is_preserved_and_arm_command_is_corrected(self) -> None:
        target = self._localizer().localize(
            _detection(),
            _measurement(-364.04996807, -4.20773123),
        )
        self.assertAlmostEqual(target.target_base_link.x_mm, -364.04996807)
        self.assertAlmostEqual(target.target_base_link.y_mm, -4.20773123)
        self.assertAlmostEqual(target.target_arm_base.x_mm, -380.0391, places=3)
        self.assertAlmostEqual(target.target_arm_base.y_mm, -20.1200, places=3)
        self.assertEqual(target.target_arm_base.z_mm, -100.0)
        self.assertFalse(target.provisional)
        self.assertEqual(
            target.metadata["nominal_target_arm_base"]["x_mm"],
            target.target_base_link.x_mm,
        )
        self.assertTrue(
            target.metadata["arm_command_calibration"]["applied"]
        )

        plan = self._runtime_planner().plan(target)
        grasp = plan.waypoint("grasp").point
        self.assertAlmostEqual(grasp.x_mm, target.target_arm_base.x_mm)
        self.assertAlmostEqual(grasp.y_mm, target.target_arm_base.y_mm)
        self.assertAlmostEqual(grasp.z_mm, target.target_arm_base.z_mm)

    def test_far_calibration_row_remains_blocked_by_real_workspace(self) -> None:
        target = self._localizer().localize(
            _detection(),
            _measurement(-411.02263265, -10.95571418),
        )
        self.assertLess(target.target_arm_base.x_mm, -430.0)
        with self.assertRaisesRegex(ValueError, "grasp X outside workspace"):
            self._runtime_planner().plan(target)

    def test_bearing_only_fallback_is_rejected_for_real_command_model(self) -> None:
        with self.assertRaisesRegex(
            ValueError,
            "requires a translation-corrected LiDAR target",
        ):
            self._localizer().localize(_detection(), _measurement())


class CommandCalibrationConfigurationTests(unittest.TestCase):
    def test_plan_only_profile_keeps_model_but_disables_roarm(self) -> None:
        config = load_runtime_config(PLAN_ONLY_PATH)
        self.assertEqual(config.get("camera", "mode"), "real")
        self.assertEqual(config.get("lidar", "mode"), "real")
        self.assertEqual(config.get("arm", "mode"), "mock")
        self.assertTrue(
            config.get("localization", "command_calibration")["enabled"]
        )
        self.assertFalse(config.get("arm", "allow_real_motion"))

    def test_raw_capture_profile_disables_fitted_model(self) -> None:
        config = load_runtime_config(CAPTURE_PATH)
        self.assertFalse(
            config.get("localization", "command_calibration")["enabled"]
        )

    def test_nonzero_legacy_y_offset_is_rejected(self) -> None:
        loaded = load_runtime_config(CONFIG_PATH)
        data = copy.deepcopy(loaded.data)
        data["planner"]["grasp_y_offset_mm"] = -15.0
        with self.assertRaisesRegex(ConfigError, "double compensation"):
            RuntimeConfig(loaded.path, data, loaded.source_paths).validate()

    def test_changed_sensor_geometry_is_rejected(self) -> None:
        loaded = load_runtime_config(CONFIG_PATH)
        data = copy.deepcopy(loaded.data)
        data["camera"]["fx_px"] = 901.0
        with self.assertRaisesRegex(
            ConfigError,
            "input mismatch for camera_fx_px",
        ):
            RuntimeConfig(loaded.path, data, loaded.source_paths).validate()

    def test_changed_grasp_height_is_rejected(self) -> None:
        loaded = load_runtime_config(CONFIG_PATH)
        data = copy.deepcopy(loaded.data)
        data["localization"]["target_z_mm"] = -90.0
        with self.assertRaisesRegex(
            ConfigError,
            "input mismatch for target_z_mm",
        ):
            RuntimeConfig(loaded.path, data, loaded.source_paths).validate()

    def test_real_arm_cannot_bypass_measured_model(self) -> None:
        loaded = load_runtime_config(CONFIG_PATH)
        data = copy.deepcopy(loaded.data)
        data["localization"]["command_calibration"]["enabled"] = False
        with self.assertRaisesRegex(
            ConfigError,
            "arm.mode=real requires the measured",
        ):
            RuntimeConfig(loaded.path, data, loaded.source_paths).validate()


if __name__ == "__main__":
    unittest.main()
