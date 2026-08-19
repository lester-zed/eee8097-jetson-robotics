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
FIXTURE_PATH = SOURCE_ROOT / "tests/fixtures/grasp_waypoint_20260819.json"


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


def _runtime_planner(config: RuntimeConfig) -> SimpleTopDownGraspPlanner:
    planner = config.section("planner")
    workspace = config.section("workspace")
    return SimpleTopDownGraspPlanner(
        approach_distance_mm=float(planner["approach_distance_mm"]),
        pregrasp_height_mm=float(planner["pregrasp_height_mm"]),
        lift_height_mm=float(planner["lift_height_mm"]),
        grasp_x_offset_mm=float(planner["grasp_x_offset_mm"]),
        grasp_y_offset_mm=float(planner["grasp_y_offset_mm"]),
        limits=WorkspaceLimits(
            min_x_mm=float(workspace["min_x_mm"]),
            max_x_mm=float(workspace["max_x_mm"]),
            min_y_mm=float(workspace["min_y_mm"]),
            max_y_mm=float(workspace["max_y_mm"]),
            min_z_mm=float(workspace["min_z_mm"]),
            max_z_mm=float(workspace["max_z_mm"]),
            max_radius_mm=float(workspace["max_radius_mm"]),
        ),
    )


class MeasuredCommandCalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = load_runtime_config(CONFIG_PATH)
        cls.section = cls.config.get("localization", "command_calibration")
        cls.calibration = AffineXYCommandCalibration.from_mapping(cls.section)
        cls.planner = _runtime_planner(cls.config)
        cls.fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))

    def test_current_model_is_bilinear_manual_teach_calibration(self) -> None:
        self.assertEqual(self.calibration.model, "bilinear_xy")
        self.assertEqual(
            self.calibration.calibration_id,
            "tissue-grasp-waypoint-20260819-v2",
        )
        self.assertAlmostEqual(self.calibration.origin_x_mm, -377.70475532)
        self.assertAlmostEqual(self.calibration.origin_y_mm, 2.01385567)
        metadata = self.calibration.to_metadata()
        self.assertEqual(metadata["model"], "bilinear_xy")
        self.assertEqual(len(metadata["x_coefficients"]), 4)
        self.assertEqual(len(metadata["y_coefficients"]), 4)

    def test_all_11_taught_grasp_samples_replay_with_expected_error(self) -> None:
        errors: list[float] = []
        for sample in self.fixture["samples"]:
            corrected = self.calibration.apply(
                Point3D(
                    sample["nominal_arm_x_mm"],
                    sample["nominal_arm_y_mm"],
                    -110.0,
                    "arm_base",
                )
            )
            predicted_grasp_x = (
                corrected.x_mm
                + float(self.config.get("planner", "grasp_x_offset_mm"))
            )
            predicted_grasp_y = (
                corrected.y_mm
                + float(self.config.get("planner", "grasp_y_offset_mm"))
            )
            errors.append(
                math.hypot(
                    predicted_grasp_x - sample["manual_arm_x_mm"],
                    predicted_grasp_y - sample["manual_arm_y_mm"],
                )
            )

        planar_rmse = math.sqrt(
            sum(value * value for value in errors) / len(errors)
        )
        self.assertEqual(len(errors), 11)
        self.assertLessEqual(planar_rmse, 5.867)
        self.assertLessEqual(max(errors), 11.8475)

    def test_far_right_sample_maps_to_expected_final_grasp(self) -> None:
        sample = self.fixture["samples"][1]
        corrected = self.calibration.apply(
            Point3D(
                sample["nominal_arm_x_mm"],
                sample["nominal_arm_y_mm"],
                -110.0,
                "arm_base",
            )
        )
        self.assertAlmostEqual(corrected.x_mm, -443.096265, places=3)
        self.assertAlmostEqual(corrected.y_mm, 51.811902, places=3)

        target = self._localizer().localize(
            _detection(),
            _measurement(
                sample["nominal_arm_x_mm"],
                sample["nominal_arm_y_mm"],
            ),
        )
        plan = self.planner.plan(target)
        grasp = plan.waypoint("grasp").point
        self.assertAlmostEqual(grasp.x_mm, -433.096265, places=3)
        self.assertAlmostEqual(grasp.y_mm, 36.811902, places=3)
        self.assertEqual(grasp.z_mm, -110.0)

    def test_far_left_sample_is_inside_new_measured_operating_region(self) -> None:
        sample = self.fixture["samples"][2]
        target = self._localizer().localize(
            _detection(),
            _measurement(
                sample["nominal_arm_x_mm"],
                sample["nominal_arm_y_mm"],
            ),
        )
        plan = self.planner.plan(target)
        grasp = plan.waypoint("grasp").point
        self.assertLess(grasp.y_mm, -95.0)
        self.assertGreaterEqual(grasp.x_mm, -450.0)

    def test_target_outside_measured_domain_is_rejected(self) -> None:
        bounds = self.calibration.input_bounds
        outside_x = bounds.max_x_mm + 1.0
        inside_y = (bounds.min_y_mm + bounds.max_y_mm) / 2.0
        with self.assertRaisesRegex(
            CalibrationDomainError,
            "outside measured command-calibration input bounds",
        ):
            self.calibration.apply(
                Point3D(outside_x, inside_y, -110.0, "arm_base")
            )

    def test_calibration_rejects_wrong_frame(self) -> None:
        with self.assertRaisesRegex(ValueError, "requires an arm_base point"):
            self.calibration.apply(
                Point3D(-380.0, 0.0, -110.0, "base_link")
            )

    def test_legacy_affine_xy_model_remains_supported(self) -> None:
        legacy = AffineXYCommandCalibration.from_mapping(
            {
                "model": "affine_xy",
                "calibration_id": "legacy-test",
                "x_coefficients": [1.0, 0.0, 10.0],
                "y_coefficients": [0.0, 1.0, -5.0],
                "input_bounds_mm": {
                    "min_x_mm": -100.0,
                    "max_x_mm": 100.0,
                    "min_y_mm": -100.0,
                    "max_y_mm": 100.0,
                },
                "output_bounds_mm": {
                    "min_x_mm": -100.0,
                    "max_x_mm": 120.0,
                    "min_y_mm": -120.0,
                    "max_y_mm": 100.0,
                },
            }
        )
        corrected = legacy.apply(Point3D(20.0, 30.0, -110.0, "arm_base"))
        self.assertEqual(corrected.x_mm, 30.0)
        self.assertEqual(corrected.y_mm, 25.0)
        self.assertEqual(corrected.z_mm, -110.0)

    def _localizer(self) -> PlanarTargetLocalizer:
        return PlanarTargetLocalizer(
            base_to_arm=IdentityBaseToArmTransform(),
            target_z_mm=-110.0,
            calibration_approved=True,
            target_z_approved=True,
            arm_command_calibration=self.calibration,
        )

    def test_raw_sensor_target_is_preserved_and_model_logged(self) -> None:
        sample = self.fixture["samples"][4]
        target = self._localizer().localize(
            _detection(),
            _measurement(
                sample["nominal_arm_x_mm"],
                sample["nominal_arm_y_mm"],
            ),
        )
        self.assertAlmostEqual(
            target.target_base_link.x_mm,
            sample["nominal_arm_x_mm"],
        )
        self.assertAlmostEqual(
            target.metadata["nominal_target_arm_base"]["y_mm"],
            sample["nominal_arm_y_mm"],
        )
        self.assertEqual(
            target.metadata["arm_command_calibration"]["model"],
            "bilinear_xy",
        )
        self.assertIn("bilinear_xy", target.metadata["localization_method"])
        self.assertFalse(target.provisional)

    def test_bearing_only_fallback_is_rejected_for_measured_model(self) -> None:
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
        self.assertEqual(
            config.get("localization", "command_calibration")["model"],
            "bilinear_xy",
        )
        self.assertFalse(config.get("arm", "allow_real_motion"))
        self.assertEqual(config.get("planner", "grasp_x_offset_mm"), 10.0)
        self.assertEqual(config.get("planner", "grasp_y_offset_mm"), -15.0)

    def test_raw_capture_profile_disables_fitted_model(self) -> None:
        config = load_runtime_config(CAPTURE_PATH)
        self.assertFalse(
            config.get("localization", "command_calibration")["enabled"]
        )

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
