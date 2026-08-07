from __future__ import annotations

import copy
import unittest
from pathlib import Path

from configuration.loader import RuntimeConfig, load_runtime_config
from roarm_manual_test import build_manual_plan, validate_manual_config

CONFIG_PATH = Path(__file__).resolve().parents[1] / "configs/modular_pipeline.yaml"

class ManualRoArmConfigTests(unittest.TestCase):
    """Keep unit tests independent from the operator's live YAML values."""

    def setUp(self) -> None:
        loaded = load_runtime_config(CONFIG_PATH)
        self.data = copy.deepcopy(loaded.data)
        self.manual = self.data.setdefault("manual_arm_test", {})

    def _config(self) -> RuntimeConfig:
        return RuntimeConfig(CONFIG_PATH, self.data)

    def _set_unconfigured_waypoints(self) -> None:
        self.manual["waypoints"] = {
            "pregrasp": [None, None, None],
            "grasp": [None, None, None],
            "lift": [None, None, None],
            "retreat": [None, None, None],
        }

    def test_safe_flags_can_be_represented(self) -> None:
        self.manual.update(enabled=False, allow_real_motion=False, confirm_clearance=False, coordinates_approved=False)
        self.assertFalse(self.manual["enabled"])
        self.assertFalse(self.manual["allow_real_motion"])
        self.assertFalse(self.manual["confirm_clearance"])
        self.assertFalse(self.manual["coordinates_approved"])
        validate_manual_config(self._config(), require_waypoints=False)

    def test_unset_waypoints_are_rejected(self) -> None:
        self._set_unconfigured_waypoints()
        with self.assertRaisesRegex(ValueError, "not configured"):
            build_manual_plan(self._config())

    def test_complete_waypoints_build_arm_base_plan(self) -> None:
        self.manual["waypoints"] = {
            "pregrasp": [250.0, 0.0, 220.0],
            "grasp": [270.0, 0.0, 180.0],
            "lift": [270.0, 0.0, 230.0],
            "retreat": [250.0, 0.0, 230.0],
        }
        plan = build_manual_plan(self._config())
        self.assertEqual([item.name for item in plan.waypoints], ["pregrasp", "grasp", "lift", "retreat"])
        self.assertTrue(all(item.point.frame_id == "arm_base" for item in plan.waypoints))
        self.assertFalse(plan.target.provisional)

    def test_read_only_validation_does_not_require_waypoints(self) -> None:
        self._set_unconfigured_waypoints()
        validate_manual_config(self._config(), require_waypoints=False)

    def test_operator_config_may_be_enabled_without_breaking_unit_tests(self) -> None:
        self.manual.update(enabled=True, allow_real_motion=True, confirm_clearance=True, coordinates_approved=True)
        self.manual["waypoints"] = {
            "pregrasp": [-310.0, 0.0, -230.0],
            "grasp": [-410.0, 0.0, -120.0],
            "lift": [-410.0, 0.0, -70.0],
            "retreat": [-350.0, 0.0, -70.0],
        }
        validate_manual_config(self._config(), require_waypoints=True)

if __name__ == "__main__":
    unittest.main()
