from __future__ import annotations

import copy
import csv
from pathlib import Path
import sys
import tempfile
import time
from types import ModuleType
import unittest
from unittest.mock import patch


SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from adapters.mock_devices import (
    MockArmAdapter,
    MockRangeSensorAdapter,
    MockVisionAdapter,
)
from configuration.loader import ConfigError, RuntimeConfig, load_runtime_config
from experiments.calibration import validate_no_motion_calibration_profile
from experiments.recorder import (
    CalibrationGroundTruth,
    ExperimentRecorder,
    RunContext,
    annotate_run,
    load_jsonl_records,
)
from experiments.statistics import summarize_records
from launch_modular_pipeline import should_execute_startup_home
from localization.target_localizer import PlanarTargetLocalizer
from localization.transforms import IdentityBaseToArmTransform
from main_modular import build_manager
from pipeline.task_manager import ModularTaskManager, TaskState
from planning.grasp_planner import SimpleTopDownGraspPlanner


CALIBRATION_CONFIG = SOURCE_ROOT / "configs/calibration_capture.yaml"


class CalibrationProfileTests(unittest.TestCase):
    def test_committed_profile_inherits_current_baseline_and_disables_arm(self) -> None:
        config = load_runtime_config(CALIBRATION_CONFIG)
        validate_no_motion_calibration_profile(config)

        self.assertEqual(config.get("camera", "mode"), "real")
        self.assertEqual(config.get("lidar", "mode"), "real")
        self.assertEqual(config.get("arm", "mode"), "mock")
        self.assertFalse(config.get("arm", "allow_real_motion"))
        self.assertFalse(config.get("manual_arm_test", "enabled"))
        self.assertEqual(config.get("planner", "grasp_y_offset_mm"), -15.0)
        self.assertFalse(config.get("localization", "calibration_approved"))
        self.assertEqual(
            [path.name for path in config.source_paths],
            ["modular_pipeline.yaml", "calibration_capture.yaml"],
        )

    def test_mock_arm_profile_skips_startup_home(self) -> None:
        config = load_runtime_config(CALIBRATION_CONFIG)
        self.assertFalse(
            should_execute_startup_home(
                config,
                {"enabled": True, "run_before_pipeline": True},
                skip_home=False,
            )
        )

    def test_calibration_manager_builds_mock_arm_without_real_arm_import(self) -> None:
        config = load_runtime_config(CALIBRATION_CONFIG)

        class _FakeVision:
            def __init__(self, **_: object) -> None:
                pass

            def close(self) -> None:
                pass

        class _FakeRange:
            def __init__(self, **_: object) -> None:
                pass

            def close(self) -> None:
                pass

        vision_module = ModuleType("vision.yolo_camera_adapter")
        vision_module.ExistingYoloCameraAdapter = _FakeVision
        lidar_module = ModuleType("lidar.range_sensor")
        lidar_module.CameraGuidedRPLidarAdapter = _FakeRange

        forbidden_arm_module = ModuleType("arm_control.arm_adapter")

        class _ForbiddenRealArm:
            def __init__(self, **_: object) -> None:
                raise AssertionError("real RoArm adapter must not be constructed")

        forbidden_arm_module.CartesianRoArmAdapter = _ForbiddenRealArm

        with patch.dict(
            sys.modules,
            {
                "vision.yolo_camera_adapter": vision_module,
                "lidar.range_sensor": lidar_module,
                "arm_control.arm_adapter": forbidden_arm_module,
            },
        ):
            manager = build_manager(config)
        try:
            self.assertIsInstance(manager.arm, MockArmAdapter)
        finally:
            manager.shutdown()

    def test_no_motion_validator_rejects_real_arm(self) -> None:
        loaded = load_runtime_config(CALIBRATION_CONFIG)
        data = copy.deepcopy(loaded.data)
        data["arm"]["mode"] = "real"
        config = RuntimeConfig(loaded.path, data, loaded.source_paths)
        with self.assertRaises(ConfigError):
            validate_no_motion_calibration_profile(config)


class TaskCompletionCallbackTests(unittest.TestCase):
    def _manager(self, callback) -> ModularTaskManager:
        return ModularTaskManager(
            vision=MockVisionAdapter(frame_width=640, frame_height=480),
            range_sensor=MockRangeSensorAdapter(distance_mm=350.0),
            localizer=PlanarTargetLocalizer(
                base_to_arm=IdentityBaseToArmTransform(),
                target_z_mm=100.0,
            ),
            planner=SimpleTopDownGraspPlanner(),
            arm=MockArmAdapter(step_delay_s=0.001),
            on_run_complete=callback,
        )

    @staticmethod
    def _wait(manager: ModularTaskManager) -> None:
        deadline = time.monotonic() + 2.0
        while manager.is_running() and time.monotonic() < deadline:
            time.sleep(0.005)

    def test_terminal_callback_receives_run_id_and_timing_once(self) -> None:
        snapshots: list[dict] = []
        manager = self._manager(snapshots.append)
        try:
            self.assertTrue(manager.start())
            self._wait(manager)
            self.assertEqual(len(snapshots), 1)
            snapshot = snapshots[0]
            self.assertEqual(snapshot["state"], TaskState.COMPLETE.value)
            self.assertFalse(snapshot["running"])
            self.assertTrue(snapshot["run_id"])
            self.assertIsNotNone(snapshot["timing"]["started_at_utc"])
            self.assertIsNotNone(snapshot["timing"]["finished_at_utc"])
            self.assertGreaterEqual(snapshot["timing"]["duration_s"], 0.0)
        finally:
            manager.shutdown()

    def test_recording_failure_does_not_change_completed_motion_state(self) -> None:
        def fail(_: dict) -> None:
            raise OSError("simulated read-only log directory")

        manager = self._manager(fail)
        try:
            self.assertTrue(manager.start())
            self._wait(manager)
            status = manager.status()
            self.assertEqual(status["state"], TaskState.COMPLETE.value)
            self.assertIn("OSError", status["recording_error"])
        finally:
            manager.shutdown()


class ExperimentRecorderTests(unittest.TestCase):
    def _config(self, output_dir: Path) -> RuntimeConfig:
        loaded = load_runtime_config(CALIBRATION_CONFIG)
        data = copy.deepcopy(loaded.data)
        data["experiment"]["output_dir"] = str(output_dir)
        return RuntimeConfig(loaded.path, data, loaded.source_paths)

    @staticmethod
    def _status(run_id: str = "run-test-001") -> dict:
        return {
            "run_id": run_id,
            "state": "COMPLETE",
            "running": False,
            "timing": {
                "started_at_utc": "2026-08-10T00:00:00.000+00:00",
                "finished_at_utc": "2026-08-10T00:00:01.500+00:00",
                "duration_s": 1.5,
            },
            "health": [],
            "detection": {
                "label": "tissue_pack",
                "confidence": 0.91,
                "center_x": 321,
                "center_y": 241,
                "bbox": [280, 180, 360, 300],
            },
            "range": {
                "distance_mm": 250.0,
                "camera_bearing_deg": 0.1,
                "lidar_bearing_deg": 180.1,
                "base_bearing_deg": 0.1,
                "sample_count": 12,
                "mad_mm": 2.0,
                "minimum_mm": 247.0,
                "maximum_mm": 253.0,
            },
            "target": {
                "target_base_link": {
                    "x_mm": 110.0,
                    "y_mm": 35.0,
                    "z_mm": -110.0,
                    "frame_id": "base_link",
                },
                "target_arm_base": {
                    "x_mm": 110.0,
                    "y_mm": 35.0,
                    "z_mm": -110.0,
                    "frame_id": "arm_base",
                },
            },
            "verification": {
                "passed": True,
                "center_shift_px": 2.0,
                "range_shift_mm": 3.0,
                "target_shift_mm": 4.0,
            },
            "plan": {
                "waypoints": [
                    {"name": name, "point": {"x_mm": index, "y_mm": 0, "z_mm": 0}}
                    for index, name in enumerate(
                        ("pregrasp", "grasp", "lift", "retreat"), start=1
                    )
                ],
                "metadata": {"grasp_y_offset_mm": -15.0},
            },
            "execution": {
                "success": True,
                "message": "mock grasp completed; no hardware command was sent",
            },
            "error": None,
            "recording_error": None,
        }

    def test_jsonl_csv_snapshot_and_calibration_error_are_written(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            recorder = ExperimentRecorder(
                config=self._config(output),
                output_dir=output,
                profile="calibration_capture",
                git_commit="abc123",
            )
            recorder.set_run_context(
                RunContext(
                    ground_truth=CalibrationGroundTruth(
                        point_id="P11", x_mm=100.0, y_mm=40.0, z_mm=-110.0
                    ),
                    repeat_index=1,
                    notes="3x3 grid centre",
                )
            )
            record = recorder.record(self._status())

            self.assertEqual(record["metrics"]["error_x_mm"], 10.0)
            self.assertEqual(record["metrics"]["error_y_mm"], -5.0)
            self.assertAlmostEqual(
                record["metrics"]["planar_error_mm"],
                (125.0) ** 0.5,
            )
            self.assertEqual(record["modes"]["arm"], "mock")
            self.assertEqual(record["config"]["planner"]["grasp_y_offset_mm"], -15.0)

            loaded = load_jsonl_records(recorder.jsonl_path)
            self.assertEqual(len(loaded), 1)
            with recorder.csv_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["point_id"], "P11")
            self.assertEqual(rows[0]["grasp_y_offset_mm"], "-15.0")

    def test_annotation_rewrites_jsonl_and_csv_consistently(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            recorder = ExperimentRecorder(
                config=self._config(output),
                output_dir=output,
                profile="real_grasp",
                git_commit="abc123",
            )
            recorder.record(self._status("run-annotate"))

            updated = annotate_run(
                jsonl_path=recorder.jsonl_path,
                csv_path=recorder.csv_path,
                run_id="run-annotate",
                ground_truth=CalibrationGroundTruth(
                    point_id="P22", x_mm=105.0, y_mm=35.0
                ),
                operator_grasp_success=False,
                notes="object slipped after lift",
            )
            self.assertEqual(updated["metrics"]["error_x_mm"], 5.0)
            self.assertFalse(
                updated["operator"]["operator_grasp_success"]
            )

            records = load_jsonl_records(recorder.jsonl_path)
            self.assertEqual(len(records), 1)
            with recorder.csv_path.open(newline="", encoding="utf-8") as handle:
                row = next(csv.DictReader(handle))
            self.assertEqual(row["operator_grasp_success"], "False")
            self.assertEqual(row["notes"], "object slipped after lift")

    def test_statistics_report_bias_rmse_failures_and_grasp_outcome(self) -> None:
        first = self._status("run-1")
        second = self._status("run-2")
        second["state"] = "ERROR"
        second["error"] = "RuntimeError: target was not detected"
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            recorder = ExperimentRecorder(
                config=self._config(output),
                output_dir=output,
                profile="calibration_capture",
                git_commit="abc123",
            )
            for index, status in enumerate((first, second), start=1):
                recorder.set_run_context(
                    RunContext(
                        ground_truth=CalibrationGroundTruth(
                            point_id="P11",
                            x_mm=100.0,
                            y_mm=40.0,
                        ),
                        repeat_index=index,
                        operator_grasp_success=index == 1,
                    )
                )
                recorder.record(status)

            summary = summarize_records(load_jsonl_records(recorder.jsonl_path))
            self.assertEqual(summary["run_count"], 2)
            self.assertEqual(summary["failure_types"]["RuntimeError"], 1)
            self.assertEqual(summary["calibration"]["sample_count"], 1)
            self.assertAlmostEqual(summary["calibration"]["bias_x_mm"], 10.0)
            self.assertAlmostEqual(
                summary["calibration"]["planar_rmse_mm"],
                (125.0) ** 0.5,
            )
            self.assertEqual(summary["grasp_outcomes"]["annotated_count"], 2)
            self.assertEqual(summary["grasp_outcomes"]["success_rate"], 0.5)


if __name__ == "__main__":
    unittest.main()
