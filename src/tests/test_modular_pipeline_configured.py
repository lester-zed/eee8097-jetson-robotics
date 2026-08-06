from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from threading import Event
import sys
import tempfile
import time
import unittest

SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from adapters.mock_devices import MockArmAdapter, MockRangeSensorAdapter, MockVisionAdapter
from arm_control.cartesian_roarm_controller import CartesianRoArmController
from configuration.loader import load_runtime_config
from lidar.range_sensor import camera_pixel_to_bearing_deg
from localization.target_localizer import PlanarTargetLocalizer
from localization.transforms import IdentityBaseToArmTransform
from pipeline.task_manager import ModularTaskManager, TaskState
from pipeline.types import Detection2D, GraspPlan, Point3D, TargetPose
from planning.grasp_planner import SimpleTopDownGraspPlanner


class ConfigurationTests(unittest.TestCase):
    def test_default_configuration_loads(self) -> None:
        config = load_runtime_config(SOURCE_ROOT / "configs/modular_pipeline.yaml")
        self.assertEqual(config.get("camera", "cx_px"), 320.0)
        self.assertEqual(config.get("arm", "mode"), "mock")


class BearingTests(unittest.TestCase):
    def test_640_width_center_is_zero_bearing(self) -> None:
        self.assertAlmostEqual(
            camera_pixel_to_bearing_deg(pixel_x=320.0, fx_px=900.0, cx_px=320.0),
            0.0,
        )


class PlannerTests(unittest.TestCase):
    def test_negative_x_pregrasp_moves_toward_origin(self) -> None:
        target = TargetPose(
            label="cup",
            confidence=0.9,
            target_base_link=Point3D(175.0, 63.0, 140.0, "base_link"),
            target_arm_base=Point3D(-175.0, -63.0, 140.0, "arm_base"),
            provisional=True,
        )
        plan = SimpleTopDownGraspPlanner(approach_distance_mm=80.0).plan(target)
        pregrasp = plan.waypoint("pregrasp").point
        self.assertLess(abs(pregrasp.x_mm), abs(target.target_arm_base.x_mm))
        self.assertLess(abs(pregrasp.y_mm), abs(target.target_arm_base.y_mm))


@dataclass
class _State:
    x_mm: float
    y_mm: float
    z_mm: float
    voltage_v: float = 12.0


class _FakeUart:
    def __init__(self, **_: object) -> None:
        self.state = _State(0.0, 0.0, 100.0)
        self.commands: list[dict] = []
        self.closed = False

    def send(self, command: dict) -> None:
        self.commands.append(dict(command))
        if command.get("T") == 104:
            self.state = _State(command["x"], command["y"], command["z"])

    def get_state(self) -> _State:
        return self.state

    def set_gripper_radians(self, angle: float) -> None:
        self.commands.append({"T": 106, "cmd": angle})

    def stop_continuous_motion(self) -> None:
        self.commands.append({"T": 123})

    def close(self) -> None:
        self.closed = True


class CartesianControllerTests(unittest.TestCase):
    def test_t104_sequence_is_generated_without_hardware(self) -> None:
        fake = _FakeUart()
        controller = CartesianRoArmController(
            port="fake",
            allow_real_motion=True,
            confirm_clearance=True,
            calibration_approved=True,
            feedback_poll_s=0.001,
            gripper_settle_s=0.0,
            uart_factory=lambda **_: fake,
        )
        target = TargetPose(
            label="cup",
            confidence=1.0,
            target_base_link=Point3D(200, 0, 100, "base_link"),
            target_arm_base=Point3D(200, 0, 100, "arm_base"),
            provisional=False,
        )
        plan = SimpleTopDownGraspPlanner(
            approach_distance_mm=50,
            pregrasp_height_mm=20,
            lift_height_mm=30,
        ).plan(target)
        result = controller.execute_grasp(plan, Event())
        self.assertTrue(result.success)
        self.assertEqual(sum(cmd.get("T") == 104 for cmd in fake.commands), 4)
        self.assertEqual(sum(cmd.get("T") == 106 for cmd in fake.commands), 2)


class PipelineTests(unittest.TestCase):
    def test_complete_mock_pipeline(self) -> None:
        manager = ModularTaskManager(
            vision=MockVisionAdapter(frame_width=640, frame_height=480),
            range_sensor=MockRangeSensorAdapter(distance_mm=350.0),
            localizer=PlanarTargetLocalizer(
                base_to_arm=IdentityBaseToArmTransform(), target_z_mm=100.0
            ),
            planner=SimpleTopDownGraspPlanner(),
            arm=MockArmAdapter(step_delay_s=0.001),
        )
        try:
            self.assertTrue(manager.start())
            deadline = time.monotonic() + 2.0
            while manager.is_running() and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertEqual(manager.status()["state"], TaskState.COMPLETE.value)
        finally:
            manager.shutdown()


if __name__ == "__main__":
    unittest.main()
