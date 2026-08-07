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
    hand_rad: float = 2.70
    raw: dict | None = None


class _FakeUart:
    def __init__(self, **_: object) -> None:
        self.state = _State(0.0, 0.0, 100.0)
        self.commands: list[dict] = []
        self.closed = False

    def send(self, command: dict) -> None:
        self.commands.append(dict(command))
        if command.get("T") == 104:
            self.state = _State(
                command["x"], command["y"], command["z"], hand_rad=command["t"], raw={"torH": 0}
            )

    def get_state(self) -> _State:
        return self.state

    def set_gripper_radians(
        self, angle: float, *, speed_steps_s: int = 100, acceleration: int = 10
    ) -> None:
        self.commands.append({
            "T": 106,
            "cmd": angle,
            "spd": speed_steps_s,
            "acc": acceleration,
        })
        self.state.hand_rad = float(angle)
        self.state.raw = {"torH": 123}

    def stop_continuous_motion(self) -> None:
        self.commands.append({"T": 123})

    def close(self) -> None:
        self.closed = True




class _BusyFakeUart(_FakeUart):
    def __init__(self, **_: object) -> None:
        super().__init__()
        self._busy_polls_remaining = 0
        self._pending_state: _State | None = None

    def send(self, command: dict) -> None:
        self.commands.append(dict(command))
        if command.get("T") == 104:
            self._pending_state = _State(
                command["x"], command["y"], command["z"], hand_rad=command["t"], raw={"torH": 0}
            )
            self._busy_polls_remaining = 2

    def get_state(self) -> _State:
        if self._busy_polls_remaining > 0:
            self._busy_polls_remaining -= 1
            from arm_control.roarm_uart import RoArmTimeoutError
            raise RoArmTimeoutError("simulated firmware busy during T=104")
        if self._pending_state is not None:
            self.state = self._pending_state
            self._pending_state = None
        return self.state


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
            gripper_speed_steps_s=1000,
            gripper_motion_margin_s=0.0,
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

        t104 = [cmd for cmd in fake.commands if cmd.get("T") == 104]
        t106 = [cmd for cmd in fake.commands if cmd.get("T") == 106]
        self.assertEqual([cmd["cmd"] for cmd in t106], [plan.gripper_open_rad, plan.gripper_closed_rad])
        self.assertEqual(
            [cmd["t"] for cmd in t104],
            [
                round(plan.gripper_open_rad, 4),
                round(plan.gripper_open_rad, 4),
                round(plan.gripper_closed_rad, 4),
                round(plan.gripper_closed_rad, 4),
            ],
        )
        self.assertIn("gripper", result.feedback)
        gripper = result.feedback["gripper"]
        self.assertGreaterEqual(
            gripper["close_wait_s"], gripper["estimated_close_motion_s"]
        )
        self.assertAlmostEqual(
            gripper["post_close_hand_rad"], plan.gripper_closed_rad, places=6
        )
        self.assertEqual(gripper["post_close_torque_raw"], 123)

    def test_legacy_initial_feedback_delay_alias_is_accepted(self) -> None:
        fake = _FakeUart()
        controller = CartesianRoArmController(
            port="fake",
            allow_real_motion=True,
            confirm_clearance=True,
            calibration_approved=True,
            initial_feedback_delay_s=0.0,
            feedback_poll_s=0.001,
            gripper_settle_s=0.0,
            gripper_speed_steps_s=1000,
            gripper_motion_margin_s=0.0,
            uart_factory=lambda **_: fake,
        )
        self.assertEqual(controller.feedback_initial_delay_s, 0.0)
        controller.close()

    def test_t104_feedback_retries_after_uart_timeout(self) -> None:
        fake = _BusyFakeUart()
        controller = CartesianRoArmController(
            port="fake",
            allow_real_motion=True,
            confirm_clearance=True,
            calibration_approved=True,
            feedback_poll_s=0.001,
            feedback_initial_delay_s=0.0,
            waypoint_timeout_s=1.0,
            uart_response_timeout_s=0.1,
            gripper_settle_s=0.0,
            gripper_speed_steps_s=1000,
            gripper_motion_margin_s=0.0,
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
        self.assertEqual(sum(cmd.get("T") == 123 for cmd in fake.commands), 0)
        self.assertIn("waypoints", result.feedback)
        for waypoint in plan.waypoints:
            diagnostics = result.feedback["waypoints"][waypoint.name]
            self.assertEqual(diagnostics["transient_t105_timeouts"], 2)
            self.assertGreaterEqual(diagnostics["t105_attempts"], 3)
            self.assertGreaterEqual(diagnostics["feedback_samples"], 1)
            self.assertLessEqual(
                diagnostics["final_error_mm"], controller.feedback_tolerance_mm
            )


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
