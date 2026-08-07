from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from threading import Event
import sys
import unittest

SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from arm_control.cartesian_roarm_controller import CartesianRoArmController
from arm_control.roarm_uart import RoArmTimeoutError
from pipeline.types import (
    CartesianWaypoint,
    GraspPlan,
    Point3D,
    TargetPose,
)


@dataclass
class _State:
    x_mm: float
    y_mm: float
    z_mm: float
    voltage_v: float = 12.0


class _BusyDuringMotionUart:
    """Simulates firmware that cannot answer the first T=105 polls after T=104."""

    def __init__(self, **_: object) -> None:
        self.state = _State(-300.0, 0.0, 230.0)
        self.target = self.state
        self.timeouts_remaining = 0
        self.commands: list[dict] = []

    def send(self, command: dict) -> None:
        self.commands.append(dict(command))
        if command.get("T") == 104:
            self.target = _State(command["x"], command["y"], command["z"])
            self.timeouts_remaining = 2

    def get_state(self) -> _State:
        if self.timeouts_remaining > 0:
            self.timeouts_remaining -= 1
            raise RoArmTimeoutError("simulated firmware busy during T=104")
        self.state = self.target
        return self.state

    def set_gripper_radians(self, angle: float) -> None:
        self.commands.append({"T": 106, "cmd": angle})

    def stop_continuous_motion(self) -> None:
        self.commands.append({"T": 123})

    def close(self) -> None:
        return None


class FourWaypointRetryTests(unittest.TestCase):
    def test_transient_t105_timeouts_do_not_abort_four_waypoint_sequence(self) -> None:
        uart = _BusyDuringMotionUart()
        controller = CartesianRoArmController(
            port="fake",
            allow_real_motion=True,
            confirm_clearance=True,
            calibration_approved=True,
            waypoint_timeout_s=1.0,
            feedback_poll_s=0.001,
            gripper_settle_s=0.0,
            initial_feedback_delay_s=0.0,
            uart_factory=lambda **_: uart,
        )
        target = TargetPose(
            label="cup",
            confidence=1.0,
            target_base_link=Point3D(-210.0, 0.0, 60.0, "base_link"),
            target_arm_base=Point3D(-210.0, 0.0, 60.0, "arm_base"),
            provisional=False,
        )
        coordinates = (
            ("pregrasp", -160.0, 0.0, 100.0),
            ("grasp", -210.0, 0.0, 60.0),
            ("lift", -210.0, 0.0, 120.0),
            ("retreat", -160.0, 0.0, 120.0),
        )
        plan = GraspPlan(
            target=target,
            waypoints=tuple(
                CartesianWaypoint(
                    name=name,
                    point=Point3D(x, y, z, "arm_base"),
                    tool_angle_rad=3.14,
                    speed=0.2,
                )
                for name, x, y, z in coordinates
            ),
            gripper_open_rad=2.70,
            gripper_closed_rad=3.14,
        )

        result = controller.execute_grasp(plan, Event())

        self.assertTrue(result.success)
        self.assertEqual(
            result.completed_waypoints,
            ("pregrasp", "grasp", "lift", "retreat"),
        )
        self.assertEqual(sum(cmd.get("T") == 104 for cmd in uart.commands), 4)
        self.assertEqual(sum(cmd.get("T") == 106 for cmd in uart.commands), 2)
        for name in ("pregrasp", "grasp", "lift", "retreat"):
            self.assertEqual(
                result.feedback["waypoints"][name]["transient_t105_timeouts"],
                2,
            )


if __name__ == "__main__":
    unittest.main()
