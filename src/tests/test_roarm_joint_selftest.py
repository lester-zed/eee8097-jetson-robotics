"""No-hardware tests for the RoArm J1-J4 motion self-test."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch


SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from arm_control.roarm_joint_selftest import (
    DEFAULT_DELTAS_DEG,
    JOINT_NAMES,
    STATE_ATTRIBUTES,
    JointSpec,
    MotionSettings,
    choose_target_degrees,
    direction_hint,
    run_joint_sequence,
)
from arm_control.roarm_uart import RoArmState


class FakeArm:
    JOINT_LIMITS_DEG = {
        1: (-180.0, 180.0),
        2: (-90.0, 90.0),
        3: (0.0, 180.0),
        4: (45.0, 180.0),
    }

    def __init__(self) -> None:
        self.angles = {1: 0.0, 2: 0.0, 3: 90.0, 4: 180.0}
        self.commands: list[tuple[int, float]] = []
        self.stop_calls = 0

    def get_state(self) -> RoArmState:
        return RoArmState(
            x_mm=300.0,
            y_mm=0.0,
            z_mm=200.0,
            base_rad=math.radians(self.angles[1]),
            shoulder_rad=math.radians(self.angles[2]),
            elbow_rad=math.radians(self.angles[3]),
            hand_rad=math.radians(self.angles[4]),
            voltage_v=12.1,
            raw={"T": 1051},
        )

    def move_single_joint_degrees(
        self,
        joint: int,
        angle_degrees: float,
        **_: object,
    ) -> None:
        self.commands.append((joint, float(angle_degrees)))
        self.angles[joint] = float(angle_degrees)

    def stop_continuous_motion(self) -> None:
        self.stop_calls += 1


class StuckJ2Arm(FakeArm):
    def move_single_joint_degrees(
        self,
        joint: int,
        angle_degrees: float,
        **kwargs: object,
    ) -> None:
        self.commands.append((joint, float(angle_degrees)))
        # Simulate J2 ignoring its outward command but accepting the return.
        if joint != 2 or math.isclose(float(angle_degrees), 0.0):
            self.angles[joint] = float(angle_degrees)


def all_specs() -> list[JointSpec]:
    return [
        JointSpec(joint, JOINT_NAMES[joint], STATE_ATTRIBUTES[joint], DEFAULT_DELTAS_DEG[joint])
        for joint in range(1, 5)
    ]


class TargetSelectionTests(unittest.TestCase):
    def test_j4_home_uses_negative_delta_inside_limit(self) -> None:
        target, delta = choose_target_degrees(
            180.0,
            -8.0,
            (45.0, 180.0),
            margin_deg=1.0,
        )
        self.assertEqual(target, 172.0)
        self.assertEqual(delta, -8.0)

    def test_preferred_direction_reverses_only_near_limit(self) -> None:
        target, delta = choose_target_degrees(
            178.0,
            10.0,
            (-180.0, 180.0),
            margin_deg=1.0,
        )
        self.assertEqual(target, 168.0)
        self.assertEqual(delta, -10.0)


class DirectionHintTests(unittest.TestCase):
    def test_reverse_mount_j1_positive_is_camera_right(self) -> None:
        hint = direction_hint(1, 10.0, 180.0)
        self.assertIn("Camera 右侧", hint)


class SequenceTests(unittest.TestCase):
    @patch("arm_control.roarm_joint_selftest.time.sleep", return_value=None)
    def test_all_joints_move_and_return_with_feedback(self, _: object) -> None:
        arm = FakeArm()
        settings = MotionSettings(
            speed_deg_s=5,
            acceleration_deg_s2=10,
            tolerance_deg=0.5,
            inactive_tolerance_deg=0.5,
            settle_margin_s=0.0,
            feedback_timeout_s=0.1,
            feedback_poll_s=0.01,
            hold_s=0.0,
        )

        results = run_joint_sequence(arm, all_specs(), settings)

        self.assertEqual(len(results), 4)
        self.assertTrue(all(result.passed for result in results))
        self.assertEqual(len(arm.commands), 8)
        self.assertEqual(arm.angles, {1: 0.0, 2: 0.0, 3: 90.0, 4: 180.0})
        self.assertEqual(arm.stop_calls, 0)

    @patch("arm_control.roarm_joint_selftest.time.sleep", return_value=None)
    def test_failure_returns_joint_then_stops_following_tests(self, _: object) -> None:
        arm = StuckJ2Arm()
        settings = MotionSettings(
            speed_deg_s=5,
            acceleration_deg_s2=10,
            tolerance_deg=0.5,
            inactive_tolerance_deg=0.5,
            settle_margin_s=0.0,
            feedback_timeout_s=0.001,
            feedback_poll_s=0.001,
            hold_s=0.0,
        )

        results = run_joint_sequence(arm, all_specs(), settings)

        self.assertEqual([result.joint for result in results], [1, 2])
        self.assertTrue(results[0].passed)
        self.assertFalse(results[1].passed)
        self.assertEqual(arm.angles[2], 0.0)
        self.assertEqual(arm.commands[-1], (2, 0.0))


if __name__ == "__main__":
    unittest.main()
