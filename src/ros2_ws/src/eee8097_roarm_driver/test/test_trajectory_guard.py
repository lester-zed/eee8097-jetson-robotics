from types import SimpleNamespace
import unittest

from eee8097_roarm_driver.joint_mapping import ARM_JOINT_NAMES, GRIPPER_JOINT_NAME
from eee8097_roarm_driver.trajectory_guard import (
    TrajectoryValidationError,
    validate_arm_trajectory,
)


def point(positions, seconds):
    return SimpleNamespace(
        positions=positions,
        time_from_start=SimpleNamespace(sec=int(seconds), nanosec=0),
    )


class TrajectoryGuardTest(unittest.TestCase):
    def test_arm_only_trajectory_is_accepted(self):
        validate_arm_trajectory(ARM_JOINT_NAMES, [point((0.0, 0.0, 1.0), 1)])

    def test_any_gripper_goal_is_hard_rejected(self):
        names = (*ARM_JOINT_NAMES, GRIPPER_JOINT_NAME)
        with self.assertRaisesRegex(TrajectoryValidationError, "gripper"):
            validate_arm_trajectory(names, [point((0.0, 0.0, 1.0, 0.5), 1)])

    def test_negative_elbow_goal_is_rejected_by_firmware_limit(self):
        with self.assertRaisesRegex(TrajectoryValidationError, "link2_to_link3"):
            validate_arm_trajectory(ARM_JOINT_NAMES, [point((0.0, 0.0, -0.1), 1)])

    def test_trajectory_time_must_increase(self):
        points = [point((0.0, 0.0, 1.0), 1), point((0.1, 0.0, 1.1), 1)]
        with self.assertRaisesRegex(TrajectoryValidationError, "increase"):
            validate_arm_trajectory(ARM_JOINT_NAMES, points)


if __name__ == "__main__":
    unittest.main()
