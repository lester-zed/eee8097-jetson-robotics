import math
import unittest

from eee8097_roarm_driver.joint_mapping import (
    hardware_to_urdf_positions,
    urdf_to_hardware_degrees,
)


class JointMappingTest(unittest.TestCase):
    def test_hardware_and_urdf_arm_mapping_round_trip(self):
        urdf = hardware_to_urdf_positions(
            base_rad=0.2,
            shoulder_rad=-0.3,
            elbow_rad=1.1,
            gripper_collision_position_rad=0.0,
        )
        expected_urdf = (-0.2, 0.3, 1.1, 0.0)
        for actual, expected in zip(urdf, expected_urdf):
            self.assertAlmostEqual(actual, expected)
        hardware = urdf_to_hardware_degrees(urdf[:3])
        expected_hardware = (
            math.degrees(0.2),
            math.degrees(-0.3),
            math.degrees(1.1),
        )
        for actual, expected in zip(hardware, expected_hardware):
            self.assertAlmostEqual(actual, expected)

    def test_fixed_gripper_collision_posture_is_bounded(self):
        with self.assertRaisesRegex(ValueError, "gripper_collision_position_rad"):
            hardware_to_urdf_positions(
                base_rad=0.0,
                shoulder_rad=0.0,
                elbow_rad=1.0,
                gripper_collision_position_rad=1.6,
            )


if __name__ == "__main__":
    unittest.main()
