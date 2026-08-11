import math

from eee8097_roarm_driver.joint_mapping import JOINT_NAMES, hardware_to_urdf_positions


def test_official_joint_names_are_stable():
    assert JOINT_NAMES == (
        "base_link_to_link1",
        "link1_to_link2",
        "link2_to_link3",
        "link3_to_gripper_link",
    )


def test_inverse_of_official_driver_zero_pose_mapping():
    actual = hardware_to_urdf_positions(
        base_rad=0.0,
        shoulder_rad=0.0,
        elbow_rad=0.0,
        hand_rad=math.pi,
    )
    assert actual == (0.0, 0.0, 0.0, 0.0)


def test_hardware_to_urdf_signs_and_gripper_offset():
    actual = hardware_to_urdf_positions(
        base_rad=-1.0,
        shoulder_rad=0.5,
        elbow_rad=1.2,
        hand_rad=2.8,
    )
    expected = (1.0, -0.5, 1.2, math.pi - 2.8)
    for observed, wanted in zip(actual, expected):
        assert math.isclose(observed, wanted, rel_tol=0.0, abs_tol=1e-12)
