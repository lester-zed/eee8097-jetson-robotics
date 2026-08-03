"""No-hardware tests for the additive reverse-mounted RoArm backend."""

import json
import math
from pathlib import Path
import sys
from threading import Event
import unittest


SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from arm_control.roarm_reverse.controller import RealRoArmController
from arm_control.roarm_reverse.transform import ReverseMountTransform
from arm_control.roarm_reverse.uart import RoArmUart


STATE_PAYLOAD = {
    "T": 1051,
    "x": 320.4,
    "y": -3.4,
    "z": 212.3,
    "b": 0.0,
    "s": 0.0,
    "e": math.pi / 2,
    "t": math.pi,
    "v": 1211,
}


class FakeSerial:
    instances = []

    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs
        self.writes = []
        self.closed = False
        self.rts = None
        self.dtr = None
        FakeSerial.instances.append(self)

    def setRTS(self, value) -> None:
        self.rts = value

    def setDTR(self, value) -> None:
        self.dtr = value

    def write(self, payload: bytes) -> int:
        self.writes.append(payload)
        return len(payload)

    def flush(self) -> None:
        pass

    def readline(self) -> bytes:
        return (json.dumps(STATE_PAYLOAD) + "\n").encode("utf-8")

    def reset_input_buffer(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True


class ReverseMountTransformTests(unittest.TestCase):
    def test_region_mapping_for_180_degree_mount(self) -> None:
        transform = ReverseMountTransform(
            mount_yaw_degrees=180.0,
            region_yaw_degrees=20.0,
        )
        self.assertAlmostEqual(transform.camera_region_to_arm_base("left"), -160.0)
        self.assertAlmostEqual(transform.camera_region_to_arm_base("center"), 180.0)
        self.assertAlmostEqual(transform.camera_region_to_arm_base("centre"), 180.0)
        self.assertAlmostEqual(transform.camera_region_to_arm_base("right"), 160.0)

    def test_xyz_conversion_is_180_degree_rotation(self) -> None:
        transform = ReverseMountTransform(mount_yaw_degrees=180.0)
        x_arm, y_arm, z_arm = transform.robot_to_arm_xyz(300.0, 50.0, 100.0)
        self.assertAlmostEqual(x_arm, -300.0, places=6)
        self.assertAlmostEqual(y_arm, -50.0, places=6)
        self.assertAlmostEqual(z_arm, 100.0, places=6)


class RoArmUartTests(unittest.TestCase):
    def setUp(self) -> None:
        FakeSerial.instances.clear()

    def test_official_serial_settings_and_t105_framing(self) -> None:
        arm = RoArmUart(serial_factory=FakeSerial)
        try:
            state = arm.get_state()
            fake = FakeSerial.instances[-1]
            self.assertEqual(fake.kwargs["baudrate"], 115200)
            self.assertIsNone(fake.kwargs["dsrdtr"])
            self.assertFalse(fake.rts)
            self.assertFalse(fake.dtr)
            self.assertEqual(fake.writes[-1], b'{"T":105}\n')
            self.assertAlmostEqual(state.voltage_v, 12.11)
        finally:
            arm.close()

    def test_t121_base_only_and_zero_speed_rejection(self) -> None:
        arm = RoArmUart(serial_factory=FakeSerial)
        try:
            arm.move_base_degrees(
                -160.0,
                speed_degrees_s=10,
                acceleration_degrees_s2=10,
            )
            command = json.loads(FakeSerial.instances[-1].writes[-1])
            self.assertEqual(
                command,
                {
                    "T": 121,
                    "joint": 1,
                    "angle": -160.0,
                    "spd": 10,
                    "acc": 10,
                },
            )
            with self.assertRaises(ValueError):
                arm.move_base_degrees(0.0, speed_degrees_s=0)
        finally:
            arm.close()


class RealControllerSafetyTests(unittest.TestCase):
    def setUp(self) -> None:
        FakeSerial.instances.clear()

    def test_both_confirmations_are_required_before_serial_open(self) -> None:
        with self.assertRaises(PermissionError):
            RealRoArmController(
                confirm_motion=False,
                confirm_clearance=True,
                serial_factory=FakeSerial,
            )
        with self.assertRaises(PermissionError):
            RealRoArmController(
                confirm_motion=True,
                confirm_clearance=False,
                serial_factory=FakeSerial,
            )
        self.assertEqual(FakeSerial.instances, [])

    def test_confirmed_initialization_reads_state_without_motion(self) -> None:
        controller = RealRoArmController(
            confirm_motion=True,
            confirm_clearance=True,
            serial_factory=FakeSerial,
        )
        try:
            commands = [json.loads(item) for item in FakeSerial.instances[-1].writes]
            self.assertTrue(commands)
            self.assertTrue(all(command["T"] == 105 for command in commands))
        finally:
            controller.close()

    def test_second_automatic_motion_is_rejected_before_tx(self) -> None:
        controller = RealRoArmController(
            confirm_motion=True,
            confirm_clearance=True,
            serial_factory=FakeSerial,
        )
        try:
            controller._automatic_motion_executed = True
            with self.assertRaisesRegex(RuntimeError, "Only one automatic"):
                controller.execute_region("center", Event())
            commands = [json.loads(item) for item in FakeSerial.instances[-1].writes]
            self.assertTrue(all(command["T"] == 105 for command in commands))
        finally:
            controller.close()


if __name__ == "__main__":
    unittest.main()
