from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys
import unittest

SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from roarm_home import execute_custom_home, resolve_home_target


@dataclass
class FakeState:
    base_rad: float = 0.0
    shoulder_rad: float = 0.0
    elbow_rad: float = math.pi / 2
    hand_rad: float = 2.7
    x_mm: float = 0.0
    y_mm: float = 0.0
    z_mm: float = 0.0
    voltage_v: float = 12.0

    @property
    def raw(self):
        return {"T": 1051, "b": self.base_rad, "s": self.shoulder_rad,
                "e": self.elbow_rad, "t": self.hand_rad, "v": 1200}


class FakeUart:
    instance = None

    def __init__(self, **kwargs):
        self.state = FakeState()
        self.commands = []
        FakeUart.instance = self

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return None

    def get_state(self):
        return self.state

    def stop_continuous_motion(self):
        self.commands.append({"T": 123})

    def send(self, command):
        self.commands.append(dict(command))

    def move_all_joints_degrees(self, *, base, shoulder, elbow, hand,
                                speed_degrees_s, acceleration_degrees_s2):
        self.commands.append({
            "T": 122, "b": base, "s": shoulder, "e": elbow, "h": hand,
            "spd": speed_degrees_s, "acc": acceleration_degrees_s2,
        })
        # Report -180 for a +180 target to verify circular-equivalence handling.
        self.state = FakeState(
            base_rad=math.radians(-180.0 if base == 180.0 else base),
            shoulder_rad=math.radians(shoulder),
            elbow_rad=math.radians(elbow),
            hand_rad=math.radians(hand),
        )


def document():
    return {
        "roarm_home": {
            "enabled": True,
            "allow_real_motion": True,
            "confirm_clearance": True,
            "confirmation_phrase": "HOME",
            "device": "fake",
            "baudrate": 115200,
            "stop_continuous_first": True,
            "enable_torque_first": True,
            "pose": {
                "base_deg": 180.0,
                "shoulder_deg": 0.0,
                "elbow_deg": 90.0,
                "hand_deg": None,
            },
            "motion": {
                "speed_deg_s": 15,
                "acceleration_deg_s2": 20,
                "timeout_s": 1.0,
                "poll_s": 0.001,
                "tolerance_deg": 5.0,
                "settle_s": 0.0,
            },
        }
    }


class HomeTests(unittest.TestCase):
    def test_null_hand_preserves_current_feedback(self):
        state = FakeState(hand_rad=2.5)
        target = resolve_home_target(document()["roarm_home"], state)
        self.assertAlmostEqual(target["hand"], math.degrees(2.5))

    def test_custom_home_sends_t122_at_increased_speed(self):
        result = execute_custom_home(
            document(),
            input_fn=lambda _: "HOME",
            uart_factory=FakeUart,
        )
        self.assertTrue(result.success)
        command = next(item for item in FakeUart.instance.commands if item["T"] == 122)
        self.assertEqual(command["b"], 180.0)
        self.assertEqual(command["spd"], 15)
        self.assertEqual(command["acc"], 20)
        self.assertAlmostEqual(result.error_deg["base"], 0.0)

    def test_motion_lock_is_required(self):
        locked = document()
        locked["roarm_home"]["allow_real_motion"] = False
        with self.assertRaises(PermissionError):
            execute_custom_home(
                locked,
                input_fn=lambda _: "HOME",
                uart_factory=FakeUart,
            )


if __name__ == "__main__":
    unittest.main()
