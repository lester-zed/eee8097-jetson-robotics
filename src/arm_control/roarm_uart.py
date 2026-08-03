from __future__ import annotations

from dataclasses import dataclass
import json
from threading import RLock
import time
from typing import Any, Callable

from utils.logger import get_logger


class RoArmError(RuntimeError):
    """Base exception for RoArm communication and validation failures."""


class RoArmTimeoutError(RoArmError):
    """Raised when the arm does not return the requested feedback."""


@dataclass(frozen=True)
class RoArmState:
    x_mm: float
    y_mm: float
    z_mm: float
    base_rad: float
    shoulder_rad: float
    elbow_rad: float
    hand_rad: float
    voltage_v: float
    raw: dict[str, Any]

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "RoArmState":
        if int(payload.get("T", -1)) != 1051:
            raise RoArmError(f"Unexpected feedback type: {payload.get('T')!r}")
        try:
            return cls(
                x_mm=float(payload["x"]),
                y_mm=float(payload["y"]),
                z_mm=float(payload["z"]),
                base_rad=float(payload["b"]),
                shoulder_rad=float(payload["s"]),
                elbow_rad=float(payload["e"]),
                hand_rad=float(payload["t"]),
                voltage_v=float(payload["v"]) / 100.0,
                raw=dict(payload),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise RoArmError(f"Malformed T=1051 feedback: {payload}") from exc


class RoArmUart:
    """Thread-safe line-delimited JSON transport for a RoArm-M2-S."""

    JOINT_LIMITS_DEG = {
        1: (-180.0, 180.0),
        2: (-90.0, 90.0),
        3: (0.0, 180.0),
        4: (45.0, 180.0),
    }
    GRIPPER_LIMITS_RAD = (1.08, 3.14)

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        *,
        read_timeout_seconds: float = 0.10,
        response_timeout_seconds: float = 2.0,
        serial_factory: Callable[..., Any] | None = None,
    ) -> None:
        if baudrate <= 0:
            raise ValueError("baudrate must be positive")
        if response_timeout_seconds <= 0:
            raise ValueError("response_timeout_seconds must be positive")

        self.logger = get_logger(self.__class__.__name__)
        self.port = port
        self.baudrate = baudrate
        self.response_timeout_seconds = response_timeout_seconds
        self._lock = RLock()
        self._closed = False

        if serial_factory is None:
            try:
                import serial
            except ModuleNotFoundError as exc:
                raise RuntimeError(
                    "pyserial is required for real RoArm UART; "
                    "install requirements.txt inside the robot container"
                ) from exc
            factory = serial.Serial
        else:
            factory = serial_factory
        self.logger.info("Opening RoArm UART: %s @ %d", port, baudrate)
        self._serial = factory(
            port=port,
            baudrate=baudrate,
            timeout=read_timeout_seconds,
            write_timeout=1.0,
            dsrdtr=None,
        )
        self._set_control_line("setRTS", False)
        self._set_control_line("setDTR", False)
        time.sleep(0.10)

    def send(self, command: dict[str, Any]) -> None:
        """Send one compact JSON command followed by a newline."""
        if "T" not in command:
            raise ValueError("A RoArm command must contain the T field")
        with self._lock:
            self._write_locked(command)

    def get_state(self) -> RoArmState:
        """Send T=105 and wait for the matching T=1051 feedback."""
        with self._lock:
            self._reset_input_buffer_locked()
            self._write_locked({"T": 105})
            deadline = time.monotonic() + self.response_timeout_seconds

            while time.monotonic() < deadline:
                raw_line = self._serial.readline()
                if not raw_line:
                    continue
                payload = self._parse_json_line(raw_line)
                if payload is None:
                    continue
                if int(payload.get("T", -1)) == 1051:
                    return RoArmState.from_payload(payload)

        raise RoArmTimeoutError(
            f"No T=1051 feedback from {self.port} within "
            f"{self.response_timeout_seconds:.1f}s"
        )

    def move_single_joint_degrees(
        self,
        joint: int,
        angle_degrees: float,
        *,
        speed_degrees_s: int = 10,
        acceleration_degrees_s2: int = 10,
    ) -> None:
        """Send the official T=121 single-joint degree command."""
        if joint not in self.JOINT_LIMITS_DEG:
            raise ValueError("joint must be 1 (base) through 4 (EoAT)")
        lower, upper = self.JOINT_LIMITS_DEG[joint]
        angle = float(angle_degrees)
        if not lower <= angle <= upper:
            raise ValueError(
                f"joint {joint} angle {angle} outside [{lower}, {upper}] degrees"
            )
        speed = self._safe_positive_int(
            speed_degrees_s,
            "speed_degrees_s",
            maximum=60,
        )
        acceleration = self._safe_positive_int(
            acceleration_degrees_s2,
            "acceleration_degrees_s2",
            maximum=254,
        )
        self.send(
            {
                "T": 121,
                "joint": joint,
                "angle": round(angle, 3),
                "spd": speed,
                "acc": acceleration,
            }
        )

    def move_all_joints_degrees(
        self,
        *,
        base: float,
        shoulder: float,
        elbow: float,
        hand: float,
        speed_degrees_s: int = 10,
        acceleration_degrees_s2: int = 10,
    ) -> None:
        """Send the official T=122 all-joints degree command."""
        values = {1: base, 2: shoulder, 3: elbow, 4: hand}
        for joint, angle in values.items():
            lower, upper = self.JOINT_LIMITS_DEG[joint]
            if not lower <= float(angle) <= upper:
                raise ValueError(
                    f"joint {joint} angle {angle} outside [{lower}, {upper}]"
                )
        speed = self._safe_positive_int(
            speed_degrees_s,
            "speed_degrees_s",
            maximum=60,
        )
        acceleration = self._safe_positive_int(
            acceleration_degrees_s2,
            "acceleration_degrees_s2",
            maximum=254,
        )
        self.send(
            {
                "T": 122,
                "b": round(float(base), 3),
                "s": round(float(shoulder), 3),
                "e": round(float(elbow), 3),
                "h": round(float(hand), 3),
                "spd": speed,
                "acc": acceleration,
            }
        )

    def set_gripper_radians(
        self,
        angle_radians: float,
        *,
        speed_steps_s: int = 100,
        acceleration: int = 10,
    ) -> None:
        """Send T=106 for the stock clamp EoAT."""
        lower, upper = self.GRIPPER_LIMITS_RAD
        angle = float(angle_radians)
        if not lower <= angle <= upper:
            raise ValueError(
                f"gripper angle {angle} outside [{lower}, {upper}] radians"
            )
        speed = self._safe_positive_int(
            speed_steps_s,
            "speed_steps_s",
            maximum=1000,
        )
        acc = self._safe_positive_int(acceleration, "acceleration", maximum=254)
        self.send(
            {
                "T": 106,
                "cmd": round(angle, 4),
                "spd": speed,
                "acc": acc,
            }
        )

    def move_initial_position(self) -> None:
        """Send T=100. This causes real motion and may block in firmware."""
        self.send({"T": 100})

    def stop_continuous_motion(self) -> None:
        """Send the official T=123 continuous-mode stop command.

        This is a cooperative stop command, not a certified emergency stop and
        not a guarantee that an already running blocking command will stop.
        """
        self.send({"T": 123, "m": 0, "axis": 0, "cmd": 0, "spd": 0})

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._serial.close()
            self._closed = True
            self.logger.info("RoArm UART closed")

    def _write_locked(self, command: dict[str, Any]) -> None:
        if self._closed:
            raise RoArmError("RoArm UART is closed")
        encoded = (
            json.dumps(command, separators=(",", ":"), ensure_ascii=True)
            + "\n"
        ).encode("utf-8")
        written = self._serial.write(encoded)
        if written != len(encoded):
            raise RoArmError(
                f"Incomplete serial write: wrote {written} of {len(encoded)} bytes"
            )
        flush = getattr(self._serial, "flush", None)
        if callable(flush):
            flush()
        self.logger.info("RoArm TX: %s", encoded.decode("utf-8").strip())

    def _reset_input_buffer_locked(self) -> None:
        reset = getattr(self._serial, "reset_input_buffer", None)
        if callable(reset):
            reset()

    def _set_control_line(self, method_name: str, value: bool) -> None:
        method = getattr(self._serial, method_name, None)
        if callable(method):
            method(value)

    def _parse_json_line(self, raw_line: bytes | str) -> dict[str, Any] | None:
        if isinstance(raw_line, bytes):
            line = raw_line.decode("utf-8", errors="replace").strip()
        else:
            line = str(raw_line).strip()
        start = line.find("{")
        end = line.rfind("}")
        if start < 0 or end < start:
            self.logger.debug("Ignoring non-JSON RoArm line: %s", line)
            return None
        try:
            payload = json.loads(line[start : end + 1])
        except json.JSONDecodeError:
            self.logger.warning("Ignoring malformed RoArm JSON: %s", line)
            return None
        return payload if isinstance(payload, dict) else None

    @staticmethod
    def _safe_positive_int(value: int, name: str, *, maximum: int) -> int:
        integer = int(value)
        # The official protocol treats zero as maximum speed/acceleration.
        if not 1 <= integer <= maximum:
            raise ValueError(f"{name} must be in [1, {maximum}], never zero")
        return integer

    def __enter__(self) -> "RoArmUart":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
