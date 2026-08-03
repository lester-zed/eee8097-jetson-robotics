"""Minimal line-delimited JSON UART transport for RoArm-M2-S."""

from __future__ import annotations

from dataclasses import dataclass
import json
import logging
from threading import RLock
import time
from typing import Any, Callable


LOGGER = logging.getLogger(__name__)


class RoArmError(RuntimeError):
    """Base exception for RoArm communication and protocol failures."""


class RoArmTimeoutError(RoArmError):
    """Raised when matching feedback is not received in time."""


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
    """Thread-safe RoArm UART using official 115200/newline JSON framing."""

    BASE_LIMIT_DEGREES = (-180.0, 180.0)
    GRIPPER_LIMIT_RADIANS = (1.08, 3.14)

    def __init__(
        self,
        port: str = "/dev/ttyROARM",
        baudrate: int = 115200,
        *,
        read_timeout_seconds: float = 0.10,
        response_timeout_seconds: float = 2.0,
        serial_factory: Callable[..., Any] | None = None,
    ) -> None:
        if int(baudrate) <= 0:
            raise ValueError("baudrate must be positive")
        if float(response_timeout_seconds) <= 0:
            raise ValueError("response_timeout_seconds must be positive")

        self.port = str(port)
        self.baudrate = int(baudrate)
        self.response_timeout_seconds = float(response_timeout_seconds)
        self._lock = RLock()
        self._closed = False

        if serial_factory is None:
            try:
                import serial
            except ModuleNotFoundError as exc:
                raise RuntimeError(
                    "pyserial is required inside the Docker environment for "
                    "real RoArm UART control"
                ) from exc
            factory = serial.Serial
        else:
            factory = serial_factory

        LOGGER.info("Opening RoArm UART: %s @ %d", self.port, self.baudrate)
        self._serial = factory(
            port=self.port,
            baudrate=self.baudrate,
            timeout=float(read_timeout_seconds),
            write_timeout=1.0,
            dsrdtr=None,
        )
        self._set_control_line("setRTS", False)
        self._set_control_line("setDTR", False)
        time.sleep(0.10)

    def get_state(self) -> RoArmState:
        """Send T=105 and wait for the corresponding T=1051 feedback."""
        with self._lock:
            self._reset_input_buffer_locked()
            self._write_locked({"T": 105})
            deadline = time.monotonic() + self.response_timeout_seconds

            while time.monotonic() < deadline:
                raw_line = self._serial.readline()
                if not raw_line:
                    continue
                payload = self._parse_json_line(raw_line)
                if payload is not None and int(payload.get("T", -1)) == 1051:
                    return RoArmState.from_payload(payload)

        raise RoArmTimeoutError(
            f"No T=1051 feedback from {self.port} within "
            f"{self.response_timeout_seconds:.1f}s"
        )

    def move_base_degrees(
        self,
        angle_degrees: float,
        *,
        speed_degrees_s: int = 10,
        acceleration_degrees_s2: int = 10,
    ) -> None:
        """Send T=121 for the base joint only."""
        lower, upper = self.BASE_LIMIT_DEGREES
        angle = float(angle_degrees)
        if not lower <= angle <= upper:
            raise ValueError(f"base angle must be within [{lower}, {upper}]")
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
                "joint": 1,
                "angle": round(angle, 3),
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
        lower, upper = self.GRIPPER_LIMIT_RADIANS
        angle = float(angle_radians)
        if not lower <= angle <= upper:
            raise ValueError(
                f"gripper angle must be within [{lower}, {upper}] radians"
            )
        speed = self._safe_positive_int(
            speed_steps_s,
            "speed_steps_s",
            maximum=1000,
        )
        acc = self._safe_positive_int(
            acceleration,
            "acceleration",
            maximum=254,
        )
        self.send(
            {
                "T": 106,
                "cmd": round(angle, 4),
                "spd": speed,
                "acc": acc,
            }
        )

    def stop_continuous_motion(self) -> None:
        """Send T=123/cmd=0; this is not a certified emergency stop."""
        self.send({"T": 123, "m": 0, "axis": 0, "cmd": 0, "spd": 0})

    def send(self, command: dict[str, Any]) -> None:
        if "T" not in command:
            raise ValueError("A RoArm command must include a T field")
        with self._lock:
            self._write_locked(command)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._serial.close()
            self._closed = True
            LOGGER.info("RoArm UART closed")

    def _write_locked(self, command: dict[str, Any]) -> None:
        if self._closed:
            raise RoArmError("RoArm UART is closed")
        encoded = (
            json.dumps(command, separators=(",", ":"), ensure_ascii=True) + "\n"
        ).encode("utf-8")
        written = self._serial.write(encoded)
        if written != len(encoded):
            raise RoArmError(
                f"Incomplete serial write: wrote {written} of {len(encoded)} bytes"
            )
        flush = getattr(self._serial, "flush", None)
        if callable(flush):
            flush()
        LOGGER.info("RoArm TX: %s", encoded.decode("utf-8").strip())

    def _reset_input_buffer_locked(self) -> None:
        reset = getattr(self._serial, "reset_input_buffer", None)
        if callable(reset):
            reset()

    def _set_control_line(self, method_name: str, value: bool) -> None:
        method = getattr(self._serial, method_name, None)
        if callable(method):
            method(value)

    @staticmethod
    def _parse_json_line(raw_line: bytes | str) -> dict[str, Any] | None:
        if isinstance(raw_line, bytes):
            line = raw_line.decode("utf-8", errors="replace").strip()
        else:
            line = str(raw_line).strip()
        start = line.find("{")
        end = line.rfind("}")
        if start < 0 or end < start:
            return None
        try:
            payload = json.loads(line[start : end + 1])
        except json.JSONDecodeError:
            return None
        return payload if isinstance(payload, dict) else None

    @staticmethod
    def _safe_positive_int(value: int, name: str, *, maximum: int) -> int:
        integer = int(value)
        # Official firmware treats zero as maximum speed/acceleration.
        if not 1 <= integer <= maximum:
            raise ValueError(f"{name} must be in [1, {maximum}], never zero")
        return integer

    def __enter__(self) -> "RoArmUart":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
