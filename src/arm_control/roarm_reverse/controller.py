"""TaskManager-compatible real backend for the reverse-mounted RoArm."""

from __future__ import annotations

import logging
import math
from threading import Event
import time
from typing import Any, Callable

from .transform import ReverseMountTransform
from .uart import RoArmState, RoArmUart


LOGGER = logging.getLogger(__name__)


class RealRoArmController:
    """First-stage real controller: low-speed base aiming only.

    It deliberately does not convert uncalibrated image pixels to x/y/z and
    does not move shoulder or elbow joints. It implements both arm method names
    used by the project's recent TaskManager versions.
    """

    mode = "real"

    def __init__(
        self,
        *,
        port: str = "/dev/ttyROARM",
        baudrate: int = 115200,
        mount_yaw_degrees: float = 180.0,
        region_yaw_degrees: float = 20.0,
        speed_degrees_s: int = 10,
        acceleration_degrees_s2: int = 10,
        settle_margin_seconds: float = 1.0,
        feedback_tolerance_degrees: float = 6.0,
        initial_base_tolerance_degrees: float = 15.0,
        confirm_motion: bool = False,
        confirm_clearance: bool = False,
        serial_factory: Callable[..., Any] | None = None,
    ) -> None:
        # Confirm before opening the serial port, so an invalid invocation has
        # no hardware side effects.
        if not confirm_motion:
            raise PermissionError(
                "Real-arm mode requires explicit motion confirmation"
            )
        if not confirm_clearance:
            raise PermissionError(
                "Reverse-mounted base motion requires clearance confirmation"
            )
        if not 1 <= int(speed_degrees_s) <= 30:
            raise ValueError("Initial real-arm speed must be in [1, 30] deg/s")
        if not 1 <= int(acceleration_degrees_s2) <= 50:
            raise ValueError("Initial acceleration must be in [1, 50] deg/s^2")
        if float(settle_margin_seconds) < 0:
            raise ValueError("settle_margin_seconds cannot be negative")

        self.transform = ReverseMountTransform(
            mount_yaw_degrees=mount_yaw_degrees,
            region_yaw_degrees=region_yaw_degrees,
        )
        self.speed_degrees_s = int(speed_degrees_s)
        self.acceleration_degrees_s2 = int(acceleration_degrees_s2)
        self.settle_margin_seconds = float(settle_margin_seconds)
        self.feedback_tolerance_degrees = float(feedback_tolerance_degrees)
        self.initial_base_tolerance_degrees = float(
            initial_base_tolerance_degrees
        )
        self.uart = RoArmUart(
            port=port,
            baudrate=baudrate,
            serial_factory=serial_factory,
        )
        self._automatic_motion_executed = False

        # T=105 identifies the device and validates feedback before movement.
        state = self.uart.get_state()
        self._validate_state(state)
        LOGGER.info(
            "Real RoArm ready: base=%.1f deg, voltage=%.2f V, mount=%.1f deg",
            math.degrees(state.base_rad),
            state.voltage_v,
            self.transform.mount_yaw_degrees,
        )

    def execute_region(self, region: str, abort_event: Event) -> bool:
        normalized = self.transform.normalize_region(region)
        if self._automatic_motion_executed:
            raise RuntimeError(
                "Only one automatic base movement is allowed per process; "
                "verify the zero pose and restart before another target"
            )

        before = self.get_state()
        current_degrees = math.degrees(before.base_rad)
        if abs(current_degrees) > self.initial_base_tolerance_degrees:
            raise RuntimeError(
                "Reverse-mount aiming requires the base to start near 0 deg; "
                f"feedback is {current_degrees:.1f} deg"
            )

        target_degrees = self.transform.camera_region_to_arm_base(normalized)
        if math.isclose(abs(target_degrees), 180.0, abs_tol=1e-6):
            target_degrees = -180.0 if current_degrees < 0.0 else 180.0

        LOGGER.warning(
            "Camera region %s maps to absolute base target %.1f deg",
            normalized,
            target_degrees,
        )
        LOGGER.warning("Reverse mount may produce a base sweep near 180 degrees")
        self._raise_if_aborted(abort_event)

        # Mark before TX: a communication failure must not allow an automatic
        # retry of a command that may already have started physical movement.
        self._automatic_motion_executed = True
        self.move_base_degrees(target_degrees, abort_event)
        self._raise_if_aborted(abort_event)
        LOGGER.info("Base aiming finished; reach/grasp remains disabled")
        return True

    def execute_grasp(self, plan: Any, abort_event: Event) -> bool:
        """Compatibility with TaskManager versions that pass a grasp plan."""
        region = getattr(plan, "region", plan)
        return self.execute_region(str(region), abort_event)

    def execute_mock_grasp(self, region: str, abort_event: Event) -> bool:
        """Compatibility with TaskManager versions that pass a region."""
        return self.execute_region(region, abort_event)

    def move_base_degrees(
        self,
        target_degrees: float,
        abort_event: Event | None = None,
    ) -> RoArmState:
        event = abort_event or Event()
        before = self.get_state()
        current_degrees = math.degrees(before.base_rad)
        # Do not assume shortest-path wrap behaviour across -180/+180.
        travel_degrees = abs(current_degrees - float(target_degrees))
        estimated_seconds = (
            travel_degrees / self.speed_degrees_s + self.settle_margin_seconds
        )
        LOGGER.warning(
            "Base %.1f -> %.1f deg; estimated %.1f s",
            current_degrees,
            target_degrees,
            estimated_seconds,
        )
        self.uart.move_base_degrees(
            target_degrees,
            speed_degrees_s=self.speed_degrees_s,
            acceleration_degrees_s2=self.acceleration_degrees_s2,
        )
        self._interruptible_wait(estimated_seconds, event)

        after = self.get_state()
        actual_degrees = math.degrees(after.base_rad)
        error = self._circular_distance_degrees(actual_degrees, target_degrees)
        if error > self.feedback_tolerance_degrees:
            raise RuntimeError(
                "Base feedback did not reach target: "
                f"target={target_degrees:.1f}, actual={actual_degrees:.1f}, "
                f"error={error:.1f} deg"
            )
        return after

    def get_state(self) -> RoArmState:
        state = self.uart.get_state()
        self._validate_state(state)
        return state

    def status(self) -> dict[str, Any]:
        state = self.get_state()
        return {
            "mode": self.mode,
            "mount_yaw_degrees": self.transform.mount_yaw_degrees,
            "base_degrees": math.degrees(state.base_rad),
            "shoulder_degrees": math.degrees(state.shoulder_rad),
            "elbow_degrees": math.degrees(state.elbow_rad),
            "hand_degrees": math.degrees(state.hand_rad),
            "x_mm": state.x_mm,
            "y_mm": state.y_mm,
            "z_mm": state.z_mm,
            "voltage_v": state.voltage_v,
        }

    def stop(self) -> None:
        LOGGER.warning(
            "Cooperative T=123 stop requested; use hardware power-off for "
            "an emergency"
        )
        self.uart.stop_continuous_motion()

    def close(self) -> None:
        self.uart.close()

    def _interruptible_wait(self, seconds: float, abort_event: Event) -> None:
        deadline = time.monotonic() + float(seconds)
        while time.monotonic() < deadline:
            if abort_event.is_set():
                try:
                    self.uart.stop_continuous_motion()
                finally:
                    raise InterruptedError("Real-arm task aborted")
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

    @staticmethod
    def _raise_if_aborted(abort_event: Event) -> None:
        if abort_event.is_set():
            raise InterruptedError("Real-arm task aborted")

    @staticmethod
    def _circular_distance_degrees(a: float, b: float) -> float:
        return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)

    @staticmethod
    def _validate_state(state: RoArmState) -> None:
        if not 7.0 <= state.voltage_v <= 13.0:
            raise RuntimeError(
                "RoArm voltage feedback is outside the expected range: "
                f"{state.voltage_v:.2f} V"
            )
