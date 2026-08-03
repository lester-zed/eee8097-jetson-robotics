from __future__ import annotations

import math
from threading import Event
import time
from typing import Any, Callable

from arm_control.coordinate_frames import MountTransform
from arm_control.roarm_uart import RoArmState, RoArmUart
from utils.logger import get_logger


class RealRoArmController:
    """Low-speed real RoArm backend for the first vision-to-arm milestone.

    This controller intentionally moves only the base joint. It does not turn
    an uncalibrated image point into x/y/z and it does not claim to perform a
    physical grasp. A future calibrated preset backend can replace this class
    without changing TaskManager.
    """

    def __init__(
        self,
        *,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        mount_yaw_degrees: float = 180.0,
        region_yaw_degrees: float = 20.0,
        prefer_positive_180: bool = True,
        speed_degrees_s: int = 10,
        acceleration_degrees_s2: int = 10,
        settle_margin_seconds: float = 1.0,
        feedback_tolerance_degrees: float = 6.0,
        initial_base_tolerance_degrees: float = 15.0,
        perform_gripper_cycle: bool = False,
        gripper_open_radians: float = 2.70,
        gripper_closed_radians: float = 3.14,
        confirm_motion: bool = False,
        confirm_clearance: bool = False,
        serial_factory: Callable[..., Any] | None = None,
    ) -> None:
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
            raise ValueError("Initial real-arm acceleration must be in [1, 50]")
        if settle_margin_seconds < 0:
            raise ValueError("settle_margin_seconds cannot be negative")

        self.logger = get_logger(self.__class__.__name__)
        self.mode = "real"
        self.transform = MountTransform(
            mount_yaw_degrees=mount_yaw_degrees,
            region_yaw_degrees=region_yaw_degrees,
            prefer_positive_180=prefer_positive_180,
        )
        self.speed_degrees_s = int(speed_degrees_s)
        self.acceleration_degrees_s2 = int(acceleration_degrees_s2)
        self.settle_margin_seconds = float(settle_margin_seconds)
        self.feedback_tolerance_degrees = float(feedback_tolerance_degrees)
        self.initial_base_tolerance_degrees = float(
            initial_base_tolerance_degrees
        )
        self.perform_gripper_cycle = bool(perform_gripper_cycle)
        self.gripper_open_radians = float(gripper_open_radians)
        self.gripper_closed_radians = float(gripper_closed_radians)
        self.uart = RoArmUart(
            port=port,
            baudrate=baudrate,
            serial_factory=serial_factory,
        )
        self._automatic_motion_executed = False

        # A real state query is mandatory before any movement is accepted.
        state = self.uart.get_state()
        self._validate_state(state)
        self.logger.info(
            "[REAL ARM] Ready: base=%.1f deg voltage=%.2f V mount_yaw=%.1f deg",
            math.degrees(state.base_rad),
            state.voltage_v,
            self.transform.mount_yaw_degrees,
        )

    def execute_region(self, region: str, abort_event: Event) -> bool:
        """Aim the base at a camera region using the reverse-mount transform."""
        normalized = self.transform.normalize_region(region)
        if self._automatic_motion_executed:
            raise RuntimeError(
                "This first-stage controller permits only one automatic base "
                "movement per process. Re-establish a verified zero pose and "
                "restart before another target."
            )

        before = self.uart.get_state()
        self._validate_state(before)
        current_base = math.degrees(before.base_rad)
        if abs(current_base) > self.initial_base_tolerance_degrees:
            raise RuntimeError(
                "Automatic reverse-mount aiming requires the base to start "
                f"near 0 deg; feedback is {current_base:.1f} deg."
            )

        target_base = self.transform.camera_region_to_arm_base(normalized)
        # At exactly 180 degrees, choose the endpoint on the same side as the
        # current feedback. This avoids assuming that firmware wraps through
        # the +/-180 discontinuity by the shortest path.
        if math.isclose(abs(target_base), 180.0, abs_tol=1e-6):
            target_base = -180.0 if current_base < 0.0 else 180.0
        self.logger.warning(
            "[REAL ARM] Camera region=%s -> absolute base target=%.1f deg",
            normalized,
            target_base,
        )
        self.logger.warning(
            "[REAL ARM] Reverse mount: this may sweep nearly 180 degrees"
        )

        self._raise_if_aborted(abort_event)
        # Set before TX so a communication/feedback failure cannot lead to an
        # automatic retry of a command that may already be moving the arm.
        self._automatic_motion_executed = True
        self.move_base_degrees(target_base, abort_event)
        self._raise_if_aborted(abort_event)

        if self.perform_gripper_cycle:
            self.logger.info("[REAL ARM] Running optional gripper test cycle")
            self.uart.set_gripper_radians(self.gripper_open_radians)
            self._interruptible_wait(1.0, abort_event)
            self.uart.set_gripper_radians(self.gripper_closed_radians)
            self._interruptible_wait(1.0, abort_event)

        self.logger.info(
            "[REAL ARM] Base aiming complete; calibrated reach/grasp is disabled"
        )
        return True

    def execute_grasp(self, plan: Any, abort_event: Event) -> bool:
        """TaskManager-compatible API used by the modular pipeline."""
        region = getattr(plan, "region", plan)
        return self.execute_region(str(region), abort_event)

    def execute_mock_grasp(self, region: str, abort_event: Event) -> bool:
        """Backward-compatible API for the user's current TaskManager."""
        return self.execute_region(region, abort_event)

    def move_base_degrees(
        self,
        target_degrees: float,
        abort_event: Event | None = None,
    ) -> RoArmState:
        abort_event = abort_event or Event()
        before = self.uart.get_state()
        self._validate_state(before)
        current_degrees = math.degrees(before.base_rad)
        # Use the conservative absolute joint-coordinate distance. Do not
        # assume firmware chooses the shortest physical path across +/-180.
        travel_degrees = abs(current_degrees - float(target_degrees))
        estimated_seconds = (
            travel_degrees / self.speed_degrees_s
            + self.settle_margin_seconds
        )

        self.logger.warning(
            "[REAL ARM] Base %.1f -> %.1f deg; estimated %.1f s",
            current_degrees,
            target_degrees,
            estimated_seconds,
        )
        self.uart.move_single_joint_degrees(
            1,
            target_degrees,
            speed_degrees_s=self.speed_degrees_s,
            acceleration_degrees_s2=self.acceleration_degrees_s2,
        )
        self._interruptible_wait(estimated_seconds, abort_event)

        after = self.uart.get_state()
        self._validate_state(after)
        actual_degrees = math.degrees(after.base_rad)
        error = self._circular_distance_degrees(actual_degrees, target_degrees)
        if error > self.feedback_tolerance_degrees:
            raise RuntimeError(
                "RoArm base feedback did not reach target: "
                f"target={target_degrees:.1f} actual={actual_degrees:.1f} "
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

    def move_to_initial_position(self, abort_event: Event | None = None) -> None:
        """Explicit T=100 helper; never called automatically."""
        event = abort_event or Event()
        self._raise_if_aborted(event)
        self.logger.warning(
            "[REAL ARM] Sending T=100; reverse-mounted home points toward robot rear"
        )
        self.uart.move_initial_position()

    def stop(self) -> None:
        self.logger.warning(
            "[REAL ARM] Cooperative stop requested; use hardware power-off for emergency"
        )
        self.uart.stop_continuous_motion()

    def close(self) -> None:
        self.uart.close()

    def _interruptible_wait(self, seconds: float, abort_event: Event) -> None:
        deadline = time.monotonic() + seconds
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
        # The official operating range is 7.0-12.6 V. A small upper allowance
        # avoids false failure from measurement tolerance while catching a bad
        # or nonsensical response before motion.
        if not 7.0 <= state.voltage_v <= 13.0:
            raise RuntimeError(
                f"RoArm voltage feedback is outside the expected range: "
                f"{state.voltage_v:.2f} V"
            )
