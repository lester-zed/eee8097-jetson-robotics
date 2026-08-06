from __future__ import annotations

import math
from threading import Event
import time
from typing import Any, Callable

from pipeline.types import CartesianWaypoint, ExecutionResult, GraspPlan, HealthResult


class CartesianRoArmController:
    """One-shot RoArm T=104 Cartesian grasp executor with T=105 feedback."""

    def __init__(
        self,
        *,
        port: str,
        baudrate: int = 115200,
        allow_real_motion: bool,
        confirm_clearance: bool,
        calibration_approved: bool,
        one_grasp_per_process: bool = True,
        feedback_tolerance_mm: float = 12.0,
        waypoint_timeout_s: float = 15.0,
        feedback_poll_s: float = 0.20,
        gripper_settle_s: float = 1.0,
        min_voltage_v: float = 7.0,
        max_voltage_v: float = 13.0,
        uart_factory: Callable[..., Any] | None = None,
    ) -> None:
        if not allow_real_motion:
            raise PermissionError("Real RoArm motion is disabled by configuration")
        if not confirm_clearance:
            raise PermissionError("Real RoArm motion requires clearance confirmation")
        if not calibration_approved:
            raise PermissionError("Real RoArm motion requires approved calibration")
        if feedback_tolerance_mm <= 0.0 or waypoint_timeout_s <= 0.0:
            raise ValueError("feedback tolerance and timeout must be positive")

        if uart_factory is None:
            from arm_control.roarm_uart import RoArmUart
            uart_factory = RoArmUart
        self.uart = uart_factory(port=port, baudrate=int(baudrate))
        self.port = str(port)
        self.mode = "real_cartesian"
        self.one_grasp_per_process = bool(one_grasp_per_process)
        self.feedback_tolerance_mm = float(feedback_tolerance_mm)
        self.waypoint_timeout_s = float(waypoint_timeout_s)
        self.feedback_poll_s = float(feedback_poll_s)
        self.gripper_settle_s = float(gripper_settle_s)
        self.min_voltage_v = float(min_voltage_v)
        self.max_voltage_v = float(max_voltage_v)
        self._executed = False
        self._moving = False
        self._last_state: Any = None

    def health_check(self) -> HealthResult:
        try:
            state = self.uart.get_state()
            self._validate_state(state)
            self._last_state = state
            return HealthResult(
                "roarm",
                True,
                "Cartesian RoArm UART and T=105 feedback ready",
                {
                    "port": self.port,
                    "voltage_v": state.voltage_v,
                    "x_mm": state.x_mm,
                    "y_mm": state.y_mm,
                    "z_mm": state.z_mm,
                },
            )
        except Exception as exc:
            return HealthResult("roarm", False, f"RoArm precheck failed: {exc}")

    def execute_grasp(self, plan: GraspPlan, abort_event: Event) -> ExecutionResult:
        if plan.target.provisional:
            raise PermissionError("Real RoArm refuses provisional target coordinates")
        if self.one_grasp_per_process and self._executed:
            raise RuntimeError("Only one automatic grasp is permitted per process")
        self._executed = True
        self._moving = True
        completed: list[str] = []
        feedback: dict[str, Any] = {}
        try:
            self._raise_if_aborted(abort_event)
            state = self.uart.get_state()
            self._validate_state(state)

            self.uart.set_gripper_radians(plan.gripper_open_rad)
            self._interruptible_wait(self.gripper_settle_s, abort_event)

            for waypoint in plan.waypoints:
                self._move_waypoint(waypoint, abort_event)
                completed.append(waypoint.name)
                if waypoint.name == "grasp":
                    self.uart.set_gripper_radians(plan.gripper_closed_rad)
                    self._interruptible_wait(self.gripper_settle_s, abort_event)

            final_state = self.uart.get_state()
            self._validate_state(final_state)
            self._last_state = final_state
            feedback = self._state_dict(final_state)
            return ExecutionResult(
                success=True,
                message="real Cartesian grasp sequence completed",
                completed_waypoints=tuple(completed),
                feedback=feedback,
            )
        except Exception:
            try:
                self.uart.stop_continuous_motion()
            except Exception:
                pass
            raise
        finally:
            self._moving = False

    def _move_waypoint(self, waypoint: CartesianWaypoint, abort_event: Event) -> None:
        self._raise_if_aborted(abort_event)
        point = waypoint.point
        if point.frame_id != "arm_base":
            raise ValueError("T=104 requires arm_base waypoint coordinates")
        values = (point.x_mm, point.y_mm, point.z_mm, waypoint.tool_angle_rad, waypoint.speed)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError(f"Non-finite Cartesian waypoint: {waypoint.name}")
        if waypoint.speed <= 0.0:
            raise ValueError("Cartesian speed must be positive")

        self.uart.send({
            "T": 104,
            "x": round(float(point.x_mm), 3),
            "y": round(float(point.y_mm), 3),
            "z": round(float(point.z_mm), 3),
            "t": round(float(waypoint.tool_angle_rad), 4),
            "spd": round(float(waypoint.speed), 4),
        })
        deadline = time.monotonic() + self.waypoint_timeout_s
        last_error = float("inf")
        while time.monotonic() < deadline:
            self._raise_if_aborted(abort_event)
            state = self.uart.get_state()
            self._validate_state(state)
            last_error = math.sqrt(
                (state.x_mm - point.x_mm) ** 2
                + (state.y_mm - point.y_mm) ** 2
                + (state.z_mm - point.z_mm) ** 2
            )
            self._last_state = state
            if last_error <= self.feedback_tolerance_mm:
                return
            self._interruptible_wait(self.feedback_poll_s, abort_event)
        raise RuntimeError(
            f"RoArm waypoint {waypoint.name} feedback timeout: "
            f"error={last_error:.1f} mm"
        )

    def status(self) -> dict[str, Any]:
        state = self._last_state
        if state is None:
            return {
                "mode": self.mode,
                "port": self.port,
                "moving": self._moving,
                "automatic_grasp_executed": self._executed,
            }
        payload = self._state_dict(state)
        payload.update({
            "mode": self.mode,
            "port": self.port,
            "moving": self._moving,
            "automatic_grasp_executed": self._executed,
        })
        return payload

    def stop(self) -> None:
        self.uart.stop_continuous_motion()

    def close(self) -> None:
        self.uart.close()

    def _validate_state(self, state: Any) -> None:
        voltage = float(state.voltage_v)
        if not self.min_voltage_v <= voltage <= self.max_voltage_v:
            raise RuntimeError(f"RoArm voltage outside configured range: {voltage:.2f} V")

    def _interruptible_wait(self, seconds: float, abort_event: Event) -> None:
        deadline = time.monotonic() + max(0.0, float(seconds))
        while time.monotonic() < deadline:
            self._raise_if_aborted(abort_event)
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

    def _raise_if_aborted(self, abort_event: Event) -> None:
        if abort_event.is_set():
            try:
                self.uart.stop_continuous_motion()
            finally:
                raise InterruptedError("Real RoArm task aborted")

    @staticmethod
    def _state_dict(state: Any) -> dict[str, Any]:
        return {
            "x_mm": float(state.x_mm),
            "y_mm": float(state.y_mm),
            "z_mm": float(state.z_mm),
            "voltage_v": float(state.voltage_v),
        }
