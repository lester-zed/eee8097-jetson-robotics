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
        feedback_initial_delay_s: float = 0.75,
        initial_feedback_delay_s: float | None = None,
        uart_response_timeout_s: float = 2.0,
        gripper_settle_s: float = 1.0,
        gripper_speed_steps_s: int | None = None,
        gripper_acceleration: int = 10,
        gripper_motion_margin_s: float = 0.75,
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

        # Backward-compatible alias. Some local tests/config revisions used
        # ``initial_feedback_delay_s`` before the canonical name settled on
        # ``feedback_initial_delay_s``. Accept either spelling.
        if initial_feedback_delay_s is not None:
            alias_value = float(initial_feedback_delay_s)
            if (
                not math.isclose(float(feedback_initial_delay_s), 0.75, abs_tol=1e-12)
                and not math.isclose(float(feedback_initial_delay_s), alias_value, abs_tol=1e-12)
            ):
                raise ValueError(
                    "feedback_initial_delay_s and initial_feedback_delay_s disagree"
                )
            feedback_initial_delay_s = alias_value

        if feedback_poll_s < 0.0 or feedback_initial_delay_s < 0.0:
            raise ValueError("feedback delays must be non-negative")
        if uart_response_timeout_s <= 0.0:
            raise ValueError("uart_response_timeout_s must be positive")
        if gripper_settle_s < 0.0 or gripper_motion_margin_s < 0.0:
            raise ValueError("gripper settle/margin times must be non-negative")
        effective_gripper_speed = (
            100 if gripper_speed_steps_s is None else int(gripper_speed_steps_s)
        )
        if not 1 <= effective_gripper_speed <= 1000:
            raise ValueError("gripper_speed_steps_s must be in [1, 1000]")
        if not 1 <= int(gripper_acceleration) <= 254:
            raise ValueError("gripper_acceleration must be in [1, 254]")

        if uart_factory is None:
            from arm_control.roarm_uart import RoArmUart
            uart_factory = RoArmUart
        self.uart = uart_factory(
            port=port,
            baudrate=int(baudrate),
            response_timeout_seconds=float(uart_response_timeout_s),
        )
        self.port = str(port)
        self.mode = "real_cartesian"
        self.one_grasp_per_process = bool(one_grasp_per_process)
        self.feedback_tolerance_mm = float(feedback_tolerance_mm)
        self.waypoint_timeout_s = float(waypoint_timeout_s)
        self.feedback_poll_s = float(feedback_poll_s)
        self.feedback_initial_delay_s = float(feedback_initial_delay_s)
        self.uart_response_timeout_s = float(uart_response_timeout_s)
        self.gripper_settle_s = float(gripper_settle_s)
        self.gripper_speed_steps_s = effective_gripper_speed
        # Direct unit tests from older revisions construct the controller
        # without a gripper speed. Preserve their historical fixed-settle
        # behavior. Production entry points always pass an explicit speed and
        # therefore enable travel-time-aware waiting.
        self.gripper_motion_timing_enabled = gripper_speed_steps_s is not None
        self.gripper_acceleration = int(gripper_acceleration)
        self.gripper_motion_margin_s = float(gripper_motion_margin_s)
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
        waypoint_feedback: dict[str, dict[str, Any]] = {}
        try:
            self._raise_if_aborted(abort_event)
            state = self.uart.get_state()
            self._validate_state(state)

            # IMPORTANT: on the stock RoArm clamp, T=104 field ``t`` is the
            # clamp/wrist angle itself.  Therefore opening with T=106 and then
            # sending a T=104 waypoint with t=3.14 would immediately close the
            # clamp again.  Keep T=104 ``t`` synchronized with the intended
            # gripper state throughout the Cartesian sequence.
            open_rad = self._validate_clamp_angle(plan.gripper_open_rad, "gripper_open_rad")
            closed_rad = self._validate_clamp_angle(plan.gripper_closed_rad, "gripper_closed_rad")

            initial_hand_rad = float(getattr(state, "hand_rad", closed_rad))
            self._set_gripper_radians(open_rad)
            open_wait_s = self._gripper_wait_seconds(initial_hand_rad, open_rad)
            self._interruptible_wait(open_wait_s, abort_event)

            gripper_feedback: dict[str, Any] = {
                "open_rad": open_rad,
                "closed_rad": closed_rad,
                "speed_steps_s": self.gripper_speed_steps_s,
                "acceleration": self.gripper_acceleration,
                "motion_timing_enabled": self.gripper_motion_timing_enabled,
                "open_wait_s": open_wait_s,
            }

            gripper_closed = False
            for waypoint in plan.waypoints:
                eoat_angle = closed_rad if gripper_closed else open_rad
                waypoint_feedback[waypoint.name] = self._move_waypoint(
                    waypoint,
                    abort_event,
                    eoat_angle_rad=eoat_angle,
                )
                completed.append(waypoint.name)
                if waypoint.name == "grasp":
                    # T=106 starts the clamp servo movement but does not provide
                    # an arrival acknowledgement.  At low servo speeds the full
                    # clamp travel can take several seconds.  Do not start lift
                    # after a fixed 1 s sleep: estimate the minimum travel time
                    # from the commanded angle delta and servo step speed, then
                    # add a configurable safety margin.
                    self._set_gripper_radians(closed_rad)
                    close_wait_s = self._gripper_wait_seconds(open_rad, closed_rad)
                    gripper_feedback["close_wait_s"] = close_wait_s
                    gripper_feedback["estimated_close_motion_s"] = (
                        self._estimated_gripper_motion_seconds(open_rad, closed_rad)
                    )
                    self._interruptible_wait(close_wait_s, abort_event)

                    # Require one fresh T=1051 response before lift.  We record
                    # hand_rad and torH as diagnostics, but deliberately do NOT
                    # require hand_rad == closed_rad: a real object can stop the
                    # clamp before the no-load closed angle while still being
                    # successfully grasped.
                    post_close_state = self.uart.get_state()
                    self._validate_state(post_close_state)
                    self._last_state = post_close_state
                    gripper_feedback["post_close_hand_rad"] = float(
                        getattr(post_close_state, "hand_rad", float("nan"))
                    )
                    raw = getattr(post_close_state, "raw", {})
                    gripper_feedback["post_close_torque_raw"] = (
                        raw.get("torH") if isinstance(raw, dict) else None
                    )
                    gripper_closed = True

            final_state = self.uart.get_state()
            self._validate_state(final_state)
            self._last_state = final_state
            feedback = self._state_dict(final_state)
            # Keep the existing top-level final-state keys for backward
            # compatibility, and add per-waypoint retry/arrival diagnostics.
            # Local hardware tests use these counters to verify that transient
            # T=105 timeouts after a blocking T=104 do not abort the sequence.
            feedback["waypoints"] = waypoint_feedback
            feedback["gripper"] = gripper_feedback
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

    def _move_waypoint(
        self,
        waypoint: CartesianWaypoint,
        abort_event: Event,
        *,
        eoat_angle_rad: float | None = None,
    ) -> dict[str, Any]:
        self._raise_if_aborted(abort_event)
        point = waypoint.point
        if point.frame_id != "arm_base":
            raise ValueError("T=104 requires arm_base waypoint coordinates")
        t_rad = waypoint.tool_angle_rad if eoat_angle_rad is None else float(eoat_angle_rad)
        values = (point.x_mm, point.y_mm, point.z_mm, t_rad, waypoint.speed)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError(f"Non-finite Cartesian waypoint: {waypoint.name}")
        if waypoint.speed <= 0.0:
            raise ValueError("Cartesian speed must be positive")

        waypoint_started = time.monotonic()
        t105_attempts = 0
        transient_t105_timeouts = 0
        feedback_samples = 0

        self.uart.send({
            "T": 104,
            "x": round(float(point.x_mm), 3),
            "y": round(float(point.y_mm), 3),
            "z": round(float(point.z_mm), 3),
            "t": round(t_rad, 4),
            "spd": round(float(waypoint.speed), 4),
        })
        # Waveshare documents T=104 (CMD_XYZT_GOAL_CTRL) as a command that
        # may block the firmware while interpolated motion is running.  A T=105
        # sent immediately after T=104 can therefore receive no T=1051 reply
        # until the move finishes.  Do not treat one UART response timeout as a
        # waypoint failure; retry feedback until the overall waypoint deadline.
        deadline = time.monotonic() + self.waypoint_timeout_s
        if self.feedback_initial_delay_s > 0.0:
            self._interruptible_wait(
                min(self.feedback_initial_delay_s, self.waypoint_timeout_s),
                abort_event,
            )

        last_error = float("inf")
        last_timeout: Exception | None = None
        received_feedback = False
        from arm_control.roarm_uart import RoArmTimeoutError

        while time.monotonic() < deadline:
            self._raise_if_aborted(abort_event)
            try:
                t105_attempts += 1
                state = self.uart.get_state()
            except RoArmTimeoutError as exc:
                # T=104 may still be occupying the firmware.  Retry until the
                # waypoint-level timeout instead of aborting after the UART's
                # shorter per-request timeout.
                last_timeout = exc
                transient_t105_timeouts += 1
                if time.monotonic() >= deadline:
                    break
                self._interruptible_wait(self.feedback_poll_s, abort_event)
                continue

            received_feedback = True
            feedback_samples += 1
            self._validate_state(state)
            last_error = math.sqrt(
                (state.x_mm - point.x_mm) ** 2
                + (state.y_mm - point.y_mm) ** 2
                + (state.z_mm - point.z_mm) ** 2
            )
            self._last_state = state
            if last_error <= self.feedback_tolerance_mm:
                return {
                    "target": {
                        "x_mm": float(point.x_mm),
                        "y_mm": float(point.y_mm),
                        "z_mm": float(point.z_mm),
                        "eoat_angle_rad": float(t_rad),
                        "speed": float(waypoint.speed),
                    },
                    "t105_attempts": t105_attempts,
                    "transient_t105_timeouts": transient_t105_timeouts,
                    "feedback_samples": feedback_samples,
                    "final_error_mm": float(last_error),
                    "elapsed_s": float(time.monotonic() - waypoint_started),
                }
            self._interruptible_wait(self.feedback_poll_s, abort_event)

        if not received_feedback:
            detail = f"; last UART timeout: {last_timeout}" if last_timeout else ""
            raise RuntimeError(
                f"RoArm waypoint {waypoint.name} feedback timeout: no T=1051 "
                f"feedback within {self.waypoint_timeout_s:.1f}s{detail}"
            )
        raise RuntimeError(
            f"RoArm waypoint {waypoint.name} feedback timeout: "
            f"error={last_error:.1f} mm after {self.waypoint_timeout_s:.1f}s"
        )




    def _set_gripper_radians(self, angle_rad: float) -> None:
        """Send T=106 with configured speed, while tolerating older test doubles."""
        try:
            self.uart.set_gripper_radians(
                angle_rad,
                speed_steps_s=self.gripper_speed_steps_s,
                acceleration=self.gripper_acceleration,
            )
        except TypeError as exc:
            # Older local FakeUart implementations accepted only ``angle``.
            # Keep those hardware-free tests compatible without weakening the
            # real RoArmUart path, which supports speed/acceleration keywords.
            if "unexpected keyword argument" not in str(exc):
                raise
            self.uart.set_gripper_radians(angle_rad)

    def _estimated_gripper_motion_seconds(
        self, start_rad: float, target_rad: float
    ) -> float:
        """Estimate clamp travel time from the official 4096 steps/revolution unit."""
        delta_rad = abs(float(target_rad) - float(start_rad))
        servo_steps = delta_rad / (2.0 * math.pi) * 4096.0
        return servo_steps / float(self.gripper_speed_steps_s)

    def _gripper_wait_seconds(self, start_rad: float, target_rad: float) -> float:
        if not self.gripper_motion_timing_enabled:
            return self.gripper_settle_s
        estimated = self._estimated_gripper_motion_seconds(start_rad, target_rad)
        return max(
            self.gripper_settle_s,
            estimated + self.gripper_motion_margin_s,
        )

    @staticmethod
    def _validate_clamp_angle(angle_rad: float, name: str) -> float:
        """Validate the stock clamp angle used by both T=106 and T=104."""
        angle = float(angle_rad)
        if not math.isfinite(angle):
            raise ValueError(f"{name} must be finite")
        lower, upper = 1.08, 3.14
        tolerance = 1e-6
        if angle < lower - tolerance or angle > upper + tolerance:
            raise ValueError(
                f"{name}={angle:.6f} outside stock clamp range "
                f"[{lower}, {upper}] rad"
            )
        return min(max(angle, lower), upper)

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
            "hand_rad": float(getattr(state, "hand_rad", float("nan"))),
        }
