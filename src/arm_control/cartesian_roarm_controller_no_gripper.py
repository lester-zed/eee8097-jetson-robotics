from __future__ import annotations

import math
from threading import Event
from typing import Any

from arm_control.cartesian_roarm_controller import CartesianRoArmController
from pipeline.types import CartesianWaypoint, ExecutionResult, GraspPlan


class GripperDisabledCartesianRoArmController(CartesianRoArmController):
    """Temporary demo controller that preserves the current EoAT angle.

    The gripper servo is currently unavailable, so this wrapper deliberately
    suppresses all explicit T=106 clamp commands. Cartesian T=104 moves still
    require a ``t`` field on RoArm-M2-S; for those moves we continuously reuse
    the hand angle read from T=105 at the start of the task instead of switching
    between the planned open/closed angles.

    A short dwell is retained at the grasp waypoint so the demonstration keeps
    the timing and visual structure of the original physical grasp sequence.

    This keeps Camera -> RPLIDAR -> localization -> planning -> Cartesian arm
    motion testable without pretending that a physical grasp took place.
    """

    disabled_reason = "temporary gripper servo fault"
    grasp_pause_s = 3.0

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._preserved_hand_rad: float | None = None
        self._disabled_closed_rad: float | None = None

    def execute_grasp(self, plan: GraspPlan, abort_event: Event) -> ExecutionResult:
        # Capture one real T=105 hand angle before the parent sequence starts.
        # Clamp tiny feedback overshoots to the stock T=104/T=106 range so the
        # remaining Cartesian waypoints stay valid without commanding a new
        # gripper pose.
        state = self.uart.get_state()
        self._validate_state(state)
        hand_rad = float(state.hand_rad)
        if not math.isfinite(hand_rad):
            raise RuntimeError("RoArm returned a non-finite EoAT angle")
        self._preserved_hand_rad = min(max(hand_rad, 1.08), 3.14)
        self._disabled_closed_rad = self._validate_clamp_angle(
            plan.gripper_closed_rad,
            "gripper_closed_rad",
        )

        result = super().execute_grasp(plan, abort_event)
        feedback = dict(result.feedback)
        gripper_feedback = dict(feedback.get("gripper", {}))
        gripper_feedback.update(
            {
                "control_enabled": False,
                "disabled_reason": self.disabled_reason,
                "preserved_hand_rad": self._preserved_hand_rad,
                "simulated_grasp_pause_s": self.grasp_pause_s,
            }
        )
        feedback["gripper"] = gripper_feedback
        return ExecutionResult(
            success=result.success,
            message=(
                "real Cartesian waypoint sequence completed with gripper "
                "control disabled"
            ),
            completed_waypoints=result.completed_waypoints,
            feedback=feedback,
        )

    def _set_gripper_radians(self, angle_rad: float) -> None:
        """Suppress explicit T=106 commands while the gripper servo is faulty."""
        return None

    def _estimated_gripper_motion_seconds(
        self, start_rad: float, target_rad: float
    ) -> float:
        return 0.0

    def _gripper_wait_seconds(self, start_rad: float, target_rad: float) -> float:
        """Skip the opening delay but pause where the original close/grasp occurred."""
        if self._disabled_closed_rad is not None and math.isclose(
            float(target_rad),
            self._disabled_closed_rad,
            rel_tol=0.0,
            abs_tol=1e-6,
        ):
            return self.grasp_pause_s
        return 0.0

    def _move_waypoint(
        self,
        waypoint: CartesianWaypoint,
        abort_event: Event,
        *,
        eoat_angle_rad: float | None = None,
    ) -> dict[str, Any]:
        if self._preserved_hand_rad is None:
            raise RuntimeError("No preserved EoAT angle available for Cartesian motion")
        return super()._move_waypoint(
            waypoint,
            abort_event,
            eoat_angle_rad=self._preserved_hand_rad,
        )

    def status(self) -> dict[str, Any]:
        payload = super().status()
        payload.update(
            {
                "gripper_control_enabled": False,
                "gripper_disabled_reason": self.disabled_reason,
                "preserved_hand_rad": self._preserved_hand_rad,
                "simulated_grasp_pause_s": self.grasp_pause_s,
            }
        )
        return payload
