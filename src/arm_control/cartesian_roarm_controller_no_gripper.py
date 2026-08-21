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

    This keeps Camera -> RPLIDAR -> localization -> planning -> Cartesian arm
    motion testable without pretending that a physical grasp took place.
    """

    disabled_reason = "temporary gripper servo fault"

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._preserved_hand_rad: float | None = None

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

        result = super().execute_grasp(plan, abort_event)
        feedback = dict(result.feedback)
        gripper_feedback = dict(feedback.get("gripper", {}))
        gripper_feedback.update(
            {
                "control_enabled": False,
                "disabled_reason": self.disabled_reason,
                "preserved_hand_rad": self._preserved_hand_rad,
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
            }
        )
        return payload
