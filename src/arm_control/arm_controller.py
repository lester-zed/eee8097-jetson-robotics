from __future__ import annotations

import time
from threading import Event
from typing import Any, Callable

from utils.logger import get_logger


class _MockArmBackend:
    """Dry-run backend. It never opens a serial port or moves hardware."""

    def __init__(self, step_delay_seconds: float = 0.35) -> None:
        self.logger = get_logger(self.__class__.__name__)
        self.mode = "mock"
        self.step_delay_seconds = float(step_delay_seconds)

    def initialize(self) -> None:
        self.logger.info("[MOCK ARM] initialize arm")

    def move_to_home(self) -> None:
        self.logger.info("[MOCK ARM] move to home")

    def open_gripper(self) -> None:
        self.logger.info("[MOCK ARM] open gripper")

    def close_gripper(self) -> None:
        self.logger.info("[MOCK ARM] close gripper")

    def move_to_pregrasp(self, region: str) -> None:
        self.logger.info("[MOCK ARM] move to %s pre-grasp pose", region)

    def move_to_grasp(self, region: str) -> None:
        self.logger.info("[MOCK ARM] move to %s grasp pose", region)

    def move_to_target_pose(self, preset_name: str = "center_grasp") -> None:
        self.logger.info("[MOCK ARM] move to preset: %s", preset_name)

    def lift_object(self) -> None:
        self.logger.info("[MOCK ARM] lift object")

    def place_object(self) -> None:
        self.logger.info("[MOCK ARM] place object")

    def execute_region(self, region: str, abort_event: Event) -> bool:
        actions = (
            self.initialize,
            self.move_to_home,
            self.open_gripper,
            lambda: self.move_to_pregrasp(region),
            lambda: self.move_to_grasp(region),
            self.close_gripper,
            self.lift_object,
            self.move_to_home,
        )
        for action in actions:
            self._raise_if_aborted(abort_event)
            action()
            self._interruptible_wait(abort_event)
        self.logger.info("[MOCK ARM] sequence completed")
        return True

    def execute_grasp(self, plan: Any, abort_event: Event) -> bool:
        region = getattr(plan, "region", plan)
        return self.execute_region(str(region), abort_event)

    def execute_mock_grasp(self, region: str, abort_event: Event) -> bool:
        return self.execute_region(region, abort_event)

    def run_demo_sequence(self) -> None:
        self.execute_region("center", Event())

    def status(self) -> dict[str, Any]:
        return {"mode": self.mode, "connected": False, "moving": False}

    def stop(self) -> None:
        self.logger.warning("[MOCK ARM] stop requested")

    def close(self) -> None:
        self.logger.info("[MOCK ARM] controller closed")

    def _interruptible_wait(self, abort_event: Event) -> None:
        deadline = time.monotonic() + self.step_delay_seconds
        while time.monotonic() < deadline:
            self._raise_if_aborted(abort_event)
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

    @staticmethod
    def _raise_if_aborted(abort_event: Event) -> None:
        if abort_event.is_set():
            raise InterruptedError("Task aborted")


class ArmController:
    """Stable facade selecting a mock or real RoArm backend.

    It supports both interfaces used by the user's recent TaskManager versions:
    ``execute_mock_grasp(region, event)`` and ``execute_grasp(plan, event)``.
    """

    def __init__(
        self,
        *,
        mode: str = "mock",
        step_delay_seconds: float = 0.35,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        mount_yaw_degrees: float = 180.0,
        region_yaw_degrees: float = 20.0,
        prefer_positive_180: bool = True,
        speed_degrees_s: int = 10,
        acceleration_degrees_s2: int = 10,
        perform_gripper_cycle: bool = False,
        confirm_motion: bool = False,
        confirm_clearance: bool = False,
        serial_factory: Callable[..., Any] | None = None,
    ) -> None:
        normalized_mode = str(mode).strip().lower()
        if normalized_mode == "mock":
            self._backend: Any = _MockArmBackend(step_delay_seconds)
        elif normalized_mode == "real":
            # Lazy import keeps mock-only tests independent of real hardware.
            from arm_control.real_roarm_controller import RealRoArmController

            self._backend = RealRoArmController(
                port=port,
                baudrate=baudrate,
                mount_yaw_degrees=mount_yaw_degrees,
                region_yaw_degrees=region_yaw_degrees,
                prefer_positive_180=prefer_positive_180,
                speed_degrees_s=speed_degrees_s,
                acceleration_degrees_s2=acceleration_degrees_s2,
                perform_gripper_cycle=perform_gripper_cycle,
                confirm_motion=confirm_motion,
                confirm_clearance=confirm_clearance,
                serial_factory=serial_factory,
            )
        else:
            raise ValueError("arm mode must be 'mock' or 'real'")

    @property
    def mode(self) -> str:
        return str(self._backend.mode)

    def execute_grasp(self, plan: Any, abort_event: Event) -> bool:
        return bool(self._backend.execute_grasp(plan, abort_event))

    def execute_mock_grasp(self, region: str, abort_event: Event) -> bool:
        return bool(self._backend.execute_mock_grasp(region, abort_event))

    def execute_region(self, region: str, abort_event: Event) -> bool:
        return bool(self._backend.execute_region(region, abort_event))

    def status(self) -> dict[str, Any]:
        return dict(self._backend.status())

    def stop(self) -> None:
        self._backend.stop()

    def close(self) -> None:
        self._backend.close()

    # Compatibility helpers retained for the repository's original demo code.
    def initialize(self) -> None:
        method = getattr(self._backend, "initialize", None)
        if not callable(method):
            raise RuntimeError("initialize() is available only in mock mode")
        method()

    def move_to_home(self) -> None:
        method = getattr(self._backend, "move_to_home", None)
        if not callable(method):
            raise RuntimeError(
                "Real reverse-mounted home is never invoked implicitly; "
                "use the dedicated safe-test command"
            )
        method()

    def open_gripper(self) -> None:
        method = getattr(self._backend, "open_gripper", None)
        if not callable(method):
            raise RuntimeError("Use the calibrated real-arm interface")
        method()

    def close_gripper(self) -> None:
        method = getattr(self._backend, "close_gripper", None)
        if not callable(method):
            raise RuntimeError("Use the calibrated real-arm interface")
        method()

    def move_to_target_pose(self, preset_name: str = "center_grasp") -> None:
        method = getattr(self._backend, "move_to_target_pose", None)
        if not callable(method):
            raise RuntimeError("Use execute_region() in real-arm aim mode")
        method(preset_name)

    def run_demo_sequence(self) -> None:
        method = getattr(self._backend, "run_demo_sequence", None)
        if not callable(method):
            raise RuntimeError("The original demo sequence is mock-only")
        method()
