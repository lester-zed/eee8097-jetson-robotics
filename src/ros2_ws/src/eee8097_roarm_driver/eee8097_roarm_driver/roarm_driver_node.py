from __future__ import annotations

import importlib
import math
import os
from pathlib import Path
import sys
from threading import RLock
import time
from typing import Any

import rclpy
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from .joint_mapping import (
    ARM_JOINT_NAMES,
    PUBLISHED_JOINT_NAMES,
    hardware_to_urdf_positions,
    reorder_arm_positions,
    urdf_to_hardware_degrees,
)
from .trajectory_guard import (
    TrajectoryValidationError,
    command_speed_degrees_s,
    duration_seconds,
    validate_arm_trajectory,
)


def _shortest_angle_error(actual: float, target: float) -> float:
    return math.atan2(math.sin(actual - target), math.cos(actual - target))


class RoArmTrajectoryDriver(Node):
    """Real RoArm state bridge and guarded arm-only trajectory action.

    No gripper command API is created.  The action server accepts exactly the
    three MoveIt arm joints and sends only T=121 joint IDs 1, 2, and 3.
    """

    def __init__(self) -> None:
        super().__init__("eee8097_roarm_driver")
        self.declare_parameter("serial_port", "/dev/ttyROARM")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("response_timeout_s", 2.0)
        self.declare_parameter("project_src", "/workspace/src")
        self.declare_parameter("allow_motion", False)
        self.declare_parameter("gripper_collision_position_rad", 0.0)
        self.declare_parameter("reconnect_interval_s", 5.0)
        self.declare_parameter("disconnect_after_failures", 3)
        self.declare_parameter("max_speed_deg_s", 15)
        self.declare_parameter("acceleration_deg_s2", 20)
        self.declare_parameter("minimum_command_period_s", 0.12)
        self.declare_parameter("final_tolerance_deg", 4.0)
        self.declare_parameter("final_timeout_s", 6.0)

        self._serial_port = str(self.get_parameter("serial_port").value)
        self._baud_rate = int(self.get_parameter("baud_rate").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._response_timeout_s = float(
            self.get_parameter("response_timeout_s").value
        )
        self._allow_motion = bool(self.get_parameter("allow_motion").value)
        self._gripper_collision_position = float(
            self.get_parameter("gripper_collision_position_rad").value
        )
        self._reconnect_interval_s = float(
            self.get_parameter("reconnect_interval_s").value
        )
        self._disconnect_after_failures = max(
            1, int(self.get_parameter("disconnect_after_failures").value)
        )
        self._max_speed_deg_s = max(
            1, min(60, int(self.get_parameter("max_speed_deg_s").value))
        )
        self._acceleration_deg_s2 = max(
            1,
            min(254, int(self.get_parameter("acceleration_deg_s2").value)),
        )
        self._minimum_command_period_s = max(
            0.02, float(self.get_parameter("minimum_command_period_s").value)
        )
        self._final_tolerance_deg = max(
            0.1, float(self.get_parameter("final_tolerance_deg").value)
        )
        self._final_timeout_s = max(
            0.5, float(self.get_parameter("final_timeout_s").value)
        )

        if self._publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        # Validate the fixed collision posture before opening the UART.
        hardware_to_urdf_positions(
            base_rad=0.0,
            shoulder_rad=0.0,
            elbow_rad=0.0,
            gripper_collision_position_rad=self._gripper_collision_position,
        )

        project_src = Path(
            os.path.expanduser(str(self.get_parameter("project_src").value))
        ).resolve()
        if not (project_src / "arm_control" / "roarm_uart.py").is_file():
            raise FileNotFoundError(
                f"project_src does not contain arm_control/roarm_uart.py: {project_src}"
            )
        if str(project_src) not in sys.path:
            sys.path.insert(0, str(project_src))
        uart_module = importlib.import_module("arm_control.roarm_uart")
        self._uart_type = uart_module.RoArmUart

        self._uart_guard = RLock()
        self._state_guard = RLock()
        self._uart: Any | None = None
        self._last_positions: tuple[float, float, float, float] | None = None
        self._last_state_time = 0.0
        self._last_connect_attempt = 0.0
        self._last_error = "not connected"
        self._consecutive_failures = 0
        self._action_active = False

        self._joint_state_publisher = self.create_publisher(
            JointState, "/joint_states", 10
        )
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self._callback_group = ReentrantCallbackGroup()
        self._stop_service = self.create_service(
            Trigger,
            "/roarm/stop",
            self._handle_stop,
            callback_group=self._callback_group,
        )
        self._reconnect_service = self.create_service(
            Trigger,
            "/roarm/reconnect",
            self._handle_reconnect,
            callback_group=self._callback_group,
        )
        self._trajectory_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/hand_controller/follow_joint_trajectory",
            execute_callback=self._execute_trajectory,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._state_timer = self.create_timer(
            1.0 / self._publish_rate_hz,
            self._poll_and_publish,
            callback_group=self._callback_group,
        )
        self._diagnostic_timer = self.create_timer(
            1.0,
            self._publish_diagnostic,
            callback_group=self._callback_group,
        )

        self._connect_uart()
        if self._allow_motion:
            self.get_logger().warning(
                "REAL ARM MOTION ENABLED. Only J1-J3 are commandable; all gripper "
                "goals remain hard-rejected. Keep the power switch reachable."
            )
        else:
            self.get_logger().info(
                "Plan/read-only mode: trajectory goals are locked. Start with "
                "allow_motion:=true only after the plan-only checks pass."
            )

    def _connect_uart(self) -> bool:
        self._last_connect_attempt = time.monotonic()
        with self._uart_guard:
            self._close_uart_locked()
            try:
                self._uart = self._uart_type(
                    port=self._serial_port,
                    baudrate=self._baud_rate,
                    response_timeout_seconds=self._response_timeout_s,
                )
            except Exception as exc:
                self._uart = None
                self._last_error = f"UART connect failed: {exc}"
                self.get_logger().error(self._last_error)
                return False
        self._consecutive_failures = 0
        self._last_error = ""
        self.get_logger().info(
            f"RoArm UART connected: {self._serial_port} @ {self._baud_rate}"
        )
        return True

    def _close_uart_locked(self) -> None:
        if self._uart is None:
            return
        try:
            self._uart.close()
        except Exception as exc:
            self.get_logger().warning(f"UART close failed: {exc}")
        finally:
            self._uart = None

    def _read_state(self) -> Any:
        with self._uart_guard:
            if self._uart is None:
                raise RuntimeError("RoArm UART is disconnected")
            return self._uart.get_state()

    def _positions_from_state(self, state: Any) -> tuple[float, float, float, float]:
        return hardware_to_urdf_positions(
            base_rad=state.base_rad,
            shoulder_rad=state.shoulder_rad,
            elbow_rad=state.elbow_rad,
            gripper_collision_position_rad=self._gripper_collision_position,
        )

    def _record_and_publish_state(self, state: Any) -> tuple[float, float, float, float]:
        positions = self._positions_from_state(state)
        with self._state_guard:
            self._last_positions = positions
            self._last_state_time = time.monotonic()
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.name = list(PUBLISHED_JOINT_NAMES)
        message.position = list(positions)
        self._joint_state_publisher.publish(message)
        return positions

    def _poll_and_publish(self) -> None:
        if self._uart is None:
            if time.monotonic() - self._last_connect_attempt >= self._reconnect_interval_s:
                self._connect_uart()
            return
        try:
            state = self._read_state()
            self._record_and_publish_state(state)
            self._consecutive_failures = 0
            self._last_error = ""
        except Exception as exc:
            self._consecutive_failures += 1
            self._last_error = f"state read failed: {exc}"
            if self._consecutive_failures == 1:
                self.get_logger().warning(self._last_error)
            if self._consecutive_failures >= self._disconnect_after_failures:
                with self._uart_guard:
                    self._close_uart_locked()
                self.get_logger().error(
                    "UART marked disconnected after repeated read failures; "
                    "automatic reconnect remains active"
                )

    def _publish_diagnostic(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "EEE8097 RoArm arm-only driver"
        status.hardware_id = self._serial_port
        if self._uart is None:
            status.level = DiagnosticStatus.ERROR
            status.message = self._last_error or "UART disconnected"
        elif self._last_error:
            status.level = DiagnosticStatus.WARN
            status.message = self._last_error
        else:
            status.level = DiagnosticStatus.OK
            status.message = "state bridge healthy"
        status.values = [
            KeyValue(key="allow_motion", value=str(self._allow_motion).lower()),
            KeyValue(key="gripper_commands", value="hard-disabled"),
            KeyValue(key="trajectory_active", value=str(self._action_active).lower()),
            KeyValue(key="consecutive_failures", value=str(self._consecutive_failures)),
        ]
        array.status = [status]
        self._diagnostic_publisher.publish(array)

    def _goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> int:
        if not self._allow_motion:
            self.get_logger().warning("Rejected trajectory: allow_motion is false")
            return GoalResponse.REJECT
        if self._uart is None:
            self.get_logger().warning("Rejected trajectory: UART is disconnected")
            return GoalResponse.REJECT
        if self._action_active:
            self.get_logger().warning("Rejected trajectory: another goal is active")
            return GoalResponse.REJECT
        try:
            validate_arm_trajectory(
                goal_request.trajectory.joint_names,
                goal_request.trajectory.points,
            )
        except TrajectoryValidationError as exc:
            self.get_logger().error(f"Rejected unsafe trajectory: {exc}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: Any) -> int:
        del goal_handle
        return CancelResponse.ACCEPT

    def _send_arm_point(self, ordered_rad: tuple[float, float, float], speed: int) -> None:
        hardware_deg = urdf_to_hardware_degrees(ordered_rad)
        with self._uart_guard:
            if self._uart is None:
                raise RuntimeError("RoArm UART disconnected during trajectory")
            # Never send the fourth joint and never use T=122 because both can command the
            # unavailable gripper servo on a stock RoArm-M2-S.
            for joint, angle in enumerate(hardware_deg, start=1):
                self._uart.move_single_joint_degrees(
                    joint=joint,
                    angle_degrees=angle,
                    speed_degrees_s=speed,
                    acceleration_degrees_s2=self._acceleration_deg_s2,
                )

    def _publish_action_feedback(
        self,
        goal_handle: Any,
        desired: tuple[float, float, float],
        actual: tuple[float, float, float],
    ) -> None:
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(ARM_JOINT_NAMES)
        feedback.desired = JointTrajectoryPoint(positions=list(desired))
        feedback.actual = JointTrajectoryPoint(positions=list(actual))
        feedback.error = JointTrajectoryPoint(
            positions=[
                _shortest_angle_error(actual[0], desired[0]),
                actual[1] - desired[1],
                actual[2] - desired[2],
            ]
        )
        goal_handle.publish_feedback(feedback)

    def _cooperative_stop(self) -> None:
        with self._uart_guard:
            if self._uart is None:
                raise RuntimeError("RoArm UART is disconnected")
            self._uart.stop_continuous_motion()

    def _execute_trajectory(self, goal_handle: Any) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        request = goal_handle.request
        names = request.trajectory.joint_names
        points = request.trajectory.points
        try:
            validate_arm_trajectory(names, points)
        except TrajectoryValidationError as exc:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            goal_handle.abort()
            return result

        self._action_active = True
        try:
            try:
                current_state = self._read_state()
                previous = self._record_and_publish_state(current_state)[:3]
            except Exception:
                with self._state_guard:
                    if self._last_positions is None:
                        raise RuntimeError("no valid real joint state is available")
                    previous = self._last_positions[:3]

            start = time.monotonic()
            previous_point_time = 0.0
            last_command_time = -math.inf
            final_index = len(points) - 1
            final_target = reorder_arm_positions(names, points[-1].positions)

            for index, point in enumerate(points):
                point_time = duration_seconds(point.time_from_start)
                is_final = index == final_index
                if not is_final and point_time - last_command_time < self._minimum_command_period_s:
                    continue
                while True:
                    if goal_handle.is_cancel_requested:
                        self._cooperative_stop()
                        goal_handle.canceled()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "trajectory canceled; cooperative stop sent"
                        return result
                    remaining = start + point_time - time.monotonic()
                    if remaining <= 0.0:
                        break
                    time.sleep(min(0.02, remaining))

                target = reorder_arm_positions(names, point.positions)
                speed = command_speed_degrees_s(
                    previous,
                    target,
                    max(0.001, point_time - previous_point_time),
                    maximum=self._max_speed_deg_s,
                )
                self._send_arm_point(target, speed)
                previous = target
                previous_point_time = point_time
                last_command_time = point_time
                with self._state_guard:
                    actual = (
                        self._last_positions[:3]
                        if self._last_positions is not None
                        else previous
                    )
                self._publish_action_feedback(goal_handle, target, actual)

            deadline = time.monotonic() + self._final_timeout_s
            last_error_deg: tuple[float, float, float] | None = None
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    self._cooperative_stop()
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "trajectory canceled during final verification"
                    return result
                state = self._read_state()
                actual = self._record_and_publish_state(state)[:3]
                errors_rad = (
                    _shortest_angle_error(actual[0], final_target[0]),
                    actual[1] - final_target[1],
                    actual[2] - final_target[2],
                )
                last_error_deg = tuple(math.degrees(value) for value in errors_rad)
                self._publish_action_feedback(goal_handle, final_target, actual)
                if max(abs(value) for value in last_error_deg) <= self._final_tolerance_deg:
                    goal_handle.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = (
                        "arm-only trajectory reached; gripper was not commanded; "
                        f"final_error_deg={last_error_deg}"
                    )
                    return result
                time.sleep(0.10)

            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = f"final feedback timeout; error_deg={last_error_deg}"
            return result
        except Exception as exc:
            try:
                self._cooperative_stop()
            except Exception:
                pass
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f"trajectory execution failed: {exc}"
            self.get_logger().error(result.error_string)
            return result
        finally:
            self._action_active = False

    def _handle_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        try:
            self._cooperative_stop()
        except Exception as exc:
            response.success = False
            response.message = f"cooperative stop failed: {exc}"
            return response
        response.success = True
        response.message = "T=123 cooperative stop sent; hardware power remains the E-stop"
        return response

    def _handle_reconnect(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        if self._action_active:
            response.success = False
            response.message = "cannot reconnect while a trajectory is active"
            return response
        response.success = self._connect_uart()
        response.message = (
            "UART reconnected" if response.success else self._last_error
        )
        return response

    def destroy_node(self) -> bool:
        self._trajectory_server.destroy()
        with self._uart_guard:
            self._close_uart_locked()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RoArmTrajectoryDriver()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
