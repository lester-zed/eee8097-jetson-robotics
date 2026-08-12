from __future__ import annotations

import importlib
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from eee8097_interfaces.srv import MoveJoint, MoveJoints, SetGripper

from .joint_mapping import JOINT_NAMES, hardware_to_urdf_positions


def _looks_like_project_src(path: Path) -> bool:
    return (
        (path / "arm_control" / "roarm_uart.py").is_file()
        and (path / "utils" / "logger.py").is_file()
    )


def _resolve_project_src(explicit: str) -> Path:
    candidates: list[Path] = []
    if explicit.strip():
        candidates.append(Path(explicit).expanduser())
    env_path = os.environ.get("EEE8097_PROJECT_SRC", "").strip()
    if env_path:
        candidates.append(Path(env_path).expanduser())

    source_file = Path(__file__).resolve()
    for parent in source_file.parents:
        candidates.extend((parent, parent / "src"))

    cwd = Path.cwd().resolve()
    for parent in (cwd, *cwd.parents):
        candidates.extend((parent, parent / "src"))

    candidates.extend(
        (
            Path.home() / "project" / "eee8097-jetson-robotics" / "src",
            Path("/workspace/src"),
        )
    )

    visited: set[Path] = set()
    for candidate in candidates:
        try:
            resolved = candidate.resolve()
        except OSError:
            continue
        if resolved in visited:
            continue
        visited.add(resolved)
        if _looks_like_project_src(resolved):
            return resolved

    searched = "\n  - ".join(str(path) for path in visited)
    raise RuntimeError(
        "Could not locate the EEE8097 project src directory containing "
        "arm_control/roarm_uart.py. Set project_src or EEE8097_PROJECT_SRC. "
        "Searched:\n  - " + searched
    )


def _load_project_types(project_src: Path) -> dict[str, Any]:
    project_src_text = str(project_src)
    if project_src_text not in sys.path:
        sys.path.insert(0, project_src_text)

    uart_module = importlib.import_module("arm_control.roarm_uart")
    home_module = importlib.import_module("roarm_home")
    return {
        "RoArmUart": uart_module.RoArmUart,
        "RoArmTimeoutError": uart_module.RoArmTimeoutError,
        "load_home_document": home_module.load_home_document,
        "resolve_home_target": home_module.resolve_home_target,
    }


def _state_degrees(state: Any) -> dict[str, float]:
    return {
        "base": math.degrees(float(state.base_rad)),
        "shoulder": math.degrees(float(state.shoulder_rad)),
        "elbow": math.degrees(float(state.elbow_rad)),
        "hand": math.degrees(float(state.hand_rad)),
    }


def _base_error_deg(actual: float, target: float) -> float:
    return (float(actual) - float(target) + 180.0) % 360.0 - 180.0


class RoArmStatePublisher(Node):
    """Single-owner ROS 2 bridge for RoArm state and guarded motion commands."""

    def __init__(self) -> None:
        super().__init__("eee8097_roarm_state_publisher")
        self.declare_parameter("serial_port", "/dev/ttyROARM")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("response_timeout_s", 2.0)
        self.declare_parameter("project_src", "")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("allow_motion", False)
        self.declare_parameter("home_config", "")

        serial_port = str(self.get_parameter("serial_port").value)
        baud_rate = int(self.get_parameter("baud_rate").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        response_timeout_s = float(self.get_parameter("response_timeout_s").value)
        project_src_value = str(self.get_parameter("project_src").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._allow_motion = bool(self.get_parameter("allow_motion").value)

        if baud_rate <= 0 or publish_rate_hz <= 0.0 or response_timeout_s <= 0.0:
            raise ValueError("baud_rate, publish_rate_hz and response_timeout_s must be positive")

        self._project_src = _resolve_project_src(project_src_value)
        project_types = _load_project_types(self._project_src)
        self._timeout_error_type = project_types["RoArmTimeoutError"]
        self._load_home_document = project_types["load_home_document"]
        self._resolve_home_target = project_types["resolve_home_target"]

        home_config_value = str(self.get_parameter("home_config").value).strip()
        self._home_config = (
            Path(home_config_value).expanduser().resolve()
            if home_config_value
            else self._project_src / "configs" / "roarm_home.yaml"
        )

        self.get_logger().info(f"Using EEE8097 project source: {self._project_src}")
        self.get_logger().info(f"Opening RoArm serial owner: {serial_port} @ {baud_rate}")

        self._uart = project_types["RoArmUart"](
            port=serial_port,
            baudrate=baud_rate,
            response_timeout_seconds=response_timeout_s,
        )

        self._publisher = self.create_publisher(JointState, "/joint_states", 10)
        self._last_warning_time = 0.0
        self._successful_samples = 0
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._poll_and_publish)

        self._move_joint_service = self.create_service(
            MoveJoint,
            "/roarm/move_joint",
            self._handle_move_joint,
        )
        self._move_joints_service = self.create_service(
            MoveJoints,
            "/roarm/move_joints",
            self._handle_move_joints,
        )
        self._gripper_service = self.create_service(
            SetGripper,
            "/roarm/set_gripper",
            self._handle_set_gripper,
        )
        self._home_service = self.create_service(
            Trigger,
            "/roarm/home",
            self._handle_home,
        )
        self._stop_service = self.create_service(
            Trigger,
            "/roarm/stop",
            self._handle_stop,
        )

        if self._allow_motion:
            self.get_logger().warning(
                "REAL MOTION ENABLED: ROS 2 command services may move the RoArm. "
                "Keep the swept volume clear and the hardware power switch reachable."
            )
        else:
            self.get_logger().info(
                "Motion services are locked. Start with allow_motion:=true only after "
                "clearing the complete arm swept volume."
            )

    def _poll_and_publish(self) -> None:
        try:
            state = self._uart.get_state()
        except self._timeout_error_type as exc:
            self._warn_throttled(f"RoArm T=105 timeout: {exc}")
            return
        except Exception as exc:
            self._warn_throttled(f"RoArm state read failed: {exc}")
            return

        positions = hardware_to_urdf_positions(
            base_rad=state.base_rad,
            shoulder_rad=state.shoulder_rad,
            elbow_rad=state.elbow_rad,
            hand_rad=state.hand_rad,
        )
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.name = list(JOINT_NAMES)
        message.position = list(positions)
        self._publisher.publish(message)

        self._successful_samples += 1
        if self._successful_samples == 1:
            text = ", ".join(
                f"{name}={value:.4f} rad" for name, value in zip(JOINT_NAMES, positions)
            )
            self.get_logger().info(f"First real JointState published: {text}")

    def _motion_allowed(self, response: Any) -> bool:
        if self._allow_motion:
            return True
        response.success = False
        response.message = (
            "Real motion is disabled. Restart the node/launch with "
            "allow_motion:=true after clearing the arm swept volume."
        )
        return False

    def _handle_move_joint(self, request: MoveJoint.Request, response: MoveJoint.Response) -> MoveJoint.Response:
        if not self._motion_allowed(response):
            return response
        try:
            self._uart.move_single_joint_degrees(
                joint=int(request.joint),
                angle_degrees=float(request.angle_deg),
                speed_degrees_s=int(request.speed_deg_s),
                acceleration_degrees_s2=int(request.acceleration_deg_s2),
            )
        except Exception as exc:
            response.success = False
            response.message = f"MoveJoint rejected: {exc}"
            return response
        response.success = True
        response.message = (
            f"T=121 accepted: joint={request.joint}, angle={request.angle_deg:.2f} deg"
        )
        return response

    def _handle_move_joints(self, request: MoveJoints.Request, response: MoveJoints.Response) -> MoveJoints.Response:
        if not self._motion_allowed(response):
            return response
        try:
            self._uart.move_all_joints_degrees(
                base=float(request.base_deg),
                shoulder=float(request.shoulder_deg),
                elbow=float(request.elbow_deg),
                hand=float(request.hand_deg),
                speed_degrees_s=int(request.speed_deg_s),
                acceleration_degrees_s2=int(request.acceleration_deg_s2),
            )
        except Exception as exc:
            response.success = False
            response.message = f"MoveJoints rejected: {exc}"
            return response
        response.success = True
        response.message = "T=122 all-joint command accepted"
        return response

    def _handle_set_gripper(self, request: SetGripper.Request, response: SetGripper.Response) -> SetGripper.Response:
        if not self._motion_allowed(response):
            return response
        try:
            self._uart.set_gripper_radians(
                float(request.angle_rad),
                speed_steps_s=int(request.speed_steps_s),
                acceleration=int(request.acceleration),
            )
        except Exception as exc:
            response.success = False
            response.message = f"SetGripper rejected: {exc}"
            return response
        response.success = True
        response.message = f"T=106 accepted: gripper={request.angle_rad:.3f} rad"
        return response

    def _handle_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if not self._motion_allowed(response):
            return response
        try:
            self._uart.stop_continuous_motion()
        except Exception as exc:
            response.success = False
            response.message = f"Stop command failed: {exc}"
            return response
        response.success = True
        response.message = "T=123 cooperative stop sent"
        return response

    def _handle_home(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if not self._motion_allowed(response):
            return response

        try:
            document = self._load_home_document(self._home_config)
            section = document["roarm_home"]
            if not bool(section.get("enabled", False)):
                raise PermissionError("roarm_home.enabled must be true")
            if not bool(section.get("allow_real_motion", False)):
                raise PermissionError("roarm_home.allow_real_motion must be true")
            if not bool(section.get("confirm_clearance", False)):
                raise PermissionError("roarm_home.confirm_clearance must be true")

            motion = section.get("motion", {})
            speed = int(motion.get("speed_deg_s", 15))
            acceleration = int(motion.get("acceleration_deg_s2", 20))
            timeout_s = float(motion.get("timeout_s", 20.0))
            poll_s = float(motion.get("poll_s", 0.25))
            tolerance_deg = float(motion.get("tolerance_deg", 5.0))
            settle_s = max(0.0, float(motion.get("settle_s", 0.8)))

            before = self._uart.get_state()
            target = self._resolve_home_target(section, before)

            if bool(section.get("stop_continuous_first", True)):
                self._uart.stop_continuous_motion()
                time.sleep(0.25)
            if bool(section.get("enable_torque_first", True)):
                self._uart.send({"T": 210, "cmd": 1})
                time.sleep(0.25)

            self._uart.move_all_joints_degrees(
                base=target["base"],
                shoulder=target["shoulder"],
                elbow=target["elbow"],
                hand=target["hand"],
                speed_degrees_s=speed,
                acceleration_degrees_s2=acceleration,
            )

            deadline = time.monotonic() + timeout_s
            last_error: dict[str, float] | None = None
            while time.monotonic() < deadline:
                time.sleep(poll_s)
                try:
                    state = self._uart.get_state()
                except Exception:
                    continue
                actual = _state_degrees(state)
                error = {
                    "base": _base_error_deg(actual["base"], target["base"]),
                    "shoulder": actual["shoulder"] - target["shoulder"],
                    "elbow": actual["elbow"] - target["elbow"],
                    "hand": actual["hand"] - target["hand"],
                }
                last_error = error
                if max(abs(value) for value in error.values()) <= tolerance_deg:
                    if settle_s:
                        time.sleep(settle_s)
                    response.success = True
                    response.message = f"Custom Home reached; error_deg={error}"
                    return response

            raise RuntimeError(f"Custom Home feedback timeout; last_error={last_error}")
        except Exception as exc:
            response.success = False
            response.message = f"Home failed: {exc}"
            return response

    def _warn_throttled(self, text: str, period_s: float = 5.0) -> None:
        now = time.monotonic()
        if now - self._last_warning_time >= period_s:
            self.get_logger().warning(text)
            self._last_warning_time = now

    def destroy_node(self) -> bool:
        uart = getattr(self, "_uart", None)
        if uart is not None:
            try:
                uart.close()
            except Exception as exc:
                self.get_logger().warning(f"Failed to close RoArm UART cleanly: {exc}")
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: RoArmStatePublisher | None = None
    try:
        node = RoArmStatePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
