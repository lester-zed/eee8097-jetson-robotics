from __future__ import annotations

import importlib
import os
from pathlib import Path
import sys
import time
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

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


def _load_roarm_uart(project_src: Path) -> tuple[type[Any], type[Exception]]:
    project_src_text = str(project_src)
    if project_src_text not in sys.path:
        sys.path.insert(0, project_src_text)
    module = importlib.import_module("arm_control.roarm_uart")
    return module.RoArmUart, module.RoArmTimeoutError


class RoArmStatePublisher(Node):
    """Read-only T=105 bridge from the real RoArm to ROS 2 JointState."""

    def __init__(self) -> None:
        super().__init__("eee8097_roarm_state_publisher")
        self.declare_parameter("serial_port", "/dev/ttyROARM")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("response_timeout_s", 2.0)
        self.declare_parameter("project_src", "")
        self.declare_parameter("frame_id", "")

        serial_port = str(self.get_parameter("serial_port").value)
        baud_rate = int(self.get_parameter("baud_rate").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        response_timeout_s = float(self.get_parameter("response_timeout_s").value)
        project_src_value = str(self.get_parameter("project_src").value)
        self._frame_id = str(self.get_parameter("frame_id").value)

        if baud_rate <= 0 or publish_rate_hz <= 0.0 or response_timeout_s <= 0.0:
            raise ValueError("baud_rate, publish_rate_hz and response_timeout_s must be positive")

        project_src = _resolve_project_src(project_src_value)
        roarm_uart_type, timeout_error_type = _load_roarm_uart(project_src)
        self._timeout_error_type = timeout_error_type
        self.get_logger().info(f"Using EEE8097 project source: {project_src}")
        self.get_logger().info(f"Opening read-only RoArm bridge: {serial_port} @ {baud_rate}")

        self._uart = roarm_uart_type(
            port=serial_port,
            baudrate=baud_rate,
            response_timeout_seconds=response_timeout_s,
        )
        self._publisher = self.create_publisher(JointState, "/joint_states", 10)
        self._last_warning_time = 0.0
        self._successful_samples = 0
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._poll_and_publish)
        self.get_logger().info("Read-only bridge ready; only T=105 feedback requests are sent.")

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
