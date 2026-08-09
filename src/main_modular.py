from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from adapters.mock_devices import MockArmAdapter, MockRangeSensorAdapter, MockVisionAdapter
from configuration.loader import RuntimeConfig, load_runtime_config
from localization.target_localizer import PlanarTargetLocalizer
from localization.transforms import ExistingArmMountTransformAdapter
from pipeline.task_manager import ModularTaskManager
from planning.grasp_planner import SimpleTopDownGraspPlanner, WorkspaceLimits


SOURCE_ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = SOURCE_ROOT / "configs/modular_pipeline.yaml"

HELP = """
Commands:
  s  Start one complete detect -> range -> recheck -> plan -> execute task
  x  Abort the current task
  p  Print complete pipeline status
  d  Print the latest detection
  l  Print the latest LiDAR measurement
  t  Print the latest target coordinates
  g  Print the latest grasp plan
  a  Print arm status
  r  Reset ERROR/ABORTED/COMPLETE to IDLE
  h  Show this help
  q  Abort and quit
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="EEE8097 configurable Camera-RPLIDAR-RoArm pipeline"
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--validate-config", action="store_true")
    return parser.parse_args()


def _typed_real_motion_confirmation(config: RuntimeConfig) -> None:
    arm_cfg = config.section("arm")
    if str(arm_cfg["mode"]).lower() != "real":
        return
    if not bool(arm_cfg.get("require_typed_confirmation", True)):
        return
    phrase = str(arm_cfg.get("confirmation_phrase", "EXECUTE ONE GRASP"))
    print("\nREAL ROARM MODE REQUESTED")
    print("Keep one hand on the hardware power switch.")
    entered = input(f"Type exactly {phrase!r} to open the RoArm serial port: ")
    if entered != phrase:
        raise PermissionError("Real-motion confirmation phrase did not match")


def build_manager(config: RuntimeConfig) -> ModularTaskManager:
    app = config.section("app")
    camera_cfg = config.section("camera")
    lidar_cfg = config.section("lidar")
    localization_cfg = config.section("localization")
    arm_mount_cfg = config.section("arm_mount")
    planner_cfg = config.section("planner")
    workspace_cfg = config.section("workspace")
    arm_cfg = config.section("arm")
    pipeline_cfg = config.section("pipeline")

    target = str(app.get("target", "cup"))
    camera_mode = str(camera_cfg.get("mode", "mock")).lower()
    if camera_mode == "real":
        from vision.yolo_camera_adapter import ExistingYoloCameraAdapter

        vision = ExistingYoloCameraAdapter(
            model_path=str(camera_cfg["model_path"]),
            camera_index=int(camera_cfg.get("index", 0)),
            target_label=target,
            confidence_threshold=float(camera_cfg.get("confidence", 0.55)),
            stable_frames=int(camera_cfg.get("stable_frames", 3)),
            stability_tolerance_px=int(
                camera_cfg.get("stability_tolerance_px", 40)
            ),
            inference_imgsz=int(camera_cfg.get("imgsz", 640)),
            device=camera_cfg.get("device"),
            show_preview=bool(
                camera_cfg.get(
                    "show_preview",
                    not bool(app.get("ssh", True)),
                )
            ),
        )
    else:
        vision = MockVisionAdapter(target_label=target)

    lidar_mode = str(lidar_cfg.get("mode", "mock")).lower()
    if lidar_mode == "real":
        from lidar.range_sensor import CameraGuidedRPLidarAdapter

        range_sensor = CameraGuidedRPLidarAdapter(
            device=str(lidar_cfg.get("device", "/dev/ttyRPLIDAR")),
            sdk_binary=lidar_cfg.get("sdk_binary"),
            baudrate=int(lidar_cfg.get("baudrate", 460800)),
            camera_fx_px=float(camera_cfg["fx_px"]),
            camera_cx_px=float(camera_cfg["cx_px"]),
            camera_yaw_in_base_deg=float(camera_cfg.get("yaw_in_base_deg", 0.0)),
            camera_origin_in_base_mm=config.vector3("camera", "origin_in_base_mm"),
            lidar_origin_in_base_mm=config.vector3("lidar", "origin_in_base_mm"),
            lidar_front_angle_deg=float(lidar_cfg.get("front_angle_deg", 0.0)),
            lidar_angle_sign=int(lidar_cfg.get("angle_sign", 1)),
            half_width_deg=float(lidar_cfg.get("target_half_width_deg", 5.0)),
            scan_count=int(lidar_cfg.get("scan_count", 3)),
            timeout_seconds=float(lidar_cfg.get("timeout_s", 12.0)),
            min_range_mm=float(lidar_cfg.get("min_range_mm", 120.0)),
            max_range_mm=float(lidar_cfg.get("max_range_mm", 12000.0)),
            min_quality=int(lidar_cfg.get("min_quality", 1)),
            min_points=int(lidar_cfg.get("min_points", 3)),
            outlier_mm=float(lidar_cfg.get("outlier_mm", 150.0)),
        )
    else:
        range_sensor = MockRangeSensorAdapter(
            distance_mm=float(lidar_cfg.get("mock_distance_mm", 350.0))
        )

    arm_origin = config.vector3("arm_mount", "origin_in_base_mm")
    transform = ExistingArmMountTransformAdapter(
        arm_origin_x_mm=arm_origin[0],
        arm_origin_y_mm=arm_origin[1],
        arm_origin_z_mm=arm_origin[2],
        mount_yaw_degrees=float(arm_mount_cfg.get("yaw_in_base_deg", 180.0)),
    )

    calibration_approved = bool(
        localization_cfg.get("calibration_approved", False)
    )
    target_z_approved = bool(
        localization_cfg.get("target_z_approved", False)
    )
    execution_calibration_ready = calibration_approved and target_z_approved

    localizer = PlanarTargetLocalizer(
        base_to_arm=transform,
        target_z_mm=float(localization_cfg.get("target_z_mm", 140.0)),
        calibration_approved=calibration_approved,
        target_z_approved=target_z_approved,
    )

    limits = WorkspaceLimits(**{
        key: float(workspace_cfg[key])
        for key in (
            "min_x_mm", "max_x_mm", "min_y_mm", "max_y_mm",
            "min_z_mm", "max_z_mm", "max_radius_mm",
        )
    })
    planner = SimpleTopDownGraspPlanner(
        approach_distance_mm=float(planner_cfg.get("approach_distance_mm", 80.0)),
        pregrasp_height_mm=float(planner_cfg.get("pregrasp_height_mm", 60.0)),
        lift_height_mm=float(planner_cfg.get("lift_height_mm", 100.0)),
        grasp_y_offset_mm=float(planner_cfg.get("grasp_y_offset_mm", 0.0)),
        tool_angle_rad=float(planner_cfg.get("tool_angle_rad", 3.14)),
        speed=float(planner_cfg.get("cartesian_speed", 0.15)),
        gripper_open_rad=float(planner_cfg.get("gripper_open_rad", 2.70)),
        gripper_closed_rad=float(planner_cfg.get("gripper_closed_rad", 3.14)),
        limits=limits,
    )

    arm_mode = str(arm_cfg.get("mode", "mock")).lower()
    if arm_mode == "real":
        from arm_control.arm_adapter import CartesianRoArmAdapter

        arm = CartesianRoArmAdapter(
            port=str(arm_cfg.get("device", "/dev/ttyROARM")),
            baudrate=int(arm_cfg.get("baudrate", 115200)),
            allow_real_motion=bool(arm_cfg.get("allow_real_motion", False)),
            confirm_clearance=bool(arm_cfg.get("confirm_clearance", False)),
            calibration_approved=execution_calibration_ready,
            one_grasp_per_process=bool(arm_cfg.get("one_grasp_per_process", True)),
            feedback_tolerance_mm=float(arm_cfg.get("feedback_tolerance_mm", 12.0)),
            waypoint_timeout_s=float(arm_cfg.get("waypoint_timeout_s", 15.0)),
            feedback_poll_s=float(arm_cfg.get("feedback_poll_s", 0.20)),
            feedback_initial_delay_s=float(
                arm_cfg.get(
                    "feedback_initial_delay_s",
                    arm_cfg.get("initial_feedback_delay_s", 0.75),
                )
            ),
            uart_response_timeout_s=float(arm_cfg.get("uart_response_timeout_s", 2.0)),
            gripper_settle_s=float(arm_cfg.get("gripper_settle_s", 1.0)),
            gripper_speed_steps_s=int(arm_cfg.get("gripper_speed_steps_s", 100)),
            gripper_acceleration=int(arm_cfg.get("gripper_acceleration", 10)),
            gripper_motion_margin_s=float(arm_cfg.get("gripper_motion_margin_s", 0.75)),
            min_voltage_v=float(arm_cfg.get("min_voltage_v", 7.0)),
            max_voltage_v=float(arm_cfg.get("max_voltage_v", 13.0)),
        )
        allow_provisional = False
    else:
        arm = MockArmAdapter()
        allow_provisional = bool(
            pipeline_cfg.get("allow_provisional_in_mock", True)
        )

    return ModularTaskManager(
        vision=vision,
        range_sensor=range_sensor,
        localizer=localizer,
        planner=planner,
        arm=arm,
        detection_timeout_s=float(app.get("detection_timeout_s", 20.0)),
        allow_provisional_execution=allow_provisional,
        pre_execute_recheck=bool(
            pipeline_cfg.get("pre_execute_recheck", True)
        ),
        recheck_timeout_s=float(
            pipeline_cfg.get("recheck_timeout_s", 8.0)
        ),
        max_recheck_center_shift_px=float(
            pipeline_cfg.get("max_recheck_center_shift_px", 60.0)
        ),
        max_recheck_target_shift_mm=float(
            pipeline_cfg.get("max_recheck_target_shift_mm", 60.0)
        ),
        max_range_mad_mm=float(
            pipeline_cfg.get("max_range_mad_mm", 60.0)
        ),
        max_range_span_mm=float(
            pipeline_cfg.get("max_range_span_mm", 250.0)
        ),
    )


def print_section(manager: ModularTaskManager, key: str) -> None:
    payload: Any = manager.status().get(key)
    print(json.dumps(payload, indent=2, ensure_ascii=False))


def main() -> int:
    args = parse_args()
    config = load_runtime_config(args.config)
    if args.validate_config:
        print(f"Configuration valid: {config.path}")
        return 0
    _typed_real_motion_confirmation(config)
    manager = build_manager(config)
    camera_mode = config.get("camera", "mode")
    lidar_mode = config.get("lidar", "mode")
    arm_mode = config.get("arm", "mode")

    print("=" * 72)
    print("EEE8097 Configurable Modular Pipeline")
    print(f"Config: {config.path}")
    print(f"Camera: {str(camera_mode).upper()}")
    print(f"RPLIDAR: {str(lidar_mode).upper()}")
    print(f"RoArm: {str(arm_mode).upper()}")
    print("=" * 72)
    print(HELP)

    try:
        while True:
            try:
                command = input("robot> ").strip().lower()
            except EOFError:
                command = "q"
            if command == "s":
                print("Task started." if manager.start() else "Start rejected; reset first.")
            elif command == "x":
                print("Abort requested." if manager.abort() else "No running task.")
            elif command == "p":
                print(json.dumps(manager.status(), indent=2, ensure_ascii=False))
            elif command == "d":
                print_section(manager, "detection")
            elif command == "l":
                print_section(manager, "range")
            elif command == "t":
                print_section(manager, "target")
            elif command == "g":
                print_section(manager, "plan")
            elif command == "a":
                print(json.dumps(manager.arm.status(), indent=2, ensure_ascii=False))
            elif command == "r":
                print("Reset to IDLE." if manager.reset() else "Reset rejected while running.")
            elif command in {"h", "help", "?"}:
                print(HELP)
            elif command == "q":
                return 0
            elif command:
                print(f"Unknown command: {command!r}")
                print(HELP)
    except KeyboardInterrupt:
        print("\nShutdown requested.")
        return 130
    finally:
        manager.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
