import argparse
from pprint import pprint

from arm_control.arm_controller import ArmController
from task.task_manager import TaskManager
from vision.yolo_camera import YoloCamera


HELP = """
Commands:
  s  Start one YOLO detection and mock-grasp task
  x  Abort the current task
  p  Print current task status
  r  Reset from ERROR or ABORTED state
  h  Show this help
  q  Abort the current task and quit
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="EEE8097 robot vision and mock-grasp controller"
    )

    parser.add_argument(
        "--ssh",
        action="store_true",
        help=(
            "Run in headless mode without cv2.imshow(). "
            "Use this option when connected through SSH."
        ),
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # 默认显示 Camera/YOLO 预览窗口。
    # 通过 SSH 运行时传入 --ssh，避免调用 cv2.imshow()。
    show_preview = not args.ssh

    print("=" * 60)
    print("EEE8097 Robot Controller")
    print(
        "Display mode:",
        "headless (SSH)" if args.ssh else "preview",
    )
    print("Arm mode: MOCK — the real RoArm will not move")
    print("=" * 60)

    vision = YoloCamera(
        camera_index=0,
        target_label="cup",
        confidence_threshold=0.55,
        stable_frames=3,
        show_preview=show_preview,
    )

    arm = ArmController()

    manager = TaskManager(
        vision=vision,
        arm=arm,
    )

    print(HELP)

    try:
        while True:
            command = input("robot> ").strip().lower()

            if command == "s":
                if manager.start():
                    print("Task started.")
                else:
                    print(
                        "Task was not started. "
                        "Use 'p' to check the current state."
                    )

            elif command == "x":
                manager.abort()

            elif command == "p":
                pprint(manager.status())

            elif command == "r":
                if manager.reset():
                    print("Task manager reset to IDLE.")
                else:
                    print(
                        "Reset failed. A task may still be running."
                    )

            elif command == "h":
                print(HELP)

            elif command == "q":
                print("Shutting down...")
                break

            elif command:
                print(
                    f"Unknown command: {command!r}. "
                    "Enter 'h' for help."
                )

    except (KeyboardInterrupt, EOFError):
        print("\nShutdown requested.")

    finally:
        manager.shutdown()
        print("Robot controller stopped.")


if __name__ == "__main__":
    main()