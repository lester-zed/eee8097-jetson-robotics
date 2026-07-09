from utils.logger import get_logger
from vision.mock_vision import MockVision
from arm_control.arm_controller import ArmController


class TaskManager:
    def __init__(self) -> None:
        self.logger = get_logger(self.__class__.__name__)
        self.vision = MockVision()
        self.arm = ArmController()

    def run(self) -> None:
        self.logger.info("Starting mock task pipeline...")

        detection = self.vision.detect()
        self.logger.info(
            "Detected target: label=%s, confidence=%.2f, x=%d, y=%d",
            detection.label,
            detection.confidence,
            detection.x,
            detection.y,
        )

        self.arm.run_demo_sequence()

        self.logger.info("Mock task pipeline finished.")
