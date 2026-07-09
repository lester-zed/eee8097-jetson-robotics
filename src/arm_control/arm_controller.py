from utils.logger import get_logger


class ArmController:
    def __init__(self) -> None:
        self.logger = get_logger(self.__class__.__name__)

    def initialize(self) -> None:
        self.logger.info("Initializing manipulator arm...")

    def move_to_home(self) -> None:
        self.logger.info("Moving arm to home position...")

    def move_to_target_pose(self) -> None:
        self.logger.info("Moving arm to target pose...")

    def open_gripper(self) -> None:
        self.logger.info("Opening gripper...")

    def close_gripper(self) -> None:
        self.logger.info("Closing gripper...")

    def run_demo_sequence(self) -> None:
        self.initialize()
        self.move_to_home()
        self.open_gripper()
        self.move_to_target_pose()
        self.close_gripper()
        self.move_to_home()
