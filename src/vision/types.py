from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class DetectionResult:
    label: str
    confidence: float
    center_x: int
    center_y: int
    bbox: Tuple[int, int, int, int]
    frame_width: int
    frame_height: int

    @property
    def region(self) -> str:
        """根据目标中心点将画面划分为左、中、右三个区域。"""
        left_boundary = self.frame_width / 3
        right_boundary = self.frame_width * 2 / 3

        if self.center_x < left_boundary:
            return "left"

        if self.center_x > right_boundary:
            return "right"

        return "centre"