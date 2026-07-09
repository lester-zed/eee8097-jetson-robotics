from dataclasses import dataclass


@dataclass
class DetectionResult:
    label: str
    confidence: float
    x: int
    y: int


class MockVision:
    def detect(self) -> DetectionResult:
        return DetectionResult(
            label="test_object",
            confidence=0.95,
            x=320,
            y=240,
        )
