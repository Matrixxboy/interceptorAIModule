"""Common helpers for boxes, geometry, and device selection."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass
class BBox:
    """Axis-aligned bounding box in pixel coordinates (xyxy)."""

    x1: float
    y1: float
    x2: float
    y2: float
    conf: float = 1.0
    cls_id: int = -1
    track_id: int = -1
    label: str = ""

    @property
    def cx(self) -> float:
        return 0.5 * (self.x1 + self.x2)

    @property
    def cy(self) -> float:
        return 0.5 * (self.y1 + self.y2)

    @property
    def width(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def height(self) -> float:
        return max(0.0, self.y2 - self.y1)

    @property
    def area(self) -> float:
        return self.width * self.height

    def as_xyxy(self) -> tuple[float, float, float, float]:
        return self.x1, self.y1, self.x2, self.y2

    def as_xywh(self) -> tuple[float, float, float, float]:
        return self.x1, self.y1, self.width, self.height

    def as_int_xywh(self) -> tuple[int, int, int, int]:
        return int(self.x1), int(self.y1), int(self.width), int(self.height)

    def clamp(self, w: int, h: int) -> "BBox":
        return BBox(
            x1=float(np.clip(self.x1, 0, w - 1)),
            y1=float(np.clip(self.y1, 0, h - 1)),
            x2=float(np.clip(self.x2, 0, w - 1)),
            y2=float(np.clip(self.y2, 0, h - 1)),
            conf=self.conf,
            cls_id=self.cls_id,
            track_id=self.track_id,
            label=self.label,
        )

    def iou(self, other: "BBox") -> float:
        ix1 = max(self.x1, other.x1)
        iy1 = max(self.y1, other.y1)
        ix2 = min(self.x2, other.x2)
        iy2 = min(self.y2, other.y2)
        iw = max(0.0, ix2 - ix1)
        ih = max(0.0, iy2 - iy1)
        inter = iw * ih
        union = self.area + other.area - inter
        return inter / union if union > 0 else 0.0


def select_torch_device(preference: str = "auto") -> str:
    """Return 'cuda' if available and requested, else 'cpu'."""
    if preference == "cpu":
        return "cpu"
    try:
        import torch

        if preference in ("auto", "cuda") and torch.cuda.is_available():
            return "cuda"
    except ImportError:
        pass
    return "cpu"


def frame_center(shape: Sequence[int]) -> tuple[float, float]:
    h, w = shape[:2]
    return w * 0.5, h * 0.5


def clip_pwm(value: float, lo: int, hi: int) -> int:
    return int(np.clip(round(value), lo, hi))
