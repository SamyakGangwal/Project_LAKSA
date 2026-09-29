"""Start/stop signal detection from the camera (pure NumPy/OpenCV, testable).

The race starts when a bright green signal is seen and ends at a red stop
signal (a red light or a stop sign).  Each colour is a hue band with high
saturation and brightness; the red band stops short of orange so orange
five-gallon buckets are not read as a stop signal.  A signal counts only
after it has been seen in several consecutive frames.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class SignalConfig:
    green_hue: tuple[int, int] = (40, 85)          # OpenCV hue, 0-179
    red_hues: tuple[tuple[int, int], ...] = ((0, 6), (172, 179))
    min_saturation: int = 130
    min_value: int = 130
    min_area_fraction: float = 0.0006             # of the searched image area
    roi_bottom_fraction: float = 0.8              # ignore the floor (bottom 20%)
    persist_frames: int = 4


@dataclass(frozen=True)
class SignalReading:
    green_fraction: float
    red_fraction: float
    green: bool
    red: bool


def _largest_blob_fraction(mask: np.ndarray) -> float:
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return 0.0
    return float(stats[1:, cv2.CC_STAT_AREA].max()) / float(mask.size)


def read_signals(bgr: np.ndarray, cfg: SignalConfig = SignalConfig()) -> SignalReading:
    height = bgr.shape[0]
    roi = bgr[: max(1, int(height * cfg.roi_bottom_fraction))]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    vivid = (s >= cfg.min_saturation) & (v >= cfg.min_value)
    green_mask = (vivid & (h >= cfg.green_hue[0]) & (h <= cfg.green_hue[1])).astype(np.uint8)
    red_mask = np.zeros_like(green_mask)
    for low, high in cfg.red_hues:
        red_mask |= (vivid & (h >= low) & (h <= high)).astype(np.uint8)
    green = _largest_blob_fraction(green_mask)
    red = _largest_blob_fraction(red_mask)
    return SignalReading(green, red, green >= cfg.min_area_fraction, red >= cfg.min_area_fraction)


class Debounce:
    """True once the condition has held for ``frames`` consecutive updates."""

    def __init__(self, frames: int) -> None:
        self.frames = frames
        self.count = 0

    def update(self, seen: bool) -> bool:
        self.count = self.count + 1 if seen else 0
        return self.count >= self.frames

    def reset(self) -> None:
        self.count = 0
