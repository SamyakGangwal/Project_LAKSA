"""Start/stop signal detection from the camera (pure NumPy/OpenCV, testable).

The competition start signal (North American event) is a painted panel, not a
light: a tall Oasis Blue board (559 x 1219 mm) standing on the ground at the
right edge of the track, 8 ft (2.44 m) down-track from the start line.  A
pinwheel pivots 813 mm up; each arm ends in a 204 mm disc (reach 407 mm from
the pivot).  Runs begin with only the red (Satin Poppy Red) arm showing; the
timer starts when the green (Satin Leafy Green) arm swings out, and teams may
go as soon as any part of the green arm is visible.

So the detector first finds the blue panel and then looks for red and green
only where the arms can reach around it.  Green turf, green clothing or
anything else green elsewhere in view is ignored.  Satin paint is far less
vivid than a lamp, so the colour bands are wide and the saturation/brightness
floors are low; ``SignalConfig`` is the place to tune them on site.

Readings are fractions of the panel's area, so a sliver of the arm counts.  The
race node starts on green *growing* past what was visible while red was up (the
CAD front view shows some green even in the red position).
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class SignalConfig:
    # OpenCV hue 0-179.  Leafy Green ~60, Poppy Red ~175-179/0-5, Oasis Blue ~100-115.
    green_hue: tuple[int, int] = (35, 90)
    red_hues: tuple[tuple[int, int], ...] = ((0, 8), (168, 179))
    blue_hue: tuple[int, int] = (95, 128)
    min_saturation: int = 70            # satin paint under indoor light
    min_value: int = 45
    red_min_saturation: int = 110       # keeps orange buckets and tan straw out
    panel_min_area_fraction: float = 0.002     # of the image (panel ~0.6 m wide at 2-4 m)
    panel_min_aspect: float = 1.3       # height / width; the board is 2.2:1
    # Search window around the panel, in panel widths/heights (arm reach 407 mm
    # = 0.73 panel widths; pivot 0.33 panel heights below the top).
    reach_x: float = 0.9
    reach_top: float = 0.15
    reach_bottom: float = 0.8
    min_signal_fraction: float = 0.006  # of the panel area: a sliver of a 204 mm disc
    persist_frames: int = 2             # "as soon as any part of the green arm is visible"
    # Without a panel (e.g. bench tests with a plain light) look in the whole
    # upper image as before; the race node only allows this when configured.
    require_panel: bool = True
    roi_bottom_fraction: float = 0.8


@dataclass(frozen=True)
class SignalReading:
    green_fraction: float               # of the panel area (of the image without a panel)
    red_fraction: float
    green: bool
    red: bool
    panel: tuple[int, int, int, int] | None = None   # x, y, w, h of the blue board


def _mask(hsv: np.ndarray, hues, min_s: int, min_v: int) -> np.ndarray:
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    vivid = (s >= min_s) & (v >= min_v)
    out = np.zeros(h.shape, np.uint8)
    for low, high in hues:
        out |= (vivid & (h >= low) & (h <= high)).astype(np.uint8)
    return cv2.morphologyEx(out, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def _largest_blob(mask: np.ndarray) -> int:
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    return int(stats[1:, cv2.CC_STAT_AREA].max()) if count > 1 else 0


def find_panel(hsv: np.ndarray, cfg: SignalConfig):
    """Largest tall blue blob: (x, y, w, h) or None."""
    blue = _mask(hsv, (cfg.blue_hue,), cfg.min_saturation, cfg.min_value)
    blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))   # bridge the arm's shadow
    count, _, stats, _ = cv2.connectedComponentsWithStats(blue, connectivity=8)
    best, best_area = None, cfg.panel_min_area_fraction * blue.size
    for i in range(1, count):
        x, y, w, h, area = stats[i]
        if area >= best_area and h >= cfg.panel_min_aspect * w:
            best, best_area = (int(x), int(y), int(w), int(h)), area
    return best


def read_signals(bgr: np.ndarray, cfg: SignalConfig = SignalConfig()) -> SignalReading:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    panel = find_panel(hsv, cfg)
    if panel is not None:
        x, y, w, h = panel
        H, W = hsv.shape[:2]
        x0, x1 = max(0, int(x - cfg.reach_x * w)), min(W, int(x + w + cfg.reach_x * w))
        y0, y1 = max(0, int(y - cfg.reach_top * h)), min(H, int(y + cfg.reach_bottom * h))
        window = hsv[y0:y1, x0:x1]
        scale = float(w * h)
    elif cfg.require_panel:
        return SignalReading(0.0, 0.0, False, False, None)
    else:
        window = hsv[: max(1, int(hsv.shape[0] * cfg.roi_bottom_fraction))]
        scale = float(window.shape[0] * window.shape[1]) * 0.1    # as if a panel filled 10%
    green = _largest_blob(_mask(window, (cfg.green_hue,), cfg.min_saturation, cfg.min_value)) / scale
    red = _largest_blob(_mask(window, cfg.red_hues, cfg.red_min_saturation, cfg.min_value)) / scale
    return SignalReading(green, red, green >= cfg.min_signal_fraction, red >= cfg.min_signal_fraction, panel)


class GreenStart:
    """Start once green grows clearly past what showed while armed (and stays for N frames).

    The baseline is the largest green seen in the first frames after ARM, while
    the red arm is up; a CAD view shows part of the green arm even then.
    """

    def __init__(self, frames: int, baseline_frames: int = 5, margin: float = 0.006, factor: float = 1.5) -> None:
        self.frames, self.baseline_frames, self.margin, self.factor = frames, baseline_frames, margin, factor
        self.reset()

    def reset(self) -> None:
        self.baseline = 0.0
        self.seen = 0
        self.count = 0

    def update(self, reading: SignalReading) -> bool:
        if self.seen < self.baseline_frames:
            self.seen += 1
            self.baseline = max(self.baseline, reading.green_fraction)
            return False
        grown = reading.green and reading.green_fraction >= self.baseline * self.factor + self.margin
        self.count = self.count + 1 if grown else 0
        return self.count >= self.frames


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
