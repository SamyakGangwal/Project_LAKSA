"""Privileged teacher: smoothed raceline + curvature speed profile + pure pursuit.

The teacher sees the true pose and the track geometry.  The student network
only sees LiDAR and the operator speed cap, so it can later drive without a
map, localization, or TF.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from tracks import Track, curvature, resample_closed
import vehicle as V


def smooth_raceline(track: Track, margin_m: float = 0.12, iterations: int = 400) -> np.ndarray:
    """Elastic-band curvature reduction inside the corridor (cheap min-curvature).

    With obstacles the band is also kept inside ``track.offset_lo/hi`` so it
    passes each box on its open side; more iterations spread each swerve over
    several metres instead of leaving a kink at the constraint.
    """
    center = track.center
    heading = track.headings()
    normal = np.column_stack([-np.sin(heading), np.cos(heading)])
    bound = np.maximum(track.half_width - (V.FOOTPRINT_HALF_WIDTH_M + V.FOOTPRINT_PADDING_M + margin_m), 0.0)
    lo, hi = -bound, bound
    if track.obstacles and track.offset_lo is not None:
        lo, hi = np.maximum(lo, track.offset_lo), np.minimum(hi, track.offset_hi)
        iterations = max(iterations, 4000)
    offset = np.clip(np.zeros(center.shape[0]), lo, hi)
    for _ in range(iterations):
        points = center + offset[:, None] * normal
        target = 0.5 * (np.roll(points, 1, axis=0) + np.roll(points, -1, axis=0))
        offset += 0.5 * np.einsum("ij,ij->i", target - points, normal)
        offset = np.clip(offset, lo, hi)
    return resample_closed(center + offset[:, None] * normal, 0.05)


@dataclass
class ExpertConfig:
    lateral_accel_mps2: float = 3.0
    accel_mps2: float = 2.5
    brake_mps2: float = 3.5
    lookahead_base_m: float = 0.45
    lookahead_gain_s: float = 0.35
    lookahead_min_m: float = 0.5
    lookahead_max_m: float = 1.6
    speed_preview_s: float = 0.25


class Expert:
    def __init__(self, track: Track, raceline: np.ndarray, config: ExpertConfig | None = None) -> None:
        self.track = track
        self.path = raceline
        self.config = config or ExpertConfig()
        self.spacing = float(np.mean(np.linalg.norm(np.diff(raceline, axis=0), axis=1)))
        kappa = np.abs(curvature(raceline))
        window = max(1, int(round(0.3 / self.spacing)))
        padded = np.concatenate([kappa[-window:], kappa, kappa[:window]])
        self.kappa = np.max(np.lib.stride_tricks.sliding_window_view(padded, 2 * window + 1), axis=1)
        # Course-style tracks: slow down in narrow sections (20" leaves ~10 cm per side).
        self.width_cap = None
        if getattr(track, "width_limits_speed", False):
            from scipy.spatial import cKDTree
            _, nearest = cKDTree(track.center).query(raceline)
            half = track.half_width[nearest]
            self.width_cap = 0.8 + 3.0 * np.maximum(half - 0.25, 0.0)
        self._index = None
        self._profile_cap = None
        self.profile = None

    def speed_profile(self, v_cap: float) -> np.ndarray:
        if self._profile_cap == v_cap:
            return self.profile
        c = self.config
        v = np.minimum(v_cap, np.sqrt(c.lateral_accel_mps2 / np.maximum(self.kappa, 1e-4)))
        if self.width_cap is not None:
            v = np.minimum(v, self.width_cap)
        n = v.size
        ds = self.spacing
        for _ in range(2):
            for i in range(1, 2 * n):
                j, k = i % n, (i - 1) % n
                v[j] = min(v[j], math.sqrt(v[k] ** 2 + 2.0 * c.accel_mps2 * ds))
            for i in range(2 * n - 2, -1, -1):
                j, k = i % n, (i + 1) % n
                v[j] = min(v[j], math.sqrt(v[k] ** 2 + 2.0 * c.brake_mps2 * ds))
        self.profile, self._profile_cap = v, v_cap
        return v

    def reset(self) -> None:
        self._index = None

    def _nearest(self, point: np.ndarray) -> int:
        n = self.path.shape[0]
        if self._index is None:
            candidates = np.arange(n)
        else:
            candidates = (self._index + np.arange(-40, 120)) % n
        d = np.sum((self.path[candidates] - point) ** 2, axis=1)
        self._index = int(candidates[int(np.argmin(d))])
        return self._index

    def command(self, rear_xy: np.ndarray, yaw: float, speed: float, v_cap: float) -> tuple[float, float]:
        c = self.config
        profile = self.speed_profile(v_cap)
        n = self.path.shape[0]
        index = self._nearest(rear_xy)
        lookahead = float(np.clip(c.lookahead_base_m + c.lookahead_gain_s * max(speed, 0.0),
                                  c.lookahead_min_m, c.lookahead_max_m))
        target = self.path[(index + int(round(lookahead / self.spacing))) % n]
        dx, dy = target - rear_xy
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        distance = max(math.hypot(local_x, local_y), 1e-3)
        steering = math.atan2(2.0 * V.WHEELBASE_M * local_y, distance * distance)
        steering = float(np.clip(steering, -V.STEER_RIGHT_MAX_RAD, V.STEER_LEFT_MAX_RAD))
        preview = int(round((c.speed_preview_s * max(speed, 0.0) + 0.2) / self.spacing))
        speed_cmd = float(np.min(profile[(index + np.arange(0, preview + 1)) % n]))
        return steering, speed_cmd
