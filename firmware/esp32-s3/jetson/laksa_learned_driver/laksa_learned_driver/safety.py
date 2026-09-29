"""Clearance governor independent of the learned policy.

The imitation-learned policy only ever saw obstacle-free corridors, so it has
no notion of slowing for something close ahead.  This governor predicts the
vehicle's swept corridor along the commanded constant-curvature arc and caps
speed so the car can always stop (with latency and a margin) before the first
LiDAR return inside that corridor.  It works on raw scan points, not on the
network's features.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GovernorConfig:
    wheelbase_m: float = 0.324
    front_overhang_m: float = 0.419      # bumper ahead of base_footprint (rear axle)
    half_width_m: float = 0.148
    lateral_margin_m: float = 0.10
    stop_margin_m: float = 0.25          # clearance kept in front of the bumper when stopped
    decel_mps2: float = 1.0              # conservative braking deceleration
    latency_s: float = 0.25              # scan age + processing + actuation
    horizon_m: float = 4.0


@dataclass(frozen=True)
class GovernorResult:
    speed_mps: float
    free_distance_m: float               # bumper to first in-path return along the arc
    blocked: bool


def path_free_distance(points_xy: np.ndarray, steering_rad: float, cfg: GovernorConfig) -> float:
    """Arc length from the front bumper to the first scan point inside the swept corridor.

    ``points_xy`` are obstacle points in the ``base_footprint`` frame
    (x forward, y left).  Returns ``cfg.horizon_m`` when the path is clear.
    """
    if points_xy.size == 0:
        return cfg.horizon_m
    x = points_xy[:, 0]
    y = points_xy[:, 1]
    corridor = cfg.half_width_m + cfg.lateral_margin_m
    curvature = math.tan(steering_rad) / cfg.wheelbase_m
    if abs(curvature) < 1e-4:
        lateral = np.abs(y)
        along = x
    else:
        radius = 1.0 / curvature                      # signed: positive turns left
        dx, dy = x, y - radius
        lateral = np.abs(np.hypot(dx, dy) - abs(radius))
        # Angle travelled around the turn centre from the rear axle to the point.
        start = math.atan2(-radius, 0.0)
        swept = np.arctan2(dy, dx) - start
        swept = np.where(curvature > 0, swept, -swept)
        swept = np.mod(swept, 2.0 * math.pi)
        along = swept * abs(radius)
    in_path = (lateral <= corridor) & (along > 0.0) & (along <= cfg.horizon_m + cfg.front_overhang_m)
    if not np.any(in_path):
        return cfg.horizon_m
    return float(max(0.0, np.min(along[in_path]) - cfg.front_overhang_m))


def govern(points_xy: np.ndarray, steering_rad: float, requested_speed_mps: float,
           cfg: GovernorConfig) -> GovernorResult:
    free = path_free_distance(points_xy, steering_rad, cfg)
    usable = free - cfg.stop_margin_m
    if usable <= 0.0:
        return GovernorResult(0.0, free, True)
    # Largest v with v*latency + v^2/(2a) <= usable.
    a, t = cfg.decel_mps2, cfg.latency_s
    v_max = -a * t + math.sqrt((a * t) ** 2 + 2.0 * a * usable)
    return GovernorResult(min(max(requested_speed_mps, 0.0), v_max), free, False)


def scan_points_base(ranges: np.ndarray, angles: np.ndarray, lidar_x_m: float) -> np.ndarray:
    """Finite scan returns (from ``scan_to_vehicle_beams``) as base-frame points."""
    finite = np.isfinite(ranges)
    r = ranges[finite]
    a = angles[finite]
    return np.column_stack([lidar_x_m + r * np.cos(a), r * np.sin(a)])
