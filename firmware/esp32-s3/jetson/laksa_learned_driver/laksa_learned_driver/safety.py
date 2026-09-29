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
    # Points already beside the body (behind the bumper) but outside the footprint
    # only matter when turning toward them: going straight or turning away never
    # brings the body closer, while a turn toward them can clip them with the
    # front corner.  Points inside the footprint itself always block.
    beside = in_path & (along < cfg.front_overhang_m) & (lateral > cfg.half_width_m)
    toward = (abs(curvature) >= 1e-4) & (np.sign(y) == math.copysign(1.0, curvature))
    in_path &= ~beside | toward
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


@dataclass(frozen=True)
class AvoidConfig:
    steer_left_max_rad: float = 0.523
    steer_right_max_rad: float = 0.288
    candidates: int = 17                 # steering arcs tried across the full range
    clearance_m: float = 0.60            # free distance ahead of the bumper that needs no detour


@dataclass(frozen=True)
class AvoidResult:
    steering_rad: float
    free_distance_m: float
    all_blocked: bool                    # no candidate arc can move at all
    avoiding: bool                       # deviated from the preferred steering


def choose_steering(points_xy: np.ndarray, preferred_rad: float, gov: GovernorConfig,
                    cfg: AvoidConfig, previous_rad: float | None = None) -> AvoidResult:
    """Keep the preferred arc if it has ``clearance_m`` free; otherwise steer around.

    An ongoing detour (``previous_rad``) is kept while it still has room, so the
    car does not flip between left and right detours from scan to scan.  Else,
    among arcs with at least ``clearance_m`` free, take the one closest to the
    preferred steering.  If none has that much room, take the arc with the most
    free distance (closest to the preferred on ties).  ``all_blocked`` means no
    arc leaves room beyond the stop margin, so only stopping or reversing helps.
    """
    preferred_free = path_free_distance(points_xy, preferred_rad, gov)
    if preferred_free >= cfg.clearance_m:
        return AvoidResult(preferred_rad, preferred_free, False, False)
    if previous_rad is not None:
        previous_free = path_free_distance(points_xy, previous_rad, gov)
        if previous_free >= cfg.clearance_m:
            return AvoidResult(previous_rad, previous_free, False, True)
    arcs = np.unique(np.append(
        np.linspace(-cfg.steer_right_max_rad, cfg.steer_left_max_rad, cfg.candidates), preferred_rad))
    free = np.array([path_free_distance(points_xy, float(a), gov) for a in arcs])
    deviation = np.abs(arcs - preferred_rad)
    roomy = free >= cfg.clearance_m
    if np.any(roomy):
        index = int(np.flatnonzero(roomy)[np.argmin(deviation[roomy])])
    else:
        best = free.max()
        near_best = np.flatnonzero(free >= best - 1e-6)
        index = int(near_best[np.argmin(deviation[near_best])])
    steering = float(arcs[index])
    return AvoidResult(steering, float(free[index]), bool(free[index] <= gov.stop_margin_m),
                       abs(steering - preferred_rad) > 1e-6)


def scan_points_base(ranges: np.ndarray, angles: np.ndarray, lidar_x_m: float) -> np.ndarray:
    """Finite scan returns (from ``scan_to_vehicle_beams``) as base-frame points."""
    finite = np.isfinite(ranges)
    r = ranges[finite]
    a = angles[finite]
    return np.column_stack([lidar_x_m + r * np.cos(a), r * np.sin(a)])
