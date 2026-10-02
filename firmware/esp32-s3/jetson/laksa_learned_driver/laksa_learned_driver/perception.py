"""Camera perception -> obstacles in the vehicle frame (pure NumPy, testable).

Two ZED sources complement the single-plane LiDAR:

* neural object detection (3-D boxes): each box's floor footprint becomes
  obstacle points; people get a larger margin and speed rules;
* the depth point cloud: points between ``min_height_m`` and ``max_height_m``
  above the *ground plane* are obstacles the LiDAR slice (~0.135 m high) can
  miss, such as low objects or overhanging edges.

The ground plane is fitted from the depth cloud itself (RANSAC), so sloped
ground counts as ground rather than as an obstacle, and LiDAR returns that land
on rising ground can be recognised as "slope, not wall".  Camera points closer
than ``near_field_min_x_m`` are ignored: ZED depth is unreliable there (worst in
low light) and the LiDAR covers that zone.

All inputs here are already in ``base_footprint`` (x forward, y left, z up).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PerceptionConfig:
    min_height_m: float = 0.05
    # bumper (0.419) + 0.20 m.  At +0.13 m (until 2026-10-01) ZED depth at its closest
    # range showed floor as obstacles ~0.15 m ahead, stopping the car again and again;
    # +0.20 m is still inside the 0.30 m stop distance, so low obstacles stay covered.
    near_field_min_x_m: float = 0.62
    ground_min_range_m: float = 0.5
    ground_max_range_m: float = 3.0
    ground_max_abs_z_m: float = 0.30
    ground_inlier_m: float = 0.03
    ground_min_inliers: int = 120
    ground_max_slope_rad: float = math.radians(20.0)
    ground_iterations: int = 40
    max_height_m: float = 0.45
    max_range_m: float = 4.0
    cloud_voxel_m: float = 0.05
    box_sample_m: float = 0.05
    person_margin_m: float = 0.25        # extra footprint inflation around people
    # 2026-10-02: 2.0 m / 1.0 m / +-60 deg stopped the car for judges standing beside
    # the track 1-3 m away. People in the path still stop it via the obstacle layer.
    person_slow_distance_m: float = 1.0
    person_slow_factor: float = 0.5
    person_stop_distance_m: float = 0.5
    person_cone_rad: float = math.radians(20.0)  # roughly the path ahead, not the sidelines


def fit_ground_plane(points_xyz: np.ndarray, cfg: PerceptionConfig, rng: np.random.Generator | None = None):
    """RANSAC ground plane z = a*x + b*y + c in base_footprint, or None.

    Only accepted when it is near the car's own ground (|c| small) and not
    steeper than ``ground_max_slope_rad``.
    """
    if points_xyz.size == 0:
        return None
    p = points_xyz[np.all(np.isfinite(points_xyz), axis=1)]
    rng_xy = np.hypot(p[:, 0], p[:, 1])
    cand = p[(rng_xy >= cfg.ground_min_range_m) & (rng_xy <= cfg.ground_max_range_m)
             & (np.abs(p[:, 2]) <= cfg.ground_max_abs_z_m) & (p[:, 0] > 0.0)]
    if cand.shape[0] < cfg.ground_min_inliers:
        return None
    rng = rng or np.random.default_rng(0)
    design = np.column_stack([cand[:, 0], cand[:, 1], np.ones(cand.shape[0])])
    best, best_count = None, 0
    for _ in range(cfg.ground_iterations):
        sample = cand[rng.choice(cand.shape[0], 3, replace=False)]
        a_mat = np.column_stack([sample[:, 0], sample[:, 1], np.ones(3)])
        try:
            coeffs = np.linalg.solve(a_mat, sample[:, 2])
        except np.linalg.LinAlgError:
            continue
        count = int(np.sum(np.abs(design @ coeffs - cand[:, 2]) <= cfg.ground_inlier_m))
        if count > best_count:
            best, best_count = coeffs, count
    if best is None or best_count < cfg.ground_min_inliers:
        return None
    inliers = np.abs(design @ best - cand[:, 2]) <= cfg.ground_inlier_m
    coeffs, *_ = np.linalg.lstsq(design[inliers], cand[inliers, 2], rcond=None)
    slope = math.atan(math.hypot(coeffs[0], coeffs[1]))
    if slope > cfg.ground_max_slope_rad or abs(coeffs[2]) > 0.15:
        return None
    return float(coeffs[0]), float(coeffs[1]), float(coeffs[2])


def ground_height(plane, x, y) -> np.ndarray:
    x = np.asarray(x, dtype=np.float64)
    if plane is None:
        return np.zeros_like(x)
    a, b, c = plane
    return a * x + b * np.asarray(y, dtype=np.float64) + c


def lidar_ground_mask(points_xy: np.ndarray, plane, lidar_height_m: float = 0.135,
                      tolerance_m: float = 0.05) -> np.ndarray:
    """True for LiDAR returns explained by rising ground (a slope), not a wall."""
    if plane is None or points_xy.size == 0:
        return np.zeros(points_xy.shape[0], dtype=bool)
    return ground_height(plane, points_xy[:, 0], points_xy[:, 1]) >= lidar_height_m - tolerance_m


def filter_cloud(points_xyz: np.ndarray, cfg: PerceptionConfig, plane=None) -> np.ndarray:
    """Obstacle-height cloud points as a voxel-thinned (N, 2) array of x, y."""
    if points_xyz.size == 0:
        return np.empty((0, 2))
    p = points_xyz[np.all(np.isfinite(points_xyz), axis=1)]
    height = p[:, 2] - ground_height(plane, p[:, 0], p[:, 1])
    keep = (height >= cfg.min_height_m) & (height <= cfg.max_height_m) \
        & (p[:, 0] >= cfg.near_field_min_x_m) \
        & (np.hypot(p[:, 0], p[:, 1]) <= cfg.max_range_m)
    xy = p[keep, :2]
    if xy.shape[0] == 0:
        return np.empty((0, 2))
    cells = np.unique(np.floor(xy / cfg.cloud_voxel_m).astype(np.int64), axis=0)
    return (cells + 0.5) * cfg.cloud_voxel_m


def box_footprint_points(corners_xyz: np.ndarray, inflate_m: float, cfg: PerceptionConfig) -> np.ndarray:
    """Sample the outline of a 3-D box's floor footprint (convex hull of its corners)."""
    xy = corners_xyz[:, :2]
    center = xy.mean(axis=0)
    hull = xy[np.argsort(np.arctan2(xy[:, 1] - center[1], xy[:, 0] - center[0]))]
    if inflate_m > 0.0:
        direction = hull - center
        norm = np.linalg.norm(direction, axis=1, keepdims=True)
        hull = hull + direction / np.maximum(norm, 1e-6) * inflate_m
    samples = [center[None, :]]
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        count = max(2, int(math.ceil(np.linalg.norm(b - a) / cfg.box_sample_m)))
        t = np.linspace(0.0, 1.0, count, endpoint=False)[:, None]
        samples.append(a + t * (b - a))
    return np.vstack(samples)


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    position_xyz: np.ndarray     # base_footprint
    corners_xyz: np.ndarray      # (8, 3) base_footprint


def detection_obstacles(detections: list[Detection], cfg: PerceptionConfig) -> np.ndarray:
    parts = []
    for det in detections:
        if np.hypot(*det.position_xyz[:2]) > cfg.max_range_m:
            continue
        inflate = cfg.person_margin_m if det.label.lower() == "person" else 0.0
        parts.append(box_footprint_points(det.corners_xyz, inflate, cfg))
    return np.vstack(parts) if parts else np.empty((0, 2))


def person_speed_rule(detections: list[Detection], cfg: PerceptionConfig,
                      front_offset_m: float = 0.419) -> tuple[float, bool, float]:
    """Return (speed factor, stop, nearest person distance ahead of the bumper)."""
    nearest = math.inf
    for det in detections:
        if det.label.lower() != "person":
            continue
        x, y = float(det.position_xyz[0]), float(det.position_xyz[1])
        if x <= 0.0 or abs(math.atan2(y, x)) > cfg.person_cone_rad:
            continue
        nearest = min(nearest, max(0.0, math.hypot(x, y) - front_offset_m))
    if nearest <= cfg.person_stop_distance_m:
        return 0.0, True, nearest
    if nearest <= cfg.person_slow_distance_m:
        return cfg.person_slow_factor, False, nearest
    return 1.0, False, nearest
