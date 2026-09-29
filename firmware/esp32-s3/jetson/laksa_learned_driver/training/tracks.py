"""Procedural closed-circuit tracks and map rasterization for F1TENTH Gym.

Tracks are random smooth loops whose centerline curvature respects the LAKSA
turning limit, with corridor widths spanning the 0.91 m competition lane up to
wide practice tracks.  The exact centerline is known, so the privileged expert
needs no map processing.  The competition course itself is *not* generated
here: its mission file marks it ``training_allowed: false``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree


@dataclass
class Track:
    name: str
    center: np.ndarray            # (N, 2) closed loop, uniform spacing, no duplicate end point
    half_width: np.ndarray        # (N,) usable half width at each centerline point
    map_stem: str                 # path without extension, as F1TENTH Gym expects
    resolution: float
    origin: tuple[float, float]
    free: np.ndarray = field(repr=False)  # bool image, row 0 = max y (ROS map convention)

    @property
    def spacing(self) -> float:
        return float(np.mean(np.linalg.norm(np.diff(self.center, axis=0), axis=1)))

    def headings(self) -> np.ndarray:
        forward = np.roll(self.center, -1, axis=0) - np.roll(self.center, 1, axis=0)
        return np.arctan2(forward[:, 1], forward[:, 0])

    def is_free(self, points: np.ndarray) -> np.ndarray:
        cols = np.floor((points[:, 0] - self.origin[0]) / self.resolution).astype(np.int64)
        rows_from_bottom = np.floor((points[:, 1] - self.origin[1]) / self.resolution).astype(np.int64)
        rows = self.free.shape[0] - 1 - rows_from_bottom
        inside = (cols >= 0) & (cols < self.free.shape[1]) & (rows >= 0) & (rows < self.free.shape[0])
        result = np.zeros(points.shape[0], dtype=bool)
        result[inside] = self.free[rows[inside], cols[inside]]
        return result


def resample_closed(points: np.ndarray, spacing: float) -> np.ndarray:
    loop = np.vstack([points, points[:1]])
    seg = np.linalg.norm(np.diff(loop, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    count = max(8, int(round(total / spacing)))
    target = np.linspace(0.0, total, count, endpoint=False)
    return np.column_stack([np.interp(target, s, loop[:, 0]), np.interp(target, s, loop[:, 1])])


def curvature(points: np.ndarray) -> np.ndarray:
    prev = np.roll(points, 1, axis=0)
    nxt = np.roll(points, -1, axis=0)
    a = np.linalg.norm(points - prev, axis=1)
    b = np.linalg.norm(nxt - points, axis=1)
    c = np.linalg.norm(nxt - prev, axis=1)
    cross = (points[:, 0] - prev[:, 0]) * (nxt[:, 1] - prev[:, 1]) - (points[:, 1] - prev[:, 1]) * (nxt[:, 0] - prev[:, 0])
    return 2.0 * cross / np.maximum(a * b * c, 1e-12)


def _random_loop(rng: np.random.Generator) -> np.ndarray:
    count = int(rng.integers(6, 13))
    theta = np.sort((np.arange(count) + rng.uniform(-0.3, 0.3, count)) * 2.0 * math.pi / count)
    radius = rng.uniform(3.0, 8.0) * (1.0 + rng.uniform(-0.35, 0.35, count))
    aspect = rng.uniform(1.0, 3.0)
    control = np.column_stack([aspect * radius * np.cos(theta), radius * np.sin(theta)])
    closed = np.vstack([control, control[:1]])
    chord = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))])
    spline = CubicSpline(chord, closed, bc_type="periodic")
    dense = spline(np.linspace(0.0, chord[-1], 4000, endpoint=False))
    return dense


def _self_clearance_ok(center: np.ndarray, width: float, spacing: float) -> bool:
    tree = cKDTree(center)
    pairs = tree.query_pairs(width + 0.8, output_type="ndarray")
    if pairs.size == 0:
        return True
    n = center.shape[0]
    gap = np.abs(pairs[:, 0] - pairs[:, 1])
    arc = np.minimum(gap, n - gap) * spacing
    return not np.any(arc > 3.0 * (width + 0.8))


def rasterize(center: np.ndarray, width_m: float, out_dir: Path, name: str, resolution: float = 0.05,
              margin_m: float = 2.0) -> tuple[str, tuple[float, float], np.ndarray]:
    out_dir.mkdir(parents=True, exist_ok=True)
    lo = center.min(axis=0) - margin_m - width_m
    hi = center.max(axis=0) + margin_m + width_m
    cols = int(math.ceil((hi[0] - lo[0]) / resolution))
    rows = int(math.ceil((hi[1] - lo[1]) / resolution))
    image = np.zeros((rows, cols), dtype=np.uint8)
    px = np.column_stack([
        (center[:, 0] - lo[0]) / resolution,
        rows - 1 - (center[:, 1] - lo[1]) / resolution,
    ])
    shift = 4
    pts = np.round(px * (1 << shift)).astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(image, [pts], isClosed=True, color=255,
                  thickness=max(1, int(round(width_m / resolution))), lineType=cv2.LINE_8, shift=shift)
    stem = out_dir / name
    cv2.imwrite(str(stem) + ".png", image)
    (Path(str(stem) + ".yaml")).write_text(
        f"image: {name}.png\nresolution: {resolution}\norigin: [{lo[0]:.6f}, {lo[1]:.6f}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )
    return str(stem), (float(lo[0]), float(lo[1])), image > 128


def generate_track(rng: np.random.Generator, out_dir: Path, name: str,
                   max_curvature: float = 0.75, spacing: float = 0.05) -> Track:
    for _ in range(200):
        width = rng.uniform(0.9, 1.25) if rng.random() < 0.45 else rng.uniform(1.25, 2.2)
        center = resample_closed(_random_loop(rng), spacing)
        if np.max(np.abs(curvature(resample_closed(center, 0.25)))) > max_curvature:
            continue
        if not _self_clearance_ok(center, width, spacing):
            continue
        if rng.random() < 0.5:
            center = center[::-1].copy()
        stem, origin, free = rasterize(center, width, out_dir, name)
        return Track(name, center, np.full(center.shape[0], 0.5 * width), stem, 0.05, origin, free)
    raise RuntimeError("could not generate a feasible track")


def load_map_track(name: str, map_yaml: Path, centerline_xy: np.ndarray, out_dir: Path,
                   spacing: float = 0.05) -> Track:
    """Wrap an existing ROS map (e.g. the held-out competition course).

    F1TENTH Gym requires the image and YAML to share one stem, so a normalized
    copy is written to ``out_dir``.
    """
    import yaml

    meta = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image = cv2.imread(str(map_yaml.parent / meta["image"]), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(meta["image"])
    free = image > 128 if not meta.get("negate", 0) else image < 128
    center = resample_closed(np.asarray(centerline_xy, dtype=np.float64), spacing)
    resolution = float(meta["resolution"])
    origin = (float(meta["origin"][0]), float(meta["origin"][1]))
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = out_dir / name
    cv2.imwrite(str(stem) + ".png", np.where(free, 255, 0).astype(np.uint8))
    Path(str(stem) + ".yaml").write_text(
        f"image: {name}.png\nresolution: {resolution}\norigin: [{origin[0]}, {origin[1]}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )
    track = Track(name, center, np.zeros(center.shape[0]), str(stem), resolution, origin, free)
    # Measure the local half width from the occupancy image along each normal.
    headings = track.headings()
    normals = np.column_stack([-np.sin(headings), np.cos(headings)])
    offsets = np.arange(0.0, 3.0, resolution * 0.5)
    half = np.zeros(center.shape[0])
    for side in (1.0, -1.0):
        probe = center[:, None, :] + side * offsets[None, :, None] * normals[:, None, :]
        free_mask = track.is_free(probe.reshape(-1, 2)).reshape(center.shape[0], offsets.size)
        first_blocked = np.argmin(free_mask, axis=1)
        reach = np.where(free_mask.all(axis=1), offsets[-1], offsets[first_blocked])
        half = reach if side > 0 else np.minimum(half, reach)
    track.half_width = half
    return track
