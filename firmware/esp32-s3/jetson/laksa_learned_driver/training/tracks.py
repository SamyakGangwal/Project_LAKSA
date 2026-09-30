"""Procedural closed-circuit tracks and map rasterization for F1TENTH Gym.

Tracks are random smooth loops whose centerline curvature respects the LAKSA
turning limit, with corridor widths spanning the 0.91 m competition lane up to
wide practice tracks.  The exact centerline is known, so the privileged expert
needs no map processing.  The competition course itself is *not* generated
here: its mission file marks it ``training_allowed: false``.

Course-style tracks (``generate_course_track``) follow the 2026 course layouts:
sections of 20", 32", 36", 41.5" and 48" width, occasional ~11 ft open areas,
and clusters of 2-9 five-gallon buckets (~0.30 m) with a way through.

Tracks can carry box obstacles (``generate_track(..., obstacles=n)``).  Each
leaves a gap of at least ``PASS_GAP_M`` on one side, and the track records the
lateral range the raceline may use near it (``offset_lo``/``offset_hi``), so
the privileged expert swerves through the open side.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree

import vehicle as V

PASS_GAP_M = 2.0 * (V.FOOTPRINT_HALF_WIDTH_M + V.FOOTPRINT_PADDING_M) + 0.40
# Raceline (rear axle) keeps this much side clearance to a box: the front corner
# swings out during a swerve and pure pursuit cuts slightly at speed.
OBSTACLE_CLEARANCE_M = 0.22
SWERVE_RADIUS_M = 2.5             # limits taper as a parabola of this radius (car's tightest: ~1.1 m)


@dataclass
class Track:
    name: str
    center: np.ndarray            # (N, 2) closed loop, uniform spacing, no duplicate end point
    half_width: np.ndarray        # (N,) usable half width at each centerline point
    map_stem: str                 # path without extension, as F1TENTH Gym expects
    resolution: float
    origin: tuple[float, float]
    free: np.ndarray = field(repr=False)  # bool image, row 0 = max y (ROS map convention)
    obstacles: list = field(default_factory=list)      # (center_xy, corners (4, 2)) per box
    offset_lo: np.ndarray | None = None                  # raceline lateral limits (+ = left)
    offset_hi: np.ndarray | None = None
    width_limits_speed: bool = False                     # expert slows in narrow sections
    no_start: tuple | None = None                        # ((x, y), radius): never start episodes here
    clearance_limits: bool = False                       # offset_lo/hi come from 2-D wall clearance

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
                   max_curvature: float = 0.75, spacing: float = 0.05, obstacles: int = 0) -> Track:
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
        track = Track(name, center, np.full(center.shape[0], 0.5 * width), stem, 0.05, origin, free)
        if obstacles:
            add_obstacles(track, rng, obstacles)
        return track
    raise RuntimeError("could not generate a feasible track")


def add_obstacles(track: Track, rng: np.random.Generator, count: int, min_separation_m: float = 4.0,
                  start_clear_m: float = 3.0, max_local_curvature: float = 0.4, shape: str = "box",
                  size_range: tuple[float, float] = (0.20, 0.45), eligible: np.ndarray | None = None,
                  natural_fraction: float = 0.6) -> None:
    """Place up to ``count`` boxes or round buckets and constrain the raceline around them.

    Can be called repeatedly (e.g. one bucket cluster per open area); limits
    and the drawn map accumulate.
    """
    n = track.center.shape[0]
    ds = track.spacing
    heading = track.headings()
    tangent = np.column_stack([np.cos(heading), np.sin(heading)])
    normal = np.column_stack([-np.sin(heading), np.cos(heading)])
    kappa = np.abs(curvature(resample_closed(track.center, ds)))
    window = int(round(1.5 / ds))
    local = np.max(np.lib.stride_tricks.sliding_window_view(np.concatenate([kappa[-window:], kappa, kappa[:window]]),
                                                            2 * window + 1), axis=1)[:n]
    arc = np.arange(n) * ds
    total = n * ds
    mask = (local < max_local_curvature) & (arc > start_clear_m) & (arc < total - start_clear_m)
    if eligible is not None:
        mask &= eligible
    candidates = np.flatnonzero(mask)
    rng.shuffle(candidates)
    wall = np.maximum(track.half_width - (V.FOOTPRINT_HALF_WIDTH_M + V.FOOTPRINT_PADDING_M + 0.12), 0.0)
    lo = track.offset_lo.copy() if track.offset_lo is not None else -wall.copy()
    hi = track.offset_hi.copy() if track.offset_hi is not None else wall.copy()
    existing = [int(np.argmin(np.sum((track.center - c) ** 2, axis=1))) for c, _ in track.obstacles]
    # Where an unobstructed raceline would run (same elastic band as the expert),
    # so most boxes sit on the natural driving line and force a swerve.
    natural = np.zeros(n)
    for _ in range(400):
        points = track.center + natural[:, None] * normal
        target = 0.5 * (np.roll(points, 1, axis=0) + np.roll(points, -1, axis=0))
        natural = np.clip(natural + 0.5 * np.einsum("ij,ij->i", target - points, normal), -wall, wall)
    placed: list[int] = []
    added = 0
    car = V.FOOTPRINT_HALF_WIDTH_M + V.FOOTPRINT_PADDING_M + OBSTACLE_CLEARANCE_M
    for index in candidates:
        if added >= count:
            break
        if any(min(abs(index - j), n - abs(index - j)) * ds < min_separation_m for j in placed + existing):
            continue
        half = float(track.half_width[index])
        size = float(rng.uniform(*size_range))
        size = min(size, 2.0 * half - PASS_GAP_M)
        if size < 0.12:
            continue
        # Lateral centre so at least one side keeps a PASS_GAP_M gap: on the
        # natural driving line 60% of the time, anywhere feasible otherwise.
        c_min, c_max = PASS_GAP_M - half + size / 2.0, half - size / 2.0
        if rng.random() < natural_fraction:
            c = float(natural[index] + rng.uniform(-0.08, 0.08))
            if -c_min < c < c_min:
                c = math.copysign(c_min, c if c != 0.0 else rng.uniform(-1.0, 1.0))
            c = float(np.clip(c, -c_max, c_max))
        else:
            side = 1.0 if rng.random() < 0.5 else -1.0
            c = side * float(rng.uniform(c_min, c_max))
        left_gap, right_gap = half - (c + size / 2.0), (c - size / 2.0) + half
        center_xy = track.center[index] + c * normal[index]
        if shape == "bucket":
            ring = np.linspace(0.0, 2.0 * math.pi, 16, endpoint=False)
            corners = center_xy + 0.5 * size * np.column_stack([np.cos(ring), np.sin(ring)])
        else:
            corners = np.array([center_xy + a * tangent[index] * size / 2.0 + b * normal[index] * size / 2.0
                                for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1))])
        # Raceline limits: exact over the box plus the car's length on either
        # side, then relaxed as a parabola (curvature 1/SWERVE_RADIUS_M) so the
        # line curves into the gap gradually instead of kinking.
        core = size / 2.0 + V.FOOTPRINT_FRONT_M + 0.45
        reach = int(math.ceil((core + math.sqrt(2.0 * SWERVE_RADIUS_M * 2.0 * half)) / ds))
        span = (index + np.arange(-reach, reach + 1)) % n
        beyond = np.maximum(np.abs(np.arange(-reach, reach + 1)) * ds - core, 0.0)
        relax = beyond ** 2 / (2.0 * SWERVE_RADIUS_M)
        new_lo, new_hi = lo.copy(), hi.copy()
        if left_gap >= right_gap:
            new_lo[span] = np.maximum(lo[span], c + size / 2.0 + car - relax)
        else:
            new_hi[span] = np.minimum(hi[span], c - size / 2.0 - car + relax)
        if np.any(new_lo[span] > new_hi[span]):
            continue
        lo, hi = new_lo, new_hi
        track.obstacles.append((center_xy, corners))
        placed.append(int(index))
        added += 1
    track.offset_lo, track.offset_hi = lo, hi
    if not track.obstacles:
        return
    # Draw the boxes into the map (occupied), so the simulated LiDAR sees them.
    image = np.where(track.free, 255, 0).astype(np.uint8)
    rows = image.shape[0]
    for _, corners in track.obstacles:
        px = np.column_stack([(corners[:, 0] - track.origin[0]) / track.resolution,
                              rows - 1 - (corners[:, 1] - track.origin[1]) / track.resolution])
        cv2.fillPoly(image, [np.round(px).astype(np.int32).reshape(-1, 1, 2)], 0)
    cv2.imwrite(track.map_stem + ".png", image)
    track.free = image > 128


COURSE_WIDTHS_M = (0.508, 0.813, 0.914, 1.054, 1.219)   # 20", 32", 36", 41.5", 48"
COURSE_WIDTH_WEIGHTS = (0.15, 0.30, 0.20, 0.15, 0.20)
OPEN_AREA_WIDTH_M = 3.35                                 # the ~11 ft wide section
BUCKET_DIAMETER_M = 0.30                                 # five-gallon bucket


def generate_course_track(rng: np.random.Generator, out_dir: Path, name: str,
                          max_curvature: float = 0.8, spacing: float = 0.05) -> Track:
    """A loop with 2026-course-style sections: varying widths, open areas, buckets."""
    for _ in range(300):
        center = resample_closed(_random_loop(rng), spacing)
        kappa = np.abs(curvature(resample_closed(center, 0.25)))
        if np.max(kappa) > max_curvature:
            continue
        n = center.shape[0]
        k_local = np.interp(np.arange(n) * spacing, np.arange(kappa.size) * 0.25, kappa)
        widths = np.empty(n)
        is_open = np.zeros(n, dtype=bool)
        index = 0
        while index < n:
            length = int(rng.uniform(3.0, 10.0) / spacing)
            span = np.arange(index, min(n, index + length))
            if rng.random() < 0.12:
                length = int(rng.uniform(5.0, 8.0) / spacing)
                span = np.arange(index, min(n, index + length))
                widths[span] = OPEN_AREA_WIDTH_M
                is_open[span] = True
            else:
                width = float(rng.choice(COURSE_WIDTHS_M, p=COURSE_WIDTH_WEIGHTS))
                if width < 0.6 and np.max(k_local[span]) > 0.45:
                    width = COURSE_WIDTHS_M[1]        # 20" path only on gentle curves
                widths[span] = width
            index += len(span)
        # Taper width changes over ~0.6 m, as the course joins sections.
        taper = int(0.6 / spacing)
        padded = np.concatenate([widths[-taper:], widths, widths[:taper]])
        widths = np.convolve(padded, np.ones(2 * taper + 1) / (2 * taper + 1), mode="same")[taper:-taper]
        if not _self_clearance_ok(center, float(np.max(widths)), spacing):
            continue
        if rng.random() < 0.5:
            center, widths, is_open = center[::-1].copy(), widths[::-1].copy(), is_open[::-1].copy()
        stem, origin, free = rasterize_variable(center, widths, out_dir, name)
        track = Track(name, center, 0.5 * widths, stem, 0.05, origin, free, width_limits_speed=True)
        # Bucket clusters (2-9) in each open area, a path around and between them.
        opens = np.flatnonzero(is_open)
        if opens.size:
            breaks = np.flatnonzero(np.diff(opens) > 1)
            for area in np.split(opens, breaks + 1):
                mask = np.zeros(n, dtype=bool)
                mask[area] = True
                add_obstacles(track, rng, int(rng.integers(2, 10)), min_separation_m=0.7, start_clear_m=3.0,
                              max_local_curvature=0.6, shape="bucket",
                              size_range=(BUCKET_DIAMETER_M, BUCKET_DIAMETER_M), eligible=mask,
                              natural_fraction=0.5)
        # A few boxes in the 36"-48" sections, as on the practice tracks.
        medium = (track.half_width >= 0.45) & ~is_open
        if rng.random() < 0.5 and medium.any():
            add_obstacles(track, rng, int(rng.integers(1, 4)), eligible=medium)
        return track
    raise RuntimeError("could not generate a feasible course-style track")


def rasterize_variable(center: np.ndarray, widths: np.ndarray, out_dir: Path, name: str,
                       resolution: float = 0.05, margin_m: float = 2.0):
    """Like ``rasterize`` but with a width per centerline point."""
    out_dir.mkdir(parents=True, exist_ok=True)
    reach = margin_m + float(np.max(widths))
    lo = center.min(axis=0) - reach
    hi = center.max(axis=0) + reach
    cols = int(math.ceil((hi[0] - lo[0]) / resolution))
    rows = int(math.ceil((hi[1] - lo[1]) / resolution))
    image = np.zeros((rows, cols), dtype=np.uint8)
    for (x, y), w in zip(center, widths):
        col = int(round((x - lo[0]) / resolution))
        row = int(round(rows - 1 - (y - lo[1]) / resolution))
        cv2.circle(image, (col, row), max(1, int(round(0.5 * w / resolution))), 255, -1)
    stem = out_dir / name
    cv2.imwrite(str(stem) + ".png", image)
    (Path(str(stem) + ".yaml")).write_text(
        f"image: {name}.png\nresolution: {resolution}\norigin: [{lo[0]:.6f}, {lo[1]:.6f}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8", newline="\n",
    )
    return str(stem), (float(lo[0]), float(lo[1])), image > 128


OBSTACLE_COURSE_DIR = Path(__file__).resolve().parent / "courses" / "obstacle_course_2026"


def load_obstacle_course(rng: np.random.Generator, out_dir: Path, name: str,
                         course_dir: Path = OBSTACLE_COURSE_DIR, hoops: bool = True) -> Track:
    """The 2026 Obstacle Course replica with this episode's buckets and hoops.

    Direction is random (the course may be run either way on the day).  The
    raceline may use the real distance to the wall on each side, so the
    expert can open up tight turns inside wide sections.
    """
    import csv
    import json

    features = json.loads((course_dir / "features.json").read_text(encoding="utf-8"))
    with open(course_dir / "centerline.csv", newline="", encoding="utf-8") as handle:
        center = np.array([(float(r["x_m"]), float(r["y_m"])) for r in csv.DictReader(handle)])
    if rng.random() < 0.5:
        center = center[::-1].copy()
    track = load_map_track(name, course_dir / "map.yaml", center, out_dir)
    track.width_limits_speed = True
    track.no_start = (tuple(features["no_start_center"]), float(features["no_start_radius_m"]))
    set_side_limits(track)
    # Buckets: 2-9 in the bucket box, a way around and between them.
    box = np.array(features["bucket_box"])
    inside = cv2.pointPolygonTest
    eligible = np.array([inside(box.astype(np.float32), (float(x), float(y)), False) >= 0 for x, y in track.center])
    low, high = features["bucket_count"]
    diameter = float(features["bucket_diameter_m"])
    add_obstacles(track, rng, int(rng.integers(low, high + 1)), min_separation_m=0.45, start_clear_m=0.0,
                  max_local_curvature=1.5, shape="bucket", size_range=(diameter, diameter), eligible=eligible,
                  natural_fraction=0.5)
    for a, b in features["hoop_lines"] if hoops else []:
        add_hoop(track, rng, np.array(a), np.array(b), float(features["hoop_inner_width_m"]))
    return track


def set_side_limits(track: Track, cap_m: float = 1.5) -> None:
    """Raceline limits from the real reach to the wall on each side (capped in open areas)."""
    heading = track.headings()
    normal = np.column_stack([-np.sin(heading), np.cos(heading)])
    offsets = np.arange(0.0, cap_m, track.resolution * 0.5)
    reach = {}
    for side in (1.0, -1.0):
        probe = track.center[:, None, :] + side * offsets[None, :, None] * normal[:, None, :]
        ok = track.is_free(probe.reshape(-1, 2)).reshape(track.center.shape[0], offsets.size)
        reach[side] = np.where(ok.all(axis=1), offsets[-1], offsets[np.argmin(ok, axis=1)])
    margin = V.FOOTPRINT_HALF_WIDTH_M + V.FOOTPRINT_PADDING_M + 0.12
    track.offset_hi = np.maximum(reach[1.0] - margin, 0.0)
    track.offset_lo = -np.maximum(reach[-1.0] - margin, 0.0)


def set_clearance_limits(track: Track, cap_m: float = 1.5, margin_m: float = 0.12) -> None:
    """Raceline limits from 2-D wall clearance (field maps).

    ``set_side_limits`` and ``half_width`` probe only along each point's
    normal, so a wall end or corner diagonally beside the path is invisible to
    them.  A driven trail that cuts a hairpin toward a wall end then lets the
    raceline clip it (the expert crashed there on a replica-derived map).
    Here every allowed offset keeps the car's margin from the nearest wall in
    any direction; where the path is narrower than that, the raceline is
    pinned to the point of most clearance.
    """
    need = V.FOOTPRINT_HALF_WIDTH_M + V.FOOTPRINT_PADDING_M + margin_m
    dist = cv2.distanceTransform(track.free.astype(np.uint8), cv2.DIST_L2, 5) * track.resolution
    heading = track.headings()
    normal = np.column_stack([-np.sin(heading), np.cos(heading)])
    offsets = np.arange(-cap_m, cap_m + 1e-9, track.resolution * 0.5)
    zero = int(np.argmin(np.abs(offsets)))
    probe = (track.center[:, None, :] + offsets[None, :, None] * normal[:, None, :]).reshape(-1, 2)
    cols = np.floor((probe[:, 0] - track.origin[0]) / track.resolution).astype(np.int64)
    rows = dist.shape[0] - 1 - np.floor((probe[:, 1] - track.origin[1]) / track.resolution).astype(np.int64)
    inside = (cols >= 0) & (cols < dist.shape[1]) & (rows >= 0) & (rows < dist.shape[0])
    clear = np.zeros(probe.shape[0])
    clear[inside] = dist[rows[inside], cols[inside]]
    clear = clear.reshape(track.center.shape[0], offsets.size)
    lo = np.empty(track.center.shape[0])
    hi = np.empty(track.center.shape[0])
    for i, row in enumerate(clear):
        # Only the free stretch around the centreline counts, never space beyond a wall.
        first = last = zero
        while first > 0 and row[first - 1] > 0.0:
            first -= 1
        while last < offsets.size - 1 and row[last + 1] > 0.0:
            last += 1
        ok = np.zeros(offsets.size, dtype=bool)
        ok[first:last + 1] = row[first:last + 1] >= need
        if not ok.any():
            lo[i] = hi[i] = offsets[first + int(np.argmax(row[first:last + 1]))]
            continue
        good = np.flatnonzero(ok)
        a = b = int(good[np.argmin(np.abs(good - zero))])      # the clear stretch nearest the centreline
        while a > 0 and ok[a - 1]:
            a -= 1
        while b < offsets.size - 1 and ok[b + 1]:
            b += 1
        lo[i], hi[i] = offsets[a], offsets[b]
    track.offset_lo, track.offset_hi = lo, hi
    track.clearance_limits = True


FIELD_MAPS_DIR = Path(__file__).resolve().parent / "field_maps"


def field_map_dirs(root: Path = FIELD_MAPS_DIR) -> list[Path]:
    """Saved console maps usable for training (a map, a trail, and the trail is a loop)."""
    import json
    usable = []
    for folder in sorted(root.glob("*")) if root.is_dir() else []:
        info = folder / "info.json"
        if (folder / "map.yaml").is_file() and (folder / "trail.csv").is_file() and info.is_file() \
                and json.loads(info.read_text(encoding="utf-8")).get("loop"):
            usable.append(folder)
    return usable


def trail_centerline(trail_xy: np.ndarray, spacing: float = 0.05) -> np.ndarray:
    """A driven path, closed and smoothed into a uniformly spaced loop."""
    from scipy.interpolate import splev, splprep
    keep = [0]
    for i in range(1, len(trail_xy)):                          # drop stationary jitter
        if np.hypot(*(trail_xy[i] - trail_xy[keep[-1]])) >= 0.15:
            keep.append(i)
    pts = trail_xy[keep]
    if np.hypot(*(pts[-1] - pts[0])) < 0.15:
        pts = pts[:-1]
    tck, _ = splprep([pts[:, 0], pts[:, 1]], s=len(pts) * 0.004, per=True)
    dense = np.column_stack(splev(np.linspace(0, 1, 8 * len(pts), endpoint=False), tck))
    return resample_closed(dense, spacing)


def load_field_map(folder: Path, rng: np.random.Generator, out_dir: Path, name: str) -> Track:
    """A place the real car explored: its saved map, the driven loop as the route,
    a random direction and a few random boxes.  Unknown map cells count as walls."""
    import csv
    with open(folder / "trail.csv", newline="", encoding="utf-8") as handle:
        trail = np.array([(float(r["x_m"]), float(r["y_m"])) for r in csv.DictReader(handle)])
    center = trail_centerline(trail)
    if rng.random() < 0.5:
        center = center[::-1].copy()
    track = load_map_track(name, folder / "map.yaml", center, out_dir)
    track.width_limits_speed = True
    set_clearance_limits(track)
    wide = track.half_width >= 0.45
    if wide.any() and rng.random() < 0.7:
        add_obstacles(track, rng, int(rng.integers(1, 5)), min_separation_m=2.0, eligible=wide)
    return track


def add_hoop(track: Track, rng: np.random.Generator, a: np.ndarray, b: np.ndarray, inner_m: float,
             post_m: float = 0.05, max_offset_m: float = 0.35) -> None:
    """A hoop on the dashed line a-b: two posts the car must pass between.

    The rules allow any point on the line, but with this car's right-steering
    limit (1.09 m radius) positions far from the natural line (e.g. hoop 3 at
    the inner end of the right loop) are not drivable, so the hoop centre is
    kept within ``max_offset_m`` of the expert's natural (unobstructed) line.
    """
    n = track.center.shape[0]
    ds = track.spacing
    length = float(np.linalg.norm(b - a))
    direction = (b - a) / max(length, 1e-6)
    half_span = 0.5 * inner_m + post_m
    if length < 2.0 * half_span:
        return
    headings = track.headings()
    normals = np.column_stack([-np.sin(headings), np.cos(headings)])
    lo0 = track.offset_lo if track.offset_lo is not None else -track.half_width
    hi0 = track.offset_hi if track.offset_hi is not None else track.half_width
    natural = np.clip(np.zeros(len(headings)), lo0, hi0)
    for _ in range(1500):
        points = track.center + natural[:, None] * normals
        target = 0.5 * (np.roll(points, 1, axis=0) + np.roll(points, -1, axis=0))
        natural = np.clip(natural + 0.5 * np.einsum("ij,ij->i", target - points, normals), lo0, hi0)
    for _ in range(40):
        middle = a + direction * float(rng.uniform(half_span, length - half_span))
        index = int(np.argmin(np.sum((track.center - middle) ** 2, axis=1)))
        heading = headings[index]
        normal = np.array([-math.sin(heading), math.cos(heading)])
        tangent = np.array([math.cos(heading), math.sin(heading)])
        c = float(np.dot(middle - track.center[index], normal))
        if abs(c - natural[index]) <= max_offset_m:
            break
    else:
        return
    allow = 0.5 * inner_m - (V.FOOTPRINT_HALF_WIDTH_M + V.FOOTPRINT_PADDING_M) - 0.02
    lo = track.offset_lo.copy() if track.offset_lo is not None else -track.half_width.copy()
    hi = track.offset_hi.copy() if track.offset_hi is not None else track.half_width.copy()
    core = V.FOOTPRINT_FRONT_M + 0.3
    reach = int(math.ceil((core + math.sqrt(2.0 * SWERVE_RADIUS_M * 1.5)) / ds))
    steps = np.arange(-reach, reach + 1)
    span = (index + steps) % n
    relax = np.maximum(np.abs(steps) * ds - core, 0.0) ** 2 / (2.0 * SWERVE_RADIUS_M)
    new_lo = np.maximum(lo[span], c - allow - relax)
    new_hi = np.minimum(hi[span], c + allow + relax)
    if np.any(new_lo > new_hi):
        return
    lo[span], hi[span] = new_lo, new_hi
    track.offset_lo, track.offset_hi = lo, hi
    for side in (1.0, -1.0):
        post = middle + side * direction * (0.5 * inner_m + 0.5 * post_m)
        corners = np.array([post + (sx * tangent + sy * normal) * post_m / 2.0
                            for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))])
        track.obstacles.append((post, corners))
    image = np.where(track.free, 255, 0).astype(np.uint8)
    rows = image.shape[0]
    for _, corners in track.obstacles[-2:]:
        px = np.column_stack([(corners[:, 0] - track.origin[0]) / track.resolution,
                              rows - 1 - (corners[:, 1] - track.origin[1]) / track.resolution])
        cv2.fillPoly(image, [np.round(px).astype(np.int32).reshape(-1, 1, 2)], 0)
    cv2.imwrite(track.map_stem + ".png", image)
    track.free = image > 128


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
