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
    # A cell counts as an obstacle only with >= 2 cloud points in it (an object
    # with some height) or an occupied neighbour cell; single-voxel depth speckle
    # once read as an obstacle 0.01 m ahead.  Hay-bale walls fill many cells.
    min_cell_points: int = 2
    # Local ground grid (ramps): ground height per cell = a low percentile of the
    # depth points in it, limited to what a ramp of at most ground_max_slope_rad
    # could reach from the car and from neighbouring cells.  A single plane fitted
    # the flat floor in front of a ramp, so the ramp itself read as an obstacle
    # (camera) and the LiDAR hitting it as a wall: the car would not drive up.
    grid_cell_m: float = 0.25
    grid_max_x_m: float = 4.0
    grid_half_y_m: float = 2.0
    grid_max_z_m: float = 1.2
    grid_min_points: int = 5
    grid_percentile: float = 15.0
    surface_max_spread_m: float = 0.12   # a cell this flat is a surface (ramp), not a wall face
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


@dataclass(frozen=True)
class GroundGrid:
    x0: float
    y0: float
    cell: float
    ground: np.ndarray        # (nx, ny) ground height, NaN = no data
    surface: np.ndarray       # (nx, ny) True where the cell's points form a surface (ramp/floor)

    def to_dict(self) -> dict:
        return {"x0": self.x0, "y0": self.y0, "cell": self.cell,
                "ground": [[None if not np.isfinite(v) else round(float(v), 3) for v in row] for row in self.ground],
                "surface": self.surface.astype(int).tolist()}

    @staticmethod
    def from_dict(data: dict) -> "GroundGrid":
        ground = np.array([[np.nan if v is None else float(v) for v in row] for row in data["ground"]], dtype=float)
        return GroundGrid(float(data["x0"]), float(data["y0"]), float(data["cell"]), ground,
                          np.asarray(data["surface"], dtype=bool).reshape(ground.shape))

    def lookup(self, x, y) -> tuple[np.ndarray, np.ndarray]:
        """(ground height, surface flag) at each point; NaN/False outside or unknown.

        The height is interpolated bilinearly between cell centres (known cells
        only), so it follows a ramp inside a cell; the point's own cell must be
        known.
        """
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        nx, ny = self.ground.shape
        ix = np.floor((x - self.x0) / self.cell).astype(np.int64)
        iy = np.floor((y - self.y0) / self.cell).astype(np.int64)
        inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        g = np.full(x.shape, np.nan)
        surf = np.zeros(x.shape, dtype=bool)
        if not np.any(inside):
            return g, surf
        own = np.full(x.shape, np.nan)
        own[inside] = self.ground[ix[inside], iy[inside]]
        surf[inside] = self.surface[ix[inside], iy[inside]]
        fx = (x - self.x0) / self.cell - 0.5
        fy = (y - self.y0) / self.cell - 0.5
        x0i, y0i = np.floor(fx).astype(np.int64), np.floor(fy).astype(np.int64)
        tx, ty = fx - x0i, fy - y0i
        num = np.zeros(x.shape)
        den = np.zeros(x.shape)
        for dx, wx in ((0, 1.0 - tx), (1, tx)):
            for dy, wy in ((0, 1.0 - ty), (1, ty)):
                cx = np.clip(x0i + dx, 0, nx - 1)
                cy = np.clip(y0i + dy, 0, ny - 1)
                v = self.ground[cx, cy]
                w = wx * wy * np.isfinite(v)
                num += w * np.nan_to_num(v)
                den += w
        known = inside & np.isfinite(own)
        g[known] = np.where(den[known] > 1e-9, num[known] / np.maximum(den[known], 1e-9), own[known])
        return g, surf


def fit_ground_grid(points_xyz: np.ndarray, cfg: PerceptionConfig) -> GroundGrid | None:
    if points_xyz.size == 0:
        return None
    p = points_xyz[np.all(np.isfinite(points_xyz), axis=1)]
    c = cfg.grid_cell_m
    nx, ny = int(round(cfg.grid_max_x_m / c)), int(round(2 * cfg.grid_half_y_m / c))
    x0, y0 = 0.0, -cfg.grid_half_y_m
    ix = np.floor((p[:, 0] - x0) / c).astype(np.int64)
    iy = np.floor((p[:, 1] - y0) / c).astype(np.int64)
    keep = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny) & (p[:, 2] <= cfg.grid_max_z_m) & (p[:, 2] >= -cfg.grid_max_z_m)
    if not np.any(keep):
        return None
    flat = ix[keep] * ny + iy[keep]
    z = p[keep, 2]
    order = np.argsort(flat, kind="stable")
    flat, z = flat[order], z[order]
    cells, starts, counts = np.unique(flat, return_index=True, return_counts=True)
    raw = np.full(nx * ny, np.nan)
    spread = np.full(nx * ny, np.inf)
    for cell_id, start, count in zip(cells, starts, counts):
        if count < cfg.grid_min_points:
            continue
        lo, mid, hi = np.percentile(z[start:start + count], [cfg.grid_percentile, 50.0,
                                                              100.0 - cfg.grid_percentile])
        # A surface (ramp/floor) cell: its median is the height at the cell centre.
        # Anything else (a wall face, clutter): the low percentile is the ground.
        raw[cell_id] = mid if hi - lo <= cfg.surface_max_spread_m else lo
        spread[cell_id] = hi - lo
    raw, spread = raw.reshape(nx, ny), spread.reshape(nx, ny)
    # Highest ground a ramp could reach: from the car (ground 0 at base_footprint)
    # and from each measured neighbour, rising at most tan(max slope) per metre.
    rise = math.tan(cfg.ground_max_slope_rad)
    xc = x0 + (np.arange(nx) + 0.5) * c
    yc = y0 + (np.arange(ny) + 0.5) * c
    bound = np.hypot(*np.meshgrid(xc, yc, indexing="ij")) * rise
    measured = np.isfinite(raw)
    ground = np.where(measured, np.minimum(raw, bound), np.nan)   # unmeasured: fall back to the plane
    for _ in range(nx + ny):
        known = np.where(np.isfinite(ground), ground, np.inf)
        best = bound.copy()
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                shifted = np.full_like(known, np.inf)
                xs = slice(max(dx, 0), nx + min(dx, 0))
                xd = slice(max(-dx, 0), nx + min(-dx, 0))
                ys = slice(max(dy, 0), ny + min(dy, 0))
                yd = slice(max(-dy, 0), ny + min(-dy, 0))
                shifted[xd, yd] = known[xs, ys]
                best = np.minimum(best, shifted + math.hypot(dx, dy) * c * rise)
        updated = np.where(measured, np.minimum(raw, best), np.nan)
        if np.allclose(np.nan_to_num(updated, nan=-9), np.nan_to_num(ground, nan=-9)):
            break
        ground = updated
    surface = np.isfinite(raw) & (spread <= cfg.surface_max_spread_m)
    return GroundGrid(x0, y0, c, ground, surface)


def lidar_ground_mask_grid(points_xy: np.ndarray, grid: GroundGrid | None, lidar_height_m: float = 0.135,
                           tolerance_m: float = 0.03) -> np.ndarray:
    """True for LiDAR returns that land on measured ramp surface at the scan height.

    Stricter than the plane version: the cell must be a surface (small height
    spread), so the face of a hay bale (spread ~0.4 m) is never masked.
    """
    if grid is None or points_xy.size == 0:
        return np.zeros(points_xy.shape[0], dtype=bool)
    g, surface = grid.lookup(points_xy[:, 0], points_xy[:, 1])
    return surface & np.isfinite(g) & (g >= lidar_height_m - tolerance_m)


def lidar_ground_mask(points_xy: np.ndarray, plane, lidar_height_m: float = 0.135,
                      tolerance_m: float = 0.05) -> np.ndarray:
    """True for LiDAR returns explained by rising ground (a slope), not a wall."""
    if plane is None or points_xy.size == 0:
        return np.zeros(points_xy.shape[0], dtype=bool)
    return ground_height(plane, points_xy[:, 0], points_xy[:, 1]) >= lidar_height_m - tolerance_m


def filter_cloud(points_xyz: np.ndarray, cfg: PerceptionConfig, plane=None,
                 grid: GroundGrid | None = None) -> np.ndarray:
    """Obstacle-height cloud points as a voxel-thinned (N, 2) array of x, y.

    Heights are measured from the local ground grid where it has data, else
    from the plane (else z = 0).
    """
    if points_xyz.size == 0:
        return np.empty((0, 2))
    p = points_xyz[np.all(np.isfinite(points_xyz), axis=1)]
    base = ground_height(plane, p[:, 0], p[:, 1])
    if grid is not None:
        local, _ = grid.lookup(p[:, 0], p[:, 1])
        base = np.where(np.isfinite(local), local, base)
    height = p[:, 2] - base
    keep = (height >= cfg.min_height_m) & (height <= cfg.max_height_m) \
        & (p[:, 0] >= cfg.near_field_min_x_m) \
        & (np.hypot(p[:, 0], p[:, 1]) <= cfg.max_range_m)
    xy = p[keep, :2]
    if xy.shape[0] == 0:
        return np.empty((0, 2))
    cells, counts = np.unique(np.floor(xy / cfg.cloud_voxel_m).astype(np.int64), axis=0, return_counts=True)
    if cfg.min_cell_points > 1 and cells.shape[0]:
        occupied = {(int(a), int(b)) for a, b in cells}
        neighbour = np.array([any((a + da, b + db) in occupied for da in (-1, 0, 1) for db in (-1, 0, 1)
                                  if da or db) for a, b in cells], dtype=bool)
        cells = cells[(counts >= cfg.min_cell_points) | neighbour]
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


def ego_compensate(points_xy: np.ndarray, commands, t_from: float, t_to: float,
                   wheelbase_m: float = 0.324) -> np.ndarray:
    """Move points seen at ``t_from`` into the vehicle frame at ``t_to``.

    ``commands`` is a time-ordered sequence of (t, speed_mps, steering_rad), each
    held until the next.  Camera obstacles arrive 0.2-0.6 s after capture; at
    1 m/s that put them up to 0.6 m farther ahead than they really were.
    """
    if points_xy.size == 0 or t_to <= t_from:
        return points_xy
    x = y = th = 0.0
    cmds = list(commands)
    for i, (t0, v, steer) in enumerate(cmds):
        t1 = cmds[i + 1][0] if i + 1 < len(cmds) else t_to
        a, b = max(t0, t_from), min(t1, t_to)
        if b <= a or v == 0.0:
            continue
        dt = b - a
        w = v * math.tan(steer) / wheelbase_m
        if abs(w) < 1e-6:
            x += v * dt * math.cos(th)
            y += v * dt * math.sin(th)
        else:
            x += v / w * (math.sin(th + w * dt) - math.sin(th))
            y += v / w * (math.cos(th) - math.cos(th + w * dt))
            th += w * dt
    c, s_ = math.cos(th), math.sin(th)
    rel = points_xy - np.array([x, y])
    return np.column_stack([c * rel[:, 0] + s_ * rel[:, 1], -s_ * rel[:, 0] + c * rel[:, 1]])
