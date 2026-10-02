"""Convert a physical LaserScan into the vehicle-frame beams the policy expects.

The RPLIDAR A2M12 is mounted at ``base_footprint`` x = 0.31542 m with its
connector facing the rear (yaw = pi).  The simulator ray-casts from that same
point along the vehicle heading, so only the yaw needs to be applied here.

``/laksa/lidar/scan_validated`` is derived from the *unfiltered* driver scan,
so returns that fall inside the chassis box used by ``lidar_filters.yaml`` are
removed here; the simulator never sees the vehicle's own body.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .scan_features import wrap_angles


@dataclass(frozen=True)
class LidarMount:
    x_m: float = 0.31542
    y_m: float = 0.0
    yaw_rad: float = float(np.pi)
    self_min_x_m: float = -0.18
    self_max_x_m: float = 0.45
    self_min_y_m: float = -0.21
    self_max_y_m: float = 0.21


def scan_to_vehicle_beams(ranges, angle_min: float, angle_increment: float,
                          range_min: float, range_max: float, mount: LidarMount):
    """Return (ranges, vehicle-frame angles) with invalid and self returns as NaN.

    Out-of-range readings above ``range_max`` and ``inf`` are reported as
    ``inf`` (no return).  Readings below ``range_min`` become NaN (ignored).
    """
    r = np.asarray(ranges, dtype=np.float64).copy()
    lidar_angles = angle_min + angle_increment * np.arange(r.size)
    angles = wrap_angles(lidar_angles + mount.yaw_rad)
    r[np.isnan(r) | (r < max(range_min, 0.0))] = np.nan
    r[np.isfinite(r) & (r > range_max)] = np.inf
    finite = np.isfinite(r)
    x = mount.x_m + np.where(finite, r, 0.0) * np.cos(angles)
    y = mount.y_m + np.where(finite, r, 0.0) * np.sin(angles)
    own_body = finite & (x >= mount.self_min_x_m) & (x <= mount.self_max_x_m) \
        & (y >= mount.self_min_y_m) & (y <= mount.self_max_y_m)
    r[own_body] = np.nan
    return r, angles


def drop_isolated_returns(ranges: np.ndarray, window: int = 2, max_jump_m: float = 0.10) -> np.ndarray:
    """Set returns with no neighbour within ``window`` beams and ``max_jump_m`` to NaN.

    The A2M12 gives ~1600 beams per turn (0.225 deg), so any real object (a hay
    bale, a cone, a 3 cm pole within ~1.7 m) covers several adjacent beams, while
    the spurious short returns seen outdoors are single beams: on 2026-10-02 the
    path read 4 m free, then 0.0-0.2 m for one scan, then 4 m again, and the car
    backed up at the start line.  NaN means "ignored", the same as a self return.
    """
    r = np.asarray(ranges, dtype=np.float64)
    out = r.copy()
    finite = np.isfinite(r)
    has_neighbour = np.zeros(r.size, dtype=bool)
    for k in range(1, window + 1):
        for shifted, valid in ((np.roll(r, k), np.roll(finite, k)), (np.roll(r, -k), np.roll(finite, -k))):
            has_neighbour |= valid & (np.abs(shifted - np.where(finite, r, 0.0)) <= max_jump_m)
    out[finite & ~has_neighbour] = np.nan
    return out
