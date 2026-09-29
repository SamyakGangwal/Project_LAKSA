"""LiDAR feature contract shared by simulator training and the Jetson runtime.

Both sides call :func:`bin_scan` with beam angles already expressed in the
vehicle frame (0 rad = forward, positive = left, REP-103) and measured from
the physical LiDAR position.  Each output bin is the *minimum* valid range in
its angular sector, which keeps obstacles conservative when the native beam
density differs between the simulator (0.25 deg) and the RPLIDAR A2M12.

This module must stay NumPy-only: it is imported by the ROS node on the Jetson.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ScanContract:
    fov_rad: float = math.pi
    bins: int = 120
    max_range_m: float = 10.0
    min_range_m: float = 0.05

    def bin_edges(self) -> np.ndarray:
        half = 0.5 * self.fov_rad
        return np.linspace(-half, half, self.bins + 1)

    def to_dict(self) -> dict:
        return {
            "fov_rad": self.fov_rad,
            "bins": self.bins,
            "max_range_m": self.max_range_m,
            "min_range_m": self.min_range_m,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "ScanContract":
        return cls(
            fov_rad=float(data["fov_rad"]),
            bins=int(data["bins"]),
            max_range_m=float(data["max_range_m"]),
            min_range_m=float(data["min_range_m"]),
        )


def wrap_angles(angles: np.ndarray) -> np.ndarray:
    return (angles + np.pi) % (2.0 * np.pi) - np.pi


def bin_scan(ranges, angles, contract: ScanContract) -> np.ndarray:
    """Return ``contract.bins`` ranges in metres, ordered right to left.

    Invalid samples (NaN, below ``min_range_m``) are ignored.  ``inf`` means
    "no return" and is treated as ``max_range_m``.  Empty bins are filled by
    linear interpolation between populated neighbours; if nothing is valid the
    whole scan is reported at ``max_range_m``.
    """
    ranges = np.asarray(ranges, dtype=np.float64).ravel()
    angles = wrap_angles(np.asarray(angles, dtype=np.float64).ravel())
    if ranges.shape != angles.shape:
        raise ValueError("ranges and angles must have the same length")

    ranges = np.where(np.isposinf(ranges), contract.max_range_m, ranges)
    valid = np.isfinite(ranges) & (ranges >= contract.min_range_m)
    half = 0.5 * contract.fov_rad
    valid &= (angles >= -half) & (angles < half)

    out = np.full(contract.bins, np.inf)
    if np.any(valid):
        width = contract.fov_rad / contract.bins
        index = np.floor((angles[valid] + half) / width).astype(np.int64)
        index = np.clip(index, 0, contract.bins - 1)
        np.minimum.at(out, index, np.minimum(ranges[valid], contract.max_range_m))

    filled = np.isfinite(out)
    if not np.any(filled):
        return np.full(contract.bins, contract.max_range_m)
    if not np.all(filled):
        positions = np.arange(contract.bins)
        out = np.interp(positions, positions[filled], out[filled])
    return out


def normalize(binned: np.ndarray, contract: ScanContract) -> np.ndarray:
    return (np.clip(binned, 0.0, contract.max_range_m) / contract.max_range_m).astype(np.float32)
