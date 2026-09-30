"""Track geometry tests (no simulator needed).

Run from this folder: python -m unittest test_tracks
"""

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import tracks  # noqa: E402
import vehicle as V  # noqa: E402

RES = 0.02
NEED = V.FOOTPRINT_HALF_WIDTH_M + V.FOOTPRINT_PADDING_M + 0.12


def corridor_track(walls: list[tuple[float, float, float, float]]) -> tracks.Track:
    """A 6 x 4 m map, free except the given wall rectangles (x0, y0, x1, y1); centreline y = 2 m."""
    free = np.ones((int(4.0 / RES), int(6.0 / RES)), dtype=bool)
    for x0, y0, x1, y1 in walls:
        free[free.shape[0] - int(round(y1 / RES)):free.shape[0] - int(round(y0 / RES)),
             int(round(x0 / RES)):int(round(x1 / RES))] = False
    center = np.column_stack([np.arange(0.5, 5.5, 0.05), np.full(100, 2.0)])
    return tracks.Track("test", center, np.full(100, 1.0), "unused", RES, (0.0, 0.0), free)


def clearance(track: tracks.Track, points: np.ndarray) -> np.ndarray:
    dist = cv2.distanceTransform(track.free.astype(np.uint8), cv2.DIST_L2, 5) * RES
    cols = np.floor(points[:, 0] / RES).astype(int)
    rows = dist.shape[0] - 1 - np.floor(points[:, 1] / RES).astype(int)
    return dist[rows, cols]


class ClearanceLimitsTest(unittest.TestCase):
    def test_wall_end_beside_the_path_limits_the_raceline(self):
        # Corridor 1.2 m wide, and a thin wall stub from the top wall down to 0.35 m above
        # the centreline at x = 3 m: probes along the normal at x != 3 never touch it.
        track = corridor_track([(0, 0, 6, 1.4), (0, 2.6, 6, 4), (2.98, 2.35, 3.02, 2.6)])
        tracks.set_clearance_limits(track)
        inner = slice(5, 95)
        for offsets in (track.offset_lo, track.offset_hi):
            points = track.center[inner] + offsets[inner, None] * np.array([0.0, 1.0])
            self.assertGreaterEqual(clearance(track, points).min(), NEED - 2 * RES)
        near = int(np.argmin(np.abs(track.center[:, 0] - 2.85)))
        self.assertLess(track.offset_hi[near], 0.6 - NEED - 0.05)          # the stub, 0.15 m ahead, counts

    def test_narrow_path_never_uses_space_beyond_a_wall(self):
        # A 0.5 m path (narrower than the car's margin on both sides) with open space
        # beyond thin walls: the raceline stays in the path, pinned near its middle.
        track = corridor_track([(0, 1.7, 6, 1.75), (0, 2.25, 6, 2.3)])
        tracks.set_clearance_limits(track)
        self.assertTrue(np.all(np.abs(track.offset_lo[5:95]) <= 0.05))
        self.assertTrue(np.all(np.abs(track.offset_hi[5:95]) <= 0.05))


if __name__ == "__main__":
    unittest.main()
