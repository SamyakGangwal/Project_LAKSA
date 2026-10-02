"""Closed-loop check: avoidance + governor + recovery turn the car around in a dead end.

A kinematic bicycle model drives into corridors closed at the far end, with a
"policy" that always steers straight ahead (the worst case: the learned network
was trained on corridors and keeps aiming at the wall).  The rule layers alone
must turn the car round without the footprint ever touching a wall.
"""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from laksa_learned_driver.recovery import (RecoveryConfig, ReverseRecovery, obstacle_side,  # noqa: E402
                                           rear_arc_free_distance, rear_free_distance)
from laksa_learned_driver.safety import (AvoidConfig, GovernorConfig, choose_steering, govern,  # noqa: E402
                                         path_free_distance)

WHEELBASE = 0.324


def dead_end(width: float, length: float = 2.0) -> np.ndarray:
    xs = np.arange(-3.0, length, 0.03)
    ys = np.arange(-width / 2, width / 2, 0.03)
    return np.vstack([np.column_stack([xs, np.full_like(xs, width / 2)]),
                      np.column_stack([xs, np.full_like(xs, -width / 2)]),
                      np.column_stack([np.full_like(ys, length), ys])])


def drive(width: float, policy_steer: float, seconds: float = 60.0, dt: float = 0.08) -> str:
    gov = GovernorConfig(stop_margin_m=0.18, min_speed_mps=0.30)          # car settings
    avoid = AvoidConfig(clearance_m=1.0)                                  # explore profile
    rec = ReverseRecovery(RecoveryConfig(reverse_speed_mps=0.30, reverse_time_s=1.2))
    world = dead_end(width)
    x = y = th = t = 0.0
    target = None
    while t < seconds:
        c, s = math.cos(th), math.sin(th)
        rel = world - [x, y]
        pts = np.column_stack([c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1]])
        pts = pts[np.hypot(pts[:, 0], pts[:, 1]) < 8.0]
        if np.any((pts[:, 0] > -0.149) & (pts[:, 0] < 0.419) & (np.abs(pts[:, 1]) < 0.148)):
            return "COLLISION"
        a = choose_steering(pts, policy_steer, gov, avoid, target)
        target = a.steering_rad if a.avoiding else None
        g = govern(pts, a.steering_rad, 0.6, gov)
        free_on = lambda arc: path_free_distance(pts, arc, gov)          # noqa: E731
        side = obstacle_side(pts, gov)
        if side == 0.0 and a.all_blocked:                                # as driver_node does
            side = -1.0 if free_on(rec.cfg.turn_left_rad) >= free_on(-rec.cfg.turn_right_rad) else 1.0
        v, steer, status = rec.step(t, a.all_blocked, rear_free_distance(pts, gov, rec.cfg), side,
                                    (g.speed_mps, a.steering_rad), free_on,
                                    lambda arc: rear_arc_free_distance(pts, arc, gov))
        if status == "BLOCKED":
            return "BLOCKED"
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += v * math.tan(steer) / WHEELBASE * dt
        t += dt
        if abs(math.atan2(math.sin(th), math.cos(th))) > math.radians(150) and x < 1.5:
            return "TURNED_AROUND"
    return "TIMEOUT"


class DeadEndUTurnTest(unittest.TestCase):
    def test_turns_around_in_corridors_from_1_2_m(self):
        for width in (1.2, 1.6, 2.0):
            for policy_steer in (0.0, 0.1):
                with self.subTest(width=width, policy_steer=policy_steer):
                    self.assertEqual(drive(width, policy_steer), "TURNED_AROUND")

    def test_too_narrow_gives_up_without_touching_a_wall(self):
        # 1.0 m is tighter than this car's turning circle: it must stop, not crash.
        self.assertEqual(drive(1.0, 0.0), "BLOCKED")


if __name__ == "__main__":
    unittest.main()
