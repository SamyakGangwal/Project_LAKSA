"""Obstacle-course sensing: LiDAR/camera speckle, camera lag compensation, recovery debounce."""

import math
import unittest

import numpy as np

from laksa_learned_driver.perception import PerceptionConfig, ego_compensate, filter_cloud
from laksa_learned_driver.recovery import RecoveryConfig, ReverseRecovery
from laksa_learned_driver.scan_adapter import drop_isolated_returns


class LidarDespeckleTest(unittest.TestCase):
    def test_single_short_return_is_dropped(self):
        r = np.full(1600, 4.0)
        r[800] = 0.6                                  # one spurious beam
        out = drop_isolated_returns(r)
        self.assertTrue(np.isnan(out[800]))
        self.assertTrue(np.all(np.isfinite(np.delete(out, 800))))

    def test_hay_bale_wall_and_thin_pole_stay(self):
        r = np.full(1600, np.inf)
        r[100:300] = np.linspace(1.0, 1.4, 200)       # bale face, many beams
        r[900:903] = 1.5                              # 3 cm pole at 1.5 m: ~1.1 deg = 3+ beams
        out = drop_isolated_returns(r)
        self.assertTrue(np.all(np.isfinite(out[100:300])))
        self.assertTrue(np.all(np.isfinite(out[900:903])))

    def test_no_return_stays_inf(self):
        out = drop_isolated_returns(np.array([np.inf, np.inf, 2.0, 2.01, np.inf]))
        self.assertTrue(np.isinf(out[0]))
        self.assertEqual(out[2], 2.0)


class CameraSpeckleTest(unittest.TestCase):
    def test_single_voxel_dropped_wall_kept(self):
        cfg = PerceptionConfig()
        speck = np.array([[1.0, 0.0, 0.2]])
        ys, zs = np.meshgrid(np.arange(-0.5, 0.5, 0.05), np.arange(0.06, 0.40, 0.05))
        wall = np.column_stack([np.full(ys.size, 2.0), ys.ravel(), zs.ravel()])
        xy = filter_cloud(np.vstack([speck, wall]), cfg)
        self.assertFalse(np.any(np.abs(xy[:, 0] - 1.0) < 0.05))
        self.assertGreater(np.sum(np.abs(xy[:, 0] - 2.0) < 0.05), 15)

    def test_small_object_with_height_kept(self):
        cfg = PerceptionConfig()
        post = np.array([[1.2, 0.01, z] for z in (0.08, 0.13, 0.18)])   # one cell, several heights
        self.assertEqual(filter_cloud(post, cfg).shape[0], 1)


class EgoCompensationTest(unittest.TestCase):
    def test_straight_motion_brings_obstacle_closer(self):
        pts = np.array([[2.0, 0.0]])
        out = ego_compensate(pts, [(0.0, 1.0, 0.0)], 10.0, 10.4)
        self.assertAlmostEqual(out[0, 0], 1.6, places=6)
        self.assertAlmostEqual(out[0, 1], 0.0, places=6)

    def test_only_motion_after_capture_counts(self):
        cmds = [(9.0, 1.0, 0.0), (10.2, 0.0, 0.0)]     # stopped 0.2 s after capture
        out = ego_compensate(np.array([[2.0, 0.0]]), cmds, 10.0, 10.5)
        self.assertAlmostEqual(out[0, 0], 1.8, places=6)

    def test_turning_left_moves_point_right(self):
        out = ego_compensate(np.array([[2.0, 0.0]]), [(0.0, 1.0, 0.3)], 0.0, 0.4)
        self.assertLess(out[0, 1], 0.0)
        self.assertLess(out[0, 0], 2.0)

    def test_no_history_no_change(self):
        pts = np.array([[2.0, 0.5]])
        np.testing.assert_allclose(ego_compensate(pts, [], 0.0, 0.5), pts)


class RecoveryDebounceTest(unittest.TestCase):
    def setUp(self):
        self.cfg = RecoveryConfig(confirm_steps=3, retry_s=3.0)
        self.rec = ReverseRecovery(self.cfg)
        self.fwd = (0.6, 0.0)

    def test_one_phantom_scan_never_reverses(self):
        self.assertEqual(self.rec.step(0.0, True, 4.0, 0.0, self.fwd)[2], "HOLD")
        self.assertEqual(self.rec.step(0.1, False, 4.0, 0.0, self.fwd), (0.6, 0.0, "LEARNED_DRIVING"))
        for i in range(2, 30):
            self.assertNotIn("RECOVERY", self.rec.step(i * 0.1, i % 2 == 0, 4.0, 0.0, self.fwd)[2])

    def test_persistent_block_starts_recovery(self):
        statuses = [self.rec.step(i * 0.1, True, 4.0, -1.0, self.fwd)[2] for i in range(3)]
        self.assertEqual(statuses, ["HOLD", "HOLD", "RECOVERY_PAUSE"])

    def test_clear_during_pause_drives_on_without_reversing(self):
        for i in range(3):
            self.rec.step(i * 0.1, True, 4.0, -1.0, self.fwd)
        self.assertEqual(self.rec.step(0.35, False, 4.0, 0.0, self.fwd), (0.6, 0.0, "LEARNED_DRIVING"))
        self.assertEqual(len(self.rec._history), 0)

    def test_give_up_resumes_when_path_clears(self):
        for i in range(3):
            self.rec.step(i * 0.1, True, 0.05, -1.0, self.fwd)          # rear blocked too
        self.assertEqual(self.rec.step(1.0, True, 0.05, -1.0, self.fwd)[2], "BLOCKED")
        self.assertEqual(self.rec.step(1.5, True, 0.05, -1.0, self.fwd)[2], "BLOCKED")
        self.assertEqual(self.rec.step(1.6, False, 0.05, 0.0, self.fwd), (0.6, 0.0, "LEARNED_DRIVING"))

    def test_give_up_retries_after_wait(self):
        for i in range(3):
            self.rec.step(i * 0.1, True, 0.05, -1.0, self.fwd)
        self.rec.step(1.0, True, 0.05, -1.0, self.fwd)                  # GIVE_UP at 1.0
        self.assertEqual(self.rec.step(3.5, True, 4.0, -1.0, self.fwd)[2], "BLOCKED")
        self.assertEqual(self.rec.step(4.1, True, 4.0, -1.0, self.fwd)[2], "RECOVERY_PAUSE")

from laksa_learned_driver.perception import (GroundGrid, fit_ground_grid, fit_ground_plane,  # noqa: E402
                                             lidar_ground_mask, lidar_ground_mask_grid)


def _scene(ramp_start=1.5, slope_deg=15.0, bale_x=None, bale_on_ramp=False, seed=0):
    """Depth points: floor, a ramp from ramp_start, optional hay-bale wall face (0.40 m tall)."""
    rng = np.random.default_rng(seed)
    xs, ys = np.meshgrid(np.arange(0.5, 4.0, 0.05), np.arange(-1.5, 1.5, 0.05), indexing="ij")
    x, y = xs.ravel(), ys.ravel()
    t = math.tan(math.radians(slope_deg))
    z = np.where(x > ramp_start, (x - ramp_start) * t, 0.0)
    pts = [np.column_stack([x, y, z + rng.normal(0, 0.01, x.size)])]
    if bale_x is not None:
        keep = pts[0][:, 0] < bale_x                      # the bale hides what is behind it
        pts[0] = pts[0][keep]
        base = (bale_x - ramp_start) * t if (bale_on_ramp and bale_x > ramp_start) else 0.0
        by, bz = np.meshgrid(np.arange(-1.5, 1.5, 0.05), np.arange(0.0, 0.40, 0.05), indexing="ij")
        pts.append(np.column_stack([np.full(by.size, bale_x), by.ravel(), base + bz.ravel()]))
    return np.vstack(pts)


class RampTest(unittest.TestCase):
    def setUp(self):
        self.cfg = PerceptionConfig()

    def test_ramp_is_not_an_obstacle_with_the_grid(self):
        cloud = _scene()
        grid = fit_ground_grid(cloud, self.cfg)
        plane = fit_ground_plane(cloud, self.cfg, np.random.default_rng(0))
        ahead = filter_cloud(cloud, self.cfg, plane, grid)
        on_path = ahead[np.abs(ahead[:, 1]) < 0.5]
        self.assertEqual(on_path.shape[0], 0, on_path[:5])

    def test_single_plane_alone_saw_the_ramp_as_obstacle(self):
        cloud = _scene()
        plane = fit_ground_plane(cloud, self.cfg, np.random.default_rng(0))
        self.assertGreater(filter_cloud(cloud, self.cfg, plane).shape[0], 20)    # the old failure

    def test_lidar_hit_on_ramp_masked_but_not_on_bale(self):
        hit_x = 1.5 + 0.135 / math.tan(math.radians(15.0))
        grid = fit_ground_grid(_scene(), self.cfg)
        self.assertTrue(lidar_ground_mask_grid(np.array([[hit_x, 0.0]]), grid)[0])
        bale_grid = fit_ground_grid(_scene(ramp_start=9.0, bale_x=2.0), self.cfg)
        self.assertFalse(lidar_ground_mask_grid(np.array([[2.0, 0.0], [2.02, 0.1]]), bale_grid).any())

    def test_hay_bale_wall_is_an_obstacle_on_flat_and_on_ramp(self):
        for on_ramp in (False, True):
            with self.subTest(on_ramp=on_ramp):
                cloud = _scene(ramp_start=1.5 if on_ramp else 9.0, bale_x=2.6, bale_on_ramp=on_ramp)
                grid = fit_ground_grid(cloud, self.cfg)
                xy = filter_cloud(cloud, self.cfg, None, grid)
                wall = xy[(np.abs(xy[:, 0] - 2.6) < 0.06) & (np.abs(xy[:, 1]) < 0.5)]
                self.assertGreater(wall.shape[0], 10)
                self.assertFalse(lidar_ground_mask_grid(np.array([[2.6, 0.0]]), grid)[0])

    def test_box_top_beyond_view_of_floor_still_obstacle(self):
        rng = np.random.default_rng(1)
        floor = np.column_stack([rng.uniform(0.5, 3.0, 3000), rng.uniform(-1, 1, 3000), np.zeros(3000)])
        top = np.column_stack([rng.uniform(2.0, 2.5, 300), rng.uniform(-0.25, 0.25, 300), np.full(300, 0.30)])
        cloud = np.vstack([floor[~((floor[:, 0] > 2.0) & (floor[:, 0] < 2.5) & (np.abs(floor[:, 1]) < 0.25))], top])
        xy = filter_cloud(cloud, self.cfg, None, fit_ground_grid(cloud, self.cfg))
        self.assertGreater(np.sum((xy[:, 0] > 2.0) & (xy[:, 0] < 2.5)), 10)

    def test_grid_round_trips_through_json(self):
        grid = fit_ground_grid(_scene(), self.cfg)
        back = GroundGrid.from_dict(grid.to_dict())
        np.testing.assert_allclose(np.nan_to_num(back.ground, nan=-1), np.nan_to_num(grid.ground, nan=-1), atol=1e-3)
        np.testing.assert_array_equal(back.surface, grid.surface)


if __name__ == "__main__":
    unittest.main()
