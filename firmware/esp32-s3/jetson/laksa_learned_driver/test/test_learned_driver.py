import ast
import math
from pathlib import Path
import unittest

import numpy as np

from laksa_learned_driver.policy import LearnedDriverPolicy, OutputContract
from laksa_learned_driver.profiles import DEFAULTS, EXPLORE_MAX_SPEED_MPS, make_profile
from laksa_learned_driver.race import MAX_SPEED_MPS, RaceManager
from laksa_learned_driver.signals import Debounce, SignalConfig, read_signals
from laksa_learned_driver.perception import (Detection, PerceptionConfig, box_footprint_points,
                                             filter_cloud, fit_ground_plane, lidar_ground_mask,
                                             person_speed_rule)
from laksa_learned_driver.smoothing import SteeringSmoother
from laksa_learned_driver.recovery import RecoveryConfig, ReverseRecovery, obstacle_side, rear_free_distance
from laksa_learned_driver.safety import (AvoidConfig, GovernorConfig, blocked_distance, choose_steering, govern,
                                         path_free_distance)
from laksa_learned_driver.scan_adapter import LidarMount, scan_to_vehicle_beams
from laksa_learned_driver.scan_features import ScanContract, bin_scan

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "models" / "laksa_tinylidarnet_v5.npz"


class ScanFeatureTest(unittest.TestCase):
    def test_bins_take_minimum_and_ordering_is_right_to_left(self):
        contract = ScanContract(fov_rad=math.pi, bins=4, max_range_m=10.0)
        angles = np.array([-1.5, -1.4, 0.1, 1.5])
        ranges = np.array([3.0, 2.0, 5.0, 7.0])
        out = bin_scan(ranges, angles, contract)
        self.assertEqual(out[0], 2.0)
        self.assertEqual(out[2], 5.0)
        self.assertEqual(out[3], 7.0)
        self.assertAlmostEqual(out[1], 3.5)  # interpolated empty bin

    def test_inf_is_max_range_and_nan_is_ignored(self):
        contract = ScanContract(bins=2, max_range_m=10.0)
        out = bin_scan([np.inf, np.nan, 1.0], [-0.5, 0.5, 0.6], contract)
        np.testing.assert_allclose(out, [10.0, 1.0])

    def test_empty_scan_reports_max_range(self):
        contract = ScanContract(bins=8)
        np.testing.assert_allclose(bin_scan([np.nan], [0.0], contract), np.full(8, contract.max_range_m))


class ScanAdapterTest(unittest.TestCase):
    def test_rear_facing_mount_maps_wall_ahead_to_center(self):
        mount = LidarMount()
        # Lidar angle pi points forward for a connector-rearward A2M12.
        ranges = np.full(360, np.inf)
        ranges[0] = 2.0     # lidar angle -pi (== +pi): forward
        ranges[180] = 1.0   # lidar angle 0: behind the vehicle
        r, a = scan_to_vehicle_beams(ranges, -math.pi, 2 * math.pi / 360, 0.15, 12.0, mount)
        self.assertAlmostEqual(a[0], 0.0, places=6)
        self.assertAlmostEqual(abs(a[180]), math.pi, places=6)
        binned = bin_scan(r, a, ScanContract(bins=121))
        self.assertAlmostEqual(binned[60], 2.0)   # centre bin sees the forward wall only

    def test_self_returns_are_removed(self):
        mount = LidarMount()
        # A 0.1 m return straight back from the lidar lands on the chassis.
        r, a = scan_to_vehicle_beams([0.1, 3.0], 0.0, math.pi / 2, 0.05, 12.0, mount)
        self.assertTrue(math.isnan(r[0]))
        self.assertEqual(r[1], 3.0)

    def test_out_of_range_is_no_return(self):
        r, _ = scan_to_vehicle_beams([20.0, 0.01], 0.0, 0.1, 0.15, 12.0, LidarMount())
        self.assertTrue(math.isinf(r[0]))
        self.assertTrue(math.isnan(r[1]))


class GovernorTest(unittest.TestCase):
    def setUp(self):
        self.cfg = GovernorConfig()

    def test_clear_path_keeps_requested_speed(self):
        points = np.array([[1.0, 1.5], [3.0, -1.2]])   # beside the corridor
        result = govern(points, 0.0, 0.24, self.cfg)
        self.assertFalse(result.blocked)
        self.assertAlmostEqual(result.speed_mps, 0.24)

    def test_obstacle_close_ahead_blocks(self):
        points = np.array([[0.419 + 0.20, 0.0]])       # 0.20 m ahead of the bumper
        result = govern(points, 0.0, 0.24, self.cfg)
        self.assertTrue(result.blocked)
        self.assertEqual(result.speed_mps, 0.0)
        self.assertAlmostEqual(result.free_distance_m, 0.20, places=6)

    def test_speed_allows_stopping_before_obstacle(self):
        free = 0.60
        result = govern(np.array([[0.419 + free, 0.05]]), 0.0, 3.0, self.cfg)
        v = result.speed_mps
        stop = v * self.cfg.latency_s + v * v / (2 * self.cfg.decel_mps2)
        self.assertLess(v, 3.0)
        self.assertAlmostEqual(stop, free - self.cfg.stop_margin_m, places=6)

    def test_turning_arc_sees_obstacle_on_the_curve_only(self):
        radius = 1.2
        steering = math.atan(self.cfg.wheelbase_m / radius)       # left turn
        on_arc = np.array([[radius * math.sin(1.0), radius * (1 - math.cos(1.0))]])
        straight_ahead = np.array([[1.3, -0.6]])
        self.assertTrue(path_free_distance(on_arc, steering, self.cfg) < self.cfg.horizon_m)
        self.assertEqual(path_free_distance(straight_ahead, steering, self.cfg), self.cfg.horizon_m)
        right = path_free_distance(on_arc * np.array([1.0, -1.0]), -steering, self.cfg)
        self.assertAlmostEqual(right, path_free_distance(on_arc, steering, self.cfg), places=6)

    def test_points_behind_are_ignored(self):
        self.assertEqual(path_free_distance(np.array([[-0.5, 0.0]]), 0.0, self.cfg), self.cfg.horizon_m)

    def test_object_beside_the_body_blocks_only_turns_toward_it(self):
        # A wall 8 cm off the left side, alongside the body (live case on the bench).
        wall = np.array([[x, 0.232] for x in np.linspace(0.0, 0.40, 9)])
        self.assertEqual(path_free_distance(wall, 0.0, self.cfg), self.cfg.horizon_m)
        self.assertEqual(path_free_distance(wall, -0.2, self.cfg), self.cfg.horizon_m)   # turning away
        self.assertEqual(path_free_distance(wall, 0.3, self.cfg), 0.0)                   # turning toward

    def test_point_inside_the_footprint_always_blocks(self):
        inside = np.array([[0.30, 0.10]])
        for steering in (-0.2, 0.0, 0.3):
            self.assertEqual(path_free_distance(inside, steering, self.cfg), 0.0)


class MinimumSpeedTest(unittest.TestCase):
    """Car settings: forward commands are >= 0.30 m/s or zero; stop at 1 ft."""

    def setUp(self):
        self.cfg = GovernorConfig(stop_margin_m=0.18, min_speed_mps=0.30)

    def test_blocked_distance_is_one_foot(self):
        self.assertAlmostEqual(blocked_distance(self.cfg), 0.30, places=6)

    def test_never_commands_a_crawl(self):
        for free in np.linspace(0.0, 1.5, 151):
            result = govern(np.array([[0.419 + free, 0.0]]), 0.0, 0.6, self.cfg)
            self.assertTrue(result.speed_mps == 0.0 or result.speed_mps >= 0.30 - 1e-9, (free, result))

    def test_keeps_moving_until_one_foot(self):
        moving = govern(np.array([[0.419 + 0.32, 0.0]]), 0.0, 0.6, self.cfg)
        self.assertFalse(moving.blocked)
        self.assertGreaterEqual(moving.speed_mps, 0.30)
        self.assertLess(moving.speed_mps, 0.6)            # still slowing for it
        stopped = govern(np.array([[0.419 + 0.30, 0.0]]), 0.0, 0.6, self.cfg)
        self.assertTrue(stopped.blocked)
        self.assertEqual(stopped.speed_mps, 0.0)

    def test_full_speed_well_before_the_obstacle(self):
        result = govern(np.array([[0.419 + 0.60, 0.0]]), 0.0, 0.6, self.cfg)
        self.assertAlmostEqual(result.speed_mps, 0.6)

    def test_slow_request_is_raised_to_the_minimum(self):
        # e.g. a person nearby halves a 0.4 m/s cap to 0.2 m/s, which would stall.
        result = govern(np.empty((0, 2)), 0.0, 0.2, self.cfg)
        self.assertAlmostEqual(result.speed_mps, 0.30)
        self.assertEqual(govern(np.empty((0, 2)), 0.0, 0.0, self.cfg).speed_mps, 0.0)

    def test_field_wall_case_is_blocked_so_recovery_runs(self):
        # 2026-10-01: best arc 0.257 m free -> 0.025 m/s for three minutes, never reversing.
        wall = np.array([[0.419 + 0.257, y] for y in np.linspace(-1.5, 1.5, 121)])
        wall = np.vstack([wall, [[x, s * 0.45] for x in np.linspace(0.0, 0.6, 20) for s in (-1, 1)]])
        self.assertTrue(choose_steering(wall, 0.0, self.cfg, AvoidConfig()).all_blocked)
        self.assertTrue(govern(wall, 0.0, 0.6, self.cfg).blocked)


class AvoidTest(unittest.TestCase):
    def setUp(self):
        self.gov = GovernorConfig()
        self.cfg = AvoidConfig()
        # A box 0.40 m ahead of the bumper, centred slightly right of the car's axis.
        self.box = np.array([[0.419 + 0.40, y] for y in np.linspace(-0.25, 0.10, 8)])

    def test_clear_preferred_arc_is_kept(self):
        result = choose_steering(np.array([[3.0, 1.5]]), 0.05, self.gov, self.cfg)
        self.assertEqual(result.steering_rad, 0.05)
        self.assertFalse(result.avoiding)

    def test_steers_around_box_toward_the_open_side(self):
        result = choose_steering(self.box, 0.0, self.gov, self.cfg)
        self.assertTrue(result.avoiding)
        self.assertFalse(result.all_blocked)
        self.assertGreater(result.steering_rad, 0.0)          # open side is left
        self.assertGreaterEqual(result.free_distance_m, self.cfg.clearance_m)
        self.assertGreaterEqual(path_free_distance(self.box, result.steering_rad, self.gov), self.cfg.clearance_m)

    def test_takes_the_smallest_detour(self):
        result = choose_steering(self.box, 0.0, self.gov, self.cfg)
        left_arcs = np.linspace(-self.cfg.steer_right_max_rad, self.cfg.steer_left_max_rad, self.cfg.candidates)
        roomy = [a for a in left_arcs if path_free_distance(self.box, a, self.gov) >= self.cfg.clearance_m]
        self.assertAlmostEqual(result.steering_rad, min(roomy, key=abs), places=9)

    def test_ongoing_detour_is_kept_while_it_has_room(self):
        first = choose_steering(self.box, 0.0, self.gov, self.cfg)
        # The policy now leans slightly right; the left detour still has room and is kept.
        again = choose_steering(self.box, -0.05, self.gov, self.cfg, previous_rad=first.steering_rad)
        self.assertEqual(again.steering_rad, first.steering_rad)

    def test_wall_across_every_arc_is_all_blocked(self):
        wall = np.array([[0.419 + 0.15, y] for y in np.linspace(-1.5, 1.5, 121)])
        wall = np.vstack([wall, [[x, s * 0.45] for x in np.linspace(0.0, 0.6, 20) for s in (-1, 1)]])
        self.assertTrue(choose_steering(wall, 0.0, self.gov, self.cfg).all_blocked)



def _frame(color_bgr=None, box=(100, 60, 140, 100), size=(240, 320)):
    import cv2
    image = np.full(size + (3,), 90, np.uint8)             # grey scene
    if color_bgr is not None:
        cv2.rectangle(image, box[:2], box[2:], color_bgr, -1)
    return image


class SignalTest(unittest.TestCase):
    def test_green_light_is_seen(self):
        reading = read_signals(_frame((40, 230, 40)))
        self.assertTrue(reading.green)
        self.assertFalse(reading.red)

    def test_red_light_is_seen(self):
        reading = read_signals(_frame((30, 30, 230)))
        self.assertTrue(reading.red)
        self.assertFalse(reading.green)

    def test_orange_bucket_is_not_a_stop_signal(self):
        self.assertFalse(read_signals(_frame((0, 140, 255))).red)       # BGR orange

    def test_floor_colours_are_ignored(self):
        self.assertFalse(read_signals(_frame((40, 230, 40), box=(100, 200, 160, 239))).green)

    def test_tiny_specks_are_ignored(self):
        self.assertFalse(read_signals(_frame((40, 230, 40), box=(100, 60, 103, 63))).green)

    def test_debounce_needs_consecutive_frames(self):
        d = Debounce(3)
        self.assertEqual([d.update(x) for x in (True, True, False, True, True, True)],
                         [False, False, False, False, False, True])


class RaceTest(unittest.TestCase):
    def test_green_starts_and_red_stops(self):
        race = RaceManager(min_run_s=3.0)
        self.assertEqual(race.arm("speed", None).profile["speed_mps"], DEFAULTS["speed"].speed_mps)
        self.assertIsNone(race.on_signals(0.0, False, False).autonomy)
        self.assertTrue(race.on_signals(1.0, True, False).autonomy)
        self.assertEqual(race.status.state, "RUNNING")
        self.assertIsNone(race.on_signals(2.0, False, True).autonomy)   # stop ignored in the first 3 s
        self.assertFalse(race.on_signals(5.0, False, True).autonomy)
        self.assertEqual(race.status.state, "FINISHED")

    def test_speed_is_clamped_to_the_trained_range(self):
        race = RaceManager()
        self.assertEqual(race.arm("obstacle", {"speed_mps": 9.0}).profile["speed_mps"], MAX_SPEED_MPS)
        self.assertEqual(RaceManager().arm("obstacle", None).profile["speed_mps"], 2.0)

    def test_explore_is_not_a_race_mode(self):
        with self.assertRaises(ValueError):
            RaceManager().arm("explore", None)


class ProfileTest(unittest.TestCase):
    def test_mode_defaults(self):
        self.assertTrue(DEFAULTS["obstacle"].avoid and DEFAULTS["obstacle"].camera)
        self.assertFalse(DEFAULTS["speed"].reverse)
        self.assertEqual(DEFAULTS["speed"].speed_mps, MAX_SPEED_MPS)

    def test_overrides_are_clamped_and_typed(self):
        p = make_profile({"mode": "obstacle", "speed_mps": "fast", "horizon_m": 99, "camera": "yes"})
        self.assertEqual(p.speed_mps, DEFAULTS["obstacle"].speed_mps)      # not a number: default
        self.assertEqual(p.horizon_m, 12.0)                                  # clamped
        self.assertTrue(p.camera)                                            # non-bool ignored: default

    def test_explore_speed_has_its_own_ceiling(self):
        self.assertEqual(make_profile({"mode": "explore", "speed_mps": 2.5}).speed_mps, EXPLORE_MAX_SPEED_MPS)
        self.assertEqual(make_profile({"mode": "obstacle", "speed_mps": 2.5}).speed_mps, 2.5)

    def test_unknown_mode_falls_back_to_explore(self):
        self.assertEqual(make_profile({"mode": "warp"}).mode, "explore")

    def test_nothing_starts_unless_armed(self):
        race = RaceManager()
        self.assertIsNone(race.on_signals(0.0, True, False).autonomy)
        self.assertEqual(race.status.state, "IDLE")

    def test_emergency_stop_aborts_the_run(self):
        race = RaceManager()
        race.arm("speed", None)
        race.on_signals(0.0, True, False)
        self.assertFalse(race.on_mission(0.5, "EMERGENCY_STOP", "").autonomy)
        self.assertEqual(race.status.state, "ABORTED")

    def test_refused_start_aborts_after_timeout(self):
        race = RaceManager(start_timeout_s=2.0)
        race.arm("speed", None)
        race.on_signals(0.0, True, False)
        self.assertIsNone(race.on_mission(1.0, "MANUAL", "VESC telemetry is stale").autonomy)
        self.assertFalse(race.on_mission(2.5, "MANUAL", "VESC telemetry is stale").autonomy)


class ReverseRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.cfg = RecoveryConfig()
        self.rec = ReverseRecovery(self.cfg)
        self.forward = (0.15, 0.1)

    def test_clear_path_passes_forward_command(self):
        self.assertEqual(self.rec.step(0.0, False, 4.0, 0.0, self.forward), (0.15, 0.1, "LEARNED_DRIVING"))

    def test_blocked_pauses_reverses_then_resumes(self):
        speed, _, status = self.rec.step(0.0, True, 4.0, -1.0, self.forward)
        self.assertEqual((speed, status), (0.0, "RECOVERY_PAUSE"))
        speed, steer, status = self.rec.step(self.cfg.pause_s + 0.01, True, 4.0, -1.0, self.forward)
        self.assertEqual(status, "RECOVERY_REVERSE")
        self.assertLess(speed, 0.0)
        self.assertLess(steer, 0.0)                       # obstacle on the right -> wheels right
        t_end = self.cfg.pause_s + self.cfg.reverse_time_s + 0.02
        self.assertEqual(self.rec.step(t_end, False, 4.0, 0.0, self.forward)[2], "RECOVERY_PAUSE")
        t_resume = t_end + self.cfg.pause_s + 0.01
        self.assertEqual(self.rec.step(t_resume, False, 4.0, 0.0, self.forward), (0.15, 0.1, "LEARNED_DRIVING"))

    def test_obstacle_on_left_turns_wheels_left_while_reversing(self):
        self.rec.step(0.0, True, 4.0, 1.0, self.forward)
        _, steer, _ = self.rec.step(self.cfg.pause_s + 0.01, True, 4.0, 1.0, self.forward)
        self.assertGreater(steer, 0.0)

    def test_rear_blocked_gives_up(self):
        self.rec.step(0.0, True, 0.1, -1.0, self.forward)
        self.assertEqual(self.rec.step(self.cfg.pause_s + 0.01, True, 0.1, -1.0, self.forward)[2], "BLOCKED")

    def test_reverse_stops_when_rear_becomes_blocked(self):
        self.rec.step(0.0, True, 4.0, -1.0, self.forward)
        self.rec.step(self.cfg.pause_s + 0.01, True, 4.0, -1.0, self.forward)
        speed, _, status = self.rec.step(self.cfg.pause_s + 0.5, True, 0.1, -1.0, self.forward)
        self.assertEqual((speed, status), (0.0, "RECOVERY_PAUSE"))

    def test_too_many_recoveries_gives_up(self):
        t = 0.0
        for _ in range(self.cfg.max_recoveries):
            self.rec.step(t, True, 4.0, -1.0, self.forward)
            t += self.cfg.pause_s + 0.01
            self.rec.step(t, True, 4.0, -1.0, self.forward)
            t += self.cfg.reverse_time_s + 0.01
            self.rec.step(t, True, 4.0, -1.0, self.forward)
            t += self.cfg.pause_s + 0.01
        self.assertEqual(self.rec.step(t, True, 4.0, -1.0, self.forward)[2], "BLOCKED")

    def test_rear_free_distance_and_side(self):
        gov = GovernorConfig()
        points = np.array([[-0.149 - 0.5, 0.0], [0.419 + 0.3, -0.1]])
        self.assertAlmostEqual(rear_free_distance(points, gov, self.cfg), 0.5, places=6)
        self.assertEqual(obstacle_side(points, gov), -1.0)


class PerceptionTest(unittest.TestCase):
    def setUp(self):
        self.cfg = PerceptionConfig()

    def test_cloud_keeps_only_obstacle_heights(self):
        cloud = np.array([
            [1.0, 0.0, 0.00],    # floor
            [1.0, 0.2, 0.10],    # low box: obstacle
            [1.5, 0.0, 0.40],    # overhanging edge: obstacle
            [1.5, 0.3, 1.20],    # tabletop high above the car: ignored
            [9.0, 0.0, 0.10],    # beyond range
        ])
        xy = filter_cloud(cloud, self.cfg)
        self.assertEqual(xy.shape[0], 2)
        self.assertTrue(np.all(xy[:, 0] < 2.0))

    def test_box_footprint_covers_the_box(self):
        corners = np.array([[x, y, z] for x in (1.0, 1.4) for y in (-0.2, 0.2) for z in (0.0, 0.5)])
        points = box_footprint_points(corners, 0.0, self.cfg)
        self.assertAlmostEqual(points[:, 0].min(), 1.0, places=6)
        self.assertAlmostEqual(points[:, 1].max(), 0.2, places=6)
        inflated = box_footprint_points(corners, 0.25, self.cfg)
        self.assertLess(inflated[:, 0].min(), 1.0 - 0.1)

    def test_person_rules(self):
        box = np.zeros((8, 3))
        near = Detection("Person", 80.0, np.array([0.419 + 0.6, 0.1, 0.8]), box)
        mid = Detection("Person", 80.0, np.array([0.419 + 1.5, 0.0, 0.8]), box)
        behind = Detection("Person", 80.0, np.array([-1.0, 0.0, 0.8]), box)
        chair = Detection("Bag", 80.0, np.array([0.8, 0.0, 0.2]), box)
        self.assertEqual(person_speed_rule([near], self.cfg)[:2], (0.0, True))
        self.assertEqual(person_speed_rule([mid], self.cfg)[:2], (0.5, False))
        self.assertEqual(person_speed_rule([behind, chair], self.cfg)[:2], (1.0, False))


class SlopeAndSmoothingTest(unittest.TestCase):
    def setUp(self):
        self.cfg = PerceptionConfig()
        rng = np.random.default_rng(1)
        x = rng.uniform(0.6, 3.0, 3000)
        y = rng.uniform(-1.5, 1.5, 3000)
        self.slope = math.radians(12.0)
        z = math.tan(self.slope) * x + rng.normal(0.0, 0.01, x.size)   # ground rising ahead
        self.ground = np.column_stack([x, y, z])

    def test_fits_a_slope(self):
        plane = fit_ground_plane(self.ground, self.cfg, np.random.default_rng(2))
        self.assertIsNotNone(plane)
        self.assertAlmostEqual(math.atan(plane[0]), self.slope, delta=math.radians(1.5))

    def test_slope_is_not_an_obstacle_but_a_box_on_it_is(self):
        plane = fit_ground_plane(self.ground, self.cfg, np.random.default_rng(2))
        box = np.array([[1.5, 0.0, math.tan(self.slope) * 1.5 + 0.15]])
        obstacles = filter_cloud(np.vstack([self.ground, box]), self.cfg, plane)
        self.assertEqual(obstacles.shape[0], 1)
        self.assertAlmostEqual(obstacles[0, 0], 1.5, delta=0.05)
        # Without the plane, the rising ground itself would look like obstacles.
        self.assertGreater(filter_cloud(self.ground, self.cfg, None).shape[0], 50)

    def test_lidar_hit_on_rising_ground_is_slope_not_wall(self):
        plane = (math.tan(self.slope), 0.0, 0.0)
        hit_distance = 0.135 / math.tan(self.slope)          # where the flat LiDAR slice meets the slope
        mask = lidar_ground_mask(np.array([[hit_distance, 0.0], [0.3, 0.0]]), plane)
        self.assertTrue(mask[0])
        self.assertFalse(mask[1])
        self.assertFalse(np.any(lidar_ground_mask(np.array([[hit_distance, 0.0]]), None)))

    def test_near_field_camera_points_ignored(self):
        edge = self.cfg.near_field_min_x_m
        near = np.array([[edge - 0.05, 0.0, 0.10], [edge + 0.05, 0.0, 0.10]])
        xy = filter_cloud(near, self.cfg)
        self.assertEqual(xy.shape[0], 1)
        self.assertGreater(xy[0, 0], edge)

    def test_camera_still_covers_the_stop_distance(self):
        # Camera-only (low) obstacles must stay visible up to the 0.30 m stop line.
        self.assertLess(self.cfg.near_field_min_x_m - 0.419, 0.30)

    def test_steering_smoother_limits_rate_and_filters(self):
        smoother = SteeringSmoother(alpha=0.4, max_rate_radps=1.0)
        out = [smoother.update(0.5 if i % 2 else -0.3, 0.08) for i in range(20)]
        self.assertTrue(all(abs(b - a) <= 1.0 * 0.08 + 1e-9 for a, b in zip(out, out[1:])))
        steady = SteeringSmoother(alpha=0.4, max_rate_radps=1.0)
        for _ in range(60):
            value = steady.update(0.3, 0.08)
        self.assertAlmostEqual(value, 0.3, places=3)


class OutputContractTest(unittest.TestCase):
    def test_asymmetric_steering_round_trip(self):
        contract = OutputContract(0.523, 0.288, 3.0, 0.324)
        for steering in (0.523, 0.2, 0.0, -0.1, -0.288):
            self.assertAlmostEqual(float(contract.decode_steering(contract.encode_steering(steering))), steering)
        self.assertAlmostEqual(float(contract.decode_steering(-5.0)), -0.288)
        self.assertAlmostEqual(float(contract.decode_steering(5.0)), 0.523)


@unittest.skipUnless(MODEL.is_file(), "trained model not present")
class ShippedModelTest(unittest.TestCase):
    def setUp(self):
        self.policy = LearnedDriverPolicy(MODEL)

    def test_outputs_respect_vehicle_limits(self):
        rng = np.random.default_rng(0)
        angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
        for _ in range(50):
            ranges = rng.uniform(0.2, 12.0, angles.size)
            for cap in (0.25, 1.0, 3.0):
                steering, speed = self.policy.act(ranges, angles, cap)
                self.assertGreaterEqual(steering, -0.288 - 1e-9)
                self.assertLessEqual(steering, 0.523 + 1e-9)
                self.assertGreaterEqual(speed, 0.0)
                self.assertLessEqual(speed, cap + 1e-9)

    def test_contract_matches_vehicle(self):
        self.assertEqual(self.policy.output.steering_right_max_rad, 0.288)
        self.assertEqual(self.policy.output.steering_left_max_rad, 0.523)
        self.assertEqual(self.policy.output.wheelbase_m, 0.324)


class RuntimeContractTest(unittest.TestCase):
    def test_node_uses_supervisor_cruise_slot_and_never_actuator_topics(self):
        source = (ROOT / "laksa_learned_driver" / "driver_node.py").read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn('"/laksa/lidar_cruise_cmd_vel"', source)
        for forbidden in ('"/laksa/command"', '"/cmd_vel"', '"/laksa/brake"', "set_drive_command", "set_armed"):
            self.assertNotIn(forbidden, source)

    def test_node_has_runtime_dependencies_only(self):
        for name in ("driver_node.py", "policy.py", "scan_adapter.py", "scan_features.py"):
            source = (ROOT / "laksa_learned_driver" / name).read_text(encoding="utf-8")
            for heavy in ("import torch", "onnxruntime", "f1tenth_gym", "gymnasium"):
                self.assertNotIn(heavy, source)


if __name__ == "__main__":
    unittest.main()
