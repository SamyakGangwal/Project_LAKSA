"""F1TENTH Gym v1.0.0 wrapper with LAKSA geometry, footprint collisions and randomization.

Gym v1.0.0 always ray-casts from the centre of gravity.  The LAKSA LiDAR sits
0.153 m further forward, so this wrapper ray-casts the scan itself from the
real sensor pose after each control period.
"""

from __future__ import annotations

import math
import sys
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from laksa_learned_driver.scan_features import ScanContract, bin_scan  # noqa: E402

from f1tenth_gym.envs.f110_env import F110Env  # noqa: E402
from f1tenth_gym.envs.track import Raceline, Track as GymTrack, TrackSpec  # noqa: E402

from tracks import Track  # noqa: E402
import vehicle as V  # noqa: E402

GYM_BEAMS = 1080
GYM_FOV = 4.7
GYM_ANGLES = -GYM_FOV / 2.0 + np.arange(GYM_BEAMS) * (GYM_FOV / (GYM_BEAMS - 1))


@dataclass
class Domain:
    """Per-episode sim-to-real randomization."""
    mu: float = 1.0489
    scan_noise_m: float = 0.0
    dropout: float = 0.0
    steer_bias_rad: float = 0.0
    speed_gain: float = 1.0
    extra_latency_steps: int = 0

    @classmethod
    def sample(cls, rng: np.random.Generator) -> "Domain":
        return cls(
            mu=float(rng.uniform(0.75, 1.1)),
            scan_noise_m=float(rng.uniform(0.0, 0.03)),
            dropout=float(rng.uniform(0.0, 0.04)),
            steer_bias_rad=float(rng.uniform(-0.02, 0.02)),
            speed_gain=float(rng.uniform(0.85, 1.1)),
            extra_latency_steps=int(rng.integers(0, 2)),
        )


def _footprint_samples() -> np.ndarray:
    front = V.FOOTPRINT_FRONT_M + V.FOOTPRINT_PADDING_M
    rear = -(V.FOOTPRINT_REAR_M + V.FOOTPRINT_PADDING_M)
    half = V.FOOTPRINT_HALF_WIDTH_M + V.FOOTPRINT_PADDING_M
    xs = np.linspace(rear, front, 13)
    ys = np.linspace(-half, half, 7)
    return np.vstack([
        np.column_stack([xs, np.full_like(xs, half)]),
        np.column_stack([xs, np.full_like(xs, -half)]),
        np.column_stack([np.full_like(ys, front), ys]),
        np.column_stack([np.full_like(ys, rear), ys]),
    ])


FOOTPRINT = _footprint_samples()


def gym_track(track: Track) -> GymTrack:
    spec = TrackSpec(name=track.name, image=Path(track.map_stem).name + ".png",
                     resolution=track.resolution, origin=(track.origin[0], track.origin[1], 0.0),
                     negate=0, occupied_thresh=0.65, free_thresh=0.196)
    occupancy = np.flipud(track.free).astype(np.float32) * 255.0   # gym expects row 0 = min y
    step = max(1, int(round(0.5 / track.spacing)))
    ref = track.center[::step]
    centerline = Raceline(xs=ref[:, 0].copy(), ys=ref[:, 1].copy(), velxs=np.ones(ref.shape[0]))
    return GymTrack(spec=spec, occupancy_map=occupancy, filepath=track.map_stem, ext=".png",
                    centerline=centerline, raceline=centerline)


class LaksaSim:
    def __init__(self, contract: ScanContract, seed: int = 0) -> None:
        self.contract = contract
        self.rng = np.random.default_rng(seed)
        self.env: F110Env | None = None
        self.track: Track | None = None
        self.domain = Domain()
        self._seed = seed

    def load(self, track: Track, domain: Domain) -> None:
        params = dict(V.GYM_PARAMS, mu=domain.mu)
        if self.env is None or self.track is None or self.track.map_stem != track.map_stem:
            self.env = F110Env(config={
                "map": gym_track(track), "params": params, "num_agents": 1, "seed": self._seed,
                "timestep": V.PHYSICS_DT_S, "integrator": "rk4", "model": "st",
                "control_input": ["speed", "steering_angle"],
                "observation_config": {"type": "original"},
                "reset_config": {"type": "cl_grid_static"},
            }, render_mode=None)
        else:
            self.env.configure({"params": params})
        self.track = track
        self.domain = domain

    def reset(self, x: float, y: float, yaw: float) -> dict:
        # Gym poses are the centre of gravity; (x, y) here is the rear axle.
        cog = (x + V.LR_M * math.cos(yaw), y + V.LR_M * math.sin(yaw))
        obs, _ = self.env.reset(options={"poses": np.array([[cog[0], cog[1], yaw]])})
        self._actions = deque([(0.0, 0.0)] * (1 + self.domain.extra_latency_steps))
        return self._observe(obs, False)

    def _lidar_scan(self, rear: np.ndarray, yaw: float) -> np.ndarray:
        lidar = rear + V.LIDAR_X_FROM_BASE_M * np.array([math.cos(yaw), math.sin(yaw)])
        scanner = self.env.sim.agents[0].scan_simulator
        return np.asarray(scanner.scan(np.array([lidar[0], lidar[1], yaw]), None), dtype=np.float64)

    def _observe(self, obs: dict, gym_collision: bool) -> dict:
        yaw = float(obs["poses_theta"][0])
        cog = np.array([obs["poses_x"][0], obs["poses_y"][0]], dtype=np.float64)
        rear = cog - V.LR_M * np.array([math.cos(yaw), math.sin(yaw)])
        c, s = math.cos(yaw), math.sin(yaw)
        world = rear + FOOTPRINT @ np.array([[c, s], [-s, c]])
        collided = bool(gym_collision) or not bool(np.all(self.track.is_free(world)))
        ranges = self._lidar_scan(rear, yaw)
        d = self.domain
        if d.scan_noise_m > 0.0:
            ranges = ranges + self.rng.normal(0.0, d.scan_noise_m, ranges.shape)
        if d.dropout > 0.0:
            ranges[self.rng.random(ranges.shape) < d.dropout] = np.inf
        return {
            "binned": bin_scan(ranges, GYM_ANGLES, self.contract),
            "rear_xy": rear,
            "yaw": yaw,
            "speed": float(obs["linear_vels_x"][0]),
            "collided": collided,
        }

    def step(self, steering: float, speed: float) -> dict:
        d = self.domain
        self._actions.append((steering + d.steer_bias_rad, speed * d.speed_gain))
        applied = self._actions.popleft()
        action = np.array([[float(np.clip(applied[0], -V.STEER_RIGHT_MAX_RAD, V.STEER_LEFT_MAX_RAD)),
                            float(applied[1])]])
        collision = False
        obs = None
        for _ in range(int(round(V.CONTROL_PERIOD_S / V.PHYSICS_DT_S))):
            obs, _, _, _, _ = self.env.step(action)
            if obs["collisions"][0]:
                collision = True
                break
        return self._observe(obs, collision)


class Progress:
    """Signed arc length travelled along the track centerline."""

    def __init__(self, track: Track, rear_xy: np.ndarray) -> None:
        self.center = track.center
        self.spacing = track.spacing
        self.n = self.center.shape[0]
        self.index = int(np.argmin(np.sum((self.center - rear_xy) ** 2, axis=1)))
        self.distance = 0.0

    def update(self, rear_xy: np.ndarray) -> float:
        candidates = (self.index + np.arange(-60, 200)) % self.n
        new = int(candidates[int(np.argmin(np.sum((self.center[candidates] - rear_xy) ** 2, axis=1)))])
        delta = (new - self.index + self.n // 2) % self.n - self.n // 2
        self.index = new
        self.distance += delta * self.spacing
        return self.distance

    @property
    def length(self) -> float:
        return self.n * self.spacing
