"""Driving profiles for the three modes (pure logic, testable).

  obstacle  Obstacle Course: fast, steers around and stops for obstacles
  speed     Speed Course: as fast as the model allows on a clear track
  explore   Trial & explore: an operator holds the run; slow, mapping and recording

A profile is applied live by the learned driver (``/laksa/drive_profile``,
JSON).  Values from the web page are clamped to safe ranges here, never
trusted as given.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace

MAX_SPEED_MPS = 3.0            # the learned model's trained maximum
MIN_SPEED_MPS = 0.2            # the drive does not run reliably below ~0.2 m/s
EXPLORE_MAX_SPEED_MPS = 1.0    # trial & explore ceiling (an operator holds the run)


@dataclass(frozen=True)
class DriveProfile:
    mode: str = "explore"
    speed_mps: float = 0.6
    avoid: bool = True               # steer around obstacles on the policy's arc
    avoid_clearance_m: float = 0.6   # free distance that needs no detour
    camera: bool = True              # ZED obstacle layer (low/overhanging objects, people)
    horizon_m: float = 4.0           # clearance-check look-ahead
    reverse: bool = True             # back away when every arc is blocked

    def to_dict(self) -> dict:
        return asdict(self)


DEFAULTS = {
    "explore": DriveProfile(mode="explore", speed_mps=0.6, avoid=True, avoid_clearance_m=0.6, camera=True,
                            horizon_m=4.0, reverse=True),
    "obstacle": DriveProfile(mode="obstacle", speed_mps=2.0, avoid=True, avoid_clearance_m=0.9, camera=True,
                             horizon_m=8.0, reverse=True),
    # The Speed Course is a clear track: the LiDAR clearance check still stops
    # for real obstacles, but the camera layer is off by default because
    # phantom camera obstacles at speed only cost time.
    "speed": DriveProfile(mode="speed", speed_mps=3.0, avoid=True, avoid_clearance_m=1.2, camera=False,
                          horizon_m=10.0, reverse=False),
}
RACE_MODES = ("obstacle", "speed")


def _clamp(value, low: float, high: float, fallback: float) -> float:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return fallback
    if value != value:                        # NaN
        return fallback
    return min(high, max(low, value))


def make_profile(data: dict) -> DriveProfile:
    """Start from the mode's defaults and apply the given overrides, clamped."""
    mode = data.get("mode") if data.get("mode") in DEFAULTS else "explore"
    base = DEFAULTS[mode]
    top = EXPLORE_MAX_SPEED_MPS if mode == "explore" else MAX_SPEED_MPS
    flag = lambda key: bool(data[key]) if isinstance(data.get(key), bool) else getattr(base, key)
    return replace(
        base,
        speed_mps=_clamp(data.get("speed_mps", base.speed_mps), MIN_SPEED_MPS, top, base.speed_mps),
        avoid=flag("avoid"),
        avoid_clearance_m=_clamp(data.get("avoid_clearance_m", base.avoid_clearance_m), 0.3, 2.0,
                                 base.avoid_clearance_m),
        camera=flag("camera"),
        horizon_m=_clamp(data.get("horizon_m", base.horizon_m), 2.0, 12.0, base.horizon_m),
        reverse=flag("reverse"),
    )
