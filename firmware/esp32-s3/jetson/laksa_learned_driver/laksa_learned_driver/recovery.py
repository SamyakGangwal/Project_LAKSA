"""Bounded reverse-away recovery when the forward path is blocked.

State machine used by the learned driver:

  FORWARD  --(forward path blocked)-->  PAUSE_IN  (brief stop before reversing)
  PAUSE_IN --(rear clear)-->            REVERSE   (slow, timed, nose swings away
                                                    from the obstacle)
  REVERSE  --(time up / rear blocked)--> PAUSE_OUT --> FORWARD
  any      --(too many recoveries, or front and rear both blocked)--> GIVE_UP

The rear corridor is checked with the LiDAR's rear-facing returns before and
during every reverse.  GIVE_UP makes the driver report BLOCKED, which the
supervisor treats as "abort autonomy and return to manual".
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from .safety import GovernorConfig


@dataclass(frozen=True)
class RecoveryConfig:
    reverse_speed_mps: float = 0.12
    reverse_time_s: float = 2.5          # ~0.3 m at the reverse speed
    reverse_steer_left_rad: float = 0.30
    reverse_steer_right_rad: float = 0.25
    pause_s: float = 0.5                 # stop between direction changes
    rear_clearance_m: float = 0.30       # required free space behind the bumper
    rear_corridor_extra_m: float = 0.08  # widen the rear corridor for the arc
    max_recoveries: int = 4
    recovery_window_s: float = 30.0


def rear_free_distance(points_xy: np.ndarray, gov: GovernorConfig, cfg: RecoveryConfig,
                       rear_overhang_m: float = 0.149) -> float:
    """Distance behind the rear bumper to the nearest return in the reverse corridor."""
    if points_xy.size == 0:
        return gov.horizon_m
    half = gov.half_width_m + gov.lateral_margin_m + cfg.rear_corridor_extra_m
    behind = (points_xy[:, 0] < -rear_overhang_m) & (np.abs(points_xy[:, 1]) <= half)
    if not np.any(behind):
        return gov.horizon_m
    return float(np.min(-points_xy[behind, 0]) - rear_overhang_m)


def obstacle_side(points_xy: np.ndarray, gov: GovernorConfig, look_ahead_m: float = 1.0) -> float:
    """+1 if the nearest forward obstacle is on the left, -1 if on the right."""
    front = gov.front_overhang_m
    band = (points_xy[:, 0] > front) & (points_xy[:, 0] < front + look_ahead_m) \
        & (np.abs(points_xy[:, 1]) <= gov.half_width_m + 0.35)
    if not np.any(band):
        return 0.0
    ahead = points_xy[band]
    nearest = ahead[np.argsort(ahead[:, 0])[:15]]
    return 1.0 if float(np.mean(nearest[:, 1])) >= 0.0 else -1.0


class ReverseRecovery:
    def __init__(self, cfg: RecoveryConfig | None = None) -> None:
        self.cfg = cfg or RecoveryConfig()
        self.state = "FORWARD"
        self._since = 0.0
        self._reverse_steer = 0.0
        self._history: deque[float] = deque()

    def reset(self) -> None:
        self.state = "FORWARD"
        self._history.clear()

    def step(self, now: float, forward_blocked: bool, rear_free_m: float, side: float,
             forward_cmd: tuple[float, float]) -> tuple[float, float, str]:
        """Return (speed_mps, steering_rad, status); negative speed is reverse."""
        cfg = self.cfg
        while self._history and now - self._history[0] > cfg.recovery_window_s:
            self._history.popleft()

        if self.state == "GIVE_UP":
            return 0.0, 0.0, "BLOCKED"

        if self.state == "FORWARD":
            if not forward_blocked:
                return forward_cmd[0], forward_cmd[1], "LEARNED_DRIVING"
            self._history.append(now)
            if len(self._history) > cfg.max_recoveries:
                self.state = "GIVE_UP"
                return 0.0, 0.0, "BLOCKED"
            # Reversing with the wheels turned toward the obstacle swings the
            # nose away from it.
            if side < 0:
                self._reverse_steer = -cfg.reverse_steer_right_rad
            elif side > 0:
                self._reverse_steer = cfg.reverse_steer_left_rad
            else:
                self._reverse_steer = 0.0
            self.state, self._since = "PAUSE_IN", now
            return 0.0, 0.0, "RECOVERY_PAUSE"

        if self.state == "PAUSE_IN":
            if now - self._since < cfg.pause_s:
                return 0.0, self._reverse_steer, "RECOVERY_PAUSE"
            if rear_free_m < cfg.rear_clearance_m:
                self.state = "GIVE_UP"          # boxed in front and rear
                return 0.0, 0.0, "BLOCKED"
            self.state, self._since = "REVERSE", now
            return -cfg.reverse_speed_mps, self._reverse_steer, "RECOVERY_REVERSE"

        if self.state == "REVERSE":
            if now - self._since >= cfg.reverse_time_s or rear_free_m < cfg.rear_clearance_m:
                self.state, self._since = "PAUSE_OUT", now
                return 0.0, 0.0, "RECOVERY_PAUSE"
            return -cfg.reverse_speed_mps, self._reverse_steer, "RECOVERY_REVERSE"

        # PAUSE_OUT
        if now - self._since < cfg.pause_s:
            return 0.0, 0.0, "RECOVERY_PAUSE"
        self.state = "FORWARD"
        if forward_blocked:
            return self.step(now, forward_blocked, rear_free_m, side, forward_cmd)
        return forward_cmd[0], forward_cmd[1], "LEARNED_DRIVING"
