"""Bounded multi-point turn when the forward path is blocked.

State machine used by the learned driver:

  FORWARD   --(forward path blocked)-->  PAUSE_IN  (brief stop before reversing)
  PAUSE_IN  --(rear clear)-->            REVERSE   (timed, nose swings away
                                                     from the obstacle)
  REVERSE   --(time up / rear blocked)--> PAUSE_OUT
  PAUSE_OUT -->                           TURN      (forward at full opposite lock,
                                                     so the nose keeps swinging)
  TURN      --(time up)-->                FORWARD
  TURN      --(its own arc blocked)-->    PAUSE_IN  (next point of the turn)
  PAUSE_IN  --(path free again)-->        FORWARD   (no reverse for a passing/phantom return)
  any       --(too many recoveries, or front and rear both blocked)--> GIVE_UP
  GIVE_UP   --(path free again)-->        FORWARD;  --(retry_s)--> a fresh recovery

Back up + forward turn changes the heading by roughly 60 degrees, so a dead end
becomes a three-point turn (about three cycles for a U-turn).  Before TURN
existed (until 2026-10-01) the car only backed up and handed straight back to
the learned policy, which drove into the same wall until it gave up.

The rear corridor is checked with the LiDAR's rear-facing returns before and
during every reverse.  GIVE_UP reports BLOCKED; the driver holds the car stopped
(status BLOCKED_WAIT, autonomy stays on) and drives on as soon as the path is free.
"""

from __future__ import annotations

import dataclasses
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from .safety import GovernorConfig, path_free_distance


@dataclass(frozen=True)
class RecoveryConfig:
    reverse_speed_mps: float = 0.12
    reverse_time_s: float = 2.5          # ~0.3 m at the reverse speed
    reverse_steer_left_rad: float = 0.30
    reverse_steer_right_rad: float = 0.25
    pause_s: float = 0.5                 # stop between direction changes
    rear_clearance_m: float = 0.20       # free space kept behind the bumper (along the reversing arc)
    rear_corridor_extra_m: float = 0.08  # widen the rear corridor for the arc
    max_recoveries: int = 8              # a U-turn takes 2-7 cycles (1.2-2 m wide, simulated)
    recovery_window_s: float = 60.0
    turn_speed_mps: float = 0.30         # forward leg of the turn (the drive's minimum)
    turn_time_s: float = 1.5             # ~0.45 m at full lock
    turn_left_rad: float = 0.523         # full lock each way
    turn_right_rad: float = 0.288
    turn_block_m: float = 0.20           # the turn's own arc counts as blocked at this
    commit_s: float = 15.0               # keep turning the same way until driving this long
    # A single LiDAR/camera frame can show a phantom return at the bumper (seen on the
    # car 2026-10-02: 4 m free, 0.0 m for one scan, 4 m again), which started a full
    # back-up at the start line.  The path must stay blocked this many steps in a
    # row before a recovery starts; until then the car only holds (speed 0).
    confirm_steps: int = 1
    # GIVE_UP is not final: it ends as soon as the path ahead is free again, and
    # after this long a fresh recovery is tried (0 = wait for a free path only).
    retry_s: float = 3.0


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


def rear_arc_free_distance(points_xy: np.ndarray, steering_rad: float, gov: GovernorConfig,
                           rear_overhang_m: float = 0.149) -> float:
    """Free distance behind the rear bumper along the reversing arc.

    Reversing with steering s follows the same circle as driving forward with s,
    so mirroring the points front-to-back turns it into the forward check with
    the rear bumper leading.  Side walls count only if the arc reaches them.
    """
    if points_xy.size == 0:
        return gov.horizon_m
    mirrored = points_xy * np.array([-1.0, 1.0])
    return path_free_distance(mirrored, steering_rad, dataclasses.replace(gov, front_overhang_m=rear_overhang_m))


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
        self._turn_steer = 0.0
        self._side = 0.0                     # obstacle side the current turn is built on
        self._last_active = -1e9             # last time any recovery state ran
        self._history: deque[float] = deque()
        self._blocked_steps = 0
        self._continuing = False             # next PAUSE_IN continues a multi-point turn
        self._cancellable = False            # PAUSE_IN may be dropped if the path clears

    def reset(self) -> None:
        self.state = "FORWARD"
        self._history.clear()
        self._blocked_steps = 0
        self._continuing = False

    def _give_up(self, now: float) -> tuple[float, float, str]:
        self.state, self._since = "GIVE_UP", now
        return 0.0, 0.0, "BLOCKED"

    def step(self, now: float, forward_blocked: bool, rear_free_m: float, side: float,
             forward_cmd: tuple[float, float],
             free_on: Callable[[float], float] | None = None,
             rear_free_on: Callable[[float], float] | None = None) -> tuple[float, float, str]:
        """Return (speed_mps, steering_rad, status); negative speed is reverse.

        ``free_on(steering)`` gives the free distance ahead along that arc; without
        it the forward TURN leg is skipped (reverse-only, as before).
        ``rear_free_on(steering)`` gives the free distance behind along the reversing
        arc; without it ``rear_free_m`` (a straight corridor behind) is used.
        """
        if self.state != "FORWARD":
            self._last_active = now
        cfg = self.cfg
        while self._history and now - self._history[0] > cfg.recovery_window_s:
            self._history.popleft()

        if self.state == "GIVE_UP":
            if not forward_blocked:
                self.reset()                    # the way ahead opened up: drive on
                return forward_cmd[0], forward_cmd[1], "LEARNED_DRIVING"
            if cfg.retry_s <= 0.0 or now - self._since < cfg.retry_s:
                return 0.0, 0.0, "BLOCKED"
            self.reset()                        # try a fresh recovery
            self._blocked_steps = cfg.confirm_steps - 1

        if self.state == "FORWARD":
            if not forward_blocked:
                self._blocked_steps = 0
                return forward_cmd[0], forward_cmd[1], "LEARNED_DRIVING"
            self._blocked_steps += 1
            if self._blocked_steps < cfg.confirm_steps:
                return 0.0, forward_cmd[1], "HOLD"
            self._blocked_steps = 0
            self._cancellable, self._continuing = not self._continuing, False
            self._history.append(now)
            if len(self._history) > cfg.max_recoveries:
                return self._give_up(now)
            # Within one manoeuvre keep the first direction: re-deciding each cycle
            # can turn back the other way and undo the heading already gained.
            if self._side != 0.0 and now - self._last_active < cfg.commit_s:
                side = self._side
            self._side = side
            self._last_active = now
            # Reversing with the wheels turned toward the obstacle swings the
            # nose away from it.
            # The forward leg then uses full lock the other way, which keeps the
            # nose swinging in the same direction (a three-point turn).
            if side < 0:
                self._reverse_steer = -cfg.reverse_steer_right_rad
                self._turn_steer = cfg.turn_left_rad
            elif side > 0:
                self._reverse_steer = cfg.reverse_steer_left_rad
                self._turn_steer = -cfg.turn_right_rad
            else:
                self._reverse_steer = 0.0
                self._turn_steer = 0.0
            self.state, self._since = "PAUSE_IN", now
            return 0.0, 0.0, "RECOVERY_PAUSE"

        if rear_free_on is not None:
            rear_free_m = rear_free_on(self._reverse_steer)
        if self.state == "PAUSE_IN":
            if not forward_blocked and self._cancellable:
                # Cleared before backing up (a passing object or a phantom return):
                # no reverse, and it does not count against the recovery budget.
                if self._history:
                    self._history.pop()
                self.state = "FORWARD"
                return forward_cmd[0], forward_cmd[1], "LEARNED_DRIVING"
            if now - self._since < cfg.pause_s:
                return 0.0, self._reverse_steer, "RECOVERY_PAUSE"
            if rear_free_m < cfg.rear_clearance_m:
                return self._give_up(now)       # boxed in front and rear (for now)
            self.state, self._since = "REVERSE", now
            return -cfg.reverse_speed_mps, self._reverse_steer, "RECOVERY_REVERSE"

        if self.state == "REVERSE":
            if now - self._since >= cfg.reverse_time_s or rear_free_m < cfg.rear_clearance_m:
                self.state, self._since = "PAUSE_OUT", now
                return 0.0, 0.0, "RECOVERY_PAUSE"
            return -cfg.reverse_speed_mps, self._reverse_steer, "RECOVERY_REVERSE"

        if self.state == "PAUSE_OUT":
            if now - self._since < cfg.pause_s:
                return 0.0, self._turn_steer, "RECOVERY_PAUSE"
            if free_on is not None and self._turn_steer != 0.0:
                self.state, self._since = "TURN", now
            else:
                self.state = "FORWARD"
                if forward_blocked:
                    self._blocked_steps = cfg.confirm_steps - 1   # already confirmed: next point now
                    self._continuing = True
                    return self.step(now, forward_blocked, rear_free_m, side, forward_cmd, free_on, rear_free_on)
                return forward_cmd[0], forward_cmd[1], "LEARNED_DRIVING"

        # TURN: forward at full lock until time is up or its own arc is blocked.
        if free_on(self._turn_steer) <= cfg.turn_block_m:
            self.state = "FORWARD"
            self._blocked_steps = cfg.confirm_steps - 1   # already confirmed: next point now
            self._continuing = True
            return self.step(now, True, rear_free_m, side, forward_cmd, free_on, rear_free_on)
        if now - self._since >= cfg.turn_time_s:
            self.state = "FORWARD"
            if forward_blocked:
                self._blocked_steps = cfg.confirm_steps - 1   # already confirmed: next point now
                self._continuing = True
                return self.step(now, forward_blocked, rear_free_m, side, forward_cmd, free_on, rear_free_on)
            return forward_cmd[0], forward_cmd[1], "LEARNED_DRIVING"
        return cfg.turn_speed_mps, self._turn_steer, "RECOVERY_TURN"
