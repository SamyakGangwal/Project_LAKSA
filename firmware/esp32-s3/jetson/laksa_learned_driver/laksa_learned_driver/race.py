"""Race sequencing: arm -> green signal -> drive -> stop signal (pure logic, testable).

Two race modes share the learned driver and differ in their drive profile
(see profiles.py): ``obstacle`` (fast, avoids obstacles) and ``speed`` (as
fast as the model allows).

The manager never commands the motor: it asks drive_supervisor to enter
LIDAR_CRUISE (the learned driver's slot) and to leave it again, and every
supervisor gate (e-stop, stale sensors, blocked path) still applies.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .profiles import DEFAULTS, MAX_SPEED_MPS, RACE_MODES, make_profile  # noqa: F401  (re-exported)


@dataclass
class RaceStatus:
    state: str = "IDLE"
    mode: str = "speed"
    speed_mps: float = DEFAULTS["speed"].speed_mps
    detail: str = ""
    started_at: float | None = None
    finished_at: float | None = None
    armed_at: float | None = None


@dataclass
class Actions:
    profile: dict | None = None        # publish to the driver
    autonomy: bool | None = None       # True = start LIDAR_CRUISE, False = stop
    events: list[str] = field(default_factory=list)


class RaceManager:
    def __init__(self, min_run_s: float = 3.0, start_timeout_s: float = 2.0,
                 auto_start_s: float = 30.0) -> None:
        self.status = RaceStatus()
        self.min_run_s = min_run_s
        self.start_timeout_s = start_timeout_s
        # Fallback if the camera never sees the green arm: start this long after
        # ARM anyway (0 = off; then only green or START NOW start the run).
        self.auto_start_s = auto_start_s

    def _set(self, state: str, detail: str = "") -> None:
        self.status.state, self.status.detail = state, detail

    def arm(self, mode: str, overrides: dict | None = None, now: float | None = None) -> Actions:
        if mode not in RACE_MODES:
            raise ValueError(f"unknown race mode {mode!r}")
        if self.status.state == "RUNNING":
            return Actions(events=["already running"])
        profile = make_profile({**(overrides or {}), "mode": mode})
        wait = f" (auto start in {self.auto_start_s:.0f} s)" if self.auto_start_s > 0 else ""
        self.status = RaceStatus(state="ARMED", mode=mode, speed_mps=profile.speed_mps,
                                 detail="waiting for the green signal" + wait, armed_at=now)
        return Actions(profile=profile.to_dict(), events=[f"armed {mode} at {profile.speed_mps:.2f} m/s"])

    def disarm(self, reason: str = "disarmed") -> Actions:
        was_running = self.status.state == "RUNNING"
        self._set("IDLE", reason)
        return Actions(autonomy=False if was_running else None, events=[reason])

    def start(self, now: float, reason: str) -> Actions:
        if self.status.state != "ARMED":
            return Actions()
        self.status.started_at = now
        self._set("RUNNING", reason)
        return Actions(autonomy=True, events=[f"start: {reason}"])

    def on_signals(self, now: float, green: bool, red: bool) -> Actions:
        if self.status.state == "ARMED" and green:
            return self.start(now, "green signal")
        if self.status.state == "RUNNING" and red and now - (self.status.started_at or now) >= self.min_run_s:
            self.status.finished_at = now
            self._set("FINISHED", "stop signal")
            return Actions(autonomy=False, events=["stop signal: finished"])
        return Actions()

    def tick(self, now: float) -> Actions:
        """Start anyway once armed for auto_start_s without seeing green."""
        armed_at = self.status.armed_at
        if self.status.state == "ARMED" and self.auto_start_s > 0 and armed_at is not None                 and now - armed_at >= self.auto_start_s:
            return self.start(now, f"no green seen in {self.auto_start_s:.0f} s: auto start")
        return Actions()

    def on_mission(self, now: float, mission: str, health: str) -> Actions:
        """Follow the supervisor: aborts, e-stops and refused starts end the run."""
        if self.status.state != "RUNNING":
            return Actions()
        if mission == "LIDAR_CRUISE":
            return Actions()
        if mission in ("EMERGENCY_STOP", "EXPLORATION_BLOCKED") or \
                now - (self.status.started_at or now) > self.start_timeout_s:
            self._set("ABORTED", f"{mission}: {health}")
            return Actions(autonomy=False, events=[f"aborted: {mission} ({health})"])
        return Actions()
