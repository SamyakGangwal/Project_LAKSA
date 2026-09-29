"""Race sequencing: arm -> green signal -> drive -> stop signal (pure logic, testable).

Two modes share the learned driver and differ in speed:

  speed     the Speed Course (default 2.5 m/s)
  obstacle  the Obstacle Course (default 2.0 m/s)

The manager never commands the motor: it asks drive_supervisor to enter
LIDAR_CRUISE (the learned driver's slot) and to leave it again, and every
supervisor gate (e-stop, stale sensors, blocked path) still applies.
"""

from __future__ import annotations

from dataclasses import dataclass, field

MODES = {"speed": 2.5, "obstacle": 2.0}
MAX_SPEED_MPS = 3.0            # the learned model's trained maximum
MIN_SPEED_MPS = 0.2            # the drive does not run reliably below ~0.2 m/s


@dataclass
class RaceStatus:
    state: str = "IDLE"
    mode: str = "speed"
    speed_mps: float = MODES["speed"]
    detail: str = ""
    started_at: float | None = None
    finished_at: float | None = None


@dataclass
class Actions:
    speed_cap: float | None = None     # publish to the driver
    autonomy: bool | None = None       # True = start LIDAR_CRUISE, False = stop
    events: list[str] = field(default_factory=list)


class RaceManager:
    def __init__(self, min_run_s: float = 3.0, start_timeout_s: float = 2.0) -> None:
        self.status = RaceStatus()
        self.min_run_s = min_run_s
        self.start_timeout_s = start_timeout_s

    def _set(self, state: str, detail: str = "") -> None:
        self.status.state, self.status.detail = state, detail

    def arm(self, mode: str, speed_mps: float | None) -> Actions:
        if mode not in MODES:
            raise ValueError(f"unknown mode {mode!r}")
        if self.status.state == "RUNNING":
            return Actions(events=["already running"])
        speed = MODES[mode] if speed_mps is None else float(speed_mps)
        speed = min(MAX_SPEED_MPS, max(MIN_SPEED_MPS, speed))
        self.status = RaceStatus(state="ARMED", mode=mode, speed_mps=speed, detail="waiting for the green signal")
        return Actions(speed_cap=speed, events=[f"armed {mode} at {speed:.2f} m/s"])

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
