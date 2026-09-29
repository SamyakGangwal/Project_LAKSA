"""Deadman re-arm rule for the console's HOLD / GO buttons.

The page sends "hold" (or "go") every 50 ms while a button is held and
"release" when it is let go.  If the heartbeat lapses while a button is held
(Wi-Fi drop, tunnel stall, browser throttled) the car brakes, and the button
must be released and pressed again before it counts: a hold that simply
resumes after the gap is ignored.  A deliberate release, STOP and REARM work
as before.
"""


class HoldLatch:
    def __init__(self) -> None:
        self.holding = False            # a hold/go stream is in progress (no release yet)
        self.awaiting_release = False   # heartbeat was lost mid-hold: ignore presses until "release"

    def lapse(self, now: float, last_heartbeat: float, timeout: float) -> bool:
        """Call periodically.  Returns True when a mid-hold heartbeat loss is first detected."""
        if self.holding and now - last_heartbeat > timeout:
            self.holding = False
            self.awaiting_release = True
            return True
        return False

    def press(self, now: float, last_heartbeat: float, timeout: float) -> bool:
        """A "hold"/"go" message.  Returns True if it counts, False if it must be ignored."""
        self.lapse(now, last_heartbeat, timeout)
        if self.awaiting_release:
            return False
        self.holding = True
        return True

    def release(self) -> None:
        self.holding = False
        self.awaiting_release = False

    def stop(self) -> None:
        """STOP ends the hold deliberately; it is not a heartbeat loss."""
        self.holding = False
