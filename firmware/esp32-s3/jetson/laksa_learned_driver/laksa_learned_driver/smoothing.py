"""Steering smoothing: low-pass filter plus a rate limit on the commanded angle.

The policy decides steering independently on every scan, so outside its
training distribution consecutive outputs can jump.  Filtering the target and
limiting how fast the command may change removes the stutter without changing
where the car is trying to go.
"""

from __future__ import annotations

import math


class SteeringSmoother:
    def __init__(self, alpha: float = 0.4, max_rate_radps: float = 1.0) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")
        if max_rate_radps <= 0.0:
            raise ValueError("max_rate_radps must be positive")
        self.alpha = alpha
        self.max_rate = max_rate_radps
        self._filtered: float | None = None
        self._output = 0.0

    def reset(self, value: float = 0.0) -> None:
        self._filtered = None
        self._output = value

    def update(self, target: float, dt: float) -> float:
        if not math.isfinite(target):
            return self._output
        dt = min(max(dt, 1e-3), 0.5)
        self._filtered = target if self._filtered is None else \
            self._filtered + self.alpha * (target - self._filtered)
        step = self.max_rate * dt
        self._output += max(-step, min(step, self._filtered - self._output))
        return self._output
