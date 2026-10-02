#!/usr/bin/env python3
"""Wait until the motor is really spinning at the commanded speed.

Exits 0 and prints SPINNING once |measured_erpm| on /laksa/vesc/state stays at
or above ratio * |target| for --hold seconds; exits 1 (TIMEOUT) if that has
not happened within --timeout.  Stale telemetry does not count as spinning.
Listens only: it never publishes.

    wait_spin.py --target 911            # eRPM
    wait_spin.py --target 911 --hold 0.5 --timeout 5 --ratio 0.8
"""

import argparse
import math
import sys
import time


class SpinDetector:
    """Pure decision: feed (time, measured_erpm, fresh); ask done()."""

    def __init__(self, target: float, ratio: float, hold: float) -> None:
        self.threshold = ratio * abs(target)
        self.hold = hold
        self.since = None
        self.peak = 0.0

    def update(self, now: float, measured: float, fresh: bool) -> bool:
        if fresh and math.isfinite(measured):
            self.peak = max(self.peak, abs(measured))
        if fresh and math.isfinite(measured) and abs(measured) >= self.threshold:
            if self.since is None:
                self.since = now
        else:
            self.since = None
        return self.since is not None and now - self.since >= self.hold


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--target", type=float, required=True, help="target eRPM (sign ignored)")
    parser.add_argument("--ratio", type=float, default=0.8)
    parser.add_argument("--hold", type=float, default=0.5, help="seconds above threshold")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    if not (math.isfinite(args.target) and args.target != 0.0 and 0.0 < args.ratio <= 1.0
            and args.hold >= 0.0 and args.timeout > 0.0):
        parser.error("need a non-zero target, 0 < ratio <= 1, hold >= 0, timeout > 0")

    import rclpy
    from laksa_interfaces.msg import VescState
    from rclpy.qos import qos_profile_sensor_data

    detector = SpinDetector(args.target, args.ratio, args.hold)
    result = {"done": False}
    rclpy.init()
    node = rclpy.create_node("laksa_wait_spin")

    def on_state(message):
        if detector.update(time.monotonic(), float(message.measured_erpm), bool(message.telemetry_fresh)):
            result["done"] = True

    node.create_subscription(VescState, "/laksa/vesc/state", on_state, qos_profile_sensor_data)
    start = time.monotonic()
    try:
        while not result["done"] and time.monotonic() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    elapsed = time.monotonic() - start
    if result["done"]:
        print(f"SPINNING after {elapsed:.2f} s (|eRPM| >= {detector.threshold:.0f} for {args.hold:.2f} s)")
        return 0
    print(f"TIMEOUT after {args.timeout:.1f} s: peak |eRPM| {detector.peak:.0f}, needed {detector.threshold:.0f} "
          f"for {args.hold:.2f} s", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
