#!/usr/bin/env python3
"""Battery voltage monitor for unattended sessions.  Listens only: never publishes.

Reads input_voltage_v from /laksa/vesc/state and every 30 s appends one line to
~/laksa_run/latest/battery_voltage.log:

    2026-09-29T18:10:00Z  14.87 V  OK

    < 14.0 V   WARNING   (informational)
    < 13.5 V   CRITICAL  banner on stdout every 30 s, BATTERY_LOW_WARNING.txt,
                         and the pause file BATTERY_PAUSE for the test loop
    no fresh reading for 60 s: TELEMETRY_LOST (expected once the XT60 is out)

Only readings with telemetry_fresh=True count: a stale VescState repeats the
last voltage the ESP32 saw, which is not the battery's voltage now.
"""

import os
import sys
import time

WARNING_V = 14.0
CRITICAL_V = 13.5
PERIOD_S = 30.0
LOST_S = 60.0


def classify(voltage):
    """Pure: 'OK', 'WARNING' or 'CRITICAL' for a fresh reading."""
    if voltage < CRITICAL_V:
        return "CRITICAL"
    if voltage < WARNING_V:
        return "WARNING"
    return "OK"


def banner(voltage):
    return ("\n╔══════════════════════════════════════════╗\n"
            f"║   ⚠  BATTERY LOW: {voltage:4.1f} V  ⚠               ║\n"
            "║   DISCONNECT THE BATTERY NOW             ║\n"
            "╚══════════════════════════════════════════╝\n")


def main() -> int:
    import rclpy
    from laksa_interfaces.msg import VescState
    from rclpy.qos import qos_profile_sensor_data

    out_dir = os.path.expanduser("~/laksa_run/latest")
    if not os.path.isdir(out_dir):
        print(f"{out_dir} does not exist", file=sys.stderr)
        return 1
    log_path = os.path.join(out_dir, "battery_voltage.log")
    warn_path = os.path.join(out_dir, "BATTERY_LOW_WARNING.txt")
    pause_path = os.path.join(out_dir, "BATTERY_PAUSE")

    state = {"v": None, "t": None}
    rclpy.init()
    node = rclpy.create_node("laksa_battery_monitor")

    def on_state(message):
        if message.telemetry_fresh:
            state["v"], state["t"] = float(message.input_voltage_v), time.monotonic()

    node.create_subscription(VescState, "/laksa/vesc/state", on_state, qos_profile_sensor_data)
    start = time.monotonic()
    next_log = start + 5.0          # first line shortly after start
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            now = time.monotonic()
            if now < next_log:
                continue
            next_log = now + PERIOD_S
            stamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            last = state["t"] if state["t"] is not None else start
            if now - last > LOST_S or state["v"] is None and now - start > LOST_S:
                line = f"{stamp}  --.-- V  TELEMETRY_LOST"
            elif state["v"] is None:
                continue
            else:
                level = classify(state["v"])
                line = f"{stamp}  {state['v']:5.2f} V  {level}"
                if level == "WARNING":
                    print(f"{stamp} battery WARNING: {state['v']:.2f} V (< {WARNING_V} V)", flush=True)
                elif level == "CRITICAL":
                    text = banner(state["v"])
                    print(text, flush=True)
                    with open(warn_path, "w", encoding="utf-8") as f:
                        f.write(f"{stamp}\n{text}")
                    open(pause_path, "a").close()
            with open(log_path, "a", encoding="utf-8") as f:
                f.write(line + "\n")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
