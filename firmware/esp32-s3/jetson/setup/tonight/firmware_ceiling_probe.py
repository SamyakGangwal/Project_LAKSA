#!/usr/bin/env python3
"""Firmware speed-ceiling probe.  MAIN BATTERY (XT60) UNPLUGGED.

Finds the ESP32 firmware's own eRPM clamp C: publishes /laksa/command at
20 Hz for +0.30, +0.50, +1.00 and -0.50 m/s (2 s each) and reads back
requested_erpm / active_erpm from /laksa/vesc/state.  With the VESC unpowered
nothing turns; requested_erpm is what the firmware would ask the VESC for.

Safety (none of these can be skipped; there is no bypass option):
  * a person must type "battery is unplugged" at a terminal (piped input is refused);
  * refuses if /laksa/command or /laksa/brake already has a publisher;
  * refuses if VESC telemetry is fresh and shows battery voltage (> 5 V);
  * aborts to brake if |measured_erpm| > 50, a second publisher appears, or on Ctrl-C;
  * ends with brake=True for 1 s, then stops publishing (the ESP32's 500 ms
    command watchdog brakes on its own if this process dies).

Writes the samples and the table to ~/laksa_run/p2_ceiling_<timestamp>.{csv,txt}.

    firmware_ceiling_probe.py --analyze CSV   re-analyze a saved run (publishes nothing)

Verdict per speed: "no" (requested ~= 4140 x v), "CLAMPED" (requested plateaus
above zero but below 95 % of 4140 x v) or "REJECTED" (requested ~0: the firmware
ignored the command and its 500 ms watchdog zeroed the output).
"""

from __future__ import annotations

import csv
import math
import os
import sys
import time

PHRASE = "battery is unplugged"
ERPM_PER_MPS = 4140.0
SPEEDS = (0.30, 0.50, 1.00, -0.50)
HOLD_S = 2.0
PLATEAU_S = 1.0            # clamp decision uses the mean over the last second of each hold
SETTLE_S = 1.0             # zero speed (no brake) before a direction change
RATE_HZ = 20.0
CLAMP_TOLERANCE = 0.05     # clamp if |mean requested| < 95 % of |expected|
MAX_MEASURED_ERPM = 50.0   # anything turning means the battery is connected
ZERO_ERPM = 1.0            # |mean requested| below this on the plateau = command rejected
POWERED_V = 5.0


def summarize(speed: float, samples: list) -> dict:
    """samples: [(t_in_hold, requested, active, dir_pending, fresh)].  Pure; unit-testable."""
    expected = ERPM_PER_MPS * speed
    plateau = [s for s in samples if s[0] >= HOLD_S - PLATEAU_S]
    row = {"speed": speed, "expected": expected, "n": len(samples), "n_plateau": len(plateau),
           "mean_requested": None, "mean_active": None, "max_abs_requested": None,
           "dir_pending": any(s[3] for s in plateau), "clamp": None}
    if not plateau:
        return row
    row["mean_requested"] = sum(s[1] for s in plateau) / len(plateau)
    row["mean_active"] = sum(s[2] for s in plateau) / len(plateau)
    row["max_abs_requested"] = max(abs(s[1]) for s in samples)
    low = abs(row["mean_requested"]) < (1.0 - CLAMP_TOLERANCE) * abs(expected)
    row["clamp"] = low
    row["kind"] = ("REJECTED" if abs(row["mean_requested"]) < ZERO_ERPM else "CLAMPED") if low else "no"
    return row


def table(rows: list) -> str:
    fmt = lambda v, f: "-" if v is None else f.format(v)
    lines = [f"{'speed_mps':>9} | {'expected_erpm':>13} | {'mean_requested_erpm':>19} | "
             f"{'mean_active_erpm':>16} | {'max|req|':>8} | {'n':>3} | clamp_detected"]
    for r in rows:
        clamp = ("no data" if r["clamp"] is None else r["kind"]) + \
                (" (direction change pending)" if r["dir_pending"] else "")
        lines.append(f"{r['speed']:>+9.2f} | {r['expected']:>13.0f} | {fmt(r['mean_requested'], '{:.0f}'):>19} | "
                     f"{fmt(r['mean_active'], '{:.0f}'):>16} | {fmt(r['max_abs_requested'], '{:.0f}'):>8} | "
                     f"{r['n_plateau']:>3} | {clamp}")
    return "\n".join(lines)


def ceiling(rows: list) -> str:
    if any(r["clamp"] is None for r in rows):
        return "INCOMPLETE: some holds had no /laksa/vesc/state samples; C not determined"
    passed = [abs(r["expected"]) for r in rows if not r["clamp"]]
    clamped = [r for r in rows if r.get("kind") == "CLAMPED"]
    rejected = [abs(r["expected"]) for r in rows if r.get("kind") == "REJECTED"]
    if not clamped and not rejected:
        return ("No clamp found up to |4140 eRPM|: supervisor cap is the sole ceiling; "
                "all caps tonight <= 1,000 eRPM.")
    parts = []
    if passed:
        parts.append(f"passed unchanged up to |{max(passed):.0f}| eRPM")
    if clamped:
        c = max(abs(r["mean_requested"]) for r in clamped)
        parts.append(f"CLAMPS at C ~= {c:.0f} eRPM")
    if rejected:
        parts.append(f"REJECTS |{min(rejected):.0f}| eRPM (command ignored, watchdog zeroes output: "
                     f"not a clamp)")
    bound = ""
    if rejected and passed and not clamped:
        bound = f"; limit C is between {max(passed):.0f} and {min(rejected):.0f} eRPM"
    return "Firmware " + "; ".join(parts) + bound


def confirm() -> bool:
    if not sys.stdin.isatty():
        print("REFUSING: the confirmation must be typed by a person at a terminal (stdin is not a TTY).")
        return False
    print("This probe publishes /laksa/command up to 1.00 m/s (4140 eRPM).")
    print("The main battery XT60 must be PHYSICALLY UNPLUGGED; the ESP32 runs from USB-C.")
    try:
        answer = input(f'Type exactly "{PHRASE}" to continue: ')
    except EOFError:
        return False
    if answer.strip() != PHRASE:
        print("Phrase did not match: exiting, nothing published.")
        return False
    return True


def analyze(path: str) -> int:
    """Re-analyze a saved probe CSV.  Publishes nothing, so no confirmation."""
    samples = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            samples.setdefault(float(r["speed_mps"]), []).append(
                (float(r["t_in_hold_s"]), float(r["requested_erpm"]), float(r["active_erpm"]),
                 r["direction_change_pending"] == "True", r["telemetry_fresh"] == "True"))
    rows = [summarize(speed, samples.get(speed, [])) for speed in SPEEDS]
    print(table(rows))
    print()
    print("RESULT: " + ceiling(rows))
    return 0


def main() -> int:
    if len(sys.argv) == 3 and sys.argv[1] == "--analyze":
        return analyze(sys.argv[2])
    if len(sys.argv) != 1:
        print("usage: firmware_ceiling_probe.py            (run the probe; battery unplugged)\n"
              "       firmware_ceiling_probe.py --analyze CSV")
        return 64
    if not confirm():
        return 2

    import rclpy
    from laksa_interfaces.msg import DriveCommand, VescState
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Bool

    stamp = time.strftime("%Y%m%dT%H%M%S")
    out_dir = os.path.expanduser("~/laksa_run")
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, f"p2_ceiling_{stamp}.csv")
    txt_path = os.path.join(out_dir, f"p2_ceiling_{stamp}.txt")
    report = []

    def say(line=""):
        print(line, flush=True)
        report.append(line)

    rclpy.init()
    node = rclpy.create_node("laksa_firmware_ceiling_probe")
    state = {"msg": None, "t": 0.0}

    def on_state(message):
        state["msg"], state["t"] = message, time.monotonic()

    node.create_subscription(VescState, "/laksa/vesc/state", on_state, qos_profile_sensor_data)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    spin(2.0)                                  # discovery + first state samples
    busy = {t: len(node.get_publishers_info_by_topic(t)) for t in ("/laksa/command", "/laksa/brake")}
    if any(busy.values()):
        say(f"REFUSING: existing publishers {busy}; nothing published.")
        node.destroy_node(); rclpy.shutdown()
        return 3
    if state["msg"] is None:
        say("REFUSING: no /laksa/vesc/state received in 2 s (ESP32 session down?); nothing published.")
        node.destroy_node(); rclpy.shutdown()
        return 3
    v = state["msg"]
    if v.telemetry_fresh and float(v.input_voltage_v) > POWERED_V:
        say(f"REFUSING: VESC telemetry is fresh at {float(v.input_voltage_v):.1f} V: the battery looks CONNECTED.")
        node.destroy_node(); rclpy.shutdown()
        return 4
    say(f"pre-check: publishers {busy}; VESC telemetry_fresh={bool(v.telemetry_fresh)} "
        f"input_voltage={float(v.input_voltage_v):.2f} V")

    command_pub = node.create_publisher(DriveCommand, "/laksa/command", 10)
    brake_pub = node.create_publisher(Bool, "/laksa/brake", 10)

    def send(speed, brake):
        command = DriveCommand()
        command.speed_mps = 0.0 if brake else float(speed)
        command.steering_angle_rad = 0.0
        command.brake = bool(brake)
        brake_pub.publish(Bool(data=bool(brake)))
        command_pub.publish(command)

    def fault():
        m = state["msg"]
        if m is not None and abs(float(m.measured_erpm)) > MAX_MEASURED_ERPM:
            return f"measured {float(m.measured_erpm):.0f} eRPM: something is turning (battery connected?)"
        if len(node.get_publishers_info_by_topic("/laksa/command")) > 1:
            return "another /laksa/command publisher appeared"
        return None

    def hold(speed, seconds, record=None):
        start = time.monotonic()
        last_seen = state["t"]
        while time.monotonic() - start < seconds:
            send(speed, brake=False)
            spin(1.0 / RATE_HZ)
            problem = fault()
            if problem:
                raise RuntimeError(problem)
            if record is not None and state["t"] != last_seen and state["msg"] is not None:
                last_seen = state["t"]
                m = state["msg"]
                record.append((time.monotonic() - start, float(m.requested_erpm), float(m.active_erpm),
                               bool(m.direction_change_pending), bool(m.telemetry_fresh)))

    rows, exit_code = [], 0
    all_samples = []
    try:
        say(f"releasing brake (/laksa/brake False), then {len(SPEEDS)} holds of {HOLD_S:.0f} s at {RATE_HZ:.0f} Hz")
        for _ in range(int(RATE_HZ * 0.5)):
            send(0.0, brake=False)
            spin(1.0 / RATE_HZ)
        previous = 0.0
        for speed in SPEEDS:
            if previous and math.copysign(1.0, speed) != math.copysign(1.0, previous):
                say(f"-> 0.00 m/s for {SETTLE_S:.0f} s (direction change)")
                hold(0.0, SETTLE_S)
            say(f"-> {speed:+.2f} m/s for {HOLD_S:.0f} s (expect {ERPM_PER_MPS * speed:+.0f} eRPM)")
            samples = []
            hold(speed, HOLD_S, samples)
            all_samples += [(speed, *s) for s in samples]
            rows.append(summarize(speed, samples))
            previous = speed
    except (RuntimeError, KeyboardInterrupt) as error:
        say(f"ABORT: {error}")
        exit_code = 1
    finally:
        for _ in range(int(RATE_HZ)):
            send(0.0, brake=True)
            spin(1.0 / RATE_HZ)
        say("brake=True sent for 1 s; publishing stopped")
        node.destroy_node()
        rclpy.shutdown()

    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["speed_mps", "t_in_hold_s", "requested_erpm", "active_erpm", "direction_change_pending",
                    "telemetry_fresh"])
        w.writerows(all_samples)
    say("")
    say(table(rows))
    say("")
    say("RESULT: " + (ceiling(rows) if exit_code == 0 else "ABORTED, C not determined"))
    with open(txt_path, "w") as f:
        f.write("\n".join(report) + "\n")
    print(f"\nsaved {txt_path} and {csv_path}")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
