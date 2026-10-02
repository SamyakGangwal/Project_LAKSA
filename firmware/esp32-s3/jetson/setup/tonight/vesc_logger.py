#!/usr/bin/env python3
"""Timestamped CSV of the drive chain, for stop-distance and brake-latency checks.

    vesc_logger.py [record] [--out CSV]      record until Ctrl-C (listens only, never publishes)
    vesc_logger.py --mark "STOP pressed"     append an event marker to the same CSV
    vesc_logger.py analyze [CSV] [--event TEXT]

Default CSV: ~/laksa_run/latest/vesc_log.csv.  One row per received message:
  /laksa/vesc/state      all VescState fields used below
  /laksa/command         speed_mps, steering_angle_rad, brake
  /laksa/brake           brake
  /laksa/mission_state   text
  /laksa/emergency_stop  estop
  /joy                   receive time only
  mark                   text (from --mark)

analyze: for every event (each mark, or else each rising edge of a brake
request on /laksa/brake or /laksa/command, or of /laksa/emergency_stop) prints
the time to VESC brake_active, the time until |measured_erpm| < 50, and the
distance travelled meanwhile (integrated vehicle_linear_velocity_mps).
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time

COLUMNS = ["t", "source", "measured_erpm", "requested_erpm", "active_erpm", "brake_active", "command_fresh",
           "telemetry_fresh", "motor_current_a", "input_current_a", "input_voltage_v", "duty_cycle",
           "vehicle_linear_velocity_mps", "tachometer", "fault_code", "speed_mps", "steering_angle_rad",
           "brake", "estop", "text"]
DEFAULT_CSV = os.path.expanduser("~/laksa_run/latest/vesc_log.csv")
STOPPED_ERPM = 50.0


def _line(values: dict) -> str:
    """One CSV line; os.write of one line with O_APPEND keeps recorder and --mark lines whole."""
    from io import StringIO
    buffer = StringIO()
    csv.writer(buffer).writerow([values.get(c, "") for c in COLUMNS])
    return buffer.getvalue()


def _append(path: str, values: dict) -> None:
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
    try:
        if new:
            os.write(fd, (",".join(COLUMNS) + "\r\n").encode())
        os.write(fd, _line(values).encode())
    finally:
        os.close(fd)


# ------------------------------------------------------------------ record
def record(path: str) -> int:
    import rclpy
    from laksa_interfaces.msg import DriveCommand, VescState
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool, String

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    rclpy.init()
    node = rclpy.create_node("laksa_vesc_logger")
    counts = {}

    def write(source, **values):
        counts[source] = counts.get(source, 0) + 1
        _append(path, {"t": f"{time.time():.4f}", "source": source, **values})

    def on_vesc(m):
        write("vesc", measured_erpm=f"{m.measured_erpm:.1f}", requested_erpm=m.requested_erpm,
              active_erpm=m.active_erpm, brake_active=int(m.brake_active), command_fresh=int(m.command_fresh),
              telemetry_fresh=int(m.telemetry_fresh), motor_current_a=f"{m.motor_current_a:.2f}",
              input_current_a=f"{m.input_current_a:.2f}", input_voltage_v=f"{m.input_voltage_v:.2f}",
              duty_cycle=f"{m.duty_cycle:.4f}", vehicle_linear_velocity_mps=f"{m.vehicle_linear_velocity_mps:.4f}",
              tachometer=m.tachometer, fault_code=m.fault_code)

    node.create_subscription(VescState, "/laksa/vesc/state", on_vesc, qos_profile_sensor_data)
    node.create_subscription(DriveCommand, "/laksa/command", lambda m: write(
        "command", speed_mps=f"{m.speed_mps:.4f}", steering_angle_rad=f"{m.steering_angle_rad:.4f}",
        brake=int(m.brake)), qos_profile_sensor_data)
    node.create_subscription(Bool, "/laksa/brake", lambda m: write("brake", brake=int(m.data)),
                             qos_profile_sensor_data)
    node.create_subscription(String, "/laksa/mission_state", lambda m: write("mission_state", text=m.data), latched)
    node.create_subscription(Bool, "/laksa/emergency_stop", lambda m: write("emergency_stop", estop=int(m.data)),
                             latched)
    node.create_subscription(Joy, "/joy", lambda m: write("joy"), qos_profile_sensor_data)
    print(f"recording to {path} (Ctrl-C to stop; mark events with: {sys.argv[0]} --mark TEXT)", flush=True)
    last = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - last > 10.0:
                last = time.monotonic()
                print("rows so far: " + ", ".join(f"{k} {v}" for k, v in sorted(counts.items())), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("rows: " + ", ".join(f"{k} {v}" for k, v in sorted(counts.items())))
    return 0


# ------------------------------------------------------------------ analyze
def load(path: str) -> list:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def find_events(rows: list, event_text: str | None = None) -> list:
    """[(t, label)] from marks, else from rising edges of brake / emergency stop requests."""
    marks = [(float(r["t"]), "mark: " + r["text"]) for r in rows if r["source"] == "mark"
             and (event_text is None or event_text in r["text"])]
    if marks or event_text is not None:
        return marks
    events, previous = [], {}
    for r in rows:
        key = value = None
        if r["source"] == "brake":
            key, value = "brake", r["brake"] == "1"
        elif r["source"] == "command":
            key, value = "command.brake", r["brake"] == "1"
        elif r["source"] == "emergency_stop":
            key, value = "emergency_stop", r["estop"] == "1"
        if key is None:
            continue
        if value and not previous.get(key, False):
            events.append((float(r["t"]), key))
        previous[key] = value
    # Brake and command.brake usually rise together: keep the first of any within 0.2 s.
    merged = []
    for t, label in sorted(events):
        if merged and t - merged[-1][0] < 0.2:
            continue
        merged.append((t, label))
    return merged


def analyze_event(rows: list, t0: float) -> dict:
    vesc = [r for r in rows if r["source"] == "vesc" and float(r["t"]) >= t0]
    out = {"t_brake_active": None, "t_stopped": None, "distance_m": None, "erpm_at_event": None}
    before = [r for r in rows if r["source"] == "vesc" and float(r["t"]) < t0]
    if before:
        out["erpm_at_event"] = float(before[-1]["measured_erpm"])
    distance, last_t, last_v = 0.0, t0, None
    if before:
        last_v = float(before[-1]["vehicle_linear_velocity_mps"])
    for r in vesc:
        t, v = float(r["t"]), float(r["vehicle_linear_velocity_mps"])
        if last_v is not None and math.isfinite(v):
            distance += 0.5 * (abs(v) + abs(last_v)) * (t - last_t)
        last_t, last_v = t, v
        if out["t_brake_active"] is None and r["brake_active"] == "1":
            out["t_brake_active"] = t - t0
        if out["t_stopped"] is None and abs(float(r["measured_erpm"])) < STOPPED_ERPM:
            out["t_stopped"] = t - t0
            out["distance_m"] = distance
            break
    return out


def total_distance(rows: list) -> float:
    vesc = [(float(r["t"]), float(r["vehicle_linear_velocity_mps"])) for r in rows if r["source"] == "vesc"]
    return sum(0.5 * (abs(a[1]) + abs(b[1])) * (b[0] - a[0]) for a, b in zip(vesc, vesc[1:]))


def analyze(path: str, event_text: str | None) -> int:
    rows = load(path)
    events = find_events(rows, event_text)
    fmt = lambda v, f: "-" if v is None else f.format(v)
    print(f"{path}: {len(rows)} rows, {sum(r['source'] == 'vesc' for r in rows)} VESC samples, "
          f"total distance {total_distance(rows):.2f} m")
    if not events:
        print("no events (no marks, and no brake / emergency-stop rising edge)")
        return 1
    print(f"{'event':<34} {'eRPM@event':>10} {'to_brake_active_s':>17} {'to_<50eRPM_s':>12} {'distance_m':>10}")
    for t0, label in events:
        a = analyze_event(rows, t0)
        print(f"{label[:34]:<34} {fmt(a['erpm_at_event'], '{:.0f}'):>10} {fmt(a['t_brake_active'], '{:.3f}'):>17} "
              f"{fmt(a['t_stopped'], '{:.3f}'):>12} {fmt(a['distance_m'], '{:.3f}'):>10}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", nargs="?", default="record", choices=["record", "analyze"])
    parser.add_argument("csv", nargs="?", default=None)
    parser.add_argument("--out", default=None, help=f"CSV path (default {DEFAULT_CSV})")
    parser.add_argument("--mark", default=None, help="append an event marker and exit")
    parser.add_argument("--event", default=None, help="analyze: only marks containing this text")
    args = parser.parse_args()
    path = os.path.expanduser(args.out or args.csv or DEFAULT_CSV)
    if args.mark is not None:
        _append(path, {"t": f"{time.time():.4f}", "source": "mark", "text": args.mark})
        print(f"marked '{args.mark}' in {path}")
        return 0
    if args.command == "analyze":
        return analyze(path, args.event)
    if not os.path.isdir(os.path.dirname(path)):
        print(f"{os.path.dirname(path)} does not exist; pass --out", file=sys.stderr)
        return 1
    return record(path)


if __name__ == "__main__":
    sys.exit(main())
