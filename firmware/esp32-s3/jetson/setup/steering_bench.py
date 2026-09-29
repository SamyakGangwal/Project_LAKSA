#!/usr/bin/env python3
"""Steering-only bench test through the ESP32 (no Xbox, traction always 0).

Publishes laksa_interfaces/DriveCommand on /laksa/command at 20 Hz with
speed_mps fixed at 0.0 and a short steering sweep.  The ESP32 centers the
wheels whenever brake is requested, so /laksa/brake is released for the sweep
and re-applied at the end and on every abort.

Aborts (brake + center) if the motor shows rotation, the VESC reports a fault,
ESP32/VESC telemetry goes stale, or another node publishes /laksa/command.
If this process dies, the ESP32's 500 ms command watchdog stops and centers.

Run with drive_supervisor stopped (it would re-apply brake continuously).
"""

import math
import sys
import time

import rclpy
from laksa_interfaces.msg import DriveCommand, VehicleState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

SPEED_MPS = 0.0                    # never changes: steering only
MAX_ABS_STEERING_RAD = 0.20
MAX_ABS_ERPM = 50.0
STATE_TIMEOUT_S = 0.6
RATE_HZ = 20.0
SEQUENCE = (                       # (steering rad, hold seconds); positive = left
    (0.0, 1.0),
    (0.15, 2.0),
    (0.0, 1.5),
    (-0.12, 2.0),
    (0.0, 1.5),
)


class SteeringBench(Node):
    def __init__(self) -> None:
        super().__init__("laksa_steering_bench")
        self.command_pub = self.create_publisher(DriveCommand, "/laksa/command", 10)
        self.brake_pub = self.create_publisher(Bool, "/laksa/brake", 10)
        self.state = None
        self.state_time = 0.0
        self.create_subscription(VehicleState, "/laksa/state", self._state_cb, qos_profile_sensor_data)

    def _state_cb(self, message: VehicleState) -> None:
        self.state = message
        self.state_time = time.monotonic()

    def send(self, steering: float, brake: bool) -> None:
        assert SPEED_MPS == 0.0
        if not math.isfinite(steering) or abs(steering) > MAX_ABS_STEERING_RAD:
            raise ValueError(f"steering {steering} outside bench limit")
        command = DriveCommand()
        command.speed_mps = SPEED_MPS
        command.steering_angle_rad = float(steering)
        command.brake = bool(brake)
        self.brake_pub.publish(Bool(data=bool(brake)))
        self.command_pub.publish(command)

    def fault(self) -> str | None:
        if self.state is None or time.monotonic() - self.state_time > STATE_TIMEOUT_S:
            return "ESP32 state stale"
        vesc = self.state.vesc
        if not vesc.telemetry_fresh:
            return "VESC telemetry stale"
        if int(vesc.fault_code) != 0:
            return f"VESC fault {int(vesc.fault_code)}"
        if abs(float(vesc.measured_erpm)) > MAX_ABS_ERPM:
            return f"motor rotation detected ({float(vesc.measured_erpm):.0f} eRPM)"
        if self.count_publishers("/laksa/command") > 1:
            return "another /laksa/command publisher is active"
        return None


def spin_for(node: SteeringBench, seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def safe_stop(node: SteeringBench) -> None:
    for _ in range(int(RATE_HZ)):
        node.send(0.0, brake=True)
        spin_for(node, 1.0 / RATE_HZ)


def main() -> int:
    rclpy.init()
    node = SteeringBench()
    exit_code = 0
    try:
        spin_for(node, 2.0)
        problem = node.fault()
        if problem:
            print(f"REFUSING TO START: {problem}", flush=True)
            return 2
        print(f"start: battery {node.state.vesc.input_voltage_v:.1f} V, speed fixed at {SPEED_MPS} m/s", flush=True)
        for target, hold in SEQUENCE:
            print(f"-> steering {target:+.3f} rad ({math.degrees(target):+.1f} deg) for {hold:.1f} s", flush=True)
            end = time.monotonic() + hold
            last_report = 0.0
            while time.monotonic() < end:
                node.send(target, brake=False)
                spin_for(node, 1.0 / RATE_HZ)
                problem = node.fault()
                if problem:
                    raise RuntimeError(problem)
                if time.monotonic() - last_report > 0.5:
                    last_report = time.monotonic()
                    s = node.state
                    print(f"   feedback: target {s.steering_target_rad:+.3f} current {s.steering_current_rad:+.3f} rad, "
                          f"eRPM {s.vesc.measured_erpm:.0f}, brake_active {s.vesc.brake_active}", flush=True)
        print("sweep complete", flush=True)
    except (RuntimeError, ValueError, KeyboardInterrupt) as error:
        print(f"ABORT: {error}", flush=True)
        exit_code = 1
    finally:
        safe_stop(node)
        print("brake re-applied, steering centered", flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
