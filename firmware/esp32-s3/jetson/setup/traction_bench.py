#!/usr/bin/env python3
"""Traction bench test through the ESP32 with the driven wheels OFF THE GROUND.

Sends short, slow speed steps on /laksa/command (steering centered), with
active braking between steps, and reports the measured VESC response.  The
m/s -> eRPM conversion happens in the ESP32 with its flashed constants, so the
measured eRPM here is the ground truth for calibrating the Jetson side.

Aborts to brake on: measured |eRPM| above MAX_ABS_ERPM, motor current above
MAX_MOTOR_CURRENT_A, a VESC fault, stale telemetry, sustained rotation opposite
to the command, or another /laksa/command publisher.  If this process dies,
the ESP32's 500 ms command watchdog brakes on its own.

Run with drive_supervisor stopped.
"""

import math
import sys
import time

import rclpy
from laksa_interfaces.msg import DriveCommand, VehicleState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

MAX_ABS_SPEED_MPS = 0.30
MAX_ABS_ERPM = 2500.0
MAX_MOTOR_CURRENT_A = 8.0
STATE_TIMEOUT_S = 0.6
RATE_HZ = 20.0
RAMP_S = 1.0                       # ramp speed steps to limit start-up current
SEQUENCE = (                       # (speed m/s, brake, seconds)
    (0.0, True, 1.0),
    (0.10, False, 3.0),
    (0.0, True, 1.5),
    (0.20, False, 3.0),
    (0.0, True, 1.5),
    (-0.10, False, 3.0),
    (0.0, True, 1.5),
)


class TractionBench(Node):
    def __init__(self) -> None:
        super().__init__("laksa_traction_bench")
        self.command_pub = self.create_publisher(DriveCommand, "/laksa/command", 10)
        self.brake_pub = self.create_publisher(Bool, "/laksa/brake", 10)
        self.state = None
        self.state_time = 0.0
        self.create_subscription(VehicleState, "/laksa/state", self._state_cb, qos_profile_sensor_data)

    def _state_cb(self, message: VehicleState) -> None:
        self.state = message
        self.state_time = time.monotonic()

    def send(self, speed: float, brake: bool) -> None:
        if not math.isfinite(speed) or abs(speed) > MAX_ABS_SPEED_MPS:
            raise ValueError(f"speed {speed} outside bench limit")
        command = DriveCommand()
        command.speed_mps = 0.0 if brake else float(speed)
        command.steering_angle_rad = 0.0
        command.brake = bool(brake)
        self.brake_pub.publish(Bool(data=bool(brake)))
        self.command_pub.publish(command)

    def fault(self, commanded: float, since_start: float) -> str | None:
        if self.state is None or time.monotonic() - self.state_time > STATE_TIMEOUT_S:
            return "ESP32 state stale"
        v = self.state.vesc
        if not v.telemetry_fresh:
            return "VESC telemetry stale"
        if int(v.fault_code) != 0:
            return f"VESC fault {int(v.fault_code)}"
        if abs(float(v.measured_erpm)) > MAX_ABS_ERPM:
            return f"motor too fast ({float(v.measured_erpm):.0f} eRPM)"
        if abs(float(v.motor_current_a)) > MAX_MOTOR_CURRENT_A:
            return f"motor current {float(v.motor_current_a):.1f} A"
        if commanded != 0.0 and since_start > 1.0 and abs(float(v.measured_erpm)) > 150 \
                and math.copysign(1.0, float(v.vehicle_linear_velocity_mps)) != math.copysign(1.0, commanded):
            return (f"wheel speed sign opposite to command (cmd {commanded:+.2f} m/s, "
                    f"measured {float(v.vehicle_linear_velocity_mps):+.3f} m/s)")
        if self.count_publishers("/laksa/command") > 1:
            return "another /laksa/command publisher is active"
        return None


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def main() -> int:
    rclpy.init()
    node = TractionBench()
    exit_code = 0
    try:
        spin_for(node, 2.0)
        problem = node.fault(0.0, 0.0)
        if problem:
            print(f"REFUSING TO START: {problem}", flush=True)
            return 2
        print(f"start: battery {node.state.vesc.input_voltage_v:.1f} V; wheels must be off the ground", flush=True)
        for speed, brake, hold in SEQUENCE:
            label = "BRAKE" if brake else f"speed {speed:+.2f} m/s"
            print(f"-> {label} for {hold:.1f} s", flush=True)
            start = time.monotonic()
            samples = []
            last_report = 0.0
            while time.monotonic() - start < hold:
                ramp = min(1.0, (time.monotonic() - start) / RAMP_S)
                node.send(speed * ramp, brake)
                spin_for(node, 1.0 / RATE_HZ)
                elapsed = time.monotonic() - start
                problem = node.fault(0.0 if brake else speed, elapsed)
                if problem:
                    raise RuntimeError(problem)
                v = node.state.vesc
                if elapsed > RAMP_S + 0.5:
                    samples.append((v.requested_erpm, v.active_erpm, v.measured_erpm,
                                    v.vehicle_linear_velocity_mps, v.motor_current_a, v.input_current_a))
                if time.monotonic() - last_report > 0.5:
                    last_report = time.monotonic()
                    print(f"   requested {v.requested_erpm} active {v.active_erpm} measured {v.measured_erpm:.0f} eRPM | "
                          f"wheel {v.vehicle_linear_velocity_mps:+.3f} m/s | motor {v.motor_current_a:+.2f} A "
                          f"input {v.input_current_a:+.2f} A | brake_active {v.brake_active} "
                          f"cmd_fresh {v.command_fresh} dir_pending {v.direction_change_pending}", flush=True)
            if samples and not brake:
                mean = [sum(column) / len(column) for column in zip(*samples)]
                print(f"   steady state: requested {mean[0]:.0f} eRPM, measured {mean[2]:.0f} eRPM, "
                      f"reported {mean[3]:+.3f} m/s for commanded {speed:+.2f} m/s, "
                      f"motor {mean[4]:+.2f} A, input {mean[5]:+.2f} A", flush=True)
        print("traction bench complete", flush=True)
    except (RuntimeError, ValueError, KeyboardInterrupt) as error:
        print(f"ABORT: {error}", flush=True)
        exit_code = 1
    finally:
        for _ in range(int(RATE_HZ)):
            node.send(0.0, brake=True)
            spin_for(node, 1.0 / RATE_HZ)
        print("brake applied, speed 0, steering centered", flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
