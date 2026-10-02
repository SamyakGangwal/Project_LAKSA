"""Terminal operator for LAKSA: a human deadman in place of the Xbox controller.

Publishes ``sensor_msgs/Joy`` exactly like the Xbox path, so drive_supervisor
keeps every one of its gates.  Sticks are always neutral: this program cannot
steer or throttle; it only says "an operator is present" and presses buttons.

  laksa_operator run [--duration 60]   operator present; holds A for 1.5 s to
                                       enter LiDAR Cruise (the learned driver).
                                       Ctrl-C, Enter, end of --duration, a closed
                                       terminal or a dropped SSH session stops
                                       /joy, and the supervisor brakes within
                                       0.5 s.  On exit it also latches E-STOP.
  laksa_operator stop                  latch the emergency stop (Xbox B), from
                                       any terminal, at any time
  laksa_operator rearm                 clear the emergency stop (Xbox Y)
  laksa_operator status                show mode, autonomy health and E-STOP
"""

from __future__ import annotations

import argparse
import select
import signal
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

A_BUTTON, B_BUTTON, Y_BUTTON = 0, 1, 3
RATE_HZ = 20.0


class Operator(Node):
    def __init__(self) -> None:
        super().__init__("laksa_terminal_operator")
        self.joy_pub = self.create_publisher(Joy, "/joy", 10)
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.mission = "UNKNOWN"
        self.health = "UNKNOWN"
        self.estop = None
        self.estop_reason = ""
        self.create_subscription(String, "/laksa/mission_state", lambda m: setattr(self, "mission", m.data), latched)
        self.create_subscription(String, "/laksa/autonomy_health", lambda m: setattr(self, "health", m.data), latched)
        self.create_subscription(Bool, "/laksa/emergency_stop", lambda m: setattr(self, "estop", m.data), latched)
        self.create_subscription(String, "/laksa/emergency_stop_reason",
                                 lambda m: setattr(self, "estop_reason", m.data), latched)

    def joy(self, pressed=()) -> None:
        message = Joy()
        message.header.stamp = self.get_clock().now().to_msg()
        message.axes = [0.0] * 8
        message.buttons = [1 if index in pressed else 0 for index in range(12)]
        self.joy_pub.publish(message)

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def pulse(self, button: int, seconds: float = 0.6) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.joy((button,))
            self.spin_for(1.0 / RATE_HZ)
        self.joy()

    def line(self) -> str:
        estop = "UNKNOWN" if self.estop is None else ("LATCHED" if self.estop else "clear")
        text = f"mode {self.mission} | autonomy health {self.health} | E-STOP {estop}"
        return text + (f" ({self.estop_reason})" if self.estop and self.estop_reason else "")


def enter_pressed() -> bool:
    """True only for a real Enter on an interactive terminal (EOF is not a press)."""
    if not sys.stdin.isatty():
        return False
    try:
        ready, _, _ = select.select([sys.stdin], [], [], 0)
    except (OSError, ValueError):
        return False
    return bool(ready) and sys.stdin.readline() != ""


def run(node: Operator, duration: float, engage_sec: float) -> int:
    print(f"OPERATOR PRESENT for up to {duration:.0f} s. Press Enter or Ctrl-C to stop.", flush=True)
    start = time.monotonic()
    last = 0.0
    reason = "duration elapsed"
    try:
        while True:
            elapsed = time.monotonic() - start
            if elapsed >= duration:
                break
            if enter_pressed():
                reason = "operator pressed Enter"
                break
            node.joy((A_BUTTON,) if elapsed < engage_sec else ())
            node.spin_for(1.0 / RATE_HZ)
            if time.monotonic() - last >= 1.0:
                last = time.monotonic()
                print(f"t={elapsed:5.1f}s  {node.line()}", flush=True)
    except KeyboardInterrupt:
        reason = "Ctrl-C / terminal closed"
    # Leave the car latched: a new run needs an explicit rearm.  Ignore a
    # second Ctrl-C while the latch is being sent.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    node.pulse(B_BUTTON)
    node.spin_for(0.3)
    try:
        print(f"STOPPED ({reason}); E-STOP latched. {node.line()}", flush=True)
    except (BrokenPipeError, OSError):
        pass
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("run", "stop", "rearm", "status"))
    parser.add_argument("--duration", type=float, default=60.0, help="run: seconds before automatic stop (max 300)")
    parser.add_argument("--engage-sec", type=float, default=1.5, help="run: seconds to hold A at the start (supervisor auto_hold_sec is 1.0)")
    args = parser.parse_args()
    if not 0.0 < args.duration <= 300.0:
        parser.error("--duration must be in (0, 300]")
    # Keep the ROS context alive through Ctrl-C so the final E-STOP press can
    # still be published; a closed terminal (SIGHUP) or kill (SIGTERM) is
    # treated exactly like Ctrl-C.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGHUP, interrupt)
    node = Operator()
    try:
        node.spin_for(2.5)   # discover the supervisor's latched state topics
        if args.command == "run":
            return run(node, args.duration, args.engage_sec)
        if args.command == "stop":
            node.pulse(B_BUTTON)
            node.spin_for(0.3)
            print(f"E-STOP sent. {node.line()}", flush=True)
        elif args.command == "rearm":
            node.pulse(Y_BUTTON)
            node.spin_for(0.3)
            print(f"REARM sent. {node.line()}", flush=True)
        else:
            print(node.line(), flush=True)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
