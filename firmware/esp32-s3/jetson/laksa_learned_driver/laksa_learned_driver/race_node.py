"""ROS 2 node: race manager (camera start/stop signals + mode and speed).

Subscribes
  /laksa/race/command          String JSON {"cmd": "arm", "mode": "speed"|"obstacle", "profile": {...}}
                               | {"cmd": "disarm"} | {"cmd": "start"} (manual start, for testing)
  ZED compressed RGB image     decoded only while ARMED or RUNNING
  /laksa/mission_state, /laksa/autonomy_health   from drive_supervisor
Publishes
  /laksa/race/state            String JSON {state, mode, speed_mps, detail, green, red} (latched)
  /laksa/drive_profile         String JSON, the learned driver's profile (latched)
  /laksa/dashboard_exploration_enabled   Bool: start/stop drive_supervisor's LIDAR_CRUISE
"""

from __future__ import annotations

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String

from .race import RaceManager
from .signals import Debounce, GreenStart, SignalConfig, read_signals


class RaceNode(Node):
    def __init__(self) -> None:
        super().__init__("race_manager")
        self.declare_parameter("image_topic", "/zed/zed_node/rgb/color/rect/image/compressed")
        self.declare_parameter("persist_frames", 2)
        self.declare_parameter("min_run_sec", 3.0)
        # Start anyway this long after ARM if green is never seen (0 = only green / START NOW).
        self.declare_parameter("auto_start_after_sec", 30.0)
        self.declare_parameter("max_rate_hz", 10.0)
        self._cfg = SignalConfig(persist_frames=int(self.get_parameter("persist_frames").value))
        self._manager = RaceManager(min_run_s=float(self.get_parameter("min_run_sec").value),
                                    auto_start_s=float(self.get_parameter("auto_start_after_sec").value))
        self._period = 1.0 / float(self.get_parameter("max_rate_hz").value)
        self._green = GreenStart(self._cfg.persist_frames)   # green growing past the armed baseline
        self._red = Debounce(self._cfg.persist_frames)
        self._last_frame = 0.0
        self._seen = {"green": 0.0, "red": 0.0}
        self._mission, self._health = "UNKNOWN", "UNKNOWN"

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(String, "/laksa/race/state", latched)
        self._profile_pub = self.create_publisher(String, "/laksa/drive_profile", latched)
        self._autonomy_pub = self.create_publisher(Bool, "/laksa/dashboard_exploration_enabled", latched)
        self.create_subscription(String, "/laksa/race/command", self._command_cb, 10)
        self.create_subscription(String, "/laksa/mission_state", self._mission_cb, latched)
        self.create_subscription(String, "/laksa/autonomy_health", self._health_cb, latched)
        self.create_subscription(CompressedImage, str(self.get_parameter("image_topic").value),
                                 self._image_cb, qos_profile_sensor_data)
        self.create_timer(0.5, self._tick)
        self._publish_state()
        self.get_logger().info("Race manager ready: ARM from the console, then show the green signal")

    def _apply(self, actions) -> None:
        if actions.profile is not None:
            self._profile_pub.publish(String(data=json.dumps(actions.profile)))
        if actions.autonomy is not None:
            self._autonomy_pub.publish(Bool(data=bool(actions.autonomy)))
        for event in actions.events:
            self.get_logger().warn(f"Race: {event}")
        if actions.profile is not None or actions.autonomy is not None or actions.events:
            self._publish_state()

    def _tick(self) -> None:
        # Catch a refused or aborted run even when no camera frames arrive, and
        # the auto-start fallback (runs without camera frames too).
        now = time.monotonic()
        self._apply(self._manager.tick(now))
        self._apply(self._manager.on_mission(now, self._mission, self._health))
        self._publish_state()

    def _publish_state(self) -> None:
        st = self._manager.status
        self._state_pub.publish(String(data=json.dumps({
            "state": st.state, "mode": st.mode, "speed_mps": round(st.speed_mps, 2), "detail": st.detail,
            "green": round(self._seen["green"], 5), "red": round(self._seen["red"], 5),
            "elapsed_s": round(time.monotonic() - st.started_at, 1) if st.started_at and st.state == "RUNNING"
            else (round(st.finished_at - st.started_at, 2) if st.finished_at and st.started_at else None),
        })))

    def _command_cb(self, message: String) -> None:
        try:
            data = json.loads(message.data)
            cmd = data.get("cmd")
            if cmd == "arm":
                self._green.reset()
                self._red.reset()
                profile = data.get("profile") if isinstance(data.get("profile"), dict) else {}
                self._apply(self._manager.arm(str(data.get("mode", "speed")), profile, now=time.monotonic()))
            elif cmd == "disarm":
                self._apply(self._manager.disarm("disarmed from the console"))
            elif cmd == "start":
                self._apply(self._manager.start(time.monotonic(), "manual start from the console"))
        except (ValueError, TypeError) as error:
            self.get_logger().error(f"Bad race command {message.data!r}: {error}")

    def _mission_cb(self, message: String) -> None:
        self._mission = message.data
        self._apply(self._manager.on_mission(time.monotonic(), self._mission, self._health))

    def _health_cb(self, message: String) -> None:
        self._health = message.data

    def _image_cb(self, message: CompressedImage) -> None:
        if self._manager.status.state not in ("ARMED", "RUNNING"):
            return
        now = time.monotonic()
        if now - self._last_frame < self._period:
            return
        self._last_frame = now
        image = cv2.imdecode(np.frombuffer(bytes(message.data), np.uint8), cv2.IMREAD_REDUCED_COLOR_2)
        if image is None:
            return
        reading = read_signals(image, self._cfg)
        self._seen = {"green": reading.green_fraction, "red": reading.red_fraction}
        green = self._green.update(reading)
        red = self._red.update(reading.red)
        self._apply(self._manager.on_signals(now, green, red))
        # Keep checking the supervisor while running (e.g. a refused start).
        self._apply(self._manager.on_mission(now, self._mission, self._health))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._autonomy_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
