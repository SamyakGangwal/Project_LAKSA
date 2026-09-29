"""LAKSA Console: map, camera, detections, status and hold-to-run control.

Binds to 127.0.0.1 by default (reach it through an SSH tunnel) or, when the
launcher sees the car's own hotspot, to that hotspot address only.  Every
request needs the token: a fixed one from ``token_file`` (generated once on the
Jetson, mode 600, never committed) or a random one per start.

    hotspot:  http://10.42.0.1:8095/?token=<fixed token>
    tunnel:   ssh -L 8095:127.0.0.1:8095 samyak@<jetson>   ->   http://localhost:8095/?token=...

Drive control publishes sensor_msgs/Joy exactly like the terminal operator
(neutral sticks; A for the first seconds of a hold, B = STOP, Y = REARM), so
drive_supervisor keeps every gate.  Releasing HOLD, closing the page or losing
the tunnel stops /joy and the supervisor brakes within 0.5 s.
"""

from __future__ import annotations

import asyncio
import base64
import json
import math
import secrets
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from aiohttp import web
from geometry_msgs.msg import PoseStamped
from laksa_interfaces.msg import VehicleState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Joy, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .scan_adapter import LidarMount, scan_to_vehicle_beams

PAGE = Path(__file__).with_name("console_page.html")
A_BUTTON, B_BUTTON, Y_BUTTON = 0, 1, 3


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Console(Node):
    def __init__(self) -> None:
        super().__init__("laksa_console")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8095)
        self.declare_parameter("heartbeat_timeout_sec", 0.3)
        self.declare_parameter("engage_hold_sec", 3.5)
        self.declare_parameter("token_file", "")
        self._host = str(self.get_parameter("host").value)
        self._port = int(self.get_parameter("port").value)
        self._timeout = float(self.get_parameter("heartbeat_timeout_sec").value)
        self._engage = float(self.get_parameter("engage_hold_sec").value)
        self._token = self._load_token(str(self.get_parameter("token_file").value))
        self._lock = threading.Lock()
        self._mount = LidarMount()

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._joy_pub = self.create_publisher(Joy, "/joy", 10)
        self._start_pub = self.create_publisher(PoseStamped, "/laksa/console/start", latched)
        self._goal_pub = self.create_publisher(PoseStamped, "/laksa/console/goal", latched)
        self._tf = Buffer()
        self._tf_listener = None
        self._clients = 0
        self._last_client_time = 0.0
        self._viewer_subs = []

        self._status = {"mission": "UNKNOWN", "health": "UNKNOWN", "estop": None, "estop_reason": "",
                        "driver": "UNKNOWN", "speed_mps": None, "battery_v": None, "vesc_fault": None,
                        "person": {}, "pose_frame": None}
        self._map_png = None
        self._map_info = None
        self._map_revision = 0
        self._pose = None
        self._trail: list[tuple[float, float]] = []
        self._lidar_xy = []
        self._detections = []
        self._jpeg = None
        self._jpeg_time = 0.0
        self._start = None
        self._goal = None
        self._last_heartbeat = 0.0
        self._hold_started = 0.0
        self._pulse = None

        s = self._status
        self.create_subscription(String, "/laksa/mission_state", lambda m: s.__setitem__("mission", m.data), latched)
        self.create_subscription(String, "/laksa/autonomy_health", lambda m: s.__setitem__("health", m.data), latched)
        self.create_subscription(Bool, "/laksa/emergency_stop", lambda m: s.__setitem__("estop", m.data), latched)
        self.create_subscription(String, "/laksa/emergency_stop_reason",
                                 lambda m: s.__setitem__("estop_reason", m.data), latched)
        self.create_subscription(String, "/laksa/exploration_status", lambda m: s.__setitem__("driver", m.data), latched)
        self.create_subscription(String, "/laksa/perception/person", self._person_cb, 10)
        self.create_subscription(String, "/laksa/perception/detections", self._detections_cb, 10)
        self.create_subscription(VehicleState, "/laksa/state", self._state_cb, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, latched)
        # Camera, LiDAR, odometry and TF are only subscribed while someone is
        # viewing the page (see _reconcile_viewer_subscriptions).
        self.create_timer(0.5, self._reconcile_viewer_subscriptions)
        self.create_timer(0.05, self._publish_joy)
        threading.Thread(target=self._serve, daemon=True).start()
        self.get_logger().warn(
            f"LAKSA Console on http://{self._host}:{self._port}/?token={self._token}  "
            f"(tunnel: ssh -L {self._port}:127.0.0.1:{self._port} <user>@<jetson>)")

    def _load_token(self, path: str) -> str:
        if path:
            try:
                token = Path(path).expanduser().read_text(encoding="utf-8").strip()
                if len(token) >= 8:
                    return token
                self.get_logger().error(f"Token in {path} is shorter than 8 characters; using a random token")
            except OSError as error:
                self.get_logger().error(f"Cannot read token file {path}: {error}; using a random token")
        return secrets.token_urlsafe(12)

    # --------------------------------------------- demand-driven subscriptions
    def _reconcile_viewer_subscriptions(self):
        with self._lock:
            wanted = self._clients > 0 or time.monotonic() - self._last_client_time < 10.0
        if wanted and not self._viewer_subs:
            self._tf_listener = TransformListener(self._tf, self)
            self._viewer_subs = [
                self.create_subscription(Odometry, "/laksa/odometry/fused", self._odom_cb, 10),
                self.create_subscription(LaserScan, "/laksa/lidar/scan_validated", self._scan_cb,
                                         qos_profile_sensor_data),
                self.create_subscription(CompressedImage, "/zed/zed_node/rgb/color/rect/image/compressed",
                                         self._image_cb, qos_profile_sensor_data),
            ]
            self.get_logger().info("Viewer connected: live camera/LiDAR/pose subscriptions on")
        elif not wanted and self._viewer_subs:
            for sub in self._viewer_subs:
                self.destroy_subscription(sub)
            self._viewer_subs = []
            if self._tf_listener is not None:
                for name in ("tf_sub", "tf_static_sub"):
                    sub = getattr(self._tf_listener, name, None)
                    if sub is not None:
                        self.destroy_subscription(sub)
                self._tf_listener = None
            self.get_logger().info("No viewers: live subscriptions off")

    # ------------------------------------------------------------ ROS inputs
    def _person_cb(self, message):
        try:
            self._status["person"] = json.loads(message.data)
        except ValueError:
            pass

    def _detections_cb(self, message):
        try:
            self._detections = json.loads(message.data)
        except ValueError:
            pass

    def _state_cb(self, message: VehicleState):
        vesc = message.vesc
        self._status.update(speed_mps=round(float(vesc.vehicle_linear_velocity_mps), 3),
                            battery_v=round(float(vesc.input_voltage_v), 2), vesc_fault=int(vesc.fault_code))

    def _map_cb(self, message: OccupancyGrid):
        info = message.info
        grid = np.asarray(message.data, dtype=np.int16).reshape(info.height, info.width)
        image = np.full(grid.shape, 205, dtype=np.uint8)            # unknown: grey
        image[(grid >= 0) & (grid < 50)] = 255                      # free: white
        image[grid >= 50] = 30                                      # occupied: black
        ok, png = cv2.imencode(".png", np.flipud(image))
        if not ok:
            return
        with self._lock:
            self._map_png = base64.b64encode(png.tobytes()).decode("ascii")
            self._map_info = {"width": info.width, "height": info.height, "resolution": info.resolution,
                              "origin_x": info.origin.position.x, "origin_y": info.origin.position.y,
                              "frame": message.header.frame_id}
            self._map_revision += 1

    def _odom_cb(self, message: Odometry):
        # Prefer the map frame (RTAB-Map); fall back to odom if no map TF exists.
        if self._tf_listener is None:
            return
        pose, frame = None, "odom"
        try:
            t = self._tf.lookup_transform("map", "base_footprint", Time()).transform
            pose, frame = (t.translation.x, t.translation.y, _yaw(t.rotation)), "map"
        except TransformException:
            p = message.pose.pose
            pose = (p.position.x, p.position.y, _yaw(p.orientation))
        with self._lock:
            self._pose = pose
            self._status["pose_frame"] = frame
            if not self._trail or math.hypot(pose[0] - self._trail[-1][0], pose[1] - self._trail[-1][1]) > 0.05:
                self._trail.append((round(pose[0], 3), round(pose[1], 3)))
                self._trail = self._trail[-2000:]

    def _scan_cb(self, message: LaserScan):
        ranges, angles = scan_to_vehicle_beams(message.ranges, float(message.angle_min),
                                               float(message.angle_increment), float(message.range_min),
                                               float(message.range_max), self._mount)
        finite = np.isfinite(ranges)
        r, a = ranges[finite][::3], angles[finite][::3]
        xy = np.column_stack([self._mount.x_m + r * np.cos(a), r * np.sin(a)])
        self._lidar_xy = np.round(xy, 3).tolist()

    def _image_cb(self, message: CompressedImage):
        # The ZED already publishes JPEG: forward the bytes, no decode/encode.
        now = time.monotonic()
        if now - self._jpeg_time < 0.4 or "jpeg" not in message.format.lower() and "jpg" not in message.format.lower():
            return
        self._jpeg, self._jpeg_time = bytes(message.data), now

    # --------------------------------------------------------- drive control
    def _joy(self, pressed=()):
        message = Joy()
        message.header.stamp = self.get_clock().now().to_msg()
        message.axes = [0.0] * 8
        message.buttons = [1 if i in pressed else 0 for i in range(12)]
        self._joy_pub.publish(message)

    def _publish_joy(self):
        now = time.monotonic()
        with self._lock:
            pulse = self._pulse if self._pulse and now < self._pulse[1] else None
            alive = now - self._last_heartbeat <= self._timeout
            engaging = alive and now - self._hold_started < self._engage
        if pulse:
            self._joy((pulse[0],))
        elif alive:
            self._joy((A_BUTTON,) if engaging else ())
        # Otherwise publish nothing: the supervisor's controller timeout brakes.

    def _set_marker(self, kind: str, x: float, y: float):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = (self._map_info or {}).get("frame") or "map"
        message.pose.position.x, message.pose.position.y = float(x), float(y)
        message.pose.orientation.w = 1.0
        if kind == "start":
            self._start = (x, y)
            self._start_pub.publish(message)
        else:
            self._goal = (x, y)
            self._goal_pub.publish(message)

    # ------------------------------------------------------------------ web
    def _snapshot(self, map_revision_seen: int) -> dict:
        with self._lock:
            state = {
                "type": "state",
                "status": dict(self._status),
                "holding": time.monotonic() - self._last_heartbeat <= self._timeout,
                "pose": self._pose,
                "trail": self._trail[-600:],
                "lidar": self._lidar_xy,
                "detections": self._detections,
                "start": self._start,
                "goal": self._goal,
                "map_revision": self._map_revision,
            }
            if self._map_png and self._map_revision != map_revision_seen:
                state["map"] = {"png": self._map_png, **self._map_info}
        return state

    def _serve(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        def check(request):
            if request.query.get("token") != self._token:
                raise web.HTTPForbidden(text="missing or wrong token")

        async def index(request):
            check(request)
            return web.Response(text=PAGE.read_text(encoding="utf-8"), content_type="text/html")

        async def camera(request):
            check(request)
            if self._jpeg is None:
                raise web.HTTPNotFound(text="no camera frame yet")
            return web.Response(body=self._jpeg, content_type="image/jpeg",
                                headers={"Cache-Control": "no-store"})

        async def ws_handler(request):
            check(request)
            ws = web.WebSocketResponse(heartbeat=2.0)
            await ws.prepare(request)
            with self._lock:
                self._clients += 1
                self._last_client_time = time.monotonic()
            seen = {"map": -1}

            async def sender():
                while not ws.closed:
                    state = self._snapshot(seen["map"])
                    if "map" in state:
                        seen["map"] = state["map_revision"]
                    await ws.send_str(json.dumps(state))
                    await asyncio.sleep(0.2)

            task = asyncio.ensure_future(sender())
            try:
                async for message in ws:
                    try:
                        data = json.loads(message.data)
                    except ValueError:
                        continue
                    now = time.monotonic()
                    kind = data.get("cmd")
                    with self._lock:
                        if kind == "hold":
                            if now - self._last_heartbeat > self._timeout:
                                self._hold_started = now
                            self._last_heartbeat = now
                        elif kind == "release":
                            self._last_heartbeat = 0.0
                        elif kind == "stop":
                            self._last_heartbeat = 0.0
                            self._pulse = (B_BUTTON, now + 0.6)
                        elif kind == "rearm":
                            self._pulse = (Y_BUTTON, now + 0.6)
                    if kind in ("start", "goal"):
                        x, y = float(data["x"]), float(data["y"])
                        if math.isfinite(x) and math.isfinite(y):
                            self._set_marker(kind, x, y)
                    elif kind == "clear_trail":
                        with self._lock:
                            self._trail = []
            finally:
                task.cancel()
                with self._lock:
                    self._last_heartbeat = 0.0
                    self._clients = max(0, self._clients - 1)
                    self._last_client_time = time.monotonic()
            return ws

        app = web.Application()
        app.router.add_get("/", index)
        app.router.add_get("/camera.jpg", camera)
        app.router.add_get("/ws", ws_handler)
        runner = web.AppRunner(app)
        loop.run_until_complete(runner.setup())
        loop.run_until_complete(web.TCPSite(runner, self._host, self._port).start())
        loop.run_forever()


def main(args=None):
    rclpy.init(args=args)
    node = Console()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
