"""ZED perception node: neural detections + depth cloud -> vehicle-frame obstacles.

Publishes (all in base_footprint):
  /laksa/perception/obstacles   sensor_msgs/PointCloud2, z = 0 obstacle points
  /laksa/perception/person      std_msgs/String JSON {factor, stop, nearest_m}
  /laksa/perception/detections  std_msgs/String JSON list of detected objects
  /laksa/perception/ground      std_msgs/String JSON {plane: [a, b, c] | null}
                                ground z = a*x + b*y + c (slope-aware heights)
Stale inputs are simply left out; the learned driver treats the camera layer
as an addition to the LiDAR, never as a replacement.  The obstacle cloud keeps
the depth cloud's capture stamp, so the driver can measure its age and shift it
by the car's motion since capture; the diagnostics report that lag.
"""

from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException
from zed_msgs.msg import ObjectsStamped

from .perception import (Detection, PerceptionConfig, detection_obstacles, filter_cloud, fit_ground_grid,
                         fit_ground_plane, person_speed_rule)


def _matrix(transform) -> tuple[np.ndarray, np.ndarray]:
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    t = transform.translation
    return rotation, np.array([t.x, t.y, t.z])


def _cloud_xyz(message: PointCloud2) -> np.ndarray:
    """x, y, z as float64 via NumPy views on the raw buffer (no per-point Python)."""
    offsets = {f.name: f.offset for f in message.fields if f.datatype == 7}   # FLOAT32
    count = int(message.width) * int(message.height)
    if count == 0 or not all(k in offsets for k in "xyz") or message.is_bigendian \
            or int(message.row_step) != int(message.width) * int(message.point_step):
        points = point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)
        array = np.asarray(points)
        if array.dtype.names:
            return np.column_stack([array["x"], array["y"], array["z"]]).astype(np.float64)
        return np.asarray(array, dtype=np.float64).reshape(-1, 3)
    data = memoryview(message.data)
    columns = [np.ndarray((count,), dtype=np.float32, buffer=data, offset=offsets[k], strides=(message.point_step,))
               for k in "xyz"]
    xyz = np.column_stack(columns).astype(np.float64)
    return xyz[np.all(np.isfinite(xyz), axis=1)]


def _static_tf_feed(node, buffer) -> None:
    """Feed only /tf_static into ``buffer``.

    This node only needs fixed sensor transforms; a full TransformListener
    would also deserialize every high-rate /tf message in Python.
    """
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage

    qos = QoSProfile(depth=100)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE

    def _on_static(message) -> None:
        for transform in message.transforms:
            buffer.set_transform_static(transform, "laksa_static_feed")

    node._laksa_static_tf_sub = node.create_subscription(TFMessage, "/tf_static", _on_static, qos)


class ZedPerception(Node):
    def __init__(self) -> None:
        super().__init__("zed_perception")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("objects_topic", "/zed/zed_node/obj_det/objects")
        self.declare_parameter("cloud_topic", "/zed/zed_node/point_cloud/cloud_registered")
        self.declare_parameter("stale_sec", 0.6)
        self.declare_parameter("min_confidence", 40.0)
        self._base = str(self.get_parameter("base_frame").value)
        self._stale = float(self.get_parameter("stale_sec").value)
        self._min_confidence = float(self.get_parameter("min_confidence").value)
        self._cfg = PerceptionConfig()
        self._tf = Buffer()
        _static_tf_feed(self, self._tf)
        self._obstacle_pub = self.create_publisher(PointCloud2, "/laksa/perception/obstacles", 10)
        self._person_pub = self.create_publisher(String, "/laksa/perception/person", 10)
        self._detections_pub = self.create_publisher(String, "/laksa/perception/detections", 10)
        self._ground_pub = self.create_publisher(String, "/laksa/perception/ground", 10)
        self._rng = np.random.default_rng(0)
        self._plane = None
        self._grid = None
        self._plane_time = 0.0
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(ObjectsStamped, str(self.get_parameter("objects_topic").value),
                                 self._objects_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, str(self.get_parameter("cloud_topic").value),
                                 self._cloud_cb, qos_profile_sensor_data)
        self.create_timer(0.1, self._publish)
        self.create_timer(1.0, self._diagnostics)
        self._detections: list[Detection] = []
        self._detections_time = 0.0
        self._cloud_xy = np.empty((0, 2))
        self._cloud_time = 0.0
        self._cloud_stamp = None             # capture stamp of the depth cloud
        self._latency: list[float] = []      # capture -> received, seconds (since last report)
        self._last_latency = math.nan
        self.create_timer(10.0, self._report_latency)
        self._last_error = ""
        self._counts = {"objects_msgs": 0, "cloud_msgs": 0}

    def _to_base(self, frame: str):
        try:
            transform = self._tf.lookup_transform(self._base, frame, Time(), timeout=Duration(seconds=0.05))
        except TransformException as error:
            self._last_error = f"TF {frame}->{self._base}: {error}"
            return None
        return _matrix(transform.transform)

    def _objects_cb(self, message: ObjectsStamped) -> None:
        self._counts["objects_msgs"] += 1
        tf = self._to_base(message.header.frame_id)
        if tf is None:
            return
        rotation, translation = tf
        detections = []
        for obj in message.objects:
            if float(obj.confidence) < self._min_confidence:
                continue
            position = rotation @ np.asarray(obj.position, dtype=np.float64) + translation
            corners = np.array([c.kp for c in obj.bounding_box_3d.corners], dtype=np.float64)
            if corners.shape != (8, 3) or not np.all(np.isfinite(corners)) or not np.any(corners):
                half = np.asarray(obj.dimensions_3d, dtype=np.float64) / 2.0
                offsets = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
                corners_base = position + offsets
            else:
                corners_base = corners @ rotation.T + translation
            if not np.all(np.isfinite(position)):
                continue
            detections.append(Detection(str(obj.label), float(obj.confidence), position, corners_base))
        self._detections, self._detections_time = detections, time.monotonic()

    def _cloud_cb(self, message: PointCloud2) -> None:
        self._counts["cloud_msgs"] += 1
        stamp_ns = Time.from_msg(message.header.stamp).nanoseconds
        if stamp_ns > 0:
            self._last_latency = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
            self._latency.append(self._last_latency)
        tf = self._to_base(message.header.frame_id)
        if tf is None:
            return
        rotation, translation = tf
        xyz = _cloud_xyz(message)
        if xyz.size:
            xyz = xyz @ rotation.T + translation
        sample = xyz if xyz.shape[0] <= 6000 else xyz[self._rng.choice(xyz.shape[0], 6000, replace=False)]
        plane = fit_ground_plane(sample, self._cfg, self._rng)
        now = time.monotonic()
        if plane is not None:
            self._plane, self._plane_time = plane, now
        use_plane = self._plane if now - self._plane_time <= self._stale else None
        self._grid = fit_ground_grid(sample, self._cfg)   # ~20 points per cell is plenty
        self._cloud_xy, self._cloud_time = filter_cloud(xyz, self._cfg, use_plane, self._grid), now
        self._cloud_stamp = message.header.stamp if stamp_ns > 0 else None

    def _publish(self) -> None:
        now = time.monotonic()
        detections = self._detections if now - self._detections_time <= self._stale else []
        cloud = self._cloud_xy if now - self._cloud_time <= self._stale else np.empty((0, 2))
        obstacles = np.vstack([cloud, detection_obstacles(detections, self._cfg)])
        fresh_cloud = now - self._cloud_time <= self._stale and self._cloud_stamp is not None
        header = Header(stamp=self._cloud_stamp if fresh_cloud else self.get_clock().now().to_msg(),
                        frame_id=self._base)
        xyz = np.column_stack([obstacles, np.zeros(obstacles.shape[0])]) if obstacles.size else np.empty((0, 3))
        self._obstacle_pub.publish(point_cloud2.create_cloud_xyz32(header, xyz.astype(np.float32).tolist()))
        factor, stop, nearest = person_speed_rule(detections, self._cfg)
        self._person_pub.publish(String(data=json.dumps({
            "factor": factor, "stop": stop,
            "nearest_m": None if math.isinf(nearest) else round(nearest, 3),
            "fresh": now - self._detections_time <= self._stale,
        })))
        plane_fresh = now - self._plane_time <= self._stale
        self._ground_pub.publish(String(data=json.dumps({
            "plane": [round(v, 5) for v in self._plane] if plane_fresh and self._plane else None,
            "slope_deg": round(math.degrees(math.atan(math.hypot(self._plane[0], self._plane[1]))), 2)
            if plane_fresh and self._plane else None,
            "grid": self._grid.to_dict() if self._grid is not None and now - self._cloud_time <= self._stale else None,
        })))
        self._detections_pub.publish(String(data=json.dumps([
            {"label": d.label, "confidence": round(d.confidence, 1),
             "x": round(float(d.position_xyz[0]), 3), "y": round(float(d.position_xyz[1]), 3)}
            for d in detections
        ])))

    def _report_latency(self) -> None:
        if not self._latency:
            return
        lat = np.asarray(self._latency)
        self._latency = []
        self.get_logger().info(
            f"Depth cloud lag (capture -> here): median {np.median(lat):.3f} s, max {lat.max():.3f} s, "
            f"{lat.size / 10.0:.1f} clouds/s")

    def _diagnostics(self) -> None:
        now = time.monotonic()
        det_age = now - self._detections_time if self._detections_time else math.inf
        cloud_age = now - self._cloud_time if self._cloud_time else math.inf
        ok = det_age <= self._stale and cloud_age <= self._stale
        status = DiagnosticStatus(level=DiagnosticStatus.OK if ok else DiagnosticStatus.WARN,
                                  name="laksa_zed_perception", hardware_id="zed2i",
                                  message="ok" if ok else (self._last_error or "camera inputs stale"))
        status.values = [
            KeyValue(key="detections", value=str(len(self._detections))),
            KeyValue(key="detections_age_sec", value=f"{det_age:.2f}"),
            KeyValue(key="cloud_obstacle_points", value=str(self._cloud_xy.shape[0])),
            KeyValue(key="cloud_age_sec", value=f"{cloud_age:.2f}"),
            KeyValue(key="cloud_capture_latency_sec", value=f"{self._last_latency:.3f}"),
            KeyValue(key="objects_msgs", value=str(self._counts["objects_msgs"])),
            KeyValue(key="cloud_msgs", value=str(self._counts["cloud_msgs"])),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZedPerception()
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
