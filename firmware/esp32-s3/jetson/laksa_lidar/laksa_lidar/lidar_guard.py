"""RPLIDAR LaserScan validation, transparent republishing, and diagnostics."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer

from .scan_quality import BAD, DEGRADED, GOOD, RateWindow, Thresholds, analyze_scan, health_state


def _sensor_qos(depth: int) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


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


class LidarGuard(Node):
    def __init__(self) -> None:
        super().__init__("lidar_guard")
        defaults = {
            "raw_scan_topic": "/scan_raw",
            "validated_scan_topic": "/laksa/lidar/scan_validated",
            "health_topic": "/laksa/lidar/health_state",
            "target_frame": "base_footprint",
            "expected_min_rate_hz": 5.0,
            "expected_max_rate_hz": 15.0,
            "stale_timeout_sec": 1.0,
            "minimum_angular_coverage_deg": 350.0,
            "maximum_consecutive_invalid": 3,
            "provisional_degraded_valid_ratio": 0.05,
            "diagnostic_publish_rate_hz": 1.0,
            "queue_depth": 5,
            "rate_window_scans": 30,
            "sample_count_tolerance_fraction": 0.02,
            "sample_count_tolerance_absolute": 4,
            "maximum_scan_time_sec": 0.5,
            # Non-blocking lookup keeps the single-threaded sensor callback cheap.
            "tf_timeout_sec": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._target_frame = self.get_parameter("target_frame").value
        self._thresholds = Thresholds(
            expected_min_rate_hz=float(self.get_parameter("expected_min_rate_hz").value),
            expected_max_rate_hz=float(self.get_parameter("expected_max_rate_hz").value),
            stale_timeout_sec=float(self.get_parameter("stale_timeout_sec").value),
            minimum_angular_coverage_deg=float(self.get_parameter("minimum_angular_coverage_deg").value),
            maximum_consecutive_invalid=int(self.get_parameter("maximum_consecutive_invalid").value),
            provisional_degraded_valid_ratio=float(
                self.get_parameter("provisional_degraded_valid_ratio").value
            ),
            sample_count_tolerance_fraction=float(self.get_parameter("sample_count_tolerance_fraction").value),
            sample_count_tolerance_absolute=int(self.get_parameter("sample_count_tolerance_absolute").value),
            maximum_scan_time_sec=float(self.get_parameter("maximum_scan_time_sec").value),
        )
        depth = int(self.get_parameter("queue_depth").value)
        self._rate = RateWindow(int(self.get_parameter("rate_window_scans").value))
        self._tf_timeout = Duration(seconds=float(self.get_parameter("tf_timeout_sec").value))
        self._tf_buffer = Buffer()
        _static_tf_feed(self, self._tf_buffer)
        self._last_arrival_monotonic = None
        self._latest_metrics = None
        self._latest_tf_valid = False
        self._consecutive_valid = 0
        self._consecutive_invalid = 0
        self._last_state = BAD
        self._last_reason = "scan_stale_or_missing"

        qos = _sensor_qos(depth)
        self._validated_pub = self.create_publisher(
            LaserScan, self.get_parameter("validated_scan_topic").value, qos
        )
        self._scan_sub = self.create_subscription(
            LaserScan, self.get_parameter("raw_scan_topic").value, self._scan_callback, qos
        )
        self._diagnostic_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._health_pub = self.create_publisher(String, self.get_parameter("health_topic").value, state_qos)
        rate = max(0.1, float(self.get_parameter("diagnostic_publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish_diagnostics)

    def _scan_callback(self, scan: LaserScan) -> None:
        now = time.monotonic()
        self._rate.add(now)
        self._last_arrival_monotonic = now
        metrics = analyze_scan(scan, self._thresholds)
        tf_valid = False
        tf_checkable = not (set(metrics.errors) - {"sample_count_mismatch"})
        if tf_checkable:
            try:
                tf_valid = self._tf_buffer.can_transform(
                    self._target_frame,
                    scan.header.frame_id,
                    Time.from_msg(scan.header.stamp),
                    timeout=self._tf_timeout,
                )
            except Exception:  # tf2 may reject malformed or out-of-buffer stamps.
                tf_valid = False
        self._latest_metrics = metrics
        self._latest_tf_valid = tf_valid
        valid = metrics.structural_valid and tf_valid
        if valid:
            self._consecutive_valid += 1
            self._consecutive_invalid = 0
            # Publish the received message itself: all header, timing, geometry,
            # ranges and intensities remain byte-for-byte semantically unchanged.
            self._validated_pub.publish(scan)
        else:
            self._consecutive_valid = 0
            self._consecutive_invalid += 1

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        age = math.inf if self._last_arrival_monotonic is None else now - self._last_arrival_monotonic
        state, reason = health_state(
            self._latest_metrics,
            self._rate.rate_hz,
            age,
            self._latest_tf_valid,
            self._consecutive_invalid,
            self._thresholds,
        )
        self._last_state, self._last_reason = state, reason
        level = {GOOD: DiagnosticStatus.OK, DEGRADED: DiagnosticStatus.WARN, BAD: DiagnosticStatus.ERROR}[state]
        metrics = self._latest_metrics
        values = {
            "health_state": state,
            "reason": reason,
            "raw_rate_hz": f"{self._rate.rate_hz:.3f}",
            "scan_age_sec": "inf" if not math.isfinite(age) else f"{age:.3f}",
            "frame_id": metrics.frame_id if metrics else "",
            "target_frame": self._target_frame,
            "tf_valid": str(self._latest_tf_valid).lower(),
            "structural_valid": str(bool(metrics and metrics.structural_valid)).lower(),
            "structural_errors": ",".join(metrics.errors) if metrics else "no_scan",
            "consecutive_valid": str(self._consecutive_valid),
            "consecutive_invalid": str(self._consecutive_invalid),
        }
        if metrics:
            values.update({
                "sample_count": str(metrics.sample_count),
                "expected_sample_count": str(metrics.expected_sample_count),
                "angular_coverage_rad": f"{metrics.angular_coverage_rad:.6f}",
                "angular_coverage_deg": f"{math.degrees(metrics.angular_coverage_rad):.3f}",
                "finite_return_count": str(metrics.finite_count),
                "finite_return_ratio": f"{metrics.finite_ratio:.6f}",
                "valid_return_count": str(metrics.valid_return_count),
                "valid_return_ratio": f"{metrics.valid_return_ratio:.6f}",
                "nan_count": str(metrics.nan_count),
                "inf_count": str(metrics.inf_count),
                "zero_count": str(metrics.zero_count),
                "below_range_count": str(metrics.below_range_count),
                "above_range_count": str(metrics.above_range_count),
                "finite_min_m": "n/a" if metrics.finite_min is None else f"{metrics.finite_min:.4f}",
                "finite_median_m": "n/a" if metrics.finite_median is None else f"{metrics.finite_median:.4f}",
                "finite_max_m": "n/a" if metrics.finite_max is None else f"{metrics.finite_max:.4f}",
            })
        status = DiagnosticStatus(
            level=level,
            name="LAKSA/RPLIDAR_A2M12",
            message=reason,
            hardware_id="rplidar_a2m12:/dev/laksa_lidar",
            values=[KeyValue(key=key, value=value) for key, value in values.items()],
        )
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostic_pub.publish(message)
        self._health_pub.publish(String(data=state))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
