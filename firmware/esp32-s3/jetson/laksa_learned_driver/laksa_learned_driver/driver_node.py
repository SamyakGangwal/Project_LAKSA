"""ROS 2 node: learned LiDAR driver in the supervisor's LiDAR Cruise slot.

The node never publishes an actuator command.  It publishes a candidate Twist on
``/laksa/lidar_cruise_cmd_vel``, which ``drive_supervisor`` only forwards after
the operator holds Xbox A for 3 s and every existing interlock passes (Xbox
freshness, ESP32/VESC telemetry, scan/odometry/ZED freshness, e-stop latch,
manual override, and the exploration speed cap).  It must replace, never run
beside, ``lidar_cruise_node.py``; a second publisher on the command topic makes
it stop and report ``CONTROL_ERROR`` so the supervisor aborts autonomy.
"""

from __future__ import annotations

import csv
import dataclasses
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String

from .policy import LearnedDriverPolicy
from .profiles import make_profile
from .recovery import RecoveryConfig, ReverseRecovery, obstacle_side, rear_free_distance
from .perception import lidar_ground_mask
from .safety import AvoidConfig, GovernorConfig, choose_steering, govern, path_free_distance, scan_points_base
from .smoothing import SteeringSmoother
from .scan_adapter import LidarMount, scan_to_vehicle_beams

MODEL_CAP_MIN_MPS = 0.25


class LearnedDriver(Node):
    def __init__(self) -> None:
        super().__init__("learned_driver")
        default_model = Path(get_package_share_directory("laksa_learned_driver")) / "models" / "laksa_tinylidarnet_v5.npz"
        self.declare_parameter("model_path", str(default_model))
        self.declare_parameter("scan_topic", "/laksa/lidar/scan_validated")
        self.declare_parameter("command_topic", "/laksa/lidar_cruise_cmd_vel")
        self.declare_parameter("enabled_topic", "/laksa/exploration_enabled")
        self.declare_parameter("status_topic", "/laksa/exploration_status")
        # The supervisor still clamps to exploration_max_erpm; this is a second,
        # policy-side cap and the conditioning input of the network.
        self.declare_parameter("speed_cap_mps", 0.24)
        self.declare_parameter("wheelbase_m", 0.324)
        self.declare_parameter("lidar_x_m", 0.31542)
        self.declare_parameter("lidar_yaw_rad", math.pi)
        self.declare_parameter("scan_timeout_sec", 0.5)
        # Clearance governor (independent of the network): stop margin in front of
        # the bumper, conservative deceleration and end-to-end latency.
        self.declare_parameter("stop_margin_m", 0.25)
        # How far ahead the clearance check looks; at 3 m/s the stopping
        # distance is ~5 m, so race mode uses a longer horizon.
        self.declare_parameter("governor_horizon_m", 4.0)
        self.declare_parameter("lateral_margin_m", 0.10)
        self.declare_parameter("brake_decel_mps2", 1.0)
        self.declare_parameter("latency_sec", 0.25)
        # Steer around obstacles: when the policy's arc has less than
        # avoid_clearance_m free, take the closest steering arc that does.
        self.declare_parameter("avoid_enabled", True)
        self.declare_parameter("avoid_clearance_m", 0.60)
        # Reverse-away recovery when every forward arc is blocked.
        self.declare_parameter("reverse_enabled", True)
        self.declare_parameter("reverse_speed_mps", 0.12)
        self.declare_parameter("reverse_time_sec", 2.5)
        self.declare_parameter("rear_clearance_m", 0.30)
        self.declare_parameter("max_recoveries", 4)
        # Camera obstacle layer from zed_perception (added to LiDAR, never replacing it).
        self.declare_parameter("use_camera", True)
        self.declare_parameter("camera_stale_sec", 0.6)
        # Steering smoothing (low-pass + rate limit) and per-session decision log.
        self.declare_parameter("steering_alpha", 0.4)
        self.declare_parameter("steering_rate_limit_radps", 1.0)
        self.declare_parameter("slope_aware_lidar", True)
        self.declare_parameter("decision_log", "")

        self._policy = LearnedDriverPolicy(str(self.get_parameter("model_path").value))
        self._cap = float(self.get_parameter("speed_cap_mps").value)
        self._wheelbase = float(self.get_parameter("wheelbase_m").value)
        self._timeout_ns = int(float(self.get_parameter("scan_timeout_sec").value) * 1e9)
        if not (math.isfinite(self._cap) and 0.0 < self._cap <= self._policy.output.speed_cap_norm_mps):
            raise ValueError("speed_cap_mps must be positive and within the trained range")
        self._mount = LidarMount(x_m=float(self.get_parameter("lidar_x_m").value),
                                 yaw_rad=float(self.get_parameter("lidar_yaw_rad").value))
        self._command_topic = str(self.get_parameter("command_topic").value)
        self._governor = GovernorConfig(
            wheelbase_m=self._wheelbase,
            stop_margin_m=float(self.get_parameter("stop_margin_m").value),
            lateral_margin_m=float(self.get_parameter("lateral_margin_m").value),
            decel_mps2=float(self.get_parameter("brake_decel_mps2").value),
            latency_s=float(self.get_parameter("latency_sec").value),
            horizon_m=float(self.get_parameter("governor_horizon_m").value),
        )

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._command_pub = self.create_publisher(Twist, self._command_topic, 10)
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), latched)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(Bool, str(self.get_parameter("enabled_topic").value), self._enabled_cb, latched)
        self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value), self._scan_cb,
                                 qos_profile_sensor_data)
        # Mode profile (obstacle / speed / explore) from the console or race manager.
        self.create_subscription(String, "/laksa/drive_profile", self._profile_cb, latched)
        self._profile_pub = self.create_publisher(String, "/laksa/drive_profile/active", latched)
        self.create_timer(0.10, self._watchdog)
        self.create_timer(1.0, self._diagnostics)
        self._telemetry_pub = self.create_publisher(String, "/laksa/driver/telemetry", 10)
        self.create_timer(0.2, self._telemetry)
        self._last_source = "none"

        self._enabled = False
        self._last_scan_ns = 0
        self._last_status = ""
        self._inference_ms = 0.0
        self._last_command = (0.0, 0.0)
        self._free_distance = float("nan")
        self._reverse_enabled = bool(self.get_parameter("reverse_enabled").value)
        self._avoid = AvoidConfig(
            steer_left_max_rad=self._policy.output.steering_left_max_rad,
            steer_right_max_rad=self._policy.output.steering_right_max_rad,
            clearance_m=float(self.get_parameter("avoid_clearance_m").value),
        ) if bool(self.get_parameter("avoid_enabled").value) else None
        self._avoid_target = None
        self._use_camera = bool(self.get_parameter("use_camera").value)
        self._camera_stale = float(self.get_parameter("camera_stale_sec").value)
        self._camera_xy = np.empty((0, 2))
        self._camera_time = 0.0
        self._person = {"factor": 1.0, "stop": False, "nearest_m": None}
        self._person_time = 0.0
        self._plane = None
        self._plane_time = 0.0
        self._slope_aware = bool(self.get_parameter("slope_aware_lidar").value)
        self._smoother = SteeringSmoother(float(self.get_parameter("steering_alpha").value),
                                          float(self.get_parameter("steering_rate_limit_radps").value))
        self._last_step_time = time.monotonic()
        # Always subscribed; a profile can switch the camera layer on and off live.
        self.create_subscription(PointCloud2, "/laksa/perception/obstacles", self._camera_cb, 10)
        self.create_subscription(String, "/laksa/perception/person", self._person_cb, 10)
        self.create_subscription(String, "/laksa/perception/ground", self._ground_cb, 10)
        self._log_writer = None
        log_path = str(self.get_parameter("decision_log").value)
        if log_path:
            Path(log_path).parent.mkdir(parents=True, exist_ok=True)
            self._log_file = open(log_path, "a", newline="", encoding="utf-8")
            self._log_writer = csv.writer(self._log_file)
            self._log_writer.writerow(["t", "status", "policy_steer", "steer_cmd", "speed_cmd", "free_m",
                                       "block_source", "lidar_free_m", "camera_free_m", "slope_deg",
                                       "person_factor", "lidar_pts", "camera_pts", "ground_filtered",
                                       "target_steer"])
        self._reverse_speed = float(self.get_parameter("reverse_speed_mps").value)
        self._recovery = ReverseRecovery(RecoveryConfig(
            reverse_speed_mps=min(self._reverse_speed, self._cap),
            reverse_time_s=float(self.get_parameter("reverse_time_sec").value),
            rear_clearance_m=float(self.get_parameter("rear_clearance_m").value),
            max_recoveries=int(self.get_parameter("max_recoveries").value),
        ))
        self._publish_status("IDLE")
        self.get_logger().info(
            f"Learned driver loaded {self._policy.metadata.get('architecture')} "
            f"({self._policy.scan.bins} bins), speed cap {self._cap:.3f} m/s"
        )

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _publish_status(self, value: str) -> None:
        if value != self._last_status:
            self._status_pub.publish(String(data=value))
            self._last_status = value
            self.get_logger().info(f"Learned driver state -> {value}")

    def _stop(self, status: str) -> None:
        self._command_pub.publish(Twist())
        self._publish_status(status)

    def _exclusive(self) -> bool:
        if self.count_publishers(self._command_topic) > 1:
            self._publish_status("CONTROL_ERROR")
            self.get_logger().error(
                f"Another publisher is active on {self._command_topic}; refusing to drive",
                throttle_duration_sec=2.0,
            )
            return False
        return True

    def _profile_cb(self, message: String) -> None:
        try:
            profile = make_profile(json.loads(message.data))
        except (ValueError, TypeError, AttributeError) as error:
            self.get_logger().error(f"Ignoring bad drive profile {message.data!r}: {error}")
            return
        self._cap = min(profile.speed_mps, self._policy.output.speed_cap_norm_mps)
        self._use_camera = profile.camera
        self._reverse_enabled = profile.reverse
        self._governor = dataclasses.replace(self._governor, horizon_m=profile.horizon_m)
        self._avoid = AvoidConfig(
            steer_left_max_rad=self._policy.output.steering_left_max_rad,
            steer_right_max_rad=self._policy.output.steering_right_max_rad,
            clearance_m=profile.avoid_clearance_m,
        ) if profile.avoid else None
        self._avoid_target = None
        self._recovery.cfg = dataclasses.replace(self._recovery.cfg,
                                                 reverse_speed_mps=min(self._reverse_speed, self._cap))
        self._profile_pub.publish(String(data=json.dumps(profile.to_dict())))
        self.get_logger().warn(f"Drive profile {profile.mode}: {profile.speed_mps:.2f} m/s, avoid={profile.avoid}, "
                               f"camera={profile.camera}, horizon={profile.horizon_m:.1f} m, reverse={profile.reverse}")

    def _camera_cb(self, message: PointCloud2) -> None:
        points = np.asarray(point_cloud2.read_points(message, field_names=("x", "y"), skip_nans=True))
        if points.dtype.names:
            points = np.column_stack([points["x"], points["y"]])
        self._camera_xy = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        self._camera_time = time.monotonic()

    def _person_cb(self, message: String) -> None:
        try:
            self._person = json.loads(message.data)
            self._person_time = time.monotonic()
        except (ValueError, TypeError):
            pass

    def _ground_cb(self, message: String) -> None:
        try:
            data = json.loads(message.data)
        except (ValueError, TypeError):
            return
        plane = data.get("plane")
        if plane and len(plane) == 3:
            self._plane, self._plane_time = tuple(float(v) for v in plane), time.monotonic()

    def _fresh_plane(self):
        if not self._slope_aware or time.monotonic() - self._plane_time > self._camera_stale:
            return None
        return self._plane

    def _camera_layer(self):
        """Fresh camera obstacle points and person rule, or LiDAR-only fallback."""
        now = time.monotonic()
        if not self._use_camera:
            return np.empty((0, 2)), 1.0, False
        points = self._camera_xy if now - self._camera_time <= self._camera_stale else np.empty((0, 2))
        if now - self._camera_time > self._camera_stale:
            self.get_logger().warning("Camera obstacle layer stale; using LiDAR only", throttle_duration_sec=5.0)
        person = self._person if now - self._person_time <= self._camera_stale else {}
        return points, float(person.get("factor", 1.0)), bool(person.get("stop", False))

    def _enabled_cb(self, message: Bool) -> None:
        enabled = bool(message.data)
        if enabled == self._enabled:
            return
        self._enabled = enabled
        self._recovery.reset()
        self._smoother.reset()
        self._avoid_target = None
        if enabled:
            self._publish_status("WAITING_FOR_SCAN")
        else:
            self._stop("IDLE")

    def _scan_cb(self, message: LaserScan) -> None:
        self._last_scan_ns = self._now_ns()
        if not self._enabled or not self._exclusive():
            return
        if not message.ranges or not math.isfinite(float(message.angle_increment)):
            self._stop("INVALID_SCAN")
            return
        started = time.perf_counter()
        ranges, angles = scan_to_vehicle_beams(
            message.ranges, float(message.angle_min), float(message.angle_increment),
            float(message.range_min), float(message.range_max), self._mount,
        )
        policy_steering, speed = self._policy.act(ranges, angles, max(self._cap, MODEL_CAP_MIN_MPS))
        speed = min(speed, self._cap)
        if not (math.isfinite(policy_steering) and math.isfinite(speed)):
            self._stop("CONTROL_ERROR")
            return
        camera_points, person_factor, person_stop = self._camera_layer()
        lidar_points = scan_points_base(ranges, angles, self._mount.x_m)
        plane = self._fresh_plane()
        ground = lidar_ground_mask(lidar_points, plane)
        if np.any(ground):
            lidar_points = lidar_points[~ground]   # rising ground ahead, not a wall
        points = np.vstack([lidar_points, camera_points]) if camera_points.size else lidar_points
        # Steer around an obstacle on the policy's arc instead of stopping for it.
        target, avoiding, all_blocked = policy_steering, False, False
        if self._avoid is not None:
            avoid = choose_steering(points, policy_steering, self._governor, self._avoid, self._avoid_target)
            target, avoiding, all_blocked = avoid.steering_rad, avoid.avoiding, avoid.all_blocked
            self._avoid_target = target if avoiding else None
        now = time.monotonic()
        steering = self._smoother.update(target, now - self._last_step_time)
        self._last_step_time = now
        if person_stop:
            # A person close ahead: stop and wait (no reversing near people).
            self._last_command = (steering, 0.0)
            self._command_pub.publish(Twist())
            self._publish_status("PERSON_STOP")
            self.get_logger().warning("Person close ahead; holding", throttle_duration_sec=2.0)
            return
        speed *= person_factor
        # The governor always checks the steering actually commanded (smoothed), so
        # while the wheels swing toward a detour the car waits instead of driving
        # into the obstacle.  Recovery starts only when no arc is free at all.
        governed = govern(points, steering, speed, self._governor)
        forward_blocked = all_blocked if self._avoid is not None else governed.blocked
        self._free_distance = governed.free_distance_m
        lidar_free = path_free_distance(lidar_points, steering, self._governor)
        camera_free = path_free_distance(camera_points, steering, self._governor) if camera_points.size \
            else self._governor.horizon_m
        source = "none" if not governed.blocked else ("camera" if camera_free < lidar_free else "lidar")
        self._last_source = source
        if self._reverse_enabled:
            speed, steering, status = self._recovery.step(
                time.monotonic(), forward_blocked,
                rear_free_distance(points, self._governor, self._recovery.cfg),
                obstacle_side(points, self._governor),
                (governed.speed_mps, steering),
            )
        elif forward_blocked:
            speed, status = 0.0, "BLOCKED"
        else:
            speed, status = governed.speed_mps, "LEARNED_DRIVING"
        if avoiding and status == "LEARNED_DRIVING":
            status = "AVOIDING"
        self._inference_ms = 1000.0 * (time.perf_counter() - started)
        if self._log_writer is not None:
            slope = None if plane is None else round(math.degrees(math.atan(math.hypot(plane[0], plane[1]))), 2)
            self._log_writer.writerow([f"{time.time():.3f}", status, f"{policy_steering:.4f}", f"{steering:.4f}",
                                       f"{speed:.3f}", f"{governed.free_distance_m:.3f}", source,
                                       f"{lidar_free:.3f}", f"{camera_free:.3f}", slope, f"{person_factor:.2f}",
                                       lidar_points.shape[0], camera_points.shape[0], int(np.sum(ground)),
                                       f"{target:.4f}"])
            self._log_file.flush()
        if forward_blocked and status in ("RECOVERY_PAUSE", "BLOCKED"):
            self.get_logger().warning(
                f"Forward path blocked by {source}: {governed.free_distance_m:.2f} m ahead of the bumper "
                f"(lidar {lidar_free:.2f} m, camera {camera_free:.2f} m, slope "
                f"{'n/a' if plane is None else f'{math.degrees(math.atan(math.hypot(plane[0], plane[1]))):.1f} deg'})",
                throttle_duration_sec=1.0)
        if status == "BLOCKED":
            # BLOCKED makes drive_supervisor abort autonomy and return to manual.
            self._last_command = (steering, 0.0)
            self._stop("BLOCKED")
            self.get_logger().warning(
                f"Path blocked {governed.free_distance_m:.2f} m ahead of the bumper and no recovery left; stopping",
                throttle_duration_sec=1.0,
            )
            return
        output = Twist()
        output.linear.x = speed
        output.angular.z = speed * math.tan(steering) / self._wheelbase
        self._command_pub.publish(output)
        self._last_command = (steering, speed)
        self._publish_status(status)

    def _watchdog(self) -> None:
        if self._enabled and (self._last_scan_ns == 0 or self._now_ns() - self._last_scan_ns > self._timeout_ns):
            self._stop("SCAN_TIMEOUT")

    def _telemetry(self) -> None:
        """Small live summary for the console (5 Hz)."""
        free = self._free_distance
        self._telemetry_pub.publish(String(data=json.dumps({
            "status": self._last_status, "speed_cmd": round(self._last_command[1], 3),
            "steer_cmd": round(self._last_command[0], 3),
            "free_m": None if not math.isfinite(free) else round(free, 2), "blocked_by": self._last_source,
            "cap": round(self._cap, 2), "inference_ms": round(self._inference_ms, 2),
        })))

    def _diagnostics(self) -> None:
        status = DiagnosticStatus(
            level=DiagnosticStatus.OK if self._last_status not in ("CONTROL_ERROR", "SCAN_TIMEOUT", "INVALID_SCAN", "BLOCKED") else DiagnosticStatus.WARN,
            name="laksa_learned_driver", hardware_id="jetson", message=self._last_status,
        )
        status.values = [
            KeyValue(key="enabled", value=str(self._enabled)),
            KeyValue(key="inference_ms", value=f"{self._inference_ms:.3f}"),
            KeyValue(key="speed_cap_mps", value=f"{self._cap:.3f}"),
            KeyValue(key="last_steering_rad", value=f"{self._last_command[0]:.4f}"),
            KeyValue(key="last_speed_mps", value=f"{self._last_command[1]:.4f}"),
            KeyValue(key="path_free_distance_m", value=f"{self._free_distance:.3f}"),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LearnedDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._command_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
