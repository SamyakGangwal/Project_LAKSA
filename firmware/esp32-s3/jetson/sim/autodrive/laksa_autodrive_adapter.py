#!/usr/bin/env python3
"""LAKSA <-> AutoDRIVE (RoboRacer digital twin) adapter: the car's software drives the simulated car.

Runs next to AutoDRIVE's own ``autodrive_bridge`` (devkit) and stands in for the
hardware the LAKSA stack expects:

  simulator                                  LAKSA stack
  /autodrive/roboracer_1/lidar      ->  /laksa/lidar/scan_validated   (LiDAR, 270 deg, forward)
  /autodrive/roboracer_1/odom       ->  /laksa/odometry/fused + TF odom->base_footprint
  (none)                            ->  /zed/zed_node/point_cloud/cloud_registered (empty: no ZED in sim)
  throttle/steering feedback, speed ->  /laksa/state, /laksa/vesc/state   (a fake ESP32 + VESC)
  /autodrive/.../throttle_command   <-  /laksa/command (DriveCommand)     speed PI + brake + 0.5 s watchdog
  /autodrive/.../steering_command   <-  /laksa/command steering

The fake ESP32 behaves like the real one: commands older than 0.5 s brake, the
brake bit brakes, and ``reject_above_mps`` (off by default) reproduces the car's
firmware speed limit (commands above it are rejected and the brake is held).
Nothing here talks to real hardware.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from laksa_interfaces.msg import DriveCommand, VehicleState
from laksa_interfaces.msg import VescState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

ERPM_PER_MPS = 4142.0          # the car's drivetrain conversion (supervisor geometry)
MAX_STEERING_NORM = 0.523      # supervisor's servo_reported_limit
LEFT_LIMIT, RIGHT_LIMIT = 0.523, 0.288


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Adapter(Node):
    def __init__(self) -> None:
        super().__init__("laksa_autodrive_adapter")
        p = self.declare_parameter
        p("vehicle", "roboracer_1")
        p("sim_max_steering_rad", 0.5236)    # road-wheel angle for steering_command = 1
        p("sim_steering_sign", 1.0)          # flip if the sim steers the wrong way
        p("kp", 0.6)                         # speed PI -> throttle [-1, 1]
        p("ki", 0.8)
        p("kff", 0.12)                       # throttle per m/s feed-forward
        p("max_throttle", 0.6)
        p("brake_gain", 1.0)                 # throttle against motion while braking
        p("command_timeout_sec", 0.5)        # ESP32 watchdog
        p("reject_above_mps", 0.0)           # >0: copy the car firmware's speed limit
        p("scan_rate_hz", 12.0)              # the real RPLIDAR rate; the sim sends 40 Hz
        p("lidar_x_m", 0.2733)
        v = str(self.get_parameter("vehicle").value)
        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self._cfg = {k: float(g(k)) for k in ("sim_max_steering_rad", "sim_steering_sign", "kp", "ki", "kff",
                                               "max_throttle", "brake_gain", "command_timeout_sec",
                                               "reject_above_mps", "scan_rate_hz", "lidar_x_m")}

        self._throttle_pub = self.create_publisher(Float32, f"/autodrive/{v}/throttle_command", 10)
        self._steering_pub = self.create_publisher(Float32, f"/autodrive/{v}/steering_command", 10)
        self._scan_pub = self.create_publisher(LaserScan, "/laksa/lidar/scan_validated", qos_profile_sensor_data)
        self._odom_pub = self.create_publisher(Odometry, "/laksa/odometry/fused", 10)
        self._zed_pub = self.create_publisher(PointCloud2, "/zed/zed_node/point_cloud/cloud_registered",
                                              qos_profile_sensor_data)
        self._state_pub = self.create_publisher(VehicleState, "/laksa/state", qos_profile_sensor_data)
        self._vesc_pub = self.create_publisher(VescState, "/laksa/vesc/state", 10)
        self._tf = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        self.create_subscription(LaserScan, f"/autodrive/{v}/lidar", self._scan_cb, 10)
        self.create_subscription(Odometry, f"/autodrive/{v}/odom", self._odom_cb, 10)
        self.create_subscription(DriveCommand, "/laksa/command", self._command_cb, 10)

        self._speed = 0.0
        self._origin = None                  # first sim pose -> odom origin
        self._last_scan_pub = 0.0
        self._cmd = DriveCommand()
        self._cmd_time = -1e9
        self._accepted = False
        self._integral = 0.0
        self._sequence = 0
        self._throttle = 0.0
        self.create_timer(0.02, self._control)       # 50 Hz like the ESP32 VESC loop
        self.create_timer(0.05, self._publish_state)  # 20 Hz
        self.create_timer(0.2, self._publish_zed)
        self.get_logger().info(f"AutoDRIVE adapter for {v}: fake ESP32 on /laksa/command, "
                               f"firmware speed limit {'off' if self._cfg['reject_above_mps'] <= 0 else self._cfg['reject_above_mps']}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------- sensors
    def _scan_cb(self, msg: LaserScan) -> None:
        now = self._now()
        if now - self._last_scan_pub < 1.0 / self._cfg["scan_rate_hz"] - 0.005:
            return
        self._last_scan_pub = now
        msg.header.frame_id = "laser"
        msg.header.stamp = self.get_clock().now().to_msg()
        self._scan_pub.publish(msg)

    def _odom_cb(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        yaw = yaw_of(pose.orientation)
        if self._origin is None:
            self._origin = (pose.position.x, pose.position.y, yaw)
        ox, oy, oyaw = self._origin
        dx, dy = pose.position.x - ox, pose.position.y - oy
        x = math.cos(-oyaw) * dx - math.sin(-oyaw) * dy
        y = math.sin(-oyaw) * dx + math.cos(-oyaw) * dy
        th = yaw - oyaw
        # Forward speed: project the sim's velocity on the heading (frame-agnostic).
        vx, vy = msg.twist.twist.linear.x, msg.twist.twist.linear.y
        forward = vx * math.cos(yaw) + vy * math.sin(yaw)
        self._speed = forward if abs(forward) <= math.hypot(vx, vy) + 1e-6 else vx
        stamp = self.get_clock().now().to_msg()
        out = Odometry()
        out.header.stamp, out.header.frame_id, out.child_frame_id = stamp, "odom", "base_footprint"
        out.pose.pose.position.x, out.pose.pose.position.y = x, y
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        out.twist.twist.linear.x = self._speed
        out.twist.twist.angular.z = msg.twist.twist.angular.z
        self._odom_pub.publish(out)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "odom", "base_footprint"
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(th / 2), math.cos(th / 2)
        self._tf.sendTransform(t)

    def _publish_static_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "base_footprint", "laser"
        t.transform.translation.x = self._cfg["lidar_x_m"]
        t.transform.translation.z = 0.096
        t.transform.rotation.w = 1.0
        self._static_tf.sendTransform(t)

    def _publish_zed(self) -> None:
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = "base_footprint"
        cloud.height, cloud.width = 1, 0
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate("xyz")]
        cloud.point_step, cloud.row_step, cloud.is_dense = 12, 0, True
        self._zed_pub.publish(cloud)

    # ---------------------------------------------------------- fake ESP32
    def _command_cb(self, msg: DriveCommand) -> None:
        limit = self._cfg["reject_above_mps"]
        if limit > 0.0 and abs(msg.speed_mps) > limit and not msg.brake:
            self._accepted = False           # like the car: rejected, the last brake state holds
            return
        self._cmd, self._cmd_time, self._accepted = msg, self._now(), True

    def _control(self) -> None:
        now, cfg = self._now(), self._cfg
        fresh = now - self._cmd_time <= cfg["command_timeout_sec"]
        braking = (not fresh) or self._cmd.brake
        target = 0.0 if braking else float(self._cmd.speed_mps)
        if braking or abs(target) < 1e-3:
            self._integral = 0.0
            throttle = -cfg["brake_gain"] * self._speed if abs(self._speed) > 0.05 else 0.0
        else:
            error = target - self._speed
            self._integral = max(-1.0, min(1.0, self._integral + error * 0.02))
            throttle = cfg["kff"] * target + cfg["kp"] * error + cfg["ki"] * self._integral
        self._throttle = max(-cfg["max_throttle"], min(cfg["max_throttle"], throttle))
        norm = 0.0 if braking else float(self._cmd.steering_angle_rad)
        road = norm * (LEFT_LIMIT if norm >= 0 else RIGHT_LIMIT) / MAX_STEERING_NORM
        steer = cfg["sim_steering_sign"] * road / cfg["sim_max_steering_rad"]
        self._throttle_pub.publish(Float32(data=float(self._throttle)))
        self._steering_pub.publish(Float32(data=float(max(-1.0, min(1.0, steer)))))

    def _publish_state(self) -> None:
        now = self._now()
        self._sequence += 1
        fresh = self._accepted and now - self._cmd_time <= self._cfg["command_timeout_sec"]
        vesc = VescState()
        vesc.stamp = self.get_clock().now().to_msg()
        vesc.command_fresh = fresh
        vesc.brake_active = (not fresh) or bool(self._cmd.brake)
        vesc.telemetry_fresh = True
        vesc.telemetry_sequence = self._sequence
        vesc.telemetry_age_ms = 20
        vesc.requested_erpm = int(self._cmd.speed_mps * ERPM_PER_MPS) if fresh else 0
        vesc.active_erpm = vesc.requested_erpm
        vesc.measured_erpm = float(self._speed * ERPM_PER_MPS)
        vesc.duty_cycle = float(self._throttle)
        vesc.input_voltage_v = 15.6
        vesc.vehicle_linear_velocity_mps = float(self._speed)
        state = VehicleState()
        state.stamp = vesc.stamp
        state.vesc = vesc
        self._vesc_pub.publish(vesc)
        self._state_pub.publish(state)


def main() -> None:
    rclpy.init()
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._throttle_pub.publish(Float32(data=0.0))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
