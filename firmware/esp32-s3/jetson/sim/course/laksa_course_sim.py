#!/usr/bin/env python3
"""The 2026 competition course replica as a live ROS simulator for the LAKSA stack.

Runs the training simulator (F1TENTH Gym v1 with the LAKSA car: 0.324 m
wheelbase, 0.523/0.288 rad steering, 270 deg LiDAR at x = 0.315 m) in real
time on a course from ``training/`` and speaks the AutoDRIVE topic contract,
so ``sim/autodrive/laksa_autodrive_adapter.py`` connects it to the real
supervisor, learned driver and console unchanged:

  /autodrive/roboracer_1/lidar            LaserScan, forward, 1080 beams over 4.7 rad
  /autodrive/roboracer_1/odom             rear-axle pose, vehicle-frame twist
  /autodrive/roboracer_1/collision_count  Int32 (the car stops on a crash until reset)
  /autodrive/roboracer_1/throttle_command Float32: speed = 24.9 m/s x throttle (as AutoDRIVE)
  /autodrive/roboracer_1/steering_command Float32: road angle = 0.5236 rad x command
  /autodrive/reset_command                Bool: back to the start pose (and new buckets)
  /map + TF map->odom                     the course map for the console (odom = start pose)

Courses: ``obstacle`` (2026 Obstacle Course replica with random buckets and
hoops, as in training) or ``obstacle_plain`` (same course, no buckets/hoops).
"""

from __future__ import annotations

import math
import sys
import tempfile
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32
from tf2_ros import StaticTransformBroadcaster

TRAINING = Path(__file__).resolve().parents[2] / "laksa_learned_driver" / "training"
sys.path.insert(0, str(TRAINING))
sys.path.insert(0, str(TRAINING.parent))

import tracks  # noqa: E402
import vehicle as V  # noqa: E402
from sim_env import GYM_ANGLES, Domain, LaksaSim  # noqa: E402
from laksa_learned_driver.scan_features import ScanContract  # noqa: E402

THROTTLE_TO_MPS = 24.9          # AutoDRIVE's measured throttle -> speed
SIM_MAX_STEERING_RAD = 0.5236   # AutoDRIVE's steering_command = 1


class CourseSim(Node):
    def __init__(self) -> None:
        super().__init__("laksa_course_sim")
        self.declare_parameter("course", "obstacle")
        self.declare_parameter("seed", 1)
        self.declare_parameter("start_index", 0)        # centreline point the car starts on
        self._course = str(self.get_parameter("course").value)
        self._rng = np.random.default_rng(int(self.get_parameter("seed").value))
        self._start_index = int(self.get_parameter("start_index").value)
        self._out = Path(tempfile.mkdtemp(prefix="laksa_course_"))
        self._sim = LaksaSim(ScanContract(), seed=int(self.get_parameter("seed").value))

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._scan_pub = self.create_publisher(LaserScan, "/autodrive/roboracer_1/lidar", 10)
        self._odom_pub = self.create_publisher(Odometry, "/autodrive/roboracer_1/odom", 10)
        self._col_pub = self.create_publisher(Int32, "/autodrive/roboracer_1/collision_count", 10)
        self._map_pub = self.create_publisher(OccupancyGrid, "/map", latched)
        self._static_tf = StaticTransformBroadcaster(self)
        self.create_subscription(Float32, "/autodrive/roboracer_1/throttle_command", self._throttle_cb, 10)
        self.create_subscription(Float32, "/autodrive/roboracer_1/steering_command", self._steering_cb, 10)
        self.create_subscription(Bool, "/autodrive/reset_command", self._reset_cb, 10)

        self._speed_cmd = 0.0
        self._steer_cmd = 0.0
        self._collisions = 0
        self._crashed = False
        self._reset_requested = False
        self._load_course()
        self.create_timer(V.CONTROL_PERIOD_S, self._step)

    # ------------------------------------------------------------- course
    def _load_course(self) -> None:
        hoops = self._course == "obstacle"
        track = tracks.load_obstacle_course(self._rng, self._out, "course", hoops=hoops)
        if self._course == "obstacle_plain":
            track.obstacles = []
        self._track = track
        self._sim.load(track, Domain())
        i = self._start_index % len(track.center)
        x, y = track.center[i]
        yaw = float(track.headings()[i])
        self._start = (float(x), float(y), yaw)
        self._obs = self._sim.reset(*self._start)
        self._yaw_prev = yaw
        self._crashed = False
        self._publish_map()
        self._publish_map_to_odom()
        self.get_logger().warn(f"Course '{self._course}' loaded: {len(track.center)} centreline points, "
                               f"{len(track.obstacles)} boxes, start ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)")

    def _publish_map(self) -> None:
        t = self._track
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.resolution = float(t.resolution)
        h, w = t.free.shape
        grid.info.width, grid.info.height = w, h
        grid.info.origin.position.x, grid.info.origin.position.y = float(t.origin[0]), float(t.origin[1])
        grid.info.origin.orientation.w = 1.0
        occupied = np.where(np.flipud(t.free), 0, 100).astype(np.int8)   # ROS rows start at min y
        grid.data = occupied.ravel().tolist()
        self._map_pub.publish(grid)

    def _publish_map_to_odom(self) -> None:
        x, y, yaw = self._start
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = "map", "odom"
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self._static_tf.sendTransform(tf)

    # ------------------------------------------------------------ commands
    def _throttle_cb(self, msg: Float32) -> None:
        self._speed_cmd = float(msg.data) * THROTTLE_TO_MPS

    def _steering_cb(self, msg: Float32) -> None:
        self._steer_cmd = float(msg.data) * SIM_MAX_STEERING_RAD

    def _reset_cb(self, msg: Bool) -> None:
        if msg.data:
            self._reset_requested = True

    # ---------------------------------------------------------------- loop
    def _step(self) -> None:
        if self._reset_requested:
            self._reset_requested = False
            self._load_course()                     # new buckets/hoops each reset
        if not self._crashed:
            self._obs = self._sim.step(self._steer_cmd, self._speed_cmd)
            if self._obs["collided"]:
                self._crashed = True
                self._collisions += 1
                self.get_logger().error(f"COLLISION #{self._collisions} at "
                                        f"({self._obs['rear_xy'][0]:.2f}, {self._obs['rear_xy'][1]:.2f}); "
                                        "car stopped until /autodrive/reset_command")
        self._publish()

    def _publish(self) -> None:
        obs = self._obs
        stamp = self.get_clock().now().to_msg()
        rear, yaw = obs["rear_xy"], float(obs["yaw"])
        ranges = self._sim._lidar_scan(np.asarray(rear), yaw)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, "lidar"
        scan.angle_min = float(GYM_ANGLES[0])
        scan.angle_max = float(GYM_ANGLES[-1])
        scan.angle_increment = float(GYM_ANGLES[1] - GYM_ANGLES[0])
        scan.scan_time = V.CONTROL_PERIOD_S
        scan.range_min, scan.range_max = 0.06, 10.0
        scan.ranges = [float(r) if r <= 10.0 else float("inf") for r in ranges]
        self._scan_pub.publish(scan)
        speed = 0.0 if self._crashed else float(obs["speed"])
        yaw_rate = math.atan2(math.sin(yaw - self._yaw_prev), math.cos(yaw - self._yaw_prev)) / V.CONTROL_PERIOD_S
        self._yaw_prev = yaw
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, "world", "roboracer_1"
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(rear[0]), float(rear[1])
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        odom.twist.twist.linear.x = speed                 # vehicle frame, like AutoDRIVE
        odom.twist.twist.angular.z = 0.0 if self._crashed else yaw_rate
        self._odom_pub.publish(odom)
        self._col_pub.publish(Int32(data=self._collisions))


def main() -> None:
    rclpy.init()
    node = CourseSim()
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
