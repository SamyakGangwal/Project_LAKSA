#!/usr/bin/env python3
"""Express ZED camera odometry as the rigidly attached vehicle-base pose."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException


def _multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    return Quaternion(
        x=left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
        y=left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
        z=left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w,
        w=left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
    )


def _rotate(rotation: Quaternion, vector: Vector3) -> Vector3:
    norm_squared = (
        rotation.x * rotation.x
        + rotation.y * rotation.y
        + rotation.z * rotation.z
        + rotation.w * rotation.w
    )
    if not math.isfinite(norm_squared) or norm_squared < 1.0e-12:
        raise ValueError("invalid ZED orientation quaternion")
    inverse = Quaternion(
        x=-rotation.x / norm_squared,
        y=-rotation.y / norm_squared,
        z=-rotation.z / norm_squared,
        w=rotation.w / norm_squared,
    )
    pure = Quaternion(x=vector.x, y=vector.y, z=vector.z, w=0.0)
    rotated = _multiply(_multiply(rotation, pure), inverse)
    return Vector3(x=rotated.x, y=rotated.y, z=rotated.z)


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


class ZedBasePoseAdapter(Node):
    """Apply camera<-base static TF to each odom<-camera ZED pose."""

    def __init__(self) -> None:
        super().__init__("zed_base_pose_adapter")
        self.declare_parameter("input_topic", "/zed/zed_node/odom")
        self.declare_parameter("output_topic", "/laksa/zed_base_pose")
        self.declare_parameter("base_frame", "base_footprint")
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._buffer = Buffer()
        self._camera_from_base = {}
        _static_tf_feed(self, self._buffer)
        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("output_topic").value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self._odom_callback,
            qos_profile_sensor_data,
        )

    def _odom_callback(self, message: Odometry) -> None:
        camera_frame = message.child_frame_id.strip()
        if not camera_frame or not message.header.frame_id.strip():
            self.get_logger().error("ZED odometry has an empty frame id", throttle_duration_sec=5.0)
            return
        try:
            # camera<-base is static: look it up once per camera frame
            # instead of querying the buffer on every odometry message.
            camera_from_base = self._camera_from_base.get(camera_frame)
            if camera_from_base is None:
                # lookup_transform(target, source) returns camera<-base here.
                camera_from_base = self._buffer.lookup_transform(
                    camera_frame,
                    self._base_frame,
                    Time(),
                    timeout=Duration(seconds=0.05),
                ).transform
                self._camera_from_base[camera_frame] = camera_from_base
            camera_pose = message.pose.pose
            offset = _rotate(camera_pose.orientation, camera_from_base.translation)
            base_orientation = _multiply(
                camera_pose.orientation, camera_from_base.rotation
            )
            norm = math.sqrt(
                base_orientation.x * base_orientation.x
                + base_orientation.y * base_orientation.y
                + base_orientation.z * base_orientation.z
                + base_orientation.w * base_orientation.w
            )
            if not math.isfinite(norm) or norm < 1.0e-12:
                raise ValueError("invalid transformed base quaternion")
            base_orientation.x /= norm
            base_orientation.y /= norm
            base_orientation.z /= norm
            base_orientation.w /= norm
        except (TransformException, ValueError) as error:
            self.get_logger().warning(
                f"Cannot transform ZED odometry to {self._base_frame}: {error}",
                throttle_duration_sec=5.0,
            )
            return

        output = PoseWithCovarianceStamped()
        output.header = message.header
        output.pose.pose.position.x = camera_pose.position.x + offset.x
        output.pose.pose.position.y = camera_pose.position.y + offset.y
        output.pose.pose.position.z = camera_pose.position.z + offset.z
        output.pose.pose.orientation = base_orientation
        # The rigid transform does not add a new stochastic measurement.
        # Preserve the ZED-provided covariance; the planar EKF selects x/y/yaw.
        output.pose.covariance = message.pose.covariance
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZedBasePoseAdapter()
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
