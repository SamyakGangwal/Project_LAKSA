"""Read-only probe: print what the learned policy would command on live scans.

Subscribes to the validated LaserScan and publishes nothing, so it can run
beside the real system without any motion authority.

    ros2 run laksa_learned_driver policy_probe --ros-args -p speed_cap_mps:=0.24
"""

from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .policy import LearnedDriverPolicy
from .scan_adapter import LidarMount, scan_to_vehicle_beams
from .safety import GovernorConfig, govern, scan_points_base
from .scan_features import bin_scan


class PolicyProbe(Node):
    def __init__(self) -> None:
        super().__init__("learned_driver_probe")
        default_model = Path(get_package_share_directory("laksa_learned_driver")) / "models" / "laksa_tinylidarnet_v2.npz"
        self.declare_parameter("model_path", str(default_model))
        self.declare_parameter("scan_topic", "/laksa/lidar/scan_validated")
        self.declare_parameter("speed_cap_mps", 0.24)
        self.declare_parameter("print_period_sec", 1.0)
        self._policy = LearnedDriverPolicy(str(self.get_parameter("model_path").value))
        self._cap = float(self.get_parameter("speed_cap_mps").value)
        self._period = float(self.get_parameter("print_period_sec").value)
        self._mount = LidarMount()
        self._last_print = 0.0
        self._count = 0
        self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value), self._scan_cb,
                                 qos_profile_sensor_data)

    def _scan_cb(self, message: LaserScan) -> None:
        self._count += 1
        started = time.perf_counter()
        ranges, angles = scan_to_vehicle_beams(
            message.ranges, float(message.angle_min), float(message.angle_increment),
            float(message.range_min), float(message.range_max), self._mount,
        )
        steering, speed = self._policy.act(ranges, angles, max(self._cap, 0.25))
        speed = min(speed, self._cap)
        governed = govern(scan_points_base(ranges, angles, self._mount.x_m), steering, speed, GovernorConfig())
        elapsed_ms = 1000.0 * (time.perf_counter() - started)
        now = time.monotonic()
        if now - self._last_print < self._period:
            return
        self._last_print = now
        binned = bin_scan(ranges, angles, self._policy.scan)
        bins = binned.size
        sectors = {
            "right": float(np.min(binned[: bins // 3])),
            "front": float(np.min(binned[bins // 3: 2 * bins // 3])),
            "left": float(np.min(binned[2 * bins // 3:])),
        }
        own_body = int(np.isnan(ranges).sum())
        self.get_logger().info(
            f"scans={self._count} steer={steering:+.3f} rad ({math.degrees(steering):+.1f} deg) "
            f"policy speed={speed:.3f} -> governed {governed.speed_mps:.3f} m/s "
            f"(path free {governed.free_distance_m:.2f} m{', BLOCKED' if governed.blocked else ''}) | nearest right {sectors['right']:.2f} front {sectors['front']:.2f} "
            f"left {sectors['left']:.2f} m | beams in={len(message.ranges)} masked={own_body} | {elapsed_ms:.2f} ms"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PolicyProbe()
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
