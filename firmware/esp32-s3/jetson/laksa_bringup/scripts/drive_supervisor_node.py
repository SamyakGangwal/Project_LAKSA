#!/usr/bin/env python3

"""Single authority for LAKSA manual and autonomous drive commands."""

import math

import rclpy
from geometry_msgs.msg import Twist
from laksa_control_math import (
    canonical_map_invalid_reason,
    limited_ackermann_command,
    map_geometry_is_sane,
    route_data_preflight_reason,
    tf_preflight_reason,
)
from drivetrain_conversion import erpm_to_speed_mps
from laksa_interfaces.msg import DriveCommand, VehicleState
from nav2_msgs.msg import Costmap, SpeedLimit
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Joy, LaserScan, PointCloud2
from std_msgs.msg import Bool, Empty, Int32, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener


VESC_FAULT_NAMES = (
    "NONE", "OVER_VOLTAGE", "UNDER_VOLTAGE", "DRV", "ABS_OVER_CURRENT",
    "OVER_TEMP_FET", "OVER_TEMP_MOTOR", "GATE_DRIVER_OVER_VOLTAGE",
    "GATE_DRIVER_UNDER_VOLTAGE", "MCU_UNDER_VOLTAGE", "WATCHDOG_RESET",
    "ENCODER_SPI", "ENCODER_SINCOS_LOW", "ENCODER_SINCOS_HIGH",
    "FLASH_CORRUPTION", "CURRENT_SENSOR_1_OFFSET", "CURRENT_SENSOR_2_OFFSET",
    "CURRENT_SENSOR_3_OFFSET", "UNBALANCED_CURRENTS", "BRAKE_DRIVER",
    "RESOLVER_LOT", "RESOLVER_DOS", "RESOLVER_LOS",
    "APP_CONFIG_FLASH_CORRUPTION", "MOTOR_CONFIG_FLASH_CORRUPTION",
    "ENCODER_NO_MAGNET", "ENCODER_MAGNET_TOO_STRONG", "PHASE_FILTER",
)


def vesc_fault_name(code: int) -> str:
    return VESC_FAULT_NAMES[code] if 0 <= code < len(VESC_FAULT_NAMES) else "UNKNOWN"


class DriveSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("drive_supervisor")

        defaults = {
            "autonomy_enabled": False,
            "actuation_enabled": False,
            "odom_topic": "/laksa/odometry/fused",
            "map_frame": "map",
            "base_frame": "base_footprint",
            "steering_axis": 2,
            "forward_axis": 1,
            "steering_sign": 1.0,
            "forward_sign": 1.0,
            "deadzone": 0.15,
            "a_button": 0,
            "b_button": 1,
            "x_button": 2,
            "y_button": 3,
            "auto_hold_sec": 3.0,
            "manual_speed_presets_erpm": [900, 1300, 1500, 2000, 3000],
            "default_manual_speed_erpm": 900,
            "exploration_max_erpm": 1000.0,
            "navigation_max_erpm": 1500.0,
            "motor_pole_pairs": 2.0,
            "gear_reduction": 11.82,
            "wheel_diameter_m": 0.109,
            "wheelbase_m": 0.324,
            "max_steering_rad": 0.523,
            "left_road_wheel_limit_rad": 0.523,
            "right_road_wheel_limit_rad": 0.288,
            "publish_rate_hz": 20.0,
            "joy_timeout_sec": 0.50,
            "nav_timeout_sec": 0.50,
            "nav_startup_timeout_sec": 5.0,
            "nav_abort_timeout_sec": 6.0,
            "state_timeout_sec": 1.0,
            "scan_timeout_sec": 0.75,
            "odom_timeout_sec": 0.75,
            "zed_cloud_timeout_sec": 1.50,
            "local_costmap_timeout_sec": 1.0,
            "max_odom_linear_speed_mps": 1.5,
            "max_odom_yaw_rate_rps": 4.0,
            "odom_jump_position_margin_m": 0.15,
            "odom_jump_yaw_margin_rad": 0.35,
            # Race mode: autonomy runs without an operator holding a controller
            # or console button.  Obstacle, stale-sensor and e-stop gates stay.
            "require_operator": True,
            # Physical emergency stop (Bool true = stop, latches like Xbox B).
            "hardware_estop_topic": "/laksa/estop_hw",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._autonomy_enabled = bool(self.get_parameter("autonomy_enabled").value)
        self._actuation_enabled = bool(self.get_parameter("actuation_enabled").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._steering_axis = int(self.get_parameter("steering_axis").value)
        self._forward_axis = int(self.get_parameter("forward_axis").value)
        self._steering_sign = float(self.get_parameter("steering_sign").value)
        self._forward_sign = float(self.get_parameter("forward_sign").value)
        self._deadzone = float(self.get_parameter("deadzone").value)
        self._a_button = int(self.get_parameter("a_button").value)
        self._b_button = int(self.get_parameter("b_button").value)
        self._x_button = int(self.get_parameter("x_button").value)
        self._y_button = int(self.get_parameter("y_button").value)
        self._auto_hold_ns = int(
            float(self.get_parameter("auto_hold_sec").value) * 1e9
        )
        self._manual_speed_presets = tuple(
            int(value)
            for value in self.get_parameter("manual_speed_presets_erpm").value
        )
        self._manual_speed_erpm = int(
            self.get_parameter("default_manual_speed_erpm").value
        )
        self._exploration_max_erpm = float(
            self.get_parameter("exploration_max_erpm").value
        )
        self._navigation_max_erpm = float(
            self.get_parameter("navigation_max_erpm").value
        )
        self._pole_pairs = float(self.get_parameter("motor_pole_pairs").value)
        self._gear_reduction = float(self.get_parameter("gear_reduction").value)
        self._wheel_diameter = float(
            self.get_parameter("wheel_diameter_m").value
        )
        self._wheelbase = float(self.get_parameter("wheelbase_m").value)
        self._max_steering = float(
            self.get_parameter("max_steering_rad").value
        )
        self._left_wheel_limit = float(
            self.get_parameter("left_road_wheel_limit_rad").value
        )
        self._right_wheel_limit = float(
            self.get_parameter("right_road_wheel_limit_rad").value
        )
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self._joy_timeout_ns = int(
            float(self.get_parameter("joy_timeout_sec").value) * 1e9
        )
        self._nav_timeout_ns = int(
            float(self.get_parameter("nav_timeout_sec").value) * 1e9
        )
        self._nav_startup_timeout_ns = int(
            float(self.get_parameter("nav_startup_timeout_sec").value) * 1e9
        )
        self._nav_abort_timeout_ns = int(
            float(self.get_parameter("nav_abort_timeout_sec").value) * 1e9
        )
        self._state_timeout_ns = int(
            float(self.get_parameter("state_timeout_sec").value) * 1e9
        )
        self._scan_timeout_ns = int(
            float(self.get_parameter("scan_timeout_sec").value) * 1e9
        )
        self._odom_timeout_ns = int(
            float(self.get_parameter("odom_timeout_sec").value) * 1e9
        )
        self._zed_cloud_timeout_ns = int(
            float(self.get_parameter("zed_cloud_timeout_sec").value) * 1e9
        )
        self._local_costmap_timeout_ns = int(
            float(self.get_parameter("local_costmap_timeout_sec").value) * 1e9
        )
        self._max_odom_linear_speed = float(
            self.get_parameter("max_odom_linear_speed_mps").value
        )
        self._max_odom_yaw_rate = float(
            self.get_parameter("max_odom_yaw_rate_rps").value
        )
        self._odom_jump_position_margin = float(
            self.get_parameter("odom_jump_position_margin_m").value
        )
        self._odom_jump_yaw_margin = float(
            self.get_parameter("odom_jump_yaw_margin_rad").value
        )
        self._vesc_telemetry_timeout_ms = self._state_timeout_ns // 1_000_000
        self._require_operator = bool(self.get_parameter("require_operator").value)

        positive = (
            self._exploration_max_erpm,
            self._navigation_max_erpm,
            self._pole_pairs,
            self._gear_reduction,
            self._wheel_diameter,
            self._wheelbase,
            self._max_steering,
            self._left_wheel_limit,
            self._right_wheel_limit,
            publish_rate,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("Drive geometry, rates, and limits must be positive")
        if not self._manual_speed_presets or any(
            value <= 0 for value in self._manual_speed_presets
        ):
            raise ValueError("Manual speed presets must be positive")
        if self._manual_speed_erpm not in self._manual_speed_presets:
            raise ValueError("Default manual speed must be one of the presets")
        if not 0.0 <= self._deadzone < 1.0:
            raise ValueError("deadzone must be in [0, 1)")

        mode_qos = QoSProfile(depth=1)
        mode_qos.reliability = ReliabilityPolicy.RELIABLE
        mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._command_pub = self.create_publisher(
            DriveCommand, "/laksa/command", 10
        )
        self._candidate_command_pub = self.create_publisher(
            DriveCommand, "/laksa/autonomy_candidate_command", 10
        )
        self._brake_pub = self.create_publisher(Bool, "/laksa/brake", 10)
        self._mode_pub = self.create_publisher(
            Bool, "/laksa/autonomous_enabled", mode_qos
        )
        self._exploration_pub = self.create_publisher(
            Bool, "/laksa/exploration_enabled", mode_qos
        )
        self._estop_pub = self.create_publisher(
            Bool, "/laksa/emergency_stop", mode_qos
        )
        self._estop_reason_pub = self.create_publisher(
            String, "/laksa/emergency_stop_reason", mode_qos
        )
        self._mission_state_pub = self.create_publisher(
            String, "/laksa/mission_state", mode_qos
        )
        self._autonomy_health_pub = self.create_publisher(
            String, "/laksa/autonomy_health", mode_qos
        )
        self._speed_limit_pub = self.create_publisher(
            SpeedLimit, "/speed_limit", 10
        )
        self._cancel_navigation_pub = self.create_publisher(
            Empty, "/laksa/cancel_navigation", 10
        )
        self.create_subscription(Joy, "/joy", self._joy_callback, 10)
        # Freshness only: raw=True skips deserializing every scan in Python.
        self.create_subscription(
            LaserScan,
            "/laksa/lidar/scan_validated",
            self._scan_callback,
            qos_profile_sensor_data,
            raw=True,
        )
        self.create_subscription(Odometry, self._odom_topic, self._odom_callback, 10)
        # Freshness only: take the serialized bytes (raw=True) instead of
        # deserializing a multi-hundred-kB cloud in Python on every message.
        self.create_subscription(
            PointCloud2,
            "/zed/zed_node/point_cloud/cloud_registered",
            self._zed_cloud_callback,
            qos_profile_sensor_data,
            raw=True,
        )
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, mode_qos)
        self.create_subscription(
            Costmap,
            "/local_costmap/costmap_raw",
            self._local_costmap_callback,
            10,
        )
        self.create_subscription(
            Twist, "/laksa/nav_cmd_vel", self._nav_callback, 10
        )
        self.create_subscription(
            Twist,
            "/laksa/lidar_cruise_cmd_vel",
            self._cruise_callback,
            10,
        )
        self.create_subscription(
            VehicleState,
            "/laksa/state",
            self._state_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            "/laksa/dashboard_navigation_enabled",
            self._dashboard_navigation_callback,
            mode_qos,
        )
        self.create_subscription(
            Bool,
            "/laksa/dashboard_exploration_enabled",
            self._dashboard_exploration_callback,
            mode_qos,
        )
        self.create_subscription(
            Bool,
            "/laksa/exploration_complete",
            self._exploration_complete_callback,
            mode_qos,
        )
        self.create_subscription(
            String,
            "/laksa/exploration_status",
            self._exploration_status_callback,
            mode_qos,
        )
        self.create_subscription(
            Int32,
            "/laksa/manual_speed_erpm",
            self._manual_speed_callback,
            mode_qos,
        )
        self.create_subscription(
            Bool,
            "/laksa/map_reset_in_progress",
            self._map_reset_callback,
            mode_qos,
        )
        self.create_service(
            SetBool,
            "/laksa/autonomy/set_armed",
            self._set_armed_callback,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("hardware_estop_topic").value),
            self._hardware_estop_callback,
            10,
        )
        self.create_timer(1.0 / publish_rate, self._update)

        self._joy_axes = []
        self._buttons = []
        self._last_joy_ns = 0
        self._last_nav_ns = 0
        self._last_cruise_ns = 0
        self._last_state_ns = 0
        self._last_scan_ns = 0
        self._last_odom_ns = 0
        self._last_zed_cloud_ns = 0
        self._last_map_ns = 0
        self._last_local_costmap_ns = 0
        self._last_accepted_odom = None
        self._odom_fault_reason = ""
        self._map_received = False
        self._map_invalid_reason = "MAP_NOT_RECEIVED"
        self._map_generation = 0
        self._last_speed_limit_publish_ns = 0
        self._vesc_telemetry_ok = False
        self._vesc_fault_code = 0
        self._nav_twist = Twist()
        self._cruise_twist = Twist()
        self._autonomous = False
        self._exploring = False
        self._autonomy_started_ns = 0
        self._map_reset_in_progress = False
        self._manual_neutral_required = True
        self._a_started_ns = 0
        self._a_latched = False
        self._estop_latched = False
        self._estop_reason = ""
        self._exploration_complete = False
        self._exploration_blocked = False
        self._previous_b = False
        self._previous_x = False
        self._previous_y = False
        self._reported_reason = ""
        self._reported_autonomy_health = ""
        self._last_health_publish_ns = 0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publish_mode()
        self.get_logger().info(
            "MANUAL mode: left stick drives and right stick steers, "
            f"hold A {self._auto_hold_ns / 1e9:.1f} s for forward-priority LiDAR cruise, "
            "X returns to manual, B latches the emergency stop, Y rearms"
        )
        if not self._actuation_enabled:
            self.get_logger().warn(
                "DRY-RUN: candidate commands are observable, physical actuation is forced to brake"
            )

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _erpm_to_mps(self, erpm: float) -> float:
        return erpm_to_speed_mps(erpm, self._pole_pairs, self._gear_reduction, self._wheel_diameter)

    def _publish_mode(self) -> None:
        message = Bool()
        message.data = self._autonomous
        self._mode_pub.publish(message)
        exploration = Bool()
        exploration.data = self._exploring
        self._exploration_pub.publish(exploration)
        estop = Bool()
        estop.data = self._estop_latched
        self._estop_pub.publish(estop)
        estop_reason = String()
        estop_reason.data = self._estop_reason if self._estop_latched else ""
        self._estop_reason_pub.publish(estop_reason)
        mission = String()
        if self._estop_latched:
            mission.data = "EMERGENCY_STOP"
        elif self._exploring:
            mission.data = "LIDAR_CRUISE"
        elif self._autonomous:
            mission.data = "NAVIGATING"
        elif self._exploration_complete:
            mission.data = "EXPLORATION_COMPLETE"
        elif self._exploration_blocked:
            mission.data = "EXPLORATION_BLOCKED"
        else:
            mission.data = "MANUAL"
        self._mission_state_pub.publish(mission)
        self._publish_speed_limit()

    def _publish_speed_limit(self) -> None:
        message = SpeedLimit()
        message.percentage = False
        if self._exploring:
            message.speed_limit = self._erpm_to_mps(self._exploration_max_erpm)
        elif self._autonomous:
            message.speed_limit = self._erpm_to_mps(self._navigation_max_erpm)
        else:
            # Nav2 defines zero absolute speed as NO_SPEED_LIMIT.
            message.speed_limit = 0.0
        self._speed_limit_pub.publish(message)
        self._last_speed_limit_publish_ns = self._now_ns()

    def _clear_nav_command(self) -> None:
        self._nav_twist = Twist()
        self._last_nav_ns = 0
        self._cruise_twist = Twist()
        self._last_cruise_ns = 0

    def _abort_autonomy(self, reason: str, blocked: bool = False) -> None:
        if not self._autonomous:
            return
        self._clear_nav_command()
        self._autonomous = False
        self._exploring = False
        self._autonomy_started_ns = 0
        self._exploration_complete = False
        self._exploration_blocked = blocked
        self._manual_neutral_required = True
        self._publish_mode()
        self._cancel_navigation_pub.publish(Empty())
        self.get_logger().error(f"Autonomy aborted: {reason}")

    def _set_autonomous(
        self, enabled: bool, reason: str, exploring: bool = False
    ) -> None:
        if enabled and not self._autonomy_enabled:
            self.get_logger().warn("Autonomous request rejected: manual-only runtime")
            return
        exploring = enabled and exploring
        if enabled:
            ready, health = self._autonomy_ready(exploring=exploring)
            if not ready:
                self._publish_autonomy_health(health)
                self.get_logger().error(
                    f"Autonomous request rejected: {health}"
                )
                return
        if enabled == self._autonomous and exploring == self._exploring:
            return
        self._clear_nav_command()
        self._autonomous = enabled
        self._exploring = exploring
        if enabled:
            self._autonomy_started_ns = self._now_ns()
            self._exploration_complete = False
            self._exploration_blocked = False
        else:
            self._autonomy_started_ns = 0
            self._manual_neutral_required = True
        self._publish_mode()
        if exploring:
            mode = "FORWARD-PRIORITY LIDAR CRUISE"
        elif enabled:
            mode = "NAVIGATE TO DASHBOARD GOAL"
        else:
            mode = "MANUAL"
        self.get_logger().warn(f"Mode -> {mode}: {reason}")

    def _dashboard_navigation_callback(self, message: Bool) -> None:
        if message.data:
            if self._estop_latched:
                self.get_logger().warning(
                    "Dashboard goal ignored while emergency stop is latched"
                )
                return
            self._set_autonomous(True, "dashboard goal", exploring=False)
        elif self._autonomous and not self._exploring:
            self._set_autonomous(False, "dashboard goal finished or canceled")

    def _set_armed_callback(self, request, response):
        if request.data:
            self._set_autonomous(True, "explicit autonomy ARM service", exploring=False)
            response.success = self._autonomous and not self._exploring
            response.message = "AUTONOMY_ARMED" if response.success else self._reported_autonomy_health
        else:
            self._set_autonomous(False, "explicit autonomy DISARM service")
            self._cancel_navigation_pub.publish(Empty())
            response.success = not self._autonomous
            response.message = "AUTONOMY_DISARMED"
        return response

    def _dashboard_exploration_callback(self, message: Bool) -> None:
        if message.data:
            if self._estop_latched:
                self.get_logger().warning(
                    "Exploration request ignored while emergency stop is latched"
                )
                return
            self._set_autonomous(True, "dashboard exploration request", exploring=True)
        elif self._exploring:
            self._set_autonomous(False, "dashboard exploration canceled")
        elif self._exploration_complete:
            self._exploration_complete = False
            self._publish_mode()
            self.get_logger().warn("Mode -> MANUAL: dashboard confirmation")

    def _exploration_complete_callback(self, message: Bool) -> None:
        if not message.data or not self._exploring:
            return
        self._clear_nav_command()
        self._autonomous = False
        self._exploring = False
        self._autonomy_started_ns = 0
        self._manual_neutral_required = True
        self._exploration_complete = True
        self._exploration_blocked = False
        self._publish_mode()
        self.get_logger().warn("Mode -> EXPLORATION COMPLETE")

    def _exploration_status_callback(self, message: String) -> None:
        if not self._exploring:
            return
        if message.data == "BLOCKED":
            self._abort_autonomy("explorer reported BLOCKED", blocked=True)
        elif message.data in ("CONTROL_ERROR", "CONTROL_REJECTED"):
            self._abort_autonomy(
                f"explorer adapter reported {message.data}", blocked=True
            )

    def _map_reset_callback(self, message: Bool) -> None:
        resetting = bool(message.data)
        if resetting == self._map_reset_in_progress:
            return
        self._map_reset_in_progress = resetting
        if resetting:
            self._last_accepted_odom = None
            self._last_odom_ns = 0
            self._odom_fault_reason = ""
            self._manual_neutral_required = True
            self._abort_autonomy("map reset entered its protected window")
            self.get_logger().warning("Map reset interlock engaged")
        else:
            self.get_logger().info("Map reset interlock released")

    def _manual_speed_callback(self, message: Int32) -> None:
        requested = int(message.data)
        if requested not in self._manual_speed_presets:
            self.get_logger().warning(
                f"Rejected unsupported manual speed preset: {requested} eRPM"
            )
            return
        self._manual_speed_erpm = requested
        self.get_logger().warn(f"Manual speed preset -> {requested} eRPM")

    def _button(self, index: int) -> bool:
        return 0 <= index < len(self._buttons) and bool(self._buttons[index])

    def _hardware_estop_callback(self, message: Bool) -> None:
        if message.data:
            self._latch_estop("hardware emergency stop")

    def _latch_estop(self, reason: str) -> None:
        if self._estop_latched:
            return
        self._estop_latched = True
        self._estop_reason = reason
        self._exploration_complete = False
        self._set_autonomous(False, reason)
        self._cancel_navigation_pub.publish(Empty())
        self._publish_mode()
        self.get_logger().error(f"EMERGENCY STOP LATCHED: {reason}")

    def _joy_callback(self, message: Joy) -> None:
        now_ns = self._now_ns()
        if (
            self._last_joy_ns == 0
            or now_ns - self._last_joy_ns > self._joy_timeout_ns
        ):
            self._manual_neutral_required = True
        self._last_joy_ns = now_ns
        self._joy_axes = list(message.axes)
        self._buttons = list(message.buttons)

        # Analog operator intent has the same immediate authority as the X/B
        # buttons. Cancel autonomy in the Joy callback itself, before the next
        # 20 Hz command publication cycle can forward another candidate.
        manual_throttle = self._axis(self._forward_axis) * self._forward_sign
        manual_steering = self._axis(self._steering_axis) * self._steering_sign
        if self._autonomous and (
            abs(manual_throttle) > self._deadzone
            or abs(manual_steering) > self._deadzone
        ):
            self._abort_autonomy("Xbox manual override")
        b_pressed = self._button(self._b_button)
        y_pressed = self._button(self._y_button)
        if b_pressed and not self._previous_b:
            self._latch_estop("Xbox B pressed")
        elif (
            y_pressed
            and not self._previous_y
            and self._estop_latched
            and not b_pressed
        ):
            if self._vesc_fault_code != 0:
                self.get_logger().error(
                    "Emergency-stop rearm rejected: VESC fault "
                    f"{vesc_fault_name(self._vesc_fault_code)} "
                    f"({self._vesc_fault_code}) is still active"
                )
            else:
                self._estop_latched = False
                self._estop_reason = ""
                self._exploration_complete = False
                self._manual_neutral_required = True
                self._publish_mode()
                self.get_logger().warn(
                    "Emergency stop REARMED by Xbox Y; MANUAL mode"
                )
        self._previous_b = b_pressed
        self._previous_y = y_pressed

        x_pressed = self._button(self._x_button)
        if x_pressed and not self._previous_x:
            self._exploration_complete = False
            self._exploration_blocked = False
            self._set_autonomous(False, "Xbox X pressed")
            self._cancel_navigation_pub.publish(Empty())
            self._publish_mode()
        self._previous_x = x_pressed

        a_pressed = self._button(self._a_button)
        if not a_pressed:
            self._a_started_ns = 0
            self._a_latched = False
        elif (
            not self._autonomous
            and not self._estop_latched
            and not self._a_latched
        ):
            if self._a_started_ns == 0:
                self._a_started_ns = self._last_joy_ns
                self.get_logger().info("A hold started")
            elif self._last_joy_ns - self._a_started_ns >= self._auto_hold_ns:
                self._a_latched = True
                self._set_autonomous(
                    True, "Xbox A held for 3 seconds", exploring=True
                )

    def _nav_callback(self, message: Twist) -> None:
        if not self._valid_twist(message, "Nav2"):
            self._nav_twist = Twist()
            self._last_nav_ns = 0
            return
        self._last_nav_ns = self._now_ns()
        self._nav_twist = message

    def _cruise_callback(self, message: Twist) -> None:
        if not self._valid_twist(message, "LiDAR cruise"):
            self._cruise_twist = Twist()
            self._last_cruise_ns = 0
            return
        self._last_cruise_ns = self._now_ns()
        self._cruise_twist = message

    def _valid_twist(self, message: Twist, source: str) -> bool:
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().error(
                f"Rejected non-finite {source} velocity command",
                throttle_duration_sec=2.0,
            )
            return False
        return True

    def _scan_callback(self, _message: bytes) -> None:
        self._last_scan_ns = self._now_ns()

    def _zed_cloud_callback(self, message: bytes) -> None:
        # An empty cloud serializes to well under 1 kB (header + field list).
        if len(message) > 1024:
            self._last_zed_cloud_ns = self._now_ns()

    def _odom_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
            *message.pose.covariance,
            *message.twist.covariance,
        )
        quaternion_norm = math.sqrt(
            pose.orientation.x * pose.orientation.x
            + pose.orientation.y * pose.orientation.y
            + pose.orientation.z * pose.orientation.z
            + pose.orientation.w * pose.orientation.w
        )
        if (
            not all(math.isfinite(float(value)) for value in values)
            or not 0.95 <= quaternion_norm <= 1.05
        ):
            self._last_odom_ns = 0
            self.get_logger().error(
                "Rejected invalid odometry sample",
                throttle_duration_sec=2.0,
            )
            return
        now_ns = self._now_ns()
        yaw = math.atan2(
            2.0 * (
                pose.orientation.w * pose.orientation.z
                + pose.orientation.x * pose.orientation.y
            ),
            1.0 - 2.0 * (
                pose.orientation.y * pose.orientation.y
                + pose.orientation.z * pose.orientation.z
            ),
        )
        if self._last_accepted_odom is not None:
            previous_ns, previous_x, previous_y, previous_yaw = (
                self._last_accepted_odom
            )
            dt = max((now_ns - previous_ns) / 1e9, 1e-3)
            distance = math.hypot(
                pose.position.x - previous_x,
                pose.position.y - previous_y,
            )
            yaw_delta = abs(
                math.atan2(
                    math.sin(yaw - previous_yaw),
                    math.cos(yaw - previous_yaw),
                )
            )
            position_limit = (
                self._max_odom_linear_speed * dt
                + self._odom_jump_position_margin
            )
            yaw_limit = self._max_odom_yaw_rate * dt + self._odom_jump_yaw_margin
            if distance > position_limit or yaw_delta > yaw_limit:
                self._last_odom_ns = 0
                self._odom_fault_reason = (
                    "odometry discontinuity "
                    f"({distance:.2f} m, {yaw_delta:.2f} rad in {dt:.3f} s)"
                )
                self.get_logger().error(
                    f"Rejected {self._odom_fault_reason}",
                    throttle_duration_sec=2.0,
                )
                return
        self._last_accepted_odom = (
            now_ns,
            float(pose.position.x),
            float(pose.position.y),
            yaw,
        )
        self._odom_fault_reason = ""
        self._last_odom_ns = now_ns

    def _map_callback(self, message: OccupancyGrid) -> None:
        self._map_received = True
        self._last_map_ns = self._now_ns()
        self._map_generation += 1
        self._map_invalid_reason = canonical_map_invalid_reason(
            int(message.info.width),
            int(message.info.height),
            float(message.info.resolution),
            len(message.data),
            str(message.header.frame_id),
            self._map_frame,
        )
        if self._map_invalid_reason:
            self.get_logger().error(
                f"Rejected canonical map: {self._map_invalid_reason}",
                throttle_duration_sec=2.0,
            )

    def _local_costmap_callback(self, message: Costmap) -> None:
        if map_geometry_is_sane(
            int(message.metadata.size_x),
            int(message.metadata.size_y),
            float(message.metadata.resolution),
        ):
            self._last_local_costmap_ns = self._now_ns()

    def _state_callback(self, message: VehicleState) -> None:
        self._last_state_ns = self._now_ns()
        self._vesc_telemetry_ok = (
            int(message.vesc.telemetry_sequence) > 0
            and int(message.vesc.telemetry_age_ms)
            <= self._vesc_telemetry_timeout_ms
        )
        self._vesc_fault_code = int(message.vesc.fault_code)
        if self._vesc_fault_code != 0:
            self._latch_estop(
                "VESC fault "
                f"{vesc_fault_name(self._vesc_fault_code)} "
                f"({self._vesc_fault_code})"
            )

    def _axis(self, index: int) -> float:
        if 0 <= index < len(self._joy_axes):
            return float(self._joy_axes[index])
        return 0.0

    def _autonomy_ready(self, exploring=None):
        if self._map_reset_in_progress:
            return False, "map reset is in progress"
        if exploring is None:
            exploring = self._exploring
        now_ns = self._now_ns()
        checks = [
            (self._last_state_ns, self._state_timeout_ns, "ESP32_STATE_STALE"),
            (self._last_scan_ns, self._scan_timeout_ns, "OBSTACLE_SOURCE_STALE: LiDAR scan"),
            (self._last_odom_ns, self._odom_timeout_ns, "ODOM_STALE"),
            (
                self._last_zed_cloud_ns,
                self._zed_cloud_timeout_ns,
                "OBSTACLE_SOURCE_STALE: ZED obstacle cloud",
            ),
        ]
        if self._require_operator:
            checks.insert(0, (self._last_joy_ns, self._joy_timeout_ns, "XBOX_STALE"))
        # Rollout cruise uses LiDAR odometry for visit memory and movement-gated
        # recovery, but remains independent of the global map and map TF.
        if not exploring:
            checks.append(
                (
                    self._last_local_costmap_ns,
                    self._local_costmap_timeout_ns,
                    "OBSTACLE_SOURCE_STALE: local costmap",
                )
            )
            failure = route_data_preflight_reason(
                now_ns,
                self._map_received,
                self._map_invalid_reason,
                checks,
            )
        else:
            failure = next(
                (
                    reason
                    for stamp, timeout, reason in checks
                    if stamp == 0 or now_ns - stamp > timeout
                ),
                None,
            )
        if failure:
            if failure == "ODOM_STALE" and self._odom_fault_reason:
                return False, f"ODOM_STALE: {self._odom_fault_reason}"
            return False, failure
        if not self._vesc_telemetry_ok:
            return False, "VESC telemetry is stale"
        if self._vesc_fault_code != 0:
            return False, f"VESC fault code {self._vesc_fault_code}"
        if not exploring:
            try:
                self._tf_buffer.lookup_transform(
                    self._map_frame,
                    self._base_frame,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException as error:
                return False, tf_preflight_reason(str(error))
        return True, "READY"

    def _publish_autonomy_health(self, health: str) -> None:
        now_ns = self._now_ns()
        if (
            health == self._reported_autonomy_health
            and now_ns - self._last_health_publish_ns < 1_000_000_000
        ):
            return
        self._reported_autonomy_health = health
        self._last_health_publish_ns = now_ns
        message = String()
        message.data = health
        self._autonomy_health_pub.publish(message)

    def _manual_command(self, joy_fresh: bool) -> DriveCommand:
        command = DriveCommand()
        if not joy_fresh:
            return command

        # A malformed/partially initialized Joy message must not count as a
        # neutral sample and silently arm propulsion.
        if not 0 <= self._forward_axis < len(self._joy_axes):
            return command

        throttle = self._axis(self._forward_axis) * self._forward_sign
        throttle = max(-1.0, min(1.0, throttle))
        if self._manual_neutral_required:
            if abs(throttle) <= self._deadzone:
                self._manual_neutral_required = False
                self.get_logger().info("Manual throttle neutral interlock released")
            return command

        steering_input = self._axis(self._steering_axis) * self._steering_sign
        steering_input = max(-1.0, min(1.0, steering_input))
        command.steering_angle_rad = steering_input * self._max_steering

        if abs(throttle) > self._deadzone:
            command.speed_mps = math.copysign(
                self._erpm_to_mps(self._manual_speed_erpm), throttle
            )
        return command

    def _autonomous_command(self, nav_fresh: bool) -> DriveCommand:
        command = DriveCommand()
        if not nav_fresh:
            return command

        source = self._cruise_twist if self._exploring else self._nav_twist
        raw_speed = float(source.linear.x)
        raw_yaw_rate = float(source.angular.z)
        mode_limit_erpm = (
            self._exploration_max_erpm
            if self._exploring
            else self._navigation_max_erpm
        )
        mode_limit = self._erpm_to_mps(mode_limit_erpm)
        command.speed_mps, command.steering_angle_rad = limited_ackermann_command(
            raw_speed,
            raw_yaw_rate,
            mode_limit,
            self._wheelbase,
            self._left_wheel_limit,
            self._right_wheel_limit,
            self._max_steering,
        )
        return command

    def _update(self) -> None:
        now_ns = self._now_ns()
        joy_fresh = (
            self._last_joy_ns != 0
            and now_ns - self._last_joy_ns <= self._joy_timeout_ns
        )
        nav_fresh = (
            (
                self._last_cruise_ns
                if self._exploring
                else self._last_nav_ns
            )
            != 0
            and now_ns
            - (self._last_cruise_ns if self._exploring else self._last_nav_ns)
            <= self._nav_timeout_ns
        )
        state_fresh = (
            self._last_state_ns != 0
            and now_ns - self._last_state_ns <= self._state_timeout_ns
        )
        # In MANUAL, report readiness for the A-button LiDAR Cruise. Global
        # point navigation performs its stricter map/odom/TF check on entry.
        autonomy_ready, autonomy_health = self._autonomy_ready(
            exploring=self._exploring or not self._autonomous
        )
        self._publish_autonomy_health(autonomy_health)
        if now_ns - self._last_speed_limit_publish_ns >= 1_000_000_000:
            self._publish_speed_limit()

        autonomy_abort_reason = ""
        if self._autonomous:
            if not joy_fresh and self._require_operator:
                autonomy_abort_reason = "Xbox controller timeout"
            elif not autonomy_ready:
                autonomy_abort_reason = autonomy_health
            elif (
                self._last_cruise_ns if self._exploring else self._last_nav_ns
            ) == 0:
                if (
                    self._autonomy_started_ns
                    and now_ns - self._autonomy_started_ns
                    > self._nav_startup_timeout_ns
                ):
                    autonomy_abort_reason = (
                        "LiDAR cruise did not produce a command"
                        if self._exploring
                        else "Nav2 did not produce a command"
                    )
            elif not nav_fresh:
                # Controller-to-behavior transitions intentionally contain
                # short command gaps (costmap clearing and Wait do not publish
                # Twist continuously). Brake immediately, but give Nav2 enough
                # time to finish that recovery step before canceling the goal.
                active_stamp = (
                    self._last_cruise_ns if self._exploring else self._last_nav_ns
                )
                if now_ns - active_stamp > self._nav_abort_timeout_ns:
                    autonomy_abort_reason = (
                        "LiDAR cruise command stream stopped"
                        if self._exploring
                        else "Nav2 command stream stopped"
                    )
            if autonomy_abort_reason:
                self._abort_autonomy(autonomy_abort_reason, blocked=True)

        command = (
            self._autonomous_command(nav_fresh)
            if self._autonomous
            else self._manual_command(joy_fresh)
        )

        reason = ""
        if self._estop_latched:
            detail = self._estop_reason or "operator request"
            reason = f"Emergency stop latched: {detail}; press Xbox Y to rearm"
            command.steering_angle_rad = 0.0
        elif self._map_reset_in_progress:
            reason = "Map reset interlock"
            command.steering_angle_rad = 0.0
        elif self._exploration_complete:
            reason = "Exploration complete; press Xbox X for manual mode"
            command.steering_angle_rad = 0.0
        elif autonomy_abort_reason:
            reason = f"Autonomy aborted: {autonomy_abort_reason}"
        elif not state_fresh:
            reason = "ESP32 state timeout"
        elif not self._vesc_telemetry_ok:
            reason = "VESC telemetry stale"
        elif self._vesc_fault_code != 0:
            reason = f"VESC fault code {self._vesc_fault_code}"
        elif self._autonomous and not autonomy_ready:
            reason = f"Autonomy unavailable: {autonomy_health}"
        elif self._autonomous and not nav_fresh:
            reason = (
                "Waiting for a fresh LiDAR cruise command"
                if self._exploring
                else "Waiting for a fresh Nav2 command"
            )
        elif not joy_fresh and (self._require_operator or not self._autonomous):
            # The Xbox controller is the operator's brake/mode escape device.
            # Never allow autonomous motion if that safety link disappears,
            # unless this runtime is explicitly operator-free (race mode).
            reason = "Xbox controller timeout"
        elif not self._autonomous and self._manual_neutral_required:
            reason = "Manual throttle must return to neutral"
        if reason:
            command.speed_mps = 0.0
            command.brake = True
            if not self._autonomous:
                # Health/map/controller faults may clear while the operator is
                # still holding throttle. Require a fresh neutral sample before
                # manual propulsion can resume.
                self._manual_neutral_required = True
            if reason != self._reported_reason:
                self.get_logger().warn(reason)
        self._reported_reason = reason
        self._candidate_command_pub.publish(command)
        if not self._actuation_enabled:
            command = DriveCommand()
            command.brake = True
        brake = Bool()
        brake.data = command.brake
        self._brake_pub.publish(brake)
        self._command_pub.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriveSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
