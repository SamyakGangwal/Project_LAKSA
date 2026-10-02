#!/usr/bin/env python3
"""Fail-closed gate between Nav2's velocity output and drive_supervisor.

Nav2's controller_server and behavior_server publish to /laksa/nav_cmd_vel_raw
(LAKSA_NAV_CMD_TOPIC in setup/dryrun_bringup.sh).  This node republishes those
Twists on /laksa/nav_cmd_vel, the supervisor's navigation input, only while the
active NavigateToPose goal has a PASS verdict for its own path.  Otherwise it
publishes nothing: the supervisor sees stale navigation input and commands zero
after nav_timeout_sec (0.25 s).

Verdict per goal (from /navigate_to_pose/_action/status):
  * A new goal starts at NONE.  Only a /plan whose header.stamp is later than
    the goal's goal_info.stamp is evaluated for it.
  * The plan passes only if all checks pass: /plan and the global costmap are
    in the map frame and the costmap is under 3 s old; the padded footprint at
    every pose (resampled at <= 0.05 m) covers no cell >= 99, no unknown cell
    and nothing outside the grid; no cusp or reverse segment (forward_only);
    curvature never above 0.914 * 1.10 1/m.
  * While forwarding, a Twist with linear.x < 0 (or non-finite) fails the goal.
  * FAIL latches for the rest of that goal: the gate publishes Empty on
    /laksa/cancel_navigation and cancels every /navigate_to_pose goal.

Every verdict is published as JSON on /laksa/path_gate and appended to
~/laksa_run/latest/path_gate.log.

The geometry (check_plan and helpers) and the verdict state machine
(GateState) do not import ROS, so setup/tonight/test_path_gate.py tests them
directly.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from dataclasses import dataclass, field

import numpy as np

# Footprint of nav2_ackermann.yaml plus its 0.02 m footprint_padding (base_footprint frame).
FOOTPRINT_X = (-0.17, 0.44)
FOOTPRINT_Y = (-0.20, 0.20)
# Smac minimum_turning_radius 1.09 m -> 0.914 1/m, with 10 % margin for the smoother.
MAX_CURVATURE = 0.914 * 1.10
RESAMPLE_M = 0.05
CURVATURE_WINDOW_M = 0.20       # chord on each side of a point for the curvature estimate
COSTMAP_MAX_AGE_S = 3.0
BLOCKED_COST = 99               # OccupancyGrid scale: 99 inscribed, 100 lethal, -1 unknown
LETHAL_COST = 100
CLEARANCE_SEARCH_M = 1.0

NONE, PASS, FAIL = "NONE", "PASS", "FAIL"
# action_msgs/GoalStatus
STATUS_ACCEPTED, STATUS_EXECUTING, STATUS_CANCELING = 1, 2, 3


# ------------------------------------------------------------------ geometry
@dataclass
class Grid:
    """An OccupancyGrid in plain numbers.  data[row, col], row = y index."""
    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    frame: str
    received: float                 # time.monotonic() when it arrived
    origin_yaw: float = 0.0


@dataclass
class PlanCheck:
    ok: bool
    reasons: list = field(default_factory=list)
    length_m: float = 0.0
    min_clearance_m: float | None = None
    poses_checked: int = 0
    cells_checked: int = 0
    cells_blocked: int = 0
    cells_unknown: int = 0
    cells_outside: int = 0          # footprint sample points that fall outside the grid
    max_curvature: float = 0.0
    reverse_segments: int = 0
    cusps: int = 0

    def as_dict(self) -> dict:
        return {"ok": self.ok, "reasons": list(self.reasons), "length_m": round(self.length_m, 3),
                "min_clearance_m": None if self.min_clearance_m is None else round(self.min_clearance_m, 3),
                "poses_checked": self.poses_checked,
                "cells": {"checked": self.cells_checked, "blocked": self.cells_blocked,
                          "unknown": self.cells_unknown, "outside_samples": self.cells_outside},
                "max_curvature": round(self.max_curvature, 3),
                "reverse_segments": self.reverse_segments, "cusps": self.cusps}


def _wrap(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def resample(poses: np.ndarray, step: float = RESAMPLE_M) -> np.ndarray:
    """Insert poses so consecutive poses are at most `step` apart.

    poses: N x 3 (x, y, yaw).  Every original pose is kept (cusps stay exact);
    yaw is interpolated along the shortest angle.
    """
    if len(poses) < 2:
        return poses.copy()
    out = [poses[:1]]
    for a, b in zip(poses[:-1], poses[1:]):
        n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / step)))
        t = np.arange(1, n + 1)[:, None] / n
        xy = a[:2] + t * (b[:2] - a[:2])
        yaw = a[2] + t[:, 0] * _wrap(b[2] - a[2])
        out.append(np.column_stack([xy, _wrap(yaw)]))
    return np.vstack(out)


def segment_directions(poses: np.ndarray) -> np.ndarray:
    """+1 forward / -1 reverse / 0 no motion per segment, from the pose heading."""
    d = np.diff(poses[:, :2], axis=0)
    heading = np.column_stack([np.cos(poses[:-1, 2]), np.sin(poses[:-1, 2])])
    dot = np.einsum("ij,ij->i", d, heading)
    length = np.hypot(d[:, 0], d[:, 1])
    out = np.sign(dot)
    out[length < 1e-6] = 0
    return out


def count_cusps(poses: np.ndarray) -> int:
    """Travel direction reversals (consecutive displacements more than 90 deg apart)."""
    d = np.diff(poses[:, :2], axis=0)
    d = d[np.hypot(d[:, 0], d[:, 1]) > 1e-6]
    if len(d) < 2:
        return 0
    return int(np.sum(np.einsum("ij,ij->i", d[:-1], d[1:]) < 0.0))


def curvature_profile(poses: np.ndarray, window_m: float = CURVATURE_WINDOW_M) -> np.ndarray:
    """Menger curvature through points about window_m before and after each point.

    Triples that straddle a cusp (travel direction reversal) are skipped: that
    is a direction change, not a turn, and the forward-only check owns it.
    """
    xy = poses[:, :2]
    if len(xy) < 3:
        return np.zeros(0)
    seg = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    d = np.diff(xy, axis=0)
    turn_back = np.concatenate([[False], np.einsum("ij,ij->i", d[:-1], d[1:]) < 0.0, [False]])
    cusp_index = np.cumsum(turn_back)          # cusps passed up to each point
    lo = np.searchsorted(s, s - window_m, side="right") - 1
    hi = np.searchsorted(s, s + window_m, side="left")
    out = []
    for i in range(len(xy)):
        a, b = lo[i], hi[i]
        if a < 0 or b >= len(xy) or a == i or b == i:
            continue
        if cusp_index[b] != cusp_index[a] or turn_back[i]:
            continue
        p, q, r = xy[a], xy[i], xy[b]
        ab, bc, ca = math.dist(p, q), math.dist(q, r), math.dist(r, p)
        area2 = abs((q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]))
        denom = ab * bc * ca
        if denom > 1e-12:
            out.append(2.0 * area2 / denom)
    return np.asarray(out)


def _footprint_samples(resolution: float) -> np.ndarray:
    """Points covering the padded footprint rectangle, spaced at most half a cell."""
    step = resolution / 2.0
    xs = np.linspace(FOOTPRINT_X[0], FOOTPRINT_X[1],
                     int(math.ceil((FOOTPRINT_X[1] - FOOTPRINT_X[0]) / step)) + 1)
    ys = np.linspace(FOOTPRINT_Y[0], FOOTPRINT_Y[1],
                     int(math.ceil((FOOTPRINT_Y[1] - FOOTPRINT_Y[0]) / step)) + 1)
    gx, gy = np.meshgrid(xs, ys)
    return np.column_stack([gx.ravel(), gy.ravel()])


def footprint_cells(grid: Grid, poses: np.ndarray):
    """Cells under the padded footprint at every pose.

    Returns (unique (row, col) inside the grid, number of footprint samples
    outside the grid).
    """
    local = _footprint_samples(grid.resolution)
    c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    wx = poses[:, 0:1] + c * local[:, 0] - s * local[:, 1]
    wy = poses[:, 1:2] + s * local[:, 0] + c * local[:, 1]
    col = np.floor((wx.ravel() - grid.origin_x) / grid.resolution).astype(np.int64)
    row = np.floor((wy.ravel() - grid.origin_y) / grid.resolution).astype(np.int64)
    h, w = grid.data.shape
    inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    cells = np.unique(np.column_stack([row[inside], col[inside]]), axis=0)
    return cells, int(np.sum(~inside))


def min_clearance(grid: Grid, poses: np.ndarray) -> float | None:
    """Smallest distance from the padded footprint to a lethal cell centre, or
    None if no lethal cell is within CLEARANCE_SEARCH_M of the path."""
    rows, cols = np.nonzero(grid.data >= LETHAL_COST)
    if len(rows) == 0:
        return None
    ox = grid.origin_x + (cols + 0.5) * grid.resolution
    oy = grid.origin_y + (rows + 0.5) * grid.resolution
    reach = CLEARANCE_SEARCH_M + math.hypot(max(abs(v) for v in FOOTPRINT_X), max(abs(v) for v in FOOTPRINT_Y))
    box = ((ox > poses[:, 0].min() - reach) & (ox < poses[:, 0].max() + reach)
           & (oy > poses[:, 1].min() - reach) & (oy < poses[:, 1].max() + reach))
    ox, oy = ox[box], oy[box]
    best = None
    for x, y, yaw in poses:
        near = (np.abs(ox - x) < reach) & (np.abs(oy - y) < reach)
        if not np.any(near):
            continue
        dx, dy = ox[near] - x, oy[near] - y
        c, s = math.cos(yaw), math.sin(yaw)
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        ex = np.maximum.reduce([FOOTPRINT_X[0] - lx, np.zeros_like(lx), lx - FOOTPRINT_X[1]])
        ey = np.maximum.reduce([FOOTPRINT_Y[0] - ly, np.zeros_like(ly), ly - FOOTPRINT_Y[1]])
        d = float(np.min(np.hypot(ex, ey)))
        best = d if best is None else min(best, d)
    if best is not None and best > CLEARANCE_SEARCH_M:
        return None
    return best


def check_plan(poses, plan_frame: str, grid: Grid | None, now: float,
               forward_only: bool = True, max_curvature: float = MAX_CURVATURE) -> PlanCheck:
    """Every check on one plan.  poses: sequence of (x, y, yaw) in plan_frame."""
    result = PlanCheck(ok=False)
    reasons = result.reasons
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    if len(poses) < 2:
        reasons.append("empty path" if len(poses) == 0 else "path has a single pose")
        return result
    if not np.all(np.isfinite(poses)):
        reasons.append("path has non-finite values")
        return result
    if plan_frame != "map":
        reasons.append(f"plan frame is '{plan_frame}', not 'map'")
    if grid is None:
        reasons.append("no global costmap received")
    else:
        if grid.frame != "map":
            reasons.append(f"costmap frame is '{grid.frame}', not 'map'")
        age = now - grid.received
        if age > COSTMAP_MAX_AGE_S:
            reasons.append(f"costmap is {age:.1f} s old (limit {COSTMAP_MAX_AGE_S:.0f} s)")
        if abs(grid.origin_yaw) > 1e-6:
            reasons.append("rotated costmap origin is not supported")

    result.length_m = float(np.sum(np.hypot(*np.diff(poses[:, :2], axis=0).T)))
    dense = resample(poses)
    result.poses_checked = len(dense)

    if grid is not None and abs(grid.origin_yaw) <= 1e-6:
        cells, outside = footprint_cells(grid, dense)
        values = grid.data[cells[:, 0], cells[:, 1]] if len(cells) else np.zeros(0)
        result.cells_checked = int(len(cells))
        result.cells_outside = outside
        result.cells_blocked = int(np.sum(values >= BLOCKED_COST))
        result.cells_unknown = int(np.sum(values < 0))
        if result.cells_blocked:
            reasons.append(f"footprint touches {result.cells_blocked} cells with cost >= {BLOCKED_COST}")
        if result.cells_unknown:
            reasons.append(f"footprint touches {result.cells_unknown} unknown cells")
        if outside:
            reasons.append("footprint leaves the costmap")
        result.min_clearance_m = min_clearance(grid, dense)

    directions = segment_directions(dense)
    result.reverse_segments = int(np.sum(directions < 0))
    result.cusps = count_cusps(dense)
    if forward_only and (result.reverse_segments or result.cusps):
        reasons.append(f"not forward-only ({result.cusps} cusps, {result.reverse_segments} reverse segments)")

    curvature = curvature_profile(dense)
    result.max_curvature = float(np.max(curvature)) if len(curvature) else 0.0
    if result.max_curvature > max_curvature:
        reasons.append(f"curvature {result.max_curvature:.3f} 1/m above {max_curvature:.3f} "
                       f"(radius {1.0 / result.max_curvature:.2f} m)")

    result.ok = not reasons
    return result


# ------------------------------------------------------------------ decisions
class GateState:
    """Verdict for the active NavigateToPose goal.  No ROS, no clocks of its own."""

    def __init__(self) -> None:
        self.goal_key = None
        self.goal_stamp = None
        self.goal_status = None
        self.verdict = NONE
        self.reasons: list = []
        self.plan_stamp = None
        self.plan_report: dict = {}

    def on_status(self, goals) -> bool:
        """goals: iterable of (key, stamp_sec, status).  Returns True when the active goal changed.

        The active goal is the newest one that is accepted, executing or
        canceling.  A different goal (or none) resets the verdict to NONE.
        """
        live = [g for g in goals if g[2] in (STATUS_ACCEPTED, STATUS_EXECUTING, STATUS_CANCELING)]
        newest = max(live, key=lambda g: g[1]) if live else None
        key = newest[0] if newest else None
        self.goal_status = newest[2] if newest else None
        if key == self.goal_key:
            return False
        self.goal_key = key
        self.goal_stamp = newest[1] if newest else None
        self.verdict, self.reasons, self.plan_stamp, self.plan_report = NONE, [], None, {}
        return True

    def on_plan(self, plan_stamp: float, check: PlanCheck) -> str:
        """Returns 'ignored', 'pass' or 'fail' ('fail' only on the transition into FAIL)."""
        if self.goal_key is None or plan_stamp <= self.goal_stamp:
            return "ignored"
        if self.verdict == FAIL:
            return "ignored"
        self.plan_stamp = plan_stamp
        self.plan_report = check.as_dict()
        if check.ok:
            self.verdict, self.reasons = PASS, []
            return "pass"
        self._fail(list(check.reasons))
        return "fail"

    def on_twist(self, linear_x: float) -> tuple:
        """Returns (forward, newly_failed)."""
        if self.goal_key is None:
            return False, False
        if not math.isfinite(linear_x) or linear_x < 0.0:
            if self.verdict == FAIL:
                return False, False
            self._fail([f"reverse or invalid velocity command (linear.x={linear_x})"])
            return False, True
        forward = self.verdict == PASS and self.goal_status in (STATUS_ACCEPTED, STATUS_EXECUTING)
        return forward, False

    def fail_external(self, reason: str) -> bool:
        if self.goal_key is None or self.verdict == FAIL:
            return False
        self._fail([reason])
        return True

    def _fail(self, reasons) -> None:
        self.verdict, self.reasons = FAIL, reasons

    def report(self) -> dict:
        return {"verdict": self.verdict, "reasons": list(self.reasons),
                "goal": self.goal_key, "goal_stamp": self.goal_stamp, "goal_status": self.goal_status,
                "plan_stamp": self.plan_stamp, **({"plan": self.plan_report} if self.plan_report else {})}


# ------------------------------------------------------------------ ROS node
def _stamp(t) -> float:
    return float(t.sec) + float(t.nanosec) * 1e-9


def _quat_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(argv=None) -> int:
    import rclpy
    from action_msgs.msg import GoalInfo, GoalStatusArray
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Path
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Empty, String

    latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class PathGate(Node):
        def __init__(self) -> None:
            super().__init__("laksa_path_gate")
            self.declare_parameter("input_topic", "/laksa/nav_cmd_vel_raw")
            self.declare_parameter("output_topic", "/laksa/nav_cmd_vel")
            self.declare_parameter("forward_only", True)
            self.declare_parameter("log_path", os.path.expanduser("~/laksa_run/latest/path_gate.log"))
            input_topic = str(self.get_parameter("input_topic").value)
            output_topic = str(self.get_parameter("output_topic").value)
            if input_topic == output_topic:
                raise SystemExit("path_gate: input_topic and output_topic must differ")
            self._forward_only = bool(self.get_parameter("forward_only").value)
            self._log = self._open_log(str(self.get_parameter("log_path").value))
            self._state = GateState()
            self._grid = None
            self._forwarded = 0
            self._dropped = 0

            self._out = self.create_publisher(Twist, output_topic, 10)
            self._report_pub = self.create_publisher(String, "/laksa/path_gate", latched)
            self._cancel_pub = self.create_publisher(Empty, "/laksa/cancel_navigation", 10)
            self._cancel = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
            self.create_subscription(Twist, input_topic, self._twist_cb, 10)
            self.create_subscription(Path, "/plan", self._plan_cb, 10)
            self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._costmap_cb, latched)
            self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self._status_cb, latched)
            self.create_timer(1.0, self._publish_report)
            self._record("start", {"input": input_topic, "output": output_topic,
                                   "forward_only": self._forward_only})

        # -- logging / reporting
        def _open_log(self, path):
            parent = os.path.dirname(path)
            # Never create ~/laksa_run/latest: the launcher owns that symlink.
            if not os.path.isdir(parent):
                self.get_logger().warn(f"{parent} does not exist: path_gate log goes to the ROS log only")
                return None
            return open(path, "a", buffering=1, encoding="utf-8")

        def _record(self, event, extra=None):
            entry = {"t": time.time(), "event": event, **self._state.report(),
                     "forwarded": self._forwarded, "dropped": self._dropped, **(extra or {})}
            line = json.dumps(entry, sort_keys=True)
            if self._log is not None:
                self._log.write(line + "\n")
            if event in ("fail", "start"):
                self.get_logger().warn(line)
            else:
                self.get_logger().info(line)
            self._publish_report()

        def _publish_report(self):
            self._report_pub.publish(String(data=json.dumps(
                {**self._state.report(), "forwarded": self._forwarded, "dropped": self._dropped}, sort_keys=True)))

        # -- failure side effects
        def _on_fail(self):
            self._cancel_pub.publish(Empty())
            if self._cancel.service_is_ready():
                self._cancel.call_async(CancelGoal.Request(goal_info=GoalInfo()))   # zero id + stamp: cancel all
            else:
                self.get_logger().error("navigate_to_pose cancel service unavailable")
            self._record("fail")

        # -- callbacks
        def _status_cb(self, message):
            goals = [(bytes(s.goal_info.goal_id.uuid).hex(), _stamp(s.goal_info.stamp), int(s.status))
                     for s in message.status_list]
            if self._state.on_status(goals):
                self._record("goal", {"active": self._state.goal_key is not None})

        def _costmap_cb(self, message):
            info = message.info
            self._grid = Grid(data=np.asarray(message.data, dtype=np.int16).reshape(info.height, info.width),
                              resolution=float(info.resolution), origin_x=float(info.origin.position.x),
                              origin_y=float(info.origin.position.y), frame=message.header.frame_id,
                              received=time.monotonic(), origin_yaw=_quat_yaw(info.origin.orientation))

        def _plan_cb(self, message):
            poses = [(p.pose.position.x, p.pose.position.y, _quat_yaw(p.pose.orientation)) for p in message.poses]
            stamp = _stamp(message.header.stamp)
            if self._state.goal_key is None or stamp <= (self._state.goal_stamp or 0.0):
                self._record("plan_ignored", {"plan_stamp_seen": stamp})
                return
            check = check_plan(poses, message.header.frame_id, self._grid, time.monotonic(),
                               forward_only=self._forward_only)
            outcome = self._state.on_plan(stamp, check)
            if outcome == "fail":
                self._on_fail()
            else:
                self._record("plan_" + outcome, {"plan_stamp_seen": stamp})

        def _twist_cb(self, message):
            forward, newly_failed = self._state.on_twist(float(message.linear.x))
            if forward:
                self._out.publish(message)
                self._forwarded += 1
                return
            self._dropped += 1
            if newly_failed:
                self._on_fail()

    import rclpy.executors
    rclpy.init(args=argv)
    node = PathGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
