"""Unit tests for path_gate's geometry checks and verdict state machine (no ROS).

    cd setup/tonight && python3 -m pytest -q test_path_gate.py
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from path_gate import (FAIL, NONE, PASS, STATUS_EXECUTING, GateState, Grid,  # noqa: E402
                       check_plan)

NOW = 1000.0


def free_grid(age=0.0, frame="map"):
    # 10 x 10 m at 0.05 m, origin (-5, -5), all free.
    return Grid(data=np.zeros((200, 200), dtype=np.int16), resolution=0.05, origin_x=-5.0, origin_y=-5.0,
                frame=frame, received=NOW - age)


def set_cells(grid, x0, x1, y0, y1, value):
    c0, c1 = int((x0 - grid.origin_x) / grid.resolution), int((x1 - grid.origin_x) / grid.resolution)
    r0, r1 = int((y0 - grid.origin_y) / grid.resolution), int((y1 - grid.origin_y) / grid.resolution)
    grid.data[r0:r1 + 1, c0:c1 + 1] = value


def straight(x0=0.0, x1=3.0, y=0.0, step=0.05):
    xs = np.arange(x0, x1 + 1e-9, step)
    return [(x, y, 0.0) for x in xs]


def arc(radius, sweep=math.pi / 2, step=0.05):
    n = int(radius * sweep / step) + 1
    th = np.linspace(0.0, sweep, n)
    # Left turn starting at the origin heading +x, centre (0, radius).
    return [(radius * math.sin(t), radius - radius * math.cos(t), t) for t in th]


def check(poses, grid=None, **kw):
    return check_plan(poses, "map", free_grid() if grid is None else grid, NOW, **kw)


# ------------------------------------------------------------- 1-8 geometry
def test_01_straight_path_in_free_space_passes():
    result = check(straight())
    assert result.ok, result.reasons
    assert result.length_m == pytest.approx(3.0, abs=0.06)
    assert result.min_clearance_m is None           # nothing lethal nearby
    assert result.cells_checked > 0


def test_02_path_hugging_a_wall_fails():
    grid = free_grid()
    set_cells(grid, -1.0, 4.0, 0.15, 0.30, 100)     # wall inside the padded footprint (|y| <= 0.20)
    result = check(straight(), grid)
    assert not result.ok
    assert any("cost >= 99" in r for r in result.reasons)
    assert result.cells_blocked > 0


def test_03_tight_arc_fails_on_curvature():
    result = check(arc(0.75))
    assert not result.ok
    assert any("curvature" in r for r in result.reasons), result.reasons
    assert result.max_curvature == pytest.approx(1 / 0.75, rel=0.05)


def test_04_path_with_a_cusp_fails_forward_only():
    forward = straight(0.0, 1.0)
    back = [(x, 0.0, 0.0) for x in np.arange(0.95, 0.5 - 1e-9, -0.05)]   # reversing, same heading
    result = check(forward + back)
    assert not result.ok
    assert any("forward-only" in r for r in result.reasons), result.reasons
    assert result.cusps == 1 and result.reverse_segments > 0


def test_05_path_crossing_one_unknown_cell_fails():
    grid = free_grid()
    set_cells(grid, 1.5, 1.5, 0.0, 0.0, -1)
    result = check(straight(), grid)
    assert not result.ok
    assert result.cells_unknown == 1
    assert any("unknown" in r for r in result.reasons)


def test_06_path_leaving_the_grid_fails():
    result = check(straight(0.0, 6.0))              # grid ends at x = 5
    assert not result.ok
    assert any("leaves the costmap" in r for r in result.reasons)


def test_07_empty_path_fails():
    result = check([])
    assert not result.ok
    assert result.reasons == ["empty path"]


def test_08_costmap_older_than_3s_fails():
    result = check(straight(), free_grid(age=3.5))
    assert not result.ok
    assert any("old" in r for r in result.reasons)
    assert check(straight(), free_grid(age=2.5)).ok


# --------------------------------------------------------- 9-11 state machine
GOAL = [("g1", 100.0, STATUS_EXECUTING)]


def passing():
    return check(straight())


def test_09_plan_stamped_before_the_goal_is_ignored():
    state = GateState()
    state.on_status(GOAL)
    assert state.on_plan(99.5, passing()) == "ignored"
    assert state.on_plan(100.0, passing()) == "ignored"     # not later than the goal
    assert state.verdict == NONE
    assert state.on_twist(0.2) == (False, False)
    assert state.on_plan(100.5, passing()) == "pass"         # its own plan counts
    assert state.verdict == PASS


def test_10_after_fail_a_pass_looking_twist_is_not_forwarded():
    state = GateState()
    state.on_status(GOAL)
    assert state.on_plan(101.0, check([])) == "fail"
    assert state.verdict == FAIL
    assert state.on_twist(0.2) == (False, False)
    assert state.on_plan(102.0, passing()) == "ignored"      # FAIL latches for this goal
    assert state.verdict == FAIL
    assert state.on_twist(0.2) == (False, False)


def test_11_negative_linear_x_is_not_forwarded_and_latches_fail():
    state = GateState()
    state.on_status(GOAL)
    state.on_plan(101.0, passing())
    assert state.on_twist(0.2) == (True, False)
    assert state.on_twist(-0.05) == (False, True)
    assert state.verdict == FAIL
    assert state.on_twist(0.2) == (False, False)
    assert state.on_twist(-0.05) == (False, False)           # already failed: no second cancel


# ------------------------------------------------------------------- extras
def test_arc_at_planner_minimum_radius_passes():
    result = check(arc(1.09))
    assert result.ok, result.reasons


def test_cusp_allowed_when_not_forward_only_and_not_mistaken_for_curvature():
    forward = straight(0.0, 1.0)
    back = [(x, 0.0, 0.0) for x in np.arange(0.95, 0.5 - 1e-9, -0.05)]
    result = check(forward + back, forward_only=False)
    assert result.ok, result.reasons


def test_wrong_frame_fails():
    assert not check_plan(straight(), "odom", free_grid(), NOW).ok
    assert not check_plan(straight(), "map", free_grid(frame="odom"), NOW).ok
    assert not check_plan(straight(), "map", None, NOW).ok


def test_clearance_is_reported():
    grid = free_grid()
    set_cells(grid, -1.0, 4.0, 0.50, 0.60, 100)
    result = check(straight(), grid)
    assert result.ok, result.reasons
    assert 0.25 < result.min_clearance_m < 0.35             # wall at y >= 0.50, footprint edge 0.20


def test_new_goal_resets_verdict_and_finished_goal_stops_forwarding():
    state = GateState()
    state.on_status(GOAL)
    state.on_plan(101.0, check([]))
    assert state.verdict == FAIL
    assert state.on_status([("g1", 100.0, 5), ("g2", 200.0, STATUS_EXECUTING)]) is True
    assert state.verdict == NONE and state.goal_key == "g2"
    state.on_plan(201.0, passing())
    assert state.on_twist(0.2) == (True, False)
    state.on_status([("g2", 200.0, 4)])                  # SUCCEEDED
    assert state.goal_key is None and state.verdict == NONE
    assert state.on_twist(0.2) == (False, False)


def test_canceling_goal_is_not_forwarded():
    state = GateState()
    state.on_status(GOAL)
    state.on_plan(101.0, passing())
    state.on_status([("g1", 100.0, 3)])                      # CANCELING
    assert state.verdict == PASS
    assert state.on_twist(0.2) == (False, False)
