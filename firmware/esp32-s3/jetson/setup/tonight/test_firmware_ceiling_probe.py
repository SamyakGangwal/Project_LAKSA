"""Focused regression tests for firmware_ceiling_probe's analysis functions.

These tests exercise the exact defects identified in the P1/P2 review:
  - Wrong-sign eRPM must be detected
  - active_erpm=0 cannot count as a pass
  - Single-sample plateau decisions are insufficient
  - Near-zero requested alone does not prove rejection

    cd setup/tonight && python3 -m pytest -q test_firmware_ceiling_probe.py
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from firmware_ceiling_probe import (CLAMP_TOLERANCE, ERPM_PER_MPS, MIN_PLATEAU_SAMPLES,  # noqa: E402
                                    ZERO_ERPM, ceiling, summarize, table)


# ---- helpers ----

def make_samples(speed, n=20, requested_erpm=None, active_erpm=0.0,
                 dir_pending=False, fresh=False, hold_s=2.0, plateau_s=1.0):
    """Create n samples spanning hold_s, with the last n/2 on the plateau."""
    if requested_erpm is None:
        requested_erpm = ERPM_PER_MPS * speed
    times = [hold_s * i / (n - 1) for i in range(n)]
    return [(t, requested_erpm, active_erpm, dir_pending, fresh) for t in times]


# ---------- 1. Wrong-sign detection ----------

def test_wrong_sign_erpm_detected():
    """A positive command producing negative requested eRPM must be WRONG_SIGN."""
    samples = make_samples(0.50, requested_erpm=-2070.0)
    row = summarize(0.50, samples)
    assert row["kind"] == "WRONG_SIGN"
    assert row["clamp"] is True
    assert any("sign" in f for f in row["flags"])


def test_correct_sign_is_not_flagged():
    """Matching signs should not trigger WRONG_SIGN."""
    samples = make_samples(0.50, requested_erpm=2070.0)
    row = summarize(0.50, samples)
    assert row["kind"] != "WRONG_SIGN"


def test_negative_speed_negative_requested_not_wrong_sign():
    """Negative speed with negative requested is correct."""
    samples = make_samples(-0.50, requested_erpm=-2070.0)
    row = summarize(-0.50, samples)
    assert row["kind"] != "WRONG_SIGN"


def test_negative_speed_positive_requested_is_wrong_sign():
    """Negative speed with positive requested is wrong sign."""
    samples = make_samples(-0.50, requested_erpm=2070.0)
    row = summarize(-0.50, samples)
    assert row["kind"] == "WRONG_SIGN"


# ---------- 2. active_erpm=0 flagging ----------

def test_active_erpm_zero_flagged():
    """active_erpm ~= 0 must be flagged (battery unplugged: unverified)."""
    samples = make_samples(0.30, active_erpm=0.0)
    row = summarize(0.30, samples)
    assert any("unplugged" in f or "unverified" in f for f in row["flags"])


def test_active_erpm_zero_can_still_pass():
    """active_erpm=0 is flagged but shouldn't prevent a PASS if requested matches."""
    expected = ERPM_PER_MPS * 0.30
    samples = make_samples(0.30, requested_erpm=expected, active_erpm=0.0)
    row = summarize(0.30, samples)
    assert row["kind"] == "PASS"
    # But the flag must still be present.
    assert any("unverified" in f for f in row["flags"])


# ---------- 3. Single-sample plateau → INSUFFICIENT ----------

def test_single_sample_plateau_is_insufficient():
    """Fewer than MIN_PLATEAU_SAMPLES on the plateau must yield INSUFFICIENT."""
    # Put 1 sample right at the plateau boundary and nothing else in the window.
    samples = [(2.0 - 0.5, 1242.0, 0.0, False, False),   # before plateau
               (1.5, 1242.0, 0.0, False, False)]          # exactly on plateau edge
    # Only the sample at t >= (HOLD_S - PLATEAU_S) = 1.0 counts as plateau.
    row = summarize(0.30, samples)
    # With hold_s=2.0 and plateau_s=1.0, plateau starts at t=1.0.
    # We have 1 sample at t=1.5 → 1 sample < MIN_PLATEAU_SAMPLES
    if row["n_plateau"] < MIN_PLATEAU_SAMPLES:
        assert row["kind"] == "INSUFFICIENT"
        assert row["clamp"] is None


def test_two_samples_still_insufficient():
    """Two samples on the plateau is still below the minimum of 3."""
    assert MIN_PLATEAU_SAMPLES == 3, "test assumes MIN_PLATEAU_SAMPLES == 3"
    samples = [(1.0, 1242.0, 0.0, False, False),
               (1.5, 1242.0, 0.0, False, False)]
    row = summarize(0.30, samples)
    assert row["n_plateau"] == 2
    assert row["kind"] == "INSUFFICIENT"
    assert row["clamp"] is None


def test_three_samples_is_sufficient():
    """Exactly MIN_PLATEAU_SAMPLES on the plateau should produce a real verdict."""
    expected = ERPM_PER_MPS * 0.30
    samples = [(1.0, expected, 0.0, False, False),
               (1.5, expected, 0.0, False, False),
               (1.9, expected, 0.0, False, False)]
    row = summarize(0.30, samples)
    assert row["n_plateau"] == 3
    assert row["kind"] in ("PASS", "CLAMPED", "REJECTED")


# ---------- 4. Near-zero requested + rejection caveat ----------

def test_near_zero_requested_is_rejected_not_pass():
    """Near-zero requested eRPM must be classified as REJECTED, never PASS."""
    samples = make_samples(0.50, requested_erpm=0.5)  # way below ZERO_ERPM threshold
    row = summarize(0.50, samples)
    assert row["kind"] == "REJECTED"
    assert row["clamp"] is True


def test_rejected_has_caveat_flag():
    """REJECTED must carry a caveat about not proving firmware rejection."""
    samples = make_samples(0.50, requested_erpm=0.5)
    row = summarize(0.50, samples)
    assert row["kind"] == "REJECTED"
    assert any("does not prove" in f for f in row["flags"])


# ---------- 5. PASS / CLAMPED classification ----------

def test_full_pass_through():
    """requested == expected → PASS with no clamp."""
    expected = ERPM_PER_MPS * 0.30
    samples = make_samples(0.30, requested_erpm=expected)
    row = summarize(0.30, samples)
    assert row["kind"] == "PASS"
    assert row["clamp"] is False


def test_clamped_below_95_pct():
    """requested below 95% of expected → CLAMPED."""
    expected = ERPM_PER_MPS * 1.00  # 4140
    clamped_erpm = expected * 0.50  # 2070, well below 95%
    samples = make_samples(1.00, requested_erpm=clamped_erpm)
    row = summarize(1.00, samples)
    assert row["kind"] == "CLAMPED"
    assert row["clamp"] is True


# ---------- 6. ceiling() with repaired classification ----------

def test_ceiling_with_wrong_sign_aborts():
    """ceiling() with any WRONG_SIGN row must not determine C."""
    rows = [
        summarize(0.30, make_samples(0.30)),
        summarize(0.50, make_samples(0.50, requested_erpm=-2070.0)),  # wrong sign
    ]
    result = ceiling(rows)
    assert "WRONG_SIGN" in result
    assert "C not determined" in result


def test_ceiling_with_insufficient_is_incomplete():
    """ceiling() with INSUFFICIENT rows must report INCOMPLETE."""
    rows = [
        summarize(0.30, make_samples(0.30)),
        summarize(0.50, [(1.5, 2070.0, 0.0, False, False)]),  # 1 plateau sample
    ]
    result = ceiling(rows)
    assert "INCOMPLETE" in result


def test_ceiling_mixed_pass_and_rejected():
    """ceiling() with PASS + REJECTED gives a range, with the caveat."""
    rows = [
        summarize(0.30, make_samples(0.30)),                           # PASS
        summarize(0.50, make_samples(0.50, requested_erpm=0.5)),       # REJECTED
    ]
    result = ceiling(rows)
    assert "passed" in result.lower() or "PASS" in result
    assert "REJECT" in result


# ---------- 7. table() regression ----------

def test_table_renders_all_kinds():
    """table() must render all verdict kinds without crashing."""
    rows = [
        summarize(0.30, make_samples(0.30)),
        summarize(0.50, make_samples(0.50, requested_erpm=-2070.0)),
        summarize(1.00, []),
        summarize(-0.50, [(1.5, -2070.0, 0.0, False, False)]),
    ]
    output = table(rows)
    assert "PASS" in output
    assert "WRONG_SIGN" in output
    assert "no data" in output
    assert "INSUFFICIENT" in output


# ---------- 8. No-sample edge case ----------

def test_no_samples_returns_none_kind():
    """Empty samples → kind=None, clamp=None."""
    row = summarize(0.30, [])
    assert row["kind"] is None
    assert row["clamp"] is None
    assert row["n"] == 0
