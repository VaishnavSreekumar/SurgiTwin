"""
test_invalid_inputs.py.

Automated unit tests for numerical integrity and input validation in SurgiTwin.
Verifies fail-closed rejection of NaNs, Infs, stale targets, and mismatched frames.
"""

import math
import pytest
from surgical_sim.safety_validator import SafetyValidator


@pytest.fixture
def validator() -> SafetyValidator:
    """Fixture providing a default safety validator."""
    return SafetyValidator(allowed_frame_id="base_link", max_staleness_sec=2.0)


def test_nan_coordinates_rejected(validator: SafetyValidator) -> None:
    """Scenario 11: NaN coordinates in any axis must fail immediately."""
    # NaN in X
    res_x = validator.validate_target(x=float("nan"), y=0.35, z=0.20, stamp_sec=1.0)
    assert res_x.is_valid is False
    assert "NaN detected in field 'x'" in res_x.reason

    # NaN in Y
    res_y = validator.validate_target(x=0.0, y=float("nan"), z=0.20, stamp_sec=1.0)
    assert res_y.is_valid is False
    assert "NaN detected in field 'y'" in res_y.reason

    # NaN in Z
    res_z = validator.validate_target(x=0.0, y=0.35, z=float("nan"), stamp_sec=1.0)
    assert res_z.is_valid is False
    assert "NaN detected in field 'z'" in res_z.reason


def test_infinity_coordinates_rejected(validator: SafetyValidator) -> None:
    """Scenario 12: Positive or negative infinity in any coordinate must be rejected."""
    res_pos = validator.validate_target(x=math.inf, y=0.35, z=0.20, stamp_sec=1.0)
    assert res_pos.is_valid is False
    assert "Infinity detected in field 'x'" in res_pos.reason

    res_neg = validator.validate_target(x=0.0, y=-math.inf, z=0.20, stamp_sec=1.0)
    assert res_neg.is_valid is False
    assert "Infinity detected in field 'y'" in res_neg.reason


def test_invalid_coordinate_frame_rejected(validator: SafetyValidator) -> None:
    """Scenario 13: Commands with unexpected coordinate frame must be rejected."""
    res = validator.validate_target(
        x=0.0, y=0.35, z=0.20, frame_id="camera_optical_frame", stamp_sec=1.0
    )
    assert res.is_valid is False
    assert "FRAME_VIOLATION" in res.reason
    assert "does not match expected 'base_link'" in res.reason


def test_stale_target_rejected(validator: SafetyValidator) -> None:
    """Scenario 14: Commands with timestamp older than max_staleness_sec are rejected."""
    now = 100.0
    stale_stamp = 95.0  # 5.0 seconds old (> 2.0s threshold)
    res = validator.validate_target(
        x=0.0, y=0.35, z=0.20, stamp_sec=stale_stamp, now_sec=now
    )
    assert res.is_valid is False
    assert "STALE_TARGET" in res.reason
    assert "exceeds threshold" in res.reason


def test_future_timestamp_rejected(validator: SafetyValidator) -> None:
    """Commands with future timestamps beyond allowable clock drift are rejected."""
    now = 100.0
    future_stamp = 102.0  # 2.0s in future (> 0.5s allowable drift)
    res = validator.validate_target(
        x=0.0, y=0.35, z=0.20, stamp_sec=future_stamp, now_sec=now
    )
    assert res.is_valid is False
    assert "TIMESTAMP_VIOLATION" in res.reason
    assert "in future" in res.reason


def test_negative_timestamp_rejected(validator: SafetyValidator) -> None:
    """Commands with negative timestamps are rejected."""
    res = validator.validate_target(x=0.0, y=0.35, z=0.20, stamp_sec=-5.0)
    assert res.is_valid is False
    assert "TIMESTAMP_VIOLATION" in res.reason
