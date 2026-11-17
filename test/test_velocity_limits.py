"""
test_velocity_limits.py.

Automated unit tests for Cartesian tip velocity limit verification in SurgiTwin.
Verifies velocity calculations, limit threshold enforcement, zero dt handling,
and negative dt handling.
"""

import pytest
from surgical_sim.safety_validator import SafetyValidator


@pytest.fixture
def validator() -> SafetyValidator:
    """Fixture providing a SafetyValidator with a 100 mm/s (0.100 m/s) cap."""
    return SafetyValidator(max_velocity_mps=0.100)


def test_initial_target_accepted_as_baseline(validator: SafetyValidator) -> None:
    """Initial target has no velocity violation and establishes spatial baseline."""
    res = validator.validate_target(x=0.0, y=0.30, z=0.20, stamp_sec=1.0)
    assert res.is_valid is True
    assert validator.total_accepted == 1


def test_acceptable_velocity_forwarded(validator: SafetyValidator) -> None:
    """Moving 50 mm in 1.0 s corresponds to 50 mm/s (< 100 mm/s) -> must be accepted."""
    validator.validate_target(x=0.0, y=0.30, z=0.20, stamp_sec=1.0)
    # Move +50 mm in X at dt = 1.0s (v = 0.050 m/s)
    res = validator.validate_target(x=0.05, y=0.30, z=0.20, stamp_sec=2.0)
    assert res.is_valid is True
    assert validator.total_accepted == 2


def test_excessive_velocity_rejected(validator: SafetyValidator) -> None:
    """Moving 150 mm in 0.5 s corresponds to 300 mm/s (> 100 mm/s) -> must be rejected."""
    validator.validate_target(x=0.0, y=0.30, z=0.20, stamp_sec=1.0)
    # Move +150 mm in X at dt = 0.5s (v = 0.300 m/s)
    res = validator.validate_target(x=0.15, y=0.30, z=0.20, stamp_sec=1.5)
    assert res.is_valid is False
    assert "VELOCITY_VIOLATION" in res.reason
    assert "exceeds limit" in res.reason
    assert validator.total_rejected == 1


def test_zero_time_interval_rejected_safely(validator: SafetyValidator) -> None:
    """Target received with identical timestamp (dt = 0.0s) must be rejected without crash."""
    validator.validate_target(x=0.0, y=0.30, z=0.20, stamp_sec=1.0)
    # Same timestamp
    res = validator.validate_target(x=0.02, y=0.30, z=0.20, stamp_sec=1.0)
    assert res.is_valid is False
    assert "Non-positive time delta" in res.reason
    assert validator.total_rejected == 1


def test_negative_time_interval_rejected(validator: SafetyValidator) -> None:
    """Target received out of chronological sequence (dt < 0) must be rejected."""
    validator.validate_target(x=0.0, y=0.30, z=0.20, stamp_sec=5.0)
    # Timestamp in the past relative to last accepted
    res = validator.validate_target(x=0.01, y=0.30, z=0.20, stamp_sec=4.0)
    assert res.is_valid is False
    assert "Non-positive time delta" in res.reason


def test_mathematical_velocity_calculation(validator: SafetyValidator) -> None:
    """Directly test the pure validate_velocity function for numerical precision."""
    # Establish valid baseline at (0.0, 0.30, 0.20)
    res_base = validator.validate_target(x=0.0, y=0.30, z=0.20, stamp_sec=1.0)
    assert res_base.is_valid is True

    # dx=0.03, dy=0.04, dz=0.0 -> dist=sqrt(0.0009 + 0.0016) = 0.05m. dt=1.0s -> v=0.05m/s (50 mm/s)
    res = validator.validate_velocity(x=0.03, y=0.34, z=0.20, timestamp_sec=2.0)
    assert res.is_valid is True
    assert pytest.approx(res.details["velocity_mps"], rel=1e-4) == 0.050
