"""
test_safety_bounds.py.

Automated unit tests for Cartesian workspace boundary enforcement in SurgiTwin.
Verifies inclusive boundary checks, inner region acceptance, and multi-axis rejection.
"""

import pytest
from surgical_sim.safety_validator import SafetyValidator, WorkspaceBounds


@pytest.fixture
def default_validator() -> SafetyValidator:
    """Fixture providing a SafetyValidator with default workspace bounds."""
    # Bounds: X in [-0.3, 0.3], Y in [0.1, 0.6], Z in [0.0, 0.4]
    return SafetyValidator()


def test_valid_target_accepted(default_validator: SafetyValidator) -> None:
    """Scenario 1: Valid target inside operational envelope must be accepted."""
    res = default_validator.validate_target(x=0.0, y=0.35, z=0.20, stamp_sec=1.0)
    assert res.is_valid is True
    assert res.reason == "VALID_SAFE_TARGET"
    assert default_validator.total_accepted == 1
    assert default_validator.total_rejected == 0


def test_target_below_min_x_rejected(default_validator: SafetyValidator) -> None:
    """Scenario 2: Target below minimum X boundary (-0.30m) must be rejected."""
    res = default_validator.validate_target(x=-0.35, y=0.35, z=0.20, stamp_sec=1.0)
    assert res.is_valid is False
    assert "X" in res.reason and "below minimum" in res.reason
    assert default_validator.total_rejected == 1


def test_target_above_max_x_rejected(default_validator: SafetyValidator) -> None:
    """Scenario 3: Target above maximum X boundary (+0.30m) must be rejected."""
    res = default_validator.validate_target(x=0.35, y=0.35, z=0.20, stamp_sec=1.0)
    assert res.is_valid is False
    assert "X" in res.reason and "above maximum" in res.reason
    assert default_validator.total_rejected == 1


def test_target_below_min_y_rejected(default_validator: SafetyValidator) -> None:
    """Scenario 4: Target below minimum Y boundary (0.10m) must be rejected."""
    res = default_validator.validate_target(x=0.0, y=0.05, z=0.20, stamp_sec=1.0)
    assert res.is_valid is False
    assert "Y" in res.reason and "below minimum" in res.reason
    assert default_validator.total_rejected == 1


def test_target_above_max_y_rejected(default_validator: SafetyValidator) -> None:
    """Scenario 5: Target above maximum Y boundary (0.60m) must be rejected."""
    res = default_validator.validate_target(x=0.0, y=0.65, z=0.20, stamp_sec=1.0)
    assert res.is_valid is False
    assert "Y" in res.reason and "above maximum" in res.reason
    assert default_validator.total_rejected == 1


def test_target_below_min_z_rejected(default_validator: SafetyValidator) -> None:
    """Scenario 6: Target below minimum Z boundary (0.00m) must be rejected (table collision)."""
    res = default_validator.validate_target(x=0.0, y=0.35, z=-0.05, stamp_sec=1.0)
    assert res.is_valid is False
    assert "Z" in res.reason and "below minimum" in res.reason
    assert default_validator.total_rejected == 1


def test_target_above_max_z_rejected(default_validator: SafetyValidator) -> None:
    """Scenario 7: Target above maximum Z boundary (0.40m) must be rejected."""
    res = default_validator.validate_target(x=0.0, y=0.35, z=0.45, stamp_sec=1.0)
    assert res.is_valid is False
    assert "Z" in res.reason and "above maximum" in res.reason
    assert default_validator.total_rejected == 1


def test_boundary_values_handled_correctly(default_validator: SafetyValidator) -> None:
    """Scenario 8: Exact boundary limits must be accepted (inclusive bounds)."""
    # Lower bound limits
    res_min = default_validator.validate_target(x=-0.30, y=0.10, z=0.00, stamp_sec=1.0)
    assert res_min.is_valid is True

    # Upper bound limits
    res_max = default_validator.validate_target(x=0.30, y=0.60, z=0.40, stamp_sec=10.0)
    assert res_max.is_valid is True


def test_custom_bounds_configuration() -> None:
    """Scenario 8b: Custom bounds specified at instantiation must be strictly enforced."""
    custom_bounds = WorkspaceBounds(
        min_x=-0.10, max_x=0.10, min_y=0.20, max_y=0.40, min_z=0.05, max_z=0.15
    )
    val = SafetyValidator(bounds=custom_bounds)

    assert val.validate_target(x=0.0, y=0.30, z=0.10, stamp_sec=1.0).is_valid is True
    assert val.validate_target(x=0.15, y=0.30, z=0.10, stamp_sec=2.0).is_valid is False
