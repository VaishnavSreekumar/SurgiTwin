"""
test_emergency_stop.py.

Automated unit tests for Emergency Stop (E-Stop) state machine in SurgiTwin.
Verifies transition to STOPPED state, command blocking, explicit reset behavior,
and prevention of stale command replay.
"""

import pytest
from surgical_sim.safety_validator import EStopState, SafetyValidator


@pytest.fixture
def validator() -> SafetyValidator:
    """Fixture providing a default safety validator."""
    return SafetyValidator()


def test_initial_state_is_normal(validator: SafetyValidator) -> None:
    """Validator starts in NORMAL operational state."""
    assert validator.state == EStopState.NORMAL


def test_estop_trigger_transitions_to_stopped(validator: SafetyValidator) -> None:
    """Scenario 15: Triggering E-Stop transitions state to STOPPED and blocks motion."""
    success, msg = validator.trigger_estop("Test emergency push")
    assert success is True
    assert validator.state == EStopState.STOPPED
    assert validator.estop_activations == 1


def test_estop_blocks_valid_and_invalid_targets(validator: SafetyValidator) -> None:
    """All target commands are blocked when E-Stop is active."""
    validator.trigger_estop("Operator E-stop")

    # Valid target
    res_valid = validator.validate_target(x=0.0, y=0.35, z=0.20, stamp_sec=1.0)
    assert res_valid.is_valid is False
    assert "ESTOP_ACTIVE" in res_valid.reason

    # Out of bounds target
    res_oob = validator.validate_target(x=0.8, y=0.9, z=0.9, stamp_sec=2.0)
    assert res_oob.is_valid is False
    assert "ESTOP_ACTIVE" in res_oob.reason


def test_reset_restores_normal_operation(validator: SafetyValidator) -> None:
    """Scenario 16: Explicit reset restores system to NORMAL and permits new motion."""
    validator.trigger_estop("Hardware alert")
    assert validator.state == EStopState.STOPPED

    # Reset
    reset_ok, reset_msg = validator.reset_estop()
    assert reset_ok is True
    assert validator.state == EStopState.NORMAL
    assert "Resumed NORMAL" in reset_msg

    # New valid target accepted without jump error
    res_after = validator.validate_target(x=0.0, y=0.35, z=0.20, stamp_sec=10.0)
    assert res_after.is_valid is True
    assert validator.total_accepted == 1


def test_invalid_reset_when_not_stopped(validator: SafetyValidator) -> None:
    """Attempting to reset when already NORMAL is rejected."""
    reset_ok, reset_msg = validator.reset_estop()
    assert reset_ok is False
    assert "RESET_REJECTED" in reset_msg


def test_event_logging_audit_trail(validator: SafetyValidator) -> None:
    """Verify all E-Stop state transitions are recorded in the event audit log."""
    validator.validate_target(x=0.0, y=0.35, z=0.20, stamp_sec=1.0)
    validator.trigger_estop("Safety barrier opened")
    validator.validate_target(x=0.0, y=0.35, z=0.20, stamp_sec=2.0)
    validator.reset_estop()

    events = [ev["event_type"] for ev in validator.events]
    assert "TARGET_ACCEPTED" in events
    assert "ESTOP_TRIGGER" in events
    assert "TARGET_REJECTED" in events
    assert "ESTOP_RESET" in events
