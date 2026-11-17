"""
safety_validator.py.

Core algorithmic safety verification module for the SurgiTwin surgical digital twin.
Provides deterministic, standalone validation of robot target coordinates, Cartesian
velocity limits, emergency stop state transitions, and numerical integrity.

Standards Awareness:
- Designed in alignment with IEC 62304 Class B safety architecture principles
  (architectural separation of safety supervision from trajectory planning).
- Risk mitigation measures per ISO 14971 (preventing out-of-bounds instrument travel
  and excessive velocity tissue trauma).
"""

from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
import math
import os
from typing import Any, Dict, List, Optional, Tuple


class EStopState(str, Enum):
    """Emergency stop operational state enumeration."""

    NORMAL = "NORMAL"
    STOPPED = "STOPPED"


@dataclass
class WorkspaceBounds:
    """3D Cartesian workspace boundaries in the robot base frame (metres)."""

    min_x: float = -0.30
    max_x: float = 0.30
    min_y: float = 0.10
    max_y: float = 0.60
    min_z: float = 0.00
    max_z: float = 0.40

    def contains(self, x: float, y: float, z: float) -> Tuple[bool, str]:
        """Check if Cartesian coordinates fall within configured boundaries (inclusive)."""
        if x < self.min_x:
            return False, f"WORKSPACE_VIOLATION: X ({x:.4f}m) below minimum ({self.min_x:.4f}m)"
        if x > self.max_x:
            return False, f"WORKSPACE_VIOLATION: X ({x:.4f}m) above maximum ({self.max_x:.4f}m)"
        if y < self.min_y:
            return False, f"WORKSPACE_VIOLATION: Y ({y:.4f}m) below minimum ({self.min_y:.4f}m)"
        if y > self.max_y:
            return False, f"WORKSPACE_VIOLATION: Y ({y:.4f}m) above maximum ({self.max_y:.4f}m)"
        if z < self.min_z:
            return False, f"WORKSPACE_VIOLATION: Z ({z:.4f}m) below minimum ({self.min_z:.4f}m)"
        if z > self.max_z:
            return False, f"WORKSPACE_VIOLATION: Z ({z:.4f}m) above maximum ({self.max_z:.4f}m)"
        return True, "WORKSPACE_OK"


@dataclass
class ValidationResult:
    """Detailed result of safety validation check."""

    is_valid: bool
    reason: str
    details: Dict[str, Any] = field(default_factory=dict)


class SafetyValidator:
    """
    Independent safety supervisor logic for surgical instrument target verification.

    Enforces workspace constraints, velocity caps, E-stop state machine, and numerical
    validity. Independent of ROS 2 runtime for high-speed offline verification.
    """

    def __init__(
        self,
        bounds: Optional[WorkspaceBounds] = None,
        max_velocity_mps: float = 0.100,  # 100 mm/s standard surgical arm limit
        max_staleness_sec: float = 2.0,
        allowed_frame_id: str = "base_link",
    ) -> None:
        """Initialize safety validator with operational parameters."""
        self.bounds = bounds if bounds is not None else WorkspaceBounds()
        self.max_velocity_mps = max_velocity_mps
        self.max_staleness_sec = max_staleness_sec
        self.allowed_frame_id = allowed_frame_id

        # Internal state
        self._state = EStopState.NORMAL
        self._last_accepted_pose: Optional[Tuple[float, float, float]] = None
        self._last_accepted_time: Optional[float] = None

        # Statistics & telemetry
        self.total_received: int = 0
        self.total_accepted: int = 0
        self.total_rejected: int = 0
        self.estop_activations: int = 0
        self.rejection_reasons: Dict[str, int] = {}
        self.events: List[Dict[str, Any]] = []

    @property
    def state(self) -> EStopState:
        """Return current E-stop state."""
        return self._state

    def trigger_estop(self, reason: str = "Operator Triggered") -> Tuple[bool, str]:
        """Transition system to STOPPED state immediately and record activation."""
        self._state = EStopState.STOPPED
        self.estop_activations += 1
        msg = f"ESTOP_ACTIVATED: {reason}"
        self._record_event("ESTOP_TRIGGER", msg, {"reason": reason})
        return True, msg

    def reset_estop(self) -> Tuple[bool, str]:
        """
        Attempt to reset system from STOPPED back to NORMAL state.

        Reset clears previous kinematic baseline to prevent velocity jumps.
        """
        if self._state == EStopState.NORMAL:
            return False, "RESET_REJECTED: E-Stop is not active"

        self._state = EStopState.NORMAL
        # Clear velocity baseline so the next motion calculates from fresh baseline
        self._last_accepted_pose = None
        self._last_accepted_time = None
        msg = "ESTOP_RESET: Resumed NORMAL operation"
        self._record_event("ESTOP_RESET", msg, {})
        return True, msg

    def validate_input_integrity(
        self,
        x: float,
        y: float,
        z: float,
        qx: float = 0.0,
        qy: float = 1.0,
        qz: float = 0.0,
        qw: float = 0.0,
        frame_id: str = "base_link",
        stamp_sec: Optional[float] = None,
        now_sec: Optional[float] = None,
    ) -> ValidationResult:
        """
        Verify numerical validity, coordinate frame, and temporal freshness.

        Fails closed on any NaN, Inf, unsupported frame, or stale timestamp.
        """
        # 1. Numerical integrity check (NaN / Inf)
        coords = [("x", x), ("y", y), ("z", z), ("qx", qx), ("qy", qy), ("qz", qz), ("qw", qw)]
        for name, val in coords:
            if math.isnan(val):
                return ValidationResult(False, f"INPUT_VIOLATION: NaN detected in field '{name}'")
            if math.isinf(val):
                return ValidationResult(False, f"INPUT_VIOLATION: Infinity detected in field '{name}'")

        # 2. Coordinate frame validation
        if self.allowed_frame_id and frame_id != self.allowed_frame_id:
            return ValidationResult(
                False,
                f"FRAME_VIOLATION: Frame '{frame_id}' does not match expected '{self.allowed_frame_id}'",
            )

        # 3. Timestamp & Staleness verification
        if stamp_sec is not None:
            if stamp_sec < 0:
                return ValidationResult(False, f"TIMESTAMP_VIOLATION: Negative timestamp ({stamp_sec}s)")

            if now_sec is not None:
                # Reject target from significantly in the future (clock skew > 0.5s)
                if stamp_sec > now_sec + 0.5:
                    return ValidationResult(
                        False,
                        f"TIMESTAMP_VIOLATION: Target timestamp in future ({stamp_sec - now_sec:.3f}s ahead)",
                    )
                # Reject stale target exceeding max_staleness_sec
                age = now_sec - stamp_sec
                if age > self.max_staleness_sec:
                    return ValidationResult(
                        False,
                        f"STALE_TARGET: Age ({age:.3f}s) exceeds threshold ({self.max_staleness_sec:.3f}s)",
                    )

        return ValidationResult(True, "INPUT_INTEGRITY_OK")

    def validate_workspace(self, x: float, y: float, z: float) -> ValidationResult:
        """Verify target position falls within Cartesian workspace envelope."""
        is_valid, reason = self.bounds.contains(x, y, z)
        return ValidationResult(is_valid, reason, {"x": x, "y": y, "z": z})

    def validate_velocity(
        self,
        x: float,
        y: float,
        z: float,
        timestamp_sec: float,
    ) -> ValidationResult:
        """
        Validate Cartesian velocity between consecutive accepted poses.

        Calculation Method:
            v = ||P_current - P_previous|| / (t_current - t_previous)
        If v > max_velocity_mps or dt <= 0, the target is rejected.
        """
        if self._last_accepted_pose is None or self._last_accepted_time is None:
            # First target: no velocity violation possible
            return ValidationResult(True, "VELOCITY_INITIAL_BASELINE", {"velocity_mps": 0.0})

        dt = timestamp_sec - self._last_accepted_time
        if dt <= 0:
            return ValidationResult(
                False,
                f"VELOCITY_VIOLATION: Non-positive time delta (dt={dt:.6f}s)",
                {"dt": dt},
            )

        dx = x - self._last_accepted_pose[0]
        dy = y - self._last_accepted_pose[1]
        dz = z - self._last_accepted_pose[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        velocity = dist / dt

        if velocity > self.max_velocity_mps:
            return ValidationResult(
                False,
                f"VELOCITY_VIOLATION: Computed velocity {velocity * 1000.0:.1f} mm/s "
                f"exceeds limit {self.max_velocity_mps * 1000.0:.1f} mm/s",
                {"velocity_mps": velocity, "limit_mps": self.max_velocity_mps, "dt": dt, "dist": dist},
            )

        return ValidationResult(True, "VELOCITY_OK", {"velocity_mps": velocity, "dt": dt, "dist": dist})

    def validate_target(
        self,
        x: float,
        y: float,
        z: float,
        qx: float = 0.0,
        qy: float = 1.0,
        qz: float = 0.0,
        qw: float = 0.0,
        frame_id: str = "base_link",
        stamp_sec: Optional[float] = None,
        now_sec: Optional[float] = None,
    ) -> ValidationResult:
        """
        Execute full safety interlock pipeline on incoming Cartesian target.

        Fail-Safe Sequence:
        1. E-Stop state check (Blocks immediately if STOPPED)
        2. Input numerical and frame integrity check
        3. 3D Workspace boundary check
        4. Cartesian velocity threshold check
        """
        self.total_received += 1
        eval_time = stamp_sec if stamp_sec is not None else (now_sec if now_sec is not None else 0.0)

        # 1. E-Stop Check
        if self._state == EStopState.STOPPED:
            reason = "ESTOP_ACTIVE: Target blocked because Emergency Stop is active"
            self._record_rejection(reason, {"x": x, "y": y, "z": z})
            return ValidationResult(False, reason, {"state": self._state})

        # 2. Input Integrity
        res_input = self.validate_input_integrity(
            x, y, z, qx, qy, qz, qw, frame_id, stamp_sec, now_sec
        )
        if not res_input.is_valid:
            self._record_rejection(res_input.reason, {"x": x, "y": y, "z": z})
            return res_input

        # 3. Workspace Bounds
        res_ws = self.validate_workspace(x, y, z)
        if not res_ws.is_valid:
            self._record_rejection(res_ws.reason, {"x": x, "y": y, "z": z})
            return res_ws

        # 4. Velocity Limits
        res_vel = self.validate_velocity(x, y, z, eval_time)
        if not res_vel.is_valid:
            self._record_rejection(res_vel.reason, {"x": x, "y": y, "z": z})
            return res_vel

        # Target validated successfully
        self.total_accepted += 1
        self._last_accepted_pose = (x, y, z)
        self._last_accepted_time = eval_time

        self._record_event(
            "TARGET_ACCEPTED",
            "Target passed all safety interlocks",
            {"x": x, "y": y, "z": z, "stamp_sec": eval_time},
        )
        return ValidationResult(True, "VALID_SAFE_TARGET", {"x": x, "y": y, "z": z})

    def _record_rejection(self, reason: str, details: Dict[str, Any]) -> None:
        """Update rejection statistics and audit history."""
        self.total_rejected += 1
        cat = reason.split(":")[0].strip()
        self.rejection_reasons[cat] = self.rejection_reasons.get(cat, 0) + 1
        self._record_event("TARGET_REJECTED", reason, details)

    def _record_event(self, event_type: str, message: str, details: Dict[str, Any]) -> None:
        """Record timestamped safety event."""
        self.events.append(
            {
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "event_type": event_type,
                "message": message,
                "state": self._state.value,
                "details": details,
            }
        )

    def export_events_csv(self, filepath: str) -> None:
        """Flush safety audit log to CSV file for traceability."""
        import csv

        expanded = os.path.expanduser(filepath)
        os.makedirs(os.path.dirname(expanded) or ".", exist_ok=True)
        fieldnames = ["timestamp", "event_type", "state", "message", "details"]
        with open(expanded, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for ev in self.events:
                writer.writerow(
                    {
                        "timestamp": ev["timestamp"],
                        "event_type": ev["event_type"],
                        "state": ev["state"],
                        "message": ev["message"],
                        "details": str(ev["details"]),
                    }
                )
