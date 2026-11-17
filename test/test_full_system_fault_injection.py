"""
test_full_system_fault_injection.py
Comprehensive Phase 5 Fault Injection & Failure Recovery Verification Suite.

Validates that SurgiTwin behaves safely and deterministically under intentional
fault injection across four major subsystems:
1. Vision and Medical Input Faults
2. Safety Subsystem Faults
3. Embedded Hardware & Communication Faults
4. ROS 2 Closed-Loop Integration Faults
"""

import os
import shutil
import tempfile
import time
import numpy as np
import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Bool

from surgical_sim.camera_calibration import CameraCalibration
from surgical_sim.coordinate_registration import CoordinateRegistration
from surgical_sim.vision_detector import VisionTargetDetector
from surgical_sim.dicom_loader import DICOMLoader
from surgical_sim.medical_volume import MedicalVolume
from surgical_sim.medical_segmentation import MedicalSegmentation
from surgical_sim.anatomical_target import AnatomicalTargetExtractor
from surgical_sim.safety_validator import SafetyValidator, WorkspaceBounds, EStopState
from surgical_sim.surgitwin_safety_controller import SurgiTwinSafetyController
from surgical_sim.robot_sim_node import SurgiTwinRobotSimNode
from surgical_sim.hardware_interface import (
    HardwareState,
    MsgType,
    ProtocolCodec,
)
from surgical_sim.microcontroller_sim import MicrocontrollerSimulator


TEST_DOMAIN_ID = 51


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init(domain_id=TEST_DOMAIN_ID)
    yield
    rclpy.shutdown()


# ==============================================================================
# SECTION 1: VISION & MEDICAL INPUT FAULTS
# ==============================================================================

def test_fault_vision_missing_or_corrupt_images():
    """Verify detector fails safely on None, empty, or degenerate image buffers."""
    detector = VisionTargetDetector()

    # 1. None image
    res_none = detector.detect(None)
    assert not res_none.detected
    assert "INVALID_INPUT" in res_none.status_message

    # 2. 0-byte/empty image
    empty_img = np.zeros((0, 0, 3), dtype=np.uint8)
    res_empty = detector.detect(empty_img)
    assert not res_empty.detected
    assert "INVALID_INPUT" in res_empty.status_message

    # 3. Flat blank image (no markers)
    blank_img = np.full((480, 640, 3), 128, dtype=np.uint8)
    res_blank = detector.detect(blank_img)
    assert not res_blank.detected


def test_fault_vision_invalid_camera_calibration():
    """Verify camera calibration rejects singular, non-invertible, or negative parameters."""
    dist = np.zeros(5)

    # Zero focal length
    bad_K = np.array([[0.0, 0.0, 320.0], [0.0, 800.0, 240.0], [0.0, 0.0, 1.0]])
    with pytest.raises(ValueError, match="Focal lengths must be positive"):
        CameraCalibration(bad_K, dist, 640, 480)

    # Negative principal point
    bad_K2 = np.array([[800.0, 0.0, -10.0], [0.0, 800.0, 240.0], [0.0, 0.0, 1.0]])
    with pytest.raises(ValueError, match="Principal point cx"):
        CameraCalibration(bad_K2, dist, 640, 480)

    # Non-positive depth projection
    good_K = np.array([[800.0, 0.0, 320.0], [0.0, 800.0, 240.0], [0.0, 0.0, 1.0]])
    calib = CameraCalibration(good_K, dist, 640, 480)
    with pytest.raises(ValueError, match="Depth must be strictly positive"):
        calib.back_project(320.0, 240.0, z=0.0)

    with pytest.raises(ValueError, match="Depth must be strictly positive"):
        calib.back_project(320.0, 240.0, z=-0.5)


def test_fault_vision_invalid_coordinate_transforms():
    """Verify coordinate registration rejects non-orthogonal or reflection matrices."""
    # Non-orthogonal matrix
    bad_T = np.eye(4)
    bad_T[:3, :3] = np.array([
        [1.0, 0.5, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    with pytest.raises(ValueError, match="not orthogonal"):
        CoordinateRegistration(bad_T)

    # Reflection matrix (det = -1.0)
    refl_T = np.eye(4)
    refl_T[:3, :3] = np.diag([-1.0, 1.0, 1.0])
    with pytest.raises(ValueError, match="determinant"):
        CoordinateRegistration(refl_T)


def test_fault_dicom_corrupted_or_missing_metadata():
    """Verify DICOM parser fails closed on corrupted files or incomplete metadata."""
    tmp = tempfile.mkdtemp(prefix="test_fault_dicom_")
    try:
        junk_path = os.path.join(tmp, "junk.dcm")
        with open(junk_path, "wb") as f:
            f.write(b"NOT_A_DICOM_FILE_AT_ALL_JUST_RANDOM_GARBAGE" * 4)

        with pytest.raises(ValueError, match="Invalid DICOM Part 10 magic bytes"):
            DICOMLoader.load_file(junk_path)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_fault_dicom_non_uniform_slice_spacing():
    """Verify volume reconstruction detects duplicate or overlapping slices."""
    tmp = tempfile.mkdtemp(prefix="test_fault_spacing_")
    try:
        DICOMLoader.generate_synthetic_series(output_dir=tmp, num_slices=6)
        slices = DICOMLoader.load_series(tmp)
        # Duplicate position between slices 2 and 3
        slices[3].image_position = slices[2].image_position

        with pytest.raises(ValueError, match="Duplicate or overlapping slice positions detected"):
            MedicalVolume.from_slices(slices)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_fault_empty_segmentation_mask_and_anatomical_target():
    """Verify segmentation failure produces no target and sets valid=False."""
    tmp = tempfile.mkdtemp(prefix="test_fault_seg_")
    try:
        DICOMLoader.generate_synthetic_series(output_dir=tmp, num_slices=6)
        slices = DICOMLoader.load_series(tmp)
        vol = MedicalVolume.from_slices(slices)

        # Threshold window far above synthetic values (HU > 5000)
        seg_res = MedicalSegmentation.segment_target(vol, hu_min=5000.0, hu_max=6000.0)
        assert not seg_res.success
        assert seg_res.voxel_count == 0

        target = AnatomicalTargetExtractor.extract_from_segmentation(vol, seg_res)
        assert not target.valid
        assert "Segmentation failed" in target.status_message or "Empty mask" in target.status_message
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ==============================================================================
# SECTION 2: SAFETY SUBSYSTEM FAULTS
# ==============================================================================

def test_fault_out_of_workspace_targets():
    """Verify safety validator catches and logs all 6 directional workspace violations."""
    bounds = WorkspaceBounds(min_x=-0.30, max_x=0.30, min_y=0.10, max_y=0.60, min_z=0.00, max_z=0.40)
    validator = SafetyValidator(bounds=bounds)

    violations = [
        (-0.35, 0.35, 0.20, "X (-0.3500m) below minimum"),
        (0.35, 0.35, 0.20, "X (0.3500m) above maximum"),
        (0.00, 0.05, 0.20, "Y (0.0500m) below minimum"),
        (0.00, 0.65, 0.20, "Y (0.6500m) above maximum"),
        (0.00, 0.35, -0.05, "Z (-0.0500m) below minimum"),
        (0.00, 0.35, 0.45, "Z (0.4500m) above maximum"),
    ]

    for x, y, z, expected_substr in violations:
        res = validator.validate_target(x, y, z)
        assert not res.is_valid
        assert "WORKSPACE_VIOLATION" in res.reason
        assert expected_substr in res.reason

    assert validator.total_rejected == len(violations)
    assert validator.total_accepted == 0


def test_fault_nan_and_infinity_injection():
    """Verify safety validator fails closed when inputs contain NaN or +/- Inf."""
    validator = SafetyValidator()

    bad_inputs = [
        (float("nan"), 0.35, 0.20),
        (0.0, float("nan"), 0.20),
        (0.0, 0.35, float("nan")),
        (float("inf"), 0.35, 0.20),
        (-float("inf"), 0.35, 0.20),
        (0.0, float("inf"), 0.20),
        (0.0, 0.35, -float("inf")),
    ]

    for x, y, z in bad_inputs:
        res = validator.validate_target(x, y, z)
        assert not res.is_valid
        assert "INPUT_VIOLATION" in res.reason


def test_fault_velocity_limit_exceeded():
    """Verify safety validator catches Cartesian step exceeding 100 mm/s limit."""
    validator = SafetyValidator(max_velocity_mps=0.100)

    # First point establishes baseline
    t0 = 1000.0
    r1 = validator.validate_target(0.0, 0.30, 0.20, stamp_sec=t0)
    assert r1.is_valid

    # Move 0.15 m (150 mm) in 0.5 s -> 300 mm/s (violates 100 mm/s limit)
    t1 = t0 + 0.5
    r2 = validator.validate_target(0.0, 0.45, 0.20, stamp_sec=t1)
    assert not r2.is_valid
    assert "VELOCITY_VIOLATION" in r2.reason
    assert "exceeds limit" in r2.reason


def test_fault_estop_state_machine_and_recovery():
    """Verify E-stop arrests motion, rejects all new commands, and enforces explicit reset."""
    validator = SafetyValidator()

    # Normal target accepted
    assert validator.validate_target(0.0, 0.35, 0.20).is_valid
    assert validator.state == EStopState.NORMAL

    # Trigger E-stop
    success, msg = validator.trigger_estop("Test fault trigger")
    assert success
    assert validator.state == EStopState.STOPPED
    assert validator.estop_activations == 1

    # In STOPPED state, any command must be rejected
    r_blocked = validator.validate_target(0.0, 0.35, 0.20)
    assert not r_blocked.is_valid
    assert "ESTOP_ACTIVE" in r_blocked.reason

    # Perform explicit reset
    ok_reset, reset_msg = validator.reset_estop()
    assert ok_reset
    assert validator.state == EStopState.NORMAL

    # Now redundant reset should fail cleanly
    bad_reset, bad_msg = validator.reset_estop()
    assert not bad_reset
    assert "RESET_REJECTED" in bad_msg

    # After valid reset, normal commands accepted again
    assert validator.validate_target(0.0, 0.35, 0.20).is_valid


# ==============================================================================
# SECTION 3: EMBEDDED & COMMUNICATION FAULTS
# ==============================================================================

def test_fault_embedded_crc_mismatch():
    """Verify protocol codec detects and rejects packets with corrupted CRC."""
    payload = ProtocolCodec.encode_joint_commands([0.0, 0.1, -0.2, 0.3, 0.0, 0.0])
    raw_packet = bytearray(ProtocolCodec.encode(MsgType.CMD_JOINTS, 1, payload))

    # Corrupt CRC byte
    raw_packet[-3] ^= 0xFF

    decoded = ProtocolCodec.decode(bytes(raw_packet))
    assert not decoded.valid
    assert "CRC failure" in decoded.error_message


def test_fault_embedded_truncated_and_framing_error():
    """Verify protocol codec rejects truncated packets or invalid header/footer bytes."""
    # Truncated packet
    short_packet = bytes([0xAA, 0x55, 0x01, 0x01])
    decoded_short = ProtocolCodec.decode(short_packet)
    assert not decoded_short.valid
    assert "too short" in decoded_short.error_message

    # Invalid start bytes
    bad_header = bytes([0x12, 0x34, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0D, 0x0A])
    decoded_header = ProtocolCodec.decode(bad_header)
    assert not decoded_header.valid
    assert "Invalid start of frame" in decoded_header.error_message

    # Invalid end bytes
    bad_footer = bytes([0xAA, 0x55, 0x01, 0x01, 0x00, 0x00, 0x00, 0xFF, 0xFF])
    decoded_footer = ProtocolCodec.decode(bad_footer)
    assert not decoded_footer.valid
    assert "Invalid end of frame" in decoded_footer.error_message


def test_fault_mcu_simulator_fault_injection():
    """Verify simulated microcontroller handles dropped packets, corrupt CRC, and disconnects."""
    mcu = MicrocontrollerSimulator()

    payload = ProtocolCodec.encode_joint_commands([0.1, 0.2, 0.3, 0.0, 0.0, 0.0])
    pkt = ProtocolCodec.encode(MsgType.CMD_JOINTS, 1, payload)

    # 1. Normal packet succeeds
    assert mcu.receive_packet(pkt)
    assert mcu.state == HardwareState.MOVING

    # 2. Injected packet drop: packet is swallowed, joint target unchanged
    mcu.fault_drop_packets = True
    new_payload = ProtocolCodec.encode_joint_commands([0.5, 0.5, 0.5, 0.0, 0.0, 0.0])
    drop_pkt = ProtocolCodec.encode(MsgType.CMD_JOINTS, 2, new_payload)
    mcu.receive_packet(drop_pkt)
    assert not np.allclose(mcu.q_target, [0.5, 0.5, 0.5, 0.0, 0.0, 0.0])
    mcu.fault_drop_packets = False

    # 3. Disconnected fault
    mcu.fault_disconnected = True
    assert not mcu.receive_packet(pkt)
    mcu.fault_disconnected = False

    # 4. Injected E-stop command halts motion and arrests velocity
    estop_pkt = ProtocolCodec.encode(MsgType.CMD_ESTOP, 3, bytes([1]))
    mcu.receive_packet(estop_pkt)
    assert mcu.state == HardwareState.ESTOPPED
    assert mcu.is_estopped
    assert np.all(mcu.dq_actual == 0.0)


# ==============================================================================
# SECTION 4: ROS 2 CLOSED-LOOP INTEGRATION FAULTS
# ==============================================================================

def test_fault_ros2_estop_during_closed_loop_motion(ros_context):
    """
    Verify that an asynchronous E-stop published during robot trajectory execution
    causes the robot controller to abort trajectory, brake joints, and cease motion.
    """
    safety_node = SurgiTwinSafetyController()
    robot_node = SurgiTwinRobotSimNode()

    executor = SingleThreadedExecutor()
    executor.add_node(safety_node)
    executor.add_node(robot_node)

    try:
        # Pre-position robot safely
        init_res = robot_node.ik_solver.solve_position(np.array([0.0, 0.30, 0.20]))
        assert init_res.success
        robot_node.plant.reset_state(init_res.q)

        # Send valid target to start motion
        target_msg = PoseStamped()
        target_msg.header.stamp = safety_node.get_clock().now().to_msg()
        target_msg.header.frame_id = "base_link"
        target_msg.pose.position = Point(x=0.0, y=0.45, z=0.20)
        target_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        target_pub = safety_node.create_publisher(PoseStamped, "/target_pose", 10)
        target_pub.publish(target_msg)

        # Spin until robot starts executing trajectory (STATE_EXECUTING)
        start_t = time.time()
        while robot_node.robot_state != SurgiTwinRobotSimNode.STATE_EXECUTING and (time.time() - start_t) < 3.0:
            executor.spin_once(timeout_sec=0.03)

        assert robot_node.robot_state == SurgiTwinRobotSimNode.STATE_EXECUTING

        # Inject Emergency Stop trigger via topic
        estop_msg = Bool()
        estop_msg.data = True
        estop_pub = safety_node.create_publisher(Bool, "/safety/emergency_stop", 10)
        estop_pub.publish(estop_msg)

        # Spin to process E-stop
        estop_t = time.time()
        while robot_node.robot_state != SurgiTwinRobotSimNode.STATE_ESTOPPED and (time.time() - estop_t) < 3.0:
            executor.spin_once(timeout_sec=0.03)

        # Assert robot immediately halted
        assert robot_node.robot_state == SurgiTwinRobotSimNode.STATE_ESTOPPED
        assert robot_node.current_trajectory is None
        assert np.max(np.abs(robot_node.plant.dq)) < 1e-3

    finally:
        executor.shutdown()
        safety_node.destroy_node()
        robot_node.destroy_node()


def test_fault_ros2_rejection_of_out_of_bounds_target(ros_context):
    """
    Verify that publishing an out-of-bounds target to /target_pose in the live ROS 2 graph
    is caught by the safety supervisor, rejected without forwarding to /safe_target_pose,
    and robot state remains IDLE.
    """
    safety_node = SurgiTwinSafetyController()
    robot_node = SurgiTwinRobotSimNode()

    executor = SingleThreadedExecutor()
    executor.add_node(safety_node)
    executor.add_node(robot_node)

    try:
        initial_rej_count = safety_node.validator.total_rejected
        initial_acc_count = safety_node.validator.total_accepted

        # Send target clearly out of bounds (Z = 0.85 m, limit is 0.40 m)
        target_msg = PoseStamped()
        target_msg.header.stamp = safety_node.get_clock().now().to_msg()
        target_msg.header.frame_id = "base_link"
        target_msg.pose.position = Point(x=0.0, y=0.35, z=0.85)
        target_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        test_pub = safety_node.create_publisher(PoseStamped, "/target_pose", 10)
        test_pub.publish(target_msg)

        for _ in range(15):
            executor.spin_once(timeout_sec=0.02)

        # Assert rejection count incremented, accepted count unchanged, robot IDLE
        assert safety_node.validator.total_rejected == initial_rej_count + 1
        assert safety_node.validator.total_accepted == initial_acc_count
        assert robot_node.robot_state == SurgiTwinRobotSimNode.STATE_IDLE

    finally:
        executor.shutdown()
        safety_node.destroy_node()
        robot_node.destroy_node()
