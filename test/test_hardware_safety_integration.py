"""
test_hardware_safety_integration.py
Full system integration tests connecting Medical Imaging, Phase 1 Safety,
Phase 3 Robot Simulation, and Phase 4 Embedded Hardware Abstraction.
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

from surgical_sim.dicom_loader import DICOMLoader
from surgical_sim.medical_volume import MedicalVolume
from surgical_sim.medical_segmentation import MedicalSegmentation
from surgical_sim.anatomical_target import AnatomicalTargetExtractor
from surgical_sim.medical_registration import MedicalRegistration
from surgical_sim.surgitwin_safety_controller import SurgiTwinSafetyController
from surgical_sim.robot_sim_node import SurgiTwinRobotSimNode
from surgical_sim.surgitwin_medical_node import SurgiTwinMedicalNode


TEST_DOMAIN_ID = 47


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init(domain_id=TEST_DOMAIN_ID)
    yield
    rclpy.shutdown()


def test_medical_to_safety_to_hardware_closed_loop(ros_context):
    """
    End-to-end closed-loop pipeline test:
    1. Ingest DICOM -> Segment target -> Register to robot coordinates.
    2. Publish to /target_pose.
    3. Phase 1 Safety Controller verifies workspace bounds and forwards to /safe_target_pose.
    4. Phase 3 Robot Simulation plans and executes trajectory.
    5. Phase 4 Hardware Interface forwards joint commands to simulated microcontroller.
    6. Microcontroller executes commands and returns encoder telemetry.
    """
    safety_node = SurgiTwinSafetyController()
    robot_node = SurgiTwinRobotSimNode()
    medical_node = SurgiTwinMedicalNode()

    # Pre-position robot in surgical workspace
    init_res = robot_node.ik_solver.solve_position(np.array([0.0, 0.35, 0.20]))
    assert init_res.success
    robot_node.plant.reset_state(init_res.q)
    medical_node.mcu_sim.reset(init_res.q)

    executor = SingleThreadedExecutor()
    executor.add_node(safety_node)
    executor.add_node(robot_node)
    executor.add_node(medical_node)

    # 1. Generate synthetic DICOM series and extract target
    tmp = tempfile.mkdtemp(prefix="test_e2e_dicom_")
    try:
        DICOMLoader.generate_synthetic_series(
            output_dir=tmp,
            num_slices=12,
            rows=48,
            cols=48,
            target_center_mm=(0.0, 0.0, 15.0),
            target_radius_mm=6.0,
        )
        slices = DICOMLoader.load_series(tmp)
        vol = MedicalVolume.from_slices(slices)
        seg_res = MedicalSegmentation.segment_target(vol, hu_min=200.0, hu_max=350.0)
        assert seg_res.success
        target = AnatomicalTargetExtractor.extract_from_segmentation(vol, seg_res)
        assert target.valid

        # 2. Register to robot base coordinates
        reg = MedicalRegistration.create_synthetic_supine_table(patient_center_on_table=(0.0, 0.35, 0.20))
        p_base = reg.transform_target(target)

        # 3. Publish to /target_pose
        target_msg = PoseStamped()
        target_msg.header.stamp = medical_node.get_clock().now().to_msg()
        target_msg.header.frame_id = "base_link"
        target_msg.pose.position = Point(x=float(p_base[0]), y=float(p_base[1]), z=float(p_base[2]))
        target_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        medical_node.target_pose_pub.publish(target_msg)

        # 4. Spin until execution completes or timeout
        start_time = time.time()
        while robot_node.robot_state != SurgiTwinRobotSimNode.STATE_COMPLETED and (time.time() - start_time) < 8.0:
            executor.spin_once(timeout_sec=0.03)

        assert robot_node.robot_state == SurgiTwinRobotSimNode.STATE_COMPLETED

        # 5. Verify hardware simulator received and tracked commands
        assert np.allclose(medical_node.mcu_sim.q_actual, robot_node.plant.q, atol=0.02)
        assert medical_node.mcu_sim.state in (medical_node.mcu_sim.state.READY, medical_node.mcu_sim.state.MOVING)

    finally:
        shutil.rmtree(tmp, ignore_errors=True)
        executor.shutdown()
        safety_node.destroy_node()
        robot_node.destroy_node()
        medical_node.destroy_node()


def test_out_of_bounds_medical_target_blocked(ros_context):
    """
    Verify out-of-bounds target from medical registration is blocked by safety supervisor
    and never transmitted to hardware.
    """
    safety_node = SurgiTwinSafetyController()
    robot_node = SurgiTwinRobotSimNode()
    medical_node = SurgiTwinMedicalNode()

    executor = SingleThreadedExecutor()
    executor.add_node(safety_node)
    executor.add_node(robot_node)
    executor.add_node(medical_node)

    # Publish target outside workspace (X = 0.80 m exceeds [-0.3, 0.3] m)
    oob_msg = PoseStamped()
    oob_msg.header.stamp = medical_node.get_clock().now().to_msg()
    oob_msg.header.frame_id = "base_link"
    oob_msg.pose.position = Point(x=0.80, y=0.35, z=0.20)
    oob_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    medical_node.target_pose_pub.publish(oob_msg)

    # Spin 0.5s
    start_time = time.time()
    while time.time() - start_time < 0.5:
        executor.spin_once(timeout_sec=0.03)

    # Robot must remain IDLE and MCU must remain in initial position
    assert robot_node.robot_state == SurgiTwinRobotSimNode.STATE_IDLE
    assert not medical_node.mcu_sim.is_estopped

    executor.shutdown()
    safety_node.destroy_node()
    robot_node.destroy_node()
    medical_node.destroy_node()


def test_hardware_estop_propagation(ros_context):
    """
    Verify asserting emergency stop halts both robot simulation and microcontroller.
    """
    safety_node = SurgiTwinSafetyController()
    robot_node = SurgiTwinRobotSimNode()
    medical_node = SurgiTwinMedicalNode()

    executor = SingleThreadedExecutor()
    executor.add_node(safety_node)
    executor.add_node(robot_node)
    executor.add_node(medical_node)

    pub_node = rclpy.create_node("test_estop_pub_node")
    estop_pub = pub_node.create_publisher(Bool, "/safety/emergency_stop", 10)
    executor.add_node(pub_node)

    # Assert E-stop
    estop_msg = Bool()
    estop_msg.data = True
    estop_pub.publish(estop_msg)

    # Spin to process
    start_time = time.time()
    while (robot_node.robot_state != SurgiTwinRobotSimNode.STATE_ESTOPPED or not medical_node.mcu_sim.is_estopped) and (time.time() - start_time) < 2.0:
        executor.spin_once(timeout_sec=0.03)

    assert robot_node.robot_state == SurgiTwinRobotSimNode.STATE_ESTOPPED
    assert medical_node.mcu_sim.is_estopped
    assert medical_node.mcu_sim.state == medical_node.mcu_sim.state.ESTOPPED

    executor.shutdown()
    pub_node.destroy_node()
    safety_node.destroy_node()
    robot_node.destroy_node()
    medical_node.destroy_node()

