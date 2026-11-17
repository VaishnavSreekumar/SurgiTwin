"""
test_robot_safety_integration.py
Full integration tests connecting Phase 1 Safety Subsystem and Phase 3 Robot Simulation.
"""

import time
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Bool, String

from surgical_sim.surgitwin_safety_controller import SurgiTwinSafetyController
from surgical_sim.robot_sim_node import SurgiTwinRobotSimNode


TEST_DOMAIN_ID = 45


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init(domain_id=TEST_DOMAIN_ID)
    yield
    rclpy.shutdown()


def test_robot_safety_closed_loop_execution(ros_context):
    """
    Test closed-loop integration:
    1. Valid target published to /target_pose.
    2. Phase 1 safety supervisor approves and forwards to /safe_target_pose.
    3. Phase 3 robot simulation receives safe pose, plans trajectory, and executes motion.
    4. End-effector arrives at target position within tolerance.
    """
    safety_node = SurgiTwinSafetyController()
    robot_node = SurgiTwinRobotSimNode()

    # Pre-position robot in surgical workspace at (0.0, 0.32, 0.20)
    init_res = robot_node.ik_solver.solve_position(np.array([0.0, 0.32, 0.20]))
    assert init_res.success
    robot_node.plant.reset_state(init_res.q)

    executor = SingleThreadedExecutor()
    executor.add_node(safety_node)
    executor.add_node(robot_node)

    # Test publisher for /target_pose
    pub_node = rclpy.create_node("test_target_publisher")
    target_pub = pub_node.create_publisher(PoseStamped, "/target_pose", 10)
    executor.add_node(pub_node)

    # Publish target in center of surgical workspace
    target_msg = PoseStamped()
    target_msg.header.stamp = pub_node.get_clock().now().to_msg()
    target_msg.header.frame_id = "base_link"
    target_msg.pose.position = Point(x=0.0, y=0.35, z=0.20)
    target_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    target_pub.publish(target_msg)

    # Spin until robot completes trajectory or timeout
    start_time = time.time()
    max_duration = 10.0  # seconds
    while robot_node.robot_state != SurgiTwinRobotSimNode.STATE_COMPLETED and (time.time() - start_time) < max_duration:
        executor.spin_once(timeout_sec=0.03)

    assert robot_node.robot_state == SurgiTwinRobotSimNode.STATE_COMPLETED, (
        f"Robot did not complete trajectory. State: {robot_node.robot_state}"
    )

    # Verify tool position is within 1.0 mm of target
    _, p_actual, _ = robot_node.kinematics.forward_kinematics(robot_node.plant.q)
    target_p = np.array([0.0, 0.35, 0.20])
    error_mm = np.linalg.norm(target_p - p_actual) * 1000.0
    assert error_mm < 1.0, f"Final tool positioning error {error_mm:.3f} mm exceeds 1.0 mm"

    executor.shutdown()
    pub_node.destroy_node()
    safety_node.destroy_node()
    robot_node.destroy_node()


def test_robot_rejects_out_of_bounds_target(ros_context):
    """
    Verify out-of-bounds target is blocked by Phase 1 safety node
    and never reaches Phase 3 robot simulation node.
    """
    safety_node = SurgiTwinSafetyController()
    robot_node = SurgiTwinRobotSimNode()

    # Pre-position robot in surgical workspace at (0.0, 0.32, 0.20)
    init_res = robot_node.ik_solver.solve_position(np.array([0.0, 0.32, 0.20]))
    assert init_res.success
    robot_node.plant.reset_state(init_res.q)

    executor = SingleThreadedExecutor()
    executor.add_node(safety_node)
    executor.add_node(robot_node)

    pub_node = rclpy.create_node("test_oob_publisher")
    target_pub = pub_node.create_publisher(PoseStamped, "/target_pose", 10)
    executor.add_node(pub_node)

    # Out-of-bounds target (x=0.8 m exceeds [ -0.3, 0.3 ])
    oob_msg = PoseStamped()
    oob_msg.header.stamp = pub_node.get_clock().now().to_msg()
    oob_msg.header.frame_id = "base_link"
    oob_msg.pose.position = Point(x=0.80, y=0.35, z=0.20)
    oob_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    target_pub.publish(oob_msg)

    # Spin for 0.5s
    start_time = time.time()
    while time.time() - start_time < 0.5:
        executor.spin_once(timeout_sec=0.03)

    # Robot must still be IDLE (never received command)
    assert robot_node.robot_state == SurgiTwinRobotSimNode.STATE_IDLE
    assert robot_node.current_trajectory is None

    executor.shutdown()
    pub_node.destroy_node()
    safety_node.destroy_node()
    robot_node.destroy_node()


def test_robot_estop_halts_motion(ros_context):
    """
    Verify that asserting E-stop immediately halts an executing robot trajectory.
    """
    safety_node = SurgiTwinSafetyController()
    robot_node = SurgiTwinRobotSimNode()

    # Pre-position robot in surgical workspace at (0.0, 0.32, 0.20)
    init_res = robot_node.ik_solver.solve_position(np.array([0.0, 0.32, 0.20]))
    assert init_res.success
    robot_node.plant.reset_state(init_res.q)

    executor = SingleThreadedExecutor()
    executor.add_node(safety_node)
    executor.add_node(robot_node)

    pub_node = rclpy.create_node("test_estop_publisher")
    target_pub = pub_node.create_publisher(PoseStamped, "/target_pose", 10)
    estop_pub = pub_node.create_publisher(Bool, "/safety/emergency_stop", 10)
    executor.add_node(pub_node)

    # Start motion to a reachable target
    target_msg = PoseStamped()
    target_msg.header.stamp = pub_node.get_clock().now().to_msg()
    target_msg.header.frame_id = "base_link"
    target_msg.pose.position = Point(x=0.10, y=0.35, z=0.20)
    target_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    target_pub.publish(target_msg)

    # Wait until EXECUTING state
    start_time = time.time()
    while robot_node.robot_state != SurgiTwinRobotSimNode.STATE_EXECUTING and time.time() - start_time < 3.0:
        executor.spin_once(timeout_sec=0.03)

    assert robot_node.robot_state == SurgiTwinRobotSimNode.STATE_EXECUTING

    # Assert E-stop!
    estop_msg = Bool()
    estop_msg.data = True
    estop_pub.publish(estop_msg)

    # Spin to process E-stop
    start_time = time.time()
    while robot_node.robot_state != SurgiTwinRobotSimNode.STATE_ESTOPPED and time.time() - start_time < 2.0:
        executor.spin_once(timeout_sec=0.03)

    assert robot_node.robot_state == SurgiTwinRobotSimNode.STATE_ESTOPPED
    assert robot_node.current_trajectory is None

    executor.shutdown()
    pub_node.destroy_node()
    safety_node.destroy_node()
    robot_node.destroy_node()
