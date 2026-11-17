"""
test_robot_simulation.py
Unit and node-level tests for SurgiTwinRobotSimNode.
"""

import time
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from surgical_sim.robot_sim_node import SurgiTwinRobotSimNode


TEST_DOMAIN_ID = 44


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init(domain_id=TEST_DOMAIN_ID)
    yield
    rclpy.shutdown()


def test_robot_sim_node_initialization(ros_context):
    """Verify robot simulation node initializes cleanly with default parameters."""
    node = SurgiTwinRobotSimNode()
    assert node.robot_state == SurgiTwinRobotSimNode.STATE_IDLE
    assert len(node.plant.q) == 6
    node.destroy_node()


def test_robot_sim_publishes_joint_states_and_tool_pose(ros_context):
    """Verify that 50 Hz timer publishes joint_states and tool_pose."""
    node = SurgiTwinRobotSimNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    received_joint_states = []
    received_tool_poses = []

    sub_node = rclpy.create_node("test_sim_sub_node")
    sub_node.create_subscription(
        JointState,
        "/joint_states",
        lambda msg: received_joint_states.append(msg),
        10,
    )
    sub_node.create_subscription(
        PoseStamped,
        "/tool_pose",
        lambda msg: received_tool_poses.append(msg),
        10,
    )
    executor.add_node(sub_node)

    # Spin for 150 ms (should produce ~7 ticks at 50 Hz)
    start_time = time.time()
    while time.time() - start_time < 0.15:
        executor.spin_once(timeout_sec=0.03)

    assert len(received_joint_states) >= 3
    assert len(received_tool_poses) >= 3

    # Check joint names
    assert received_joint_states[0].name == node.kinematics.JOINT_NAMES
    assert len(received_joint_states[0].position) == 6

    executor.shutdown()
    sub_node.destroy_node()
    node.destroy_node()


def test_robot_sim_reset_home_service(ros_context):
    """Verify /robot/reset_home service resets robot configuration."""
    node = SurgiTwinRobotSimNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Offset robot joints
    node.plant.q += 0.2

    client_node = rclpy.create_node("test_client_node")
    client = client_node.create_client(Trigger, "/robot/reset_home")
    executor.add_node(client_node)

    assert client.wait_for_service(timeout_sec=2.0)

    req = Trigger.Request()
    future = client.call_async(req)

    start_time = time.time()
    while not future.done() and time.time() - start_time < 2.0:
        executor.spin_once(timeout_sec=0.05)

    assert future.done()
    res = future.result()
    assert res.success
    assert np.allclose(node.plant.q, node.kinematics.SURGICAL_HOME_Q, atol=1e-5)

    executor.shutdown()
    client_node.destroy_node()
    node.destroy_node()

