"""
test_safety_integration.py.

Live ROS 2 node integration test for SurgiTwinSafetyController.
Spawns the safety supervisor node in a ROS 2 test executor, publishes
targets, triggers E-Stop, and validates that only safe targets reach /safe_target_pose.

Uses isolated ROS domain ID and deterministic event-driven synchronization to prevent
timing jitter and multi-process DDS cross-talk.
"""

import json
import os
import time
from typing import List

from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from surgical_sim.safety_validator import EStopState
from surgical_sim.surgitwin_safety_controller import SurgiTwinSafetyController


@pytest.fixture(scope="module")
def ros_context():
    """Manage rclpy lifecycle with isolated domain ID for the test module."""
    # Isolate test on domain 42 unless specified, preventing cross-talk with host nodes
    test_domain = int(os.environ.get("SURGITWIN_TEST_DOMAIN_ID", "42"))
    rclpy.init(domain_id=test_domain)
    yield
    rclpy.shutdown()


def test_safety_node_live_forwarding_and_rejection(ros_context) -> None:
    """
    Scenario 17, 18, 19: Full ROS 2 node communication test.

    Verifies:
    1. Valid PoseStamped on /target_pose is published to /safe_target_pose.
    2. Out-of-bounds PoseStamped is rejected and NOT published.
    3. E-Stop stops all publications.
    4. Reset restores safe forwarding.
    5. Node handles multiple invalid commands without crashing.
    """
    safety_node = SurgiTwinSafetyController()
    test_client = rclpy.create_node("safety_test_client")

    executor = SingleThreadedExecutor()
    executor.add_node(safety_node)
    executor.add_node(test_client)

    received_safe_poses: List[PoseStamped] = []
    received_events: List[str] = []

    def safe_pose_cb(msg: PoseStamped):
        received_safe_poses.append(msg)

    def event_cb(msg: String):
        received_events.append(msg.data)

    test_client.create_subscription(PoseStamped, "/safe_target_pose", safe_pose_cb, 10)
    test_client.create_subscription(String, "/safety/events", event_cb, 10)
    target_pub = test_client.create_publisher(PoseStamped, "/target_pose", 10)
    estop_pub = test_client.create_publisher(Bool, "/safety/emergency_stop", 10)

    def build_pose(x: float, y: float, z: float, frame: str = "base_link") -> PoseStamped:
        p = PoseStamped()
        p.header.stamp = test_client.get_clock().now().to_msg()
        p.header.frame_id = frame
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        p.pose.orientation.w = 1.0
        return p

    def spin_until_safe_pose(expected_count: int, timeout_sec: float = 3.0) -> bool:
        start = time.time()
        while len(received_safe_poses) < expected_count and (time.time() - start) < timeout_sec:
            executor.spin_once(timeout_sec=0.05)
        # Extra settling spin to verify no spurious duplicate publications
        extra = time.time()
        while (time.time() - extra) < 0.15:
            executor.spin_once(timeout_sec=0.02)
        return len(received_safe_poses) == expected_count

    def spin_until_rejection(expected_rejections: int, timeout_sec: float = 3.0) -> bool:
        start = time.time()
        while safety_node.validator.total_rejected < expected_rejections and (time.time() - start) < timeout_sec:
            executor.spin_once(timeout_sec=0.05)
        return safety_node.validator.total_rejected >= expected_rejections

    def spin_until_state(expected_state: EStopState, timeout_sec: float = 3.0) -> bool:
        start = time.time()
        while safety_node.validator.state != expected_state and (time.time() - start) < timeout_sec:
            executor.spin_once(timeout_sec=0.05)
        return safety_node.validator.state == expected_state

    # --- Step 1: Valid Target ---
    valid_pose_1 = build_pose(0.0, 0.35, 0.20)
    target_pub.publish(valid_pose_1)
    assert spin_until_safe_pose(1), "Valid target should be forwarded to /safe_target_pose"
    assert len(received_safe_poses) == 1
    assert pytest.approx(received_safe_poses[0].pose.position.x) == 0.0
    assert pytest.approx(received_safe_poses[0].pose.position.y) == 0.35

    # --- Step 2: Out of Bounds Target ---
    prev_rej = safety_node.validator.total_rejected
    oob_pose = build_pose(0.80, 0.35, 0.20)  # X=0.80 exceeds 0.30 max
    target_pub.publish(oob_pose)
    assert spin_until_rejection(prev_rej + 1), "Out-of-bounds target must trigger rejection"
    assert len(received_safe_poses) == 1, "Out of bounds target must NOT be forwarded"

    # --- Step 3: Trigger E-Stop via Topic ---
    estop_msg = Bool()
    estop_msg.data = True
    estop_pub.publish(estop_msg)
    assert spin_until_state(EStopState.STOPPED), "E-Stop state machine must transition to STOPPED"

    # --- Step 4: Valid target while E-Stop is active ---
    prev_rej = safety_node.validator.total_rejected
    valid_during_stop = build_pose(0.02, 0.35, 0.20)
    target_pub.publish(valid_during_stop)
    assert spin_until_rejection(prev_rej + 1), "Commands during E-Stop must be rejected"
    assert len(received_safe_poses) == 1, "Commands must be blocked while E-Stop is active"

    # --- Step 5: Reset E-Stop via Service ---
    reset_client = test_client.create_client(Trigger, "/safety/reset")
    assert reset_client.wait_for_service(timeout_sec=2.0)

    req = Trigger.Request()
    future = reset_client.call_async(req)
    while not future.done():
        executor.spin_once(timeout_sec=0.05)

    assert future.result().success is True
    assert safety_node.validator.state == EStopState.NORMAL

    # --- Step 6: Valid target after Reset ---
    valid_pose_2 = build_pose(0.05, 0.35, 0.20)
    target_pub.publish(valid_pose_2)
    assert spin_until_safe_pose(2), "New valid target after reset must be forwarded exactly once"
    assert len(received_safe_poses) == 2, "Received count must be exactly 2"
    assert pytest.approx(received_safe_poses[1].pose.position.x) == 0.05

    # --- Step 7: Sequential Malformed Input Burst (Does not crash node) ---
    prev_rej = safety_node.validator.total_rejected
    for _ in range(5):
        nan_pose = build_pose(float("nan"), 0.35, 0.20)
        target_pub.publish(nan_pose)
        executor.spin_once(timeout_sec=0.02)
        frame_pose = build_pose(0.0, 0.35, 0.20, frame="unknown_frame")
        target_pub.publish(frame_pose)
        executor.spin_once(timeout_sec=0.02)

    assert spin_until_rejection(prev_rej + 10), "Malformed inputs must be rejected"
    assert len(received_safe_poses) == 2, "Malformed inputs must never reach safe pose topic"
    assert safety_node.validator.state == EStopState.NORMAL

    # Clean teardown
    safety_node.destroy_node()
    test_client.destroy_node()
