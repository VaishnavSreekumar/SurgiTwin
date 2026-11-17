"""
test_vision_integration.py.

Live ROS 2 integration test for VisionTargetNode and SurgiTwinSafetyController.
Verifies the complete end-to-end perception-to-safety pipeline:
  Synthetic Image -> Vision Detection -> 3D Back-Projection -> Coordinate Registration
  -> /target_pose -> Safety Supervisor -> /safe_target_pose

Uses isolated ROS domain ID and deterministic event-driven synchronization.
"""

import os
import time
from typing import List

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image

from surgical_sim.surgitwin_safety_controller import SurgiTwinSafetyController
from surgical_sim.vision_detector import VisionTargetDetector
from surgical_sim.vision_target_node import VisionTargetNode


@pytest.fixture(scope="module")
def ros_context():
    """Manage rclpy lifecycle with isolated domain ID for the vision test module."""
    test_domain = int(os.environ.get("SURGITWIN_TEST_DOMAIN_ID", "42"))
    rclpy.init(domain_id=test_domain)
    yield
    rclpy.shutdown()


def test_end_to_end_vision_to_safety_pipeline(ros_context) -> None:
    """
    Scenario 15, 16, 17: Full end-to-end vision perception to safety validation.

    1. Image with valid marker published to camera/image_raw.
    2. Vision node detects marker, back-projects to 3D, registers to base_link,
       and publishes to /target_pose.
    3. Safety supervisor validates workspace bounds and forwards to /safe_target_pose.
    4. Image without marker is handled cleanly and fails closed (no pose published).
    5. Out-of-bounds visual target is intercepted and rejected by safety supervisor.
    """
    vision_node = VisionTargetNode()
    safety_node = SurgiTwinSafetyController()
    test_client = rclpy.create_node("vision_test_client")

    executor = SingleThreadedExecutor()
    executor.add_node(vision_node)
    executor.add_node(safety_node)
    executor.add_node(test_client)

    bridge = CvBridge()
    received_target_poses: List[PoseStamped] = []
    received_safe_poses: List[PoseStamped] = []

    def target_pose_cb(msg: PoseStamped):
        received_target_poses.append(msg)

    def safe_pose_cb(msg: PoseStamped):
        received_safe_poses.append(msg)

    test_client.create_subscription(PoseStamped, "/target_pose", target_pose_cb, 10)
    test_client.create_subscription(PoseStamped, "/safe_target_pose", safe_pose_cb, 10)
    img_pub = test_client.create_publisher(Image, "camera/image_raw", 10)

    def spin_until_condition(condition_fn, timeout_sec: float = 3.0) -> bool:
        start = time.time()
        while not condition_fn() and (time.time() - start) < timeout_sec:
            executor.spin_once(timeout_sec=0.05)
        extra = time.time()
        while (time.time() - extra) < 0.15:
            executor.spin_once(timeout_sec=0.02)
        return condition_fn()

    # --- Step 1: Publish Image with Marker at Optical Center (320, 240) ---
    # Maps via synthetic overhead camera (Z=0.60m) to robot home (0.0, 0.35, 0.20)m
    img_valid = VisionTargetDetector.generate_synthetic_aruco_image(
        width=640, height=480, marker_id=0, center_x=320, center_y=240, marker_size_px=100
    )
    img_msg = bridge.cv2_to_imgmsg(img_valid, encoding="bgr8")
    img_msg.header.stamp = test_client.get_clock().now().to_msg()
    img_msg.header.frame_id = "camera_optical_frame"
    img_pub.publish(img_msg)

    # Assert target reaches /target_pose and then /safe_target_pose
    assert spin_until_condition(lambda: len(received_safe_poses) == 1), (
        "Vision-detected safe target must be forwarded through Safety Supervisor"
    )

    assert len(received_target_poses) == 1
    assert len(received_safe_poses) == 1
    safe_pose = received_safe_poses[0].pose.position
    assert pytest.approx(safe_pose.x, abs=0.03) == 0.00
    assert pytest.approx(safe_pose.y, abs=0.03) == 0.35
    assert pytest.approx(safe_pose.z, abs=0.03) == 0.20

    # --- Step 2: Publish Image Without Any Target ---
    img_blank = np.ones((480, 640, 3), dtype=np.uint8) * 128
    blank_msg = bridge.cv2_to_imgmsg(img_blank, encoding="bgr8")
    blank_msg.header.stamp = test_client.get_clock().now().to_msg()
    blank_msg.header.frame_id = "camera_optical_frame"
    img_pub.publish(blank_msg)

    # Spin to allow processing
    spin_until_condition(lambda: False, timeout_sec=0.3)
    # Must fail closed: counts remain exactly 1
    assert len(received_target_poses) == 1, "Blank frame must NOT generate /target_pose"
    assert len(received_safe_poses) == 1, "Blank frame must NOT generate /safe_target_pose"

    # --- Step 3: Publish Image with Out-of-Bounds Target ---
    # Set working depth to 0.80m so that marker at center_x=570 px back-projects to
    # X_cam = (570 - 320) * 0.80 / 554.26 = 0.361m > 0.30m (workspace limit X in [-0.3, 0.3])
    vision_node.working_depth = 0.80
    img_oob = VisionTargetDetector.generate_synthetic_aruco_image(
        width=640, height=480, marker_id=0, center_x=570, center_y=240, marker_size_px=60
    )
    oob_msg = bridge.cv2_to_imgmsg(img_oob, encoding="bgr8")
    oob_msg.header.stamp = test_client.get_clock().now().to_msg()
    oob_msg.header.frame_id = "camera_optical_frame"
    img_pub.publish(oob_msg)

    # Wait for vision node to publish to /target_pose
    assert spin_until_condition(lambda: len(received_target_poses) == 2), (
        "Vision node must publish detected coordinates to /target_pose"
    )
    # But Safety Supervisor must intercept and reject: safe pose count stays 1!
    assert len(received_safe_poses) == 1, (
        "Out-of-bounds vision target must be REJECTED by Safety Supervisor"
    )
    assert safety_node.validator.total_rejected >= 1

    # Teardown
    vision_node.destroy_node()
    safety_node.destroy_node()
    test_client.destroy_node()
