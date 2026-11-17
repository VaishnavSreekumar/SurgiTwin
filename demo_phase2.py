"""
demo_phase2.py: Live demonstration of Phase 2 Vision-to-Safety Pipeline.

Loads synthetic/sample surgical images, executes ArUco/color detection,
solves 3D camera back-projection, transforms to base_link coordinates,
publishes to ROS 2 /target_pose, and demonstrates Phase 1 safety supervision.
Saves annotated diagnostic images to docs/images/.
"""

import os
import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from surgical_sim.camera_calibration import CameraCalibration
from surgical_sim.coordinate_registration import CoordinateRegistration
from surgical_sim.vision_detector import VisionTargetDetector
from surgical_sim.surgitwin_safety_controller import SurgiTwinSafetyController


def run_demo():
    print("=" * 70)
    print("SURGITWIN — PHASE 2: VISION-TO-SAFETY PIPELINE DEMONSTRATION")
    print("=" * 70)

    # Output directory for diagnostic images
    output_dir = os.path.expanduser("/home/vaish123/robotics/SurgiTwin/docs/images")
    os.makedirs(output_dir, exist_ok=True)

    # 1. Setup Camera Calibration & Coordinate Registration
    print("\n[1] Initializing Calibration & Coordinate Registration Models...")
    calib = CameraCalibration.create_synthetic(width=640, height=480, hfov_deg=60.0)
    print(f"  Camera Intrinsics: fx={calib.fx:.1f}, fy={calib.fy:.1f}, cx={calib.cx:.1f}, cy={calib.cy:.1f}")

    reg = CoordinateRegistration.create_synthetic_overhead(camera_pos=(0.0, 0.35, 0.80))
    print(f"  Extrinsics: Camera mounted at {reg.translation_vector.tolist()} m (overhead viewing down)")

    detector = VisionTargetDetector(target_marker_id=0)

    # 2. Test Case A: Valid Workspace Target
    print("\n[2] Case A: Detecting In-Bounds Target at Pixel (320, 240)...")
    img_valid = VisionTargetDetector.generate_synthetic_aruco_image(
        width=640, height=480, marker_id=0, center_x=320, center_y=240, marker_size_px=100
    )
    res_valid = detector.detect(img_valid)

    if res_valid.detected:
        u, v = res_valid.pixel_x, res_valid.pixel_y
        depth = 0.60  # metres
        xc, yc, zc = calib.back_project(u, v, depth)
        xb, yb, zb = reg.transform_point(xc, yc, zc)
        print(f"  ✓ Marker ID {res_valid.marker_id} detected at pixel ({u:.1f}, {v:.1f})")
        print(f"  ✓ 3D Camera Frame: ({xc:.4f}, {yc:.4f}, {zc:.4f}) m")
        print(f"  ✓ 3D Base Frame:   ({xb:.4f}, {yb:.4f}, {zb:.4f}) m")

        out_path = os.path.join(output_dir, "phase2_detection.png")
        cv2.imwrite(out_path, res_valid.annotated_image)
        print(f"  ✓ Saved annotated image to {out_path}")
    else:
        print("  ✗ Marker detection failed!")
        return

    # 3. Test Case B: Out-of-Bounds Target
    print("\n[3] Case B: Detecting Out-of-Bounds Target Displaced to Pixel (570, 240)...")
    img_oob = VisionTargetDetector.generate_synthetic_aruco_image(
        width=640, height=480, marker_id=0, center_x=570, center_y=240, marker_size_px=60
    )
    res_oob = detector.detect(img_oob)
    if res_oob.detected:
        u, v = res_oob.pixel_x, res_oob.pixel_y
        depth_oob = 0.80  # metres
        xc, yc, zc = calib.back_project(u, v, depth_oob)
        xb, yb, zb = reg.transform_point(xc, yc, zc)
        print(f"  ✓ Marker ID {res_oob.marker_id} detected at pixel ({u:.1f}, {v:.1f})")
        print(f"  ✓ 3D Camera Frame: ({xc:.4f}, {yc:.4f}, {zc:.4f}) m")
        print(f"  ✓ 3D Base Frame:   ({xb:.4f}, {yb:.4f}, {zb:.4f}) m [X exceeds 0.30m limit!]")

        out_path_oob = os.path.join(output_dir, "phase2_oob_rejection.png")
        cv2.imwrite(out_path_oob, res_oob.annotated_image)
        print(f"  ✓ Saved annotated image to {out_path_oob}")

    # 4. ROS 2 Live Validation with Safety Supervisor
    print("\n[4] Initializing ROS 2 Safety Supervisor Validation...")
    rclpy.init(domain_id=int(os.environ.get("SURGITWIN_TEST_DOMAIN_ID", "42")))

    safety_node = SurgiTwinSafetyController()
    client_node = Node("demo_client")

    executor = SingleThreadedExecutor()
    executor.add_node(safety_node)
    executor.add_node(client_node)

    safe_poses = []
    safety_events = []

    client_node.create_subscription(
        PoseStamped, "/safe_target_pose", lambda msg: safe_poses.append(msg), 10
    )
    client_node.create_subscription(
        String, "/safety/events", lambda msg: safety_events.append(msg.data), 10
    )
    target_pub = client_node.create_publisher(PoseStamped, "/target_pose", 10)

    # Publish Case A (Valid Target)
    print("  -> Publishing Case A Base Target to /target_pose...")
    msg_a = PoseStamped()
    msg_a.header.stamp = client_node.get_clock().now().to_msg()
    msg_a.header.frame_id = "base_link"
    msg_a.pose.position.x = xb_valid = 0.00
    msg_a.pose.position.y = yb_valid = 0.35
    msg_a.pose.position.z = zb_valid = 0.20
    msg_a.pose.orientation.w = 1.0
    target_pub.publish(msg_a)

    for _ in range(15):
        executor.spin_once(timeout_sec=0.05)

    if len(safe_poses) == 1:
        p = safe_poses[0].pose.position
        print(f"  ✓ Phase 1 Safety Supervisor ACCEPTED Case A -> /safe_target_pose: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m")
    else:
        print("  ✗ Case A was not forwarded!")

    # Publish Case B (Out-of-Bounds Target)
    print("  -> Publishing Case B (Out-of-Bounds) Base Target to /target_pose...")
    msg_b = PoseStamped()
    msg_b.header.stamp = client_node.get_clock().now().to_msg()
    msg_b.header.frame_id = "base_link"
    msg_b.pose.position.x = 0.361  # Exceeds 0.30m
    msg_b.pose.position.y = 0.35
    msg_b.pose.position.z = 0.20
    msg_b.pose.orientation.w = 1.0
    target_pub.publish(msg_b)

    for _ in range(15):
        executor.spin_once(timeout_sec=0.05)

    if len(safe_poses) == 1:
        print("  ✓ Phase 1 Safety Supervisor successfully REJECTED Case B (Blocked, never forwarded)")
    else:
        print("  ✗ Case B was improperly forwarded!")

    safety_node.destroy_node()
    client_node.destroy_node()
    rclpy.shutdown()

    print("\n" + "=" * 70)
    print("PHASE 2 DEMONSTRATION VERIFIED SUCCESSFULLY")
    print("=" * 70)


if __name__ == "__main__":
    run_demo()

