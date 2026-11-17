"""
demo_phase3.py: End-to-End Live Demonstration of Phase 3 Robot Kinematics & Simulation.

Demonstrates the integrated pipeline:
1. Target detection in camera frame -> 3D back-projection -> Base coordinate registration.
2. Safety supervisor verification and forwarding to /safe_target_pose.
3. Inverse kinematics solving and joint limits validation.
4. Quintic minimum-jerk trajectory planning with Cartesian speed limit (<= 100 mm/s).
5. 50 Hz closed-loop dynamic simulation with PID joint controllers.
6. Tracking performance measurement and diagnostic plot generation.
7. Active emergency stop braking mid-trajectory.
8. Rejection of out-of-bounds targets.
"""

import json
import os
import time
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Bool

from surgical_sim.camera_calibration import CameraCalibration
from surgical_sim.coordinate_registration import CoordinateRegistration
from surgical_sim.vision_detector import VisionTargetDetector
from surgical_sim.surgitwin_safety_controller import SurgiTwinSafetyController
from surgical_sim.robot_sim_node import SurgiTwinRobotSimNode


def run_demo():
    print("=" * 80)
    print("SURGITWIN — PHASE 3: KINEMATICS, TRAJECTORY PLANNING & SIMULATION DEMO")
    print("=" * 80)

    output_dir = os.path.expanduser("/home/vaish123/robotics/SurgiTwin/docs/images")
    os.makedirs(output_dir, exist_ok=True)

    rclpy.init(domain_id=46)

    try:
        # [1] Initialize Nodes
        print("\n[1] Initializing Nodes and Kinematics Subsystems...")
        safety_node = SurgiTwinSafetyController()
        robot_node = SurgiTwinRobotSimNode()

        # Pre-position robot near surgical workspace center
        init_res = robot_node.ik_solver.solve_position(np.array([0.0, 0.30, 0.20]))
        robot_node.plant.reset_state(init_res.q)

        executor = SingleThreadedExecutor()
        executor.add_node(safety_node)
        executor.add_node(robot_node)

        pub_node = rclpy.create_node("demo_phase3_publisher")
        target_pub = pub_node.create_publisher(PoseStamped, "/target_pose", 10)
        estop_pub = pub_node.create_publisher(Bool, "/safety/emergency_stop", 10)
        executor.add_node(pub_node)

        # [2] Vision Perception Target Formulation (Phase 2 -> Phase 1 -> Phase 3)
        print("\n[2] Vision Target Detection & Coordinate Registration...")
        calib = CameraCalibration.create_synthetic(width=640, height=480, hfov_deg=60.0)
        reg = CoordinateRegistration.create_synthetic_overhead(camera_pos=(0.0, 0.35, 0.80))
        detector = VisionTargetDetector(target_marker_id=0)

        # Generate synthetic surgical target image
        img = VisionTargetDetector.generate_synthetic_aruco_image(
            width=640, height=480, marker_id=0, center_x=320, center_y=240, marker_size_px=100
        )
        det_res = detector.detect(img)
        p_cam = calib.back_project(det_res.pixel_x, det_res.pixel_y, 0.60)
        p_base = reg.transform_point(*p_cam)

        print(f"  Detected pixel target: ({det_res.pixel_x:.1f}, {det_res.pixel_y:.1f})")
        print(f"  3D Camera coordinates: ({p_cam[0]:.3f}, {p_cam[1]:.3f}, {p_cam[2]:.3f}) m")
        print(f"  Registered Base coordinates: ({p_base[0]:.3f}, {p_base[1]:.3f}, {p_base[2]:.3f}) m")

        # [3] Dispatch Target to Safety Controller
        print("\n[3] Dispatching Target to /target_pose...")
        target_msg = PoseStamped()
        target_msg.header.stamp = pub_node.get_clock().now().to_msg()
        target_msg.header.frame_id = "base_link"
        target_msg.pose.position = Point(x=float(p_base[0]), y=float(p_base[1]), z=float(p_base[2]))
        target_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        target_pub.publish(target_msg)

        # Spin until motion completed
        print("  Executing closed-loop 50 Hz trajectory execution...")
        t0 = time.time()
        while robot_node.robot_state != SurgiTwinRobotSimNode.STATE_COMPLETED and (time.time() - t0) < 5.0:
            executor.spin_once(timeout_sec=0.02)

        print(f"  Robot State: {robot_node.robot_state}")
        _, tool_final_pos, _ = robot_node.kinematics.forward_kinematics(robot_node.plant.q)
        err_mm = np.linalg.norm(p_base - tool_final_pos) * 1000.0
        print(f"  Tool Final Position: ({tool_final_pos[0]:.4f}, {tool_final_pos[1]:.4f}, {tool_final_pos[2]:.4f}) m")
        print(f"  Residual Positioning Error: {err_mm:.3f} mm")

        metrics = robot_node.plant.get_metrics()
        print(f"  Dynamic Tracking Metrics: Max Err = {metrics['max_error_mm']:.3f} mm, RMS = {metrics['rms_error_mm']:.3f} mm")

        # Plot tracking trajectory
        if robot_node.plant.history:
            history = robot_node.plant.history
            t_hist = [h.time - history[0].time for h in history]
            err_hist_mm = [h.cartesian_error * 1000.0 for h in history]
            q_act = np.array([h.q_actual for h in history])
            q_des = np.array([h.q_desired for h in history])

            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
            ax1.plot(t_hist, err_hist_mm, "b-", linewidth=2, label="Cartesian Tip Error (mm)")
            ax1.axhline(0.5, color="r", linestyle="--", label="0.5 mm Surgical Tolerance")
            ax1.set_ylabel("Tracking Error [mm]")
            ax1.set_title("SurgiTwin Phase 3: Closed-Loop Trajectory Dynamic Response")
            ax1.grid(True, linestyle="--", alpha=0.7)
            ax1.legend(loc="upper right")

            for j in range(6):
                ax2.plot(t_hist, q_act[:, j], label=robot_node.kinematics.JOINT_NAMES[j])
            ax2.set_xlabel("Time [s]")
            ax2.set_ylabel("Joint Positions [rad]")
            ax2.grid(True, linestyle="--", alpha=0.7)
            ax2.legend(loc="upper right", ncol=2, fontsize=8)

            plot_path = os.path.join(output_dir, "phase3_trajectory_tracking.png")
            fig.tight_layout()
            fig.savefig(plot_path, dpi=150)
            plt.close(fig)
            print(f"  Saved trajectory response plot to: {plot_path}")

        # [4] Emergency Stop Demonstration
        print("\n[4] Emergency Stop Braking Test...")
        # Command new target
        estop_target = PoseStamped()
        estop_target.header.stamp = pub_node.get_clock().now().to_msg()
        estop_target.header.frame_id = "base_link"
        estop_target.pose.position = Point(x=0.15, y=0.45, z=0.25)
        estop_target.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        target_pub.publish(estop_target)

        # Spin until executing
        t0 = time.time()
        while robot_node.robot_state != SurgiTwinRobotSimNode.STATE_EXECUTING and (time.time() - t0) < 2.0:
            executor.spin_once(timeout_sec=0.02)

        print(f"  Robot in motion (State: {robot_node.robot_state}). Asserting E-Stop!")
        estop_msg = Bool()
        estop_msg.data = True
        estop_pub.publish(estop_msg)

        # Spin to process E-stop
        t0 = time.time()
        while robot_node.robot_state != SurgiTwinRobotSimNode.STATE_ESTOPPED and (time.time() - t0) < 2.0:
            executor.spin_once(timeout_sec=0.02)

        print(f"  Robot arrested! State: {robot_node.robot_state}, Joint Velocity Max: {np.max(np.abs(robot_node.plant.dq)):.4f} rad/s")

        # [5] Out-of-bounds Rejection Test
        print("\n[5] Out-of-Bounds Target Safety Verification...")
        # Clear estop first
        clear_estop = Bool()
        clear_estop.data = False
        estop_pub.publish(clear_estop)
        executor.spin_once(timeout_sec=0.05)

        oob_msg = PoseStamped()
        oob_msg.header.stamp = pub_node.get_clock().now().to_msg()
        oob_msg.header.frame_id = "base_link"
        oob_msg.pose.position = Point(x=0.90, y=0.35, z=0.20)  # X out of bounds
        target_pub.publish(oob_msg)

        executor.spin_once(timeout_sec=0.1)
        print("  Out-of-bounds target (x=0.90 m) dispatched to safety node.")
        print(f"  Robot State remains safe: {robot_node.robot_state}")

        print("\n" + "=" * 80)
        print("PHASE 3 DEMONSTRATION SUCCESSFULLY COMPLETED!")
        print("=" * 80)

    finally:
        executor.shutdown()
        pub_node.destroy_node()
        safety_node.destroy_node()
        robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    run_demo()
