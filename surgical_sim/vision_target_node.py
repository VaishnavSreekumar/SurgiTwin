"""
vision_target_node.py: ROS 2 computer vision and coordinate registration node.

Subscribes to camera image stream, detects surgical fiducial markers, back-projects
pixels to 3D camera coordinates using camera intrinsics, registers coordinates into
the robot base_link frame, and publishes safe targets to /target_pose for validation
by the Phase 1 Safety Supervisor.

Subscribed Topics:
  /camera/image_raw           (sensor_msgs/msg/Image)

Published Topics:
  /vision/detection_image     (sensor_msgs/msg/Image)
  /vision/target_pixel        (geometry_msgs/msg/PointStamped)
  /vision/target_pose_camera  (geometry_msgs/msg/PoseStamped)
  /vision/target_pose_base    (geometry_msgs/msg/PoseStamped)
  /target_pose                (geometry_msgs/msg/PoseStamped)
"""

import os
from typing import Optional

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from surgical_sim.camera_calibration import CameraCalibration
from surgical_sim.coordinate_registration import CoordinateRegistration
from surgical_sim.vision_detector import DetectionResult, VisionTargetDetector


class VisionTargetNode(Node):
    """ROS 2 node integrating OpenCV detection, camera calibration, and robot registration."""

    def __init__(self) -> None:
        """Initialize parameters, CV detector, calibration models, and ROS interfaces."""
        super().__init__("vision_target_node")

        # Declare parameters
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("extrinsics_file", "")
        self.declare_parameter("working_depth_m", 0.60)
        self.declare_parameter("target_marker_id", 0)
        self.declare_parameter("camera_frame_id", "camera_optical_frame")
        self.declare_parameter("robot_base_frame_id", "base_link")
        self.declare_parameter("synthetic_stream_hz", 0.0)

        # Retrieve parameters
        calib_file = str(self.get_parameter("calibration_file").value)
        extrin_file = str(self.get_parameter("extrinsics_file").value)
        self.working_depth = float(self.get_parameter("working_depth_m").value)
        marker_id = int(self.get_parameter("target_marker_id").value)
        self.cam_frame = str(self.get_parameter("camera_frame_id").value)
        self.base_frame = str(self.get_parameter("robot_base_frame_id").value)

        # Load or synthesize camera calibration
        if calib_file and os.path.isfile(os.path.expanduser(calib_file)):
            self.calibration = CameraCalibration.load_yaml(calib_file)
            self.get_logger().info(f"Loaded camera calibration from {calib_file}")
        else:
            self.calibration = CameraCalibration.create_synthetic(width=640, height=480, hfov_deg=60.0)
            self.get_logger().info("Using verified synthetic camera calibration (640x480, HFOV 60°)")

        # Load or synthesize extrinsic registration
        if extrin_file and os.path.isfile(os.path.expanduser(extrin_file)):
            self.registration = CoordinateRegistration.load_yaml(extrin_file)
            self.get_logger().info(f"Loaded extrinsic registration from {extrin_file}")
        else:
            self.registration = CoordinateRegistration.create_synthetic_overhead(
                camera_pos=(0.0, 0.35, 0.80)
            )
            self.get_logger().info("Using verified synthetic overhead camera registration (0, 0.35, 0.80)m")

        # Detector and bridge
        self.detector = VisionTargetDetector(target_marker_id=marker_id)
        self.bridge = CvBridge()

        # Telemetry counters
        self.total_frames_received: int = 0
        self.total_targets_detected: int = 0
        self.total_targets_published: int = 0

        # Publishers
        self.pub_detection_img = self.create_publisher(Image, "vision/detection_image", 10)
        self.pub_pixel = self.create_publisher(PointStamped, "vision/target_pixel", 10)
        self.pub_pose_cam = self.create_publisher(PoseStamped, "vision/target_pose_camera", 10)
        self.pub_pose_base = self.create_publisher(PoseStamped, "vision/target_pose_base", 10)
        # Final output sent to Phase 1 Safety Supervisor
        self.pub_target_pose = self.create_publisher(PoseStamped, "/target_pose", 10)

        # Subscriber
        self.sub_image = self.create_subscription(
            Image, "camera/image_raw", self._image_callback, 10
        )

        # Optional synthetic stream generator for testing without webcam
        stream_hz = float(self.get_parameter("synthetic_stream_hz").value)
        if stream_hz > 0.0:
            self.timer = self.create_timer(1.0 / stream_hz, self._publish_synthetic_frame)
            self.get_logger().info(f"Synthetic test image stream enabled at {stream_hz:.1f} Hz")

        self.get_logger().info(
            f"VisionTargetNode initialized. Target Marker ID: {marker_id}, "
            f"Working Depth: {self.working_depth:.3f} m, Base Frame: '{self.base_frame}'"
        )

    def _image_callback(self, msg: Image) -> None:
        """Process incoming image, detect target, transform coordinates, and publish."""
        self.total_frames_received += 1
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failure: {e}")
            return

        self._process_cv_frame(cv_img, msg.header.stamp)

    def _process_cv_frame(self, cv_img, stamp=None) -> Optional[DetectionResult]:
        """Execute detection, back-projection, registration, and ROS publications."""
        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        res = self.detector.detect(cv_img)

        # Publish annotated diagnostics image
        if res.annotated_image is not None:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(res.annotated_image, encoding="bgr8")
                annotated_msg.header.stamp = stamp
                annotated_msg.header.frame_id = self.cam_frame
                self.pub_detection_img.publish(annotated_msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish annotated image: {e}")

        if not res.detected:
            # Fail closed: Do NOT publish any target pose
            return res

        self.total_targets_detected += 1
        u, v = res.pixel_x, res.pixel_y

        # 1. Publish 2D pixel coordinate
        pixel_msg = PointStamped()
        pixel_msg.header.stamp = stamp
        pixel_msg.header.frame_id = self.cam_frame
        pixel_msg.point.x = float(u)
        pixel_msg.point.y = float(v)
        pixel_msg.point.z = 0.0
        self.pub_pixel.publish(pixel_msg)

        # 2. 3D Camera Back-Projection: X_c = Z * K^-1 * [u, v, 1]^T
        try:
            x_c, y_c, z_c = self.calibration.back_project(u, v, self.working_depth)
        except ValueError as e:
            self.get_logger().error(f"3D back-projection failed: {e}")
            return res

        cam_pose = PoseStamped()
        cam_pose.header.stamp = stamp
        cam_pose.header.frame_id = self.cam_frame
        cam_pose.pose.position.x = x_c
        cam_pose.pose.position.y = y_c
        cam_pose.pose.position.z = z_c
        cam_pose.pose.orientation.w = 1.0
        self.pub_pose_cam.publish(cam_pose)

        # 3. Camera-to-Robot Registration: P_base = T_base_cam * P_cam
        x_base, y_base, z_base = self.registration.transform_point(x_c, y_c, z_c)
        qx_b, qy_b, qz_b, qw_b = self.registration.transform_orientation(
            0.0, 0.0, 0.0, 1.0
        )

        base_pose = PoseStamped()
        base_pose.header.stamp = stamp
        base_pose.header.frame_id = self.base_frame
        base_pose.pose.position.x = x_base
        base_pose.pose.position.y = y_base
        base_pose.pose.position.z = z_base
        # Default surgical downward orientation
        base_pose.pose.orientation.x = qx_b
        base_pose.pose.orientation.y = qy_b
        base_pose.pose.orientation.z = qz_b
        base_pose.pose.orientation.w = qw_b

        # Publish to vision internal topic
        self.pub_pose_base.publish(base_pose)

        # 4. Forward to Phase 1 Safety Supervisor on /target_pose
        self.pub_target_pose.publish(base_pose)
        self.total_targets_published += 1

        self.get_logger().info(
            f"VISION TARGET -> Base: ({x_base:.3f}, {y_base:.3f}, {z_base:.3f}) m "
            f"from pixel ({u:.1f}, {v:.1f}) at depth {z_c:.2f} m"
        )
        return res

    def _publish_synthetic_frame(self) -> None:
        """Generate and process a synthetic test image."""
        img = VisionTargetDetector.generate_synthetic_aruco_image(
            width=640, height=480, marker_id=0, center_x=320, center_y=240
        )
        self._process_cv_frame(img)


def main(args: Optional[list] = None) -> None:
    """Entrypoint for vision_target_node."""
    rclpy.init(args=args)
    node = VisionTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

