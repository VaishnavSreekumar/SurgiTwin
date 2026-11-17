"""
surgitwin_medical_node.py
SurgiTwin Medical Imaging & Embedded Hardware ROS 2 Node.

Bridges:
1. Medical imaging pipeline: DICOM loader -> Volume reconstruction -> Segmentation ->
   Target extraction -> Coordinate registration -> publishes /target_pose to Phase 1 Safety Supervisor.
2. Embedded hardware interface: subscribes to /joint_states and /safety/emergency_stop,
   transmits binary protocol packets to microcontroller simulator, and publishes
   hardware encoder telemetry to /hardware/joint_feedback and /hardware/status.
"""

import json
import os
import tempfile
from typing import Optional
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from surgical_sim.dicom_loader import DICOMLoader
from surgical_sim.medical_volume import MedicalVolume
from surgical_sim.medical_segmentation import MedicalSegmentation, SegmentationResult
from surgical_sim.anatomical_target import AnatomicalTarget, AnatomicalTargetExtractor
from surgical_sim.medical_registration import MedicalRegistration
from surgical_sim.hardware_interface import (
    HardwareInterface,
    HardwareMode,
    HardwareState,
    JointTelemetry,
    ProtocolCodec,
)
from surgical_sim.microcontroller_sim import MicrocontrollerSimulator


class SurgiTwinMedicalNode(Node):
    """
    ROS 2 Node orchestrating medical imaging ingestion and embedded hardware communication.
    """

    def __init__(self):
        super().__init__("surgitwin_medical_node")

        # Declare parameters
        self.declare_parameter("dicom_directory", "")
        self.declare_parameter("hardware_mode", "SIMULATION")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("telemetry_rate_hz", 50.0)

        self.dicom_dir = self.get_parameter("dicom_directory").value
        hw_mode_str = self.get_parameter("hardware_mode").value
        serial_port = self.get_parameter("serial_port").value
        self.rate_hz = self.get_parameter("telemetry_rate_hz").value

        # Subsystems
        self.registration = MedicalRegistration.create_synthetic_supine_table(
            patient_center_on_table=(0.0, 0.35, 0.18)
        )
        self.active_volume: Optional[MedicalVolume] = None
        self.active_target: Optional[AnatomicalTarget] = None

        # Hardware interface setup
        hw_mode = HardwareMode.SERIAL if hw_mode_str.upper() == "SERIAL" and serial_port else HardwareMode.SIMULATION
        self.hw_interface = HardwareInterface(mode=hw_mode, serial_port=serial_port or None)
        self.mcu_sim = MicrocontrollerSimulator(update_rate_hz=self.rate_hz)

        if hw_mode == HardwareMode.SIMULATION:
            self.hw_interface.virtual_transceiver = self.mcu_sim

        self.hw_interface.connect()

        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self.target_pose_pub = self.create_publisher(PoseStamped, "/target_pose", qos_reliable)
        self.volume_status_pub = self.create_publisher(String, "/medical/volume_status", qos_reliable)
        self.seg_status_pub = self.create_publisher(String, "/medical/segmentation_status", qos_reliable)
        self.hw_feedback_pub = self.create_publisher(JointState, "/hardware/joint_feedback", qos_reliable)
        self.hw_status_pub = self.create_publisher(String, "/hardware/status", qos_reliable)

        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joint_states,
            qos_reliable,
        )
        self.estop_sub = self.create_subscription(
            Bool,
            "/safety/emergency_stop",
            self.on_emergency_stop,
            qos_reliable,
        )

        # Services
        self.process_service = self.create_service(
            Trigger,
            "/medical/process_and_target",
            self.on_process_and_target_srv,
        )

        # Telemetry loop timer (50 Hz)
        self.telemetry_timer = self.create_timer(1.0 / self.rate_hz, self.on_telemetry_tick)

        self.get_logger().info(
            f"SurgiTwin Medical Node active. Hardware mode: {self.hw_interface.mode.value}, "
            f"Connected: {self.hw_interface.is_connected}"
        )

    def on_joint_states(self, msg: JointState):
        """
        Forward joint state commands from Phase 3 simulation to embedded hardware bus.
        """
        if len(msg.position) >= 6 and self.hw_interface.is_connected:
            self.hw_interface.send_joint_command(msg.position[:6])

    def on_emergency_stop(self, msg: Bool):
        """
        Propagate E-stop line to microcontroller.
        """
        if self.hw_interface.is_connected:
            self.hw_interface.send_estop(msg.data)
            if msg.data:
                self.get_logger().warn("Hardware E-Stop packet asserted!")

    def on_process_and_target_srv(self, request, response):
        """
        Execute full medical imaging workflow:
        DICOM -> Volume -> Segmentation -> Target -> Registration -> /target_pose.
        """
        try:
            # Generate or load synthetic DICOM dataset
            temp_dir = tempfile.mkdtemp(prefix="surgitwin_dicom_")
            DICOMLoader.generate_synthetic_series(
                temp_dir,
                num_slices=16,
                rows=64,
                cols=64,
                target_center_mm=(0.0, 10.0, 20.0),
                target_radius_mm=8.0,
            )
            slices = DICOMLoader.load_series(temp_dir)
            self.active_volume = MedicalVolume.from_slices(slices)

            # Volumetric status publication
            vol_msg = String()
            vol_msg.data = json.dumps({
                "slices": self.active_volume.nz,
                "rows": self.active_volume.ny,
                "cols": self.active_volume.nx,
                "spacing_mm": [self.active_volume.spacing_z, self.active_volume.spacing_y, self.active_volume.spacing_x],
            })
            self.volume_status_pub.publish(vol_msg)

            # Segmentation
            seg_res = MedicalSegmentation.segment_target(self.active_volume, hu_min=200.0, hu_max=400.0)
            if not seg_res.success:
                response.success = False
                response.message = f"Segmentation failed: {seg_res.status_message}"
                return response

            seg_msg = String()
            seg_msg.data = json.dumps({
                "success": True,
                "voxel_count": seg_res.voxel_count,
                "volume_mm3": seg_res.volume_mm3,
            })
            self.seg_status_pub.publish(seg_msg)

            # Extract anatomical target
            self.active_target = AnatomicalTargetExtractor.extract_from_segmentation(self.active_volume, seg_res)
            if not self.active_target.valid:
                response.success = False
                response.message = f"Target extraction failed: {self.active_target.status_message}"
                return response

            # Register to robot base coordinates
            p_base = self.registration.transform_target(self.active_target)

            # Publish to /target_pose for Phase 1 Safety Supervisor
            target_pose_msg = PoseStamped()
            target_pose_msg.header.stamp = self.get_clock().now().to_msg()
            target_pose_msg.header.frame_id = "base_link"
            target_pose_msg.pose.position = Point(x=float(p_base[0]), y=float(p_base[1]), z=float(p_base[2]))
            target_pose_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            self.target_pose_pub.publish(target_pose_msg)

            response.success = True
            response.message = (
                f"Medical target extracted and published to /target_pose: "
                f"({p_base[0]:.3f}, {p_base[1]:.3f}, {p_base[2]:.3f}) m"
            )
            self.get_logger().info(response.message)
            return response

        except Exception as e:
            response.success = False
            response.message = f"Medical processing error: {e}"
            self.get_logger().error(response.message)
            return response

    def on_telemetry_tick(self):
        """
        Process microcontroller firmware loop and publish feedback at 50 Hz.
        """
        if self.hw_interface.mode == HardwareMode.SIMULATION:
            telemetry = self.mcu_sim.step(dt=1.0 / self.rate_hz)
            self.hw_interface.hardware_state = telemetry.state
            self.hw_interface.last_telemetry = telemetry

            # Publish sensor_msgs/JointState
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.header.frame_id = "base_link"
            js.name = [
                "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
            ]
            js.position = telemetry.joint_positions_rad.tolist()
            js.velocity = self.mcu_sim.dq_actual.tolist()
            self.hw_feedback_pub.publish(js)

            # Publish status
            st = String()
            st.data = json.dumps({
                "state": telemetry.state.name,
                "is_estopped": telemetry.is_estopped,
                "fault_code": telemetry.fault_code,
                "timestamp_ms": telemetry.timestamp_ms,
            })
            self.hw_status_pub.publish(st)


def main(args=None):
    rclpy.init(args=args)
    node = SurgiTwinMedicalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
