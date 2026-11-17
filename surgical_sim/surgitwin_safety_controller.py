"""
surgitwin_safety_controller.py.

ROS 2 supervisory safety controller node for SurgiTwin.
Intercepts all commanded Cartesian robot targets on /target_pose, validates them against
workspace bounds, velocity limits, temporal staleness, and E-Stop state, and only forwards
verified safe targets to /safe_target_pose.

Subscribed Topics:
  /target_pose             (geometry_msgs/msg/PoseStamped)
  /safety/emergency_stop   (std_msgs/msg/Bool)

Published Topics:
  /safe_target_pose        (geometry_msgs/msg/PoseStamped)
  /safety/status           (std_msgs/msg/String)
  /safety/events           (std_msgs/msg/String)

Services:
  /safety/estop            (std_srvs/srv/Trigger)
  /safety/reset            (std_srvs/srv/Trigger)
"""

import json
import os
from typing import Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from surgical_sim.safety_validator import SafetyValidator, WorkspaceBounds


class SurgiTwinSafetyController(Node):
    """Supervisory ROS 2 safety controller enforcing fail-closed bounds and E-Stop."""

    def __init__(self) -> None:
        """Initialize parameters, communication interfaces, and safety validator."""
        super().__init__("surgitwin_safety_controller")

        # Declare ROS 2 parameters with safe surgical defaults
        self.declare_parameter("workspace_min_x", -0.30)
        self.declare_parameter("workspace_max_x", 0.30)
        self.declare_parameter("workspace_min_y", 0.10)
        self.declare_parameter("workspace_max_y", 0.60)
        self.declare_parameter("workspace_min_z", 0.00)
        self.declare_parameter("workspace_max_z", 0.40)
        self.declare_parameter("max_velocity_mps", 0.100)
        self.declare_parameter("max_staleness_sec", 2.0)
        self.declare_parameter("allowed_frame_id", "base_link")
        self.declare_parameter("metrics_log_path", "~/surgitwin_safety_events.csv")
        self.declare_parameter("status_publish_rate_hz", 5.0)

        # Retrieve parameter values
        bounds = WorkspaceBounds(
            min_x=float(self.get_parameter("workspace_min_x").value),
            max_x=float(self.get_parameter("workspace_max_x").value),
            min_y=float(self.get_parameter("workspace_min_y").value),
            max_y=float(self.get_parameter("workspace_max_y").value),
            min_z=float(self.get_parameter("workspace_min_z").value),
            max_z=float(self.get_parameter("workspace_max_z").value),
        )
        max_vel = float(self.get_parameter("max_velocity_mps").value)
        max_stale = float(self.get_parameter("max_staleness_sec").value)
        frame_id = str(self.get_parameter("allowed_frame_id").value)
        self.log_path = os.path.expanduser(str(self.get_parameter("metrics_log_path").value))

        # Instantiate pure logic validator
        self.validator = SafetyValidator(
            bounds=bounds,
            max_velocity_mps=max_vel,
            max_staleness_sec=max_stale,
            allowed_frame_id=frame_id,
        )

        # Reliable QoS for safety-critical command forwarding
        cmd_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Publishers
        self.safe_pose_pub = self.create_publisher(PoseStamped, "/safe_target_pose", cmd_qos)
        self.status_pub = self.create_publisher(String, "/safety/status", 10)
        self.events_pub = self.create_publisher(String, "/safety/events", 10)

        # Subscriptions
        self.target_sub = self.create_subscription(
            PoseStamped, "/target_pose", self._target_pose_callback, cmd_qos
        )
        self.estop_sub = self.create_subscription(
            Bool, "/safety/emergency_stop", self._estop_topic_callback, 10
        )

        # Services
        self.estop_srv = self.create_service(Trigger, "/safety/estop", self._estop_service_callback)
        self.reset_srv = self.create_service(Trigger, "/safety/reset", self._reset_service_callback)

        # Periodic status heartbeat timer
        rate = float(self.get_parameter("status_publish_rate_hz").value)
        timer_period = 1.0 / rate if rate > 0 else 0.2
        self.status_timer = self.create_timer(timer_period, self._publish_status_heartbeat)

        self.get_logger().info(
            f"SurgiTwin Safety Controller active. Workspace X:[{bounds.min_x}, {bounds.max_x}], "
            f"Y:[{bounds.min_y}, {bounds.max_y}], Z:[{bounds.min_z}, {bounds.max_z}], "
            f"MaxVel: {max_vel * 1000.0:.0f} mm/s, Frame: '{frame_id}'"
        )

    def _target_pose_callback(self, msg: PoseStamped) -> None:
        """Evaluate incoming target against all safety interlocks and publish if safe."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        stamp_sec = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)

        # If timestamp is zero, use arrival time
        if stamp_sec == 0.0:
            stamp_sec = now_sec

        pos = msg.pose.position
        orient = msg.pose.orientation
        frame_id = msg.header.frame_id

        # Validate through decoupled safety engine
        res = self.validator.validate_target(
            x=pos.x,
            y=pos.y,
            z=pos.z,
            qx=orient.x,
            qy=orient.y,
            qz=orient.z,
            qw=orient.w,
            frame_id=frame_id,
            stamp_sec=stamp_sec,
            now_sec=now_sec,
        )

        if res.is_valid:
            # Forward only verified safe target
            self.safe_pose_pub.publish(msg)
            self.get_logger().info(
                f"ACCEPTED: Target ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) m -> /safe_target_pose"
            )
            self._publish_event("ACCEPTED", res.reason, {"x": pos.x, "y": pos.y, "z": pos.z})
        else:
            # Fail closed: reject command and log reason
            self.get_logger().warn(
                f"REJECTED: Target ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) m - Reason: {res.reason}"
            )
            self._publish_event("REJECTED", res.reason, {"x": pos.x, "y": pos.y, "z": pos.z})

    def _estop_topic_callback(self, msg: Bool) -> None:
        """Handle E-Stop activation triggered via topic boolean signal."""
        if msg.data:
            success, message = self.validator.trigger_estop("Topic /safety/emergency_stop (True)")
            self.get_logger().error(f"E-STOP ENGAGED: {message}")
            self._publish_event("ESTOP_TRIGGERED", message, {})

    def _estop_service_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle E-Stop trigger service call."""
        success, message = self.validator.trigger_estop("Service Call /safety/estop")
        response.success = success
        response.message = message
        self.get_logger().error(f"E-STOP ENGAGED: {message}")
        self._publish_event("ESTOP_TRIGGERED", message, {})
        return response

    def _reset_service_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle E-Stop reset service call."""
        success, message = self.validator.reset_estop()
        response.success = success
        response.message = message
        if success:
            self.get_logger().info(f"E-STOP RESET: {message}")
            self._publish_event("ESTOP_RESET", message, {})
        else:
            self.get_logger().warn(f"E-STOP RESET REJECTED: {message}")
        return response

    def _publish_status_heartbeat(self) -> None:
        """Publish real-time telemetry and safety status on /safety/status."""
        payload = {
            "state": self.validator.state.value,
            "total_received": self.validator.total_received,
            "total_accepted": self.validator.total_accepted,
            "total_rejected": self.validator.total_rejected,
            "estop_activations": self.validator.estop_activations,
            "rejection_reasons": self.validator.rejection_reasons,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def _publish_event(self, event_type: str, reason: str, details: dict) -> None:
        """Publish discrete safety event on /safety/events."""
        event_dict = {
            "event_type": event_type,
            "reason": reason,
            "details": details,
            "state": self.validator.state.value,
        }
        msg = String()
        msg.data = json.dumps(event_dict)
        self.events_pub.publish(msg)

    def destroy_node(self) -> None:
        """Flush safety audit events to disk before destruction."""
        try:
            self.validator.export_events_csv(self.log_path)
            self.get_logger().info(f"Safety events flushed to {self.log_path}")
        except Exception as e:
            self.get_logger().warn(f"Failed to export safety events CSV: {e}")
        super().destroy_node()


def main(args: Optional[list] = None) -> None:
    """Entry point for surgitwin_safety_controller."""
    rclpy.init(args=args)
    node = SurgiTwinSafetyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
