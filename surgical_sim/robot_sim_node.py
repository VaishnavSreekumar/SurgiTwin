"""
robot_sim_node.py
SurgiTwin ROS 2 Robotic Manipulator Simulation Node.

Integrates forward/inverse kinematics, quintic polynomial trajectory generation,
PID joint control, and mechanical plant simulation.
Subscribes to /safe_target_pose from the Phase 1 safety supervisor,
solves inverse kinematics, plans dynamically safe trajectories, simulates
50 Hz closed-loop execution, and publishes /joint_states and /tool_pose.
"""

import json
from typing import Optional
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from surgical_sim.robot_kinematics import RobotKinematics
from surgical_sim.inverse_kinematics import InverseKinematicsSolver, IKResult
from surgical_sim.trajectory_planner import TrajectoryPlanner, JointTrajectory
from surgical_sim.robot_controller import RobotPlantSimulator


class SurgiTwinRobotSimNode(Node):
    """
    ROS 2 node for 6-DOF surgical manipulator simulation.
    """

    STATE_IDLE = "IDLE"
    STATE_PLANNING = "PLANNING"
    STATE_EXECUTING = "EXECUTING"
    STATE_COMPLETED = "COMPLETED"
    STATE_ESTOPPED = "ESTOPPED"
    STATE_ERROR = "ERROR"

    def __init__(self):
        super().__init__("surgitwin_robot_sim_node")

        # Declare parameters
        self.declare_parameter("sim_rate_hz", 50.0)
        self.declare_parameter("max_cartesian_velocity", 0.10)  # 100 mm/s
        self.declare_parameter("max_joint_velocity", 1.0)       # 1.0 rad/s
        self.declare_parameter("tool_length", 0.150)            # 150 mm
        self.declare_parameter("ik_tolerance_mm", 0.5)          # 0.5 mm

        self.sim_rate_hz = self.get_parameter("sim_rate_hz").value
        self.dt = 1.0 / self.sim_rate_hz
        max_c_vel = self.get_parameter("max_cartesian_velocity").value
        max_j_vel = self.get_parameter("max_joint_velocity").value
        tool_len = self.get_parameter("tool_length").value
        ik_tol = self.get_parameter("ik_tolerance_mm").value / 1000.0

        # Subsystems
        self.kinematics = RobotKinematics(tool_length=tool_len)
        self.ik_solver = InverseKinematicsSolver(kinematics=self.kinematics, position_tolerance=ik_tol)
        self.planner = TrajectoryPlanner(
            kinematics=self.kinematics,
            max_cartesian_velocity=max_c_vel,
            max_joint_velocity=max_j_vel,
        )
        self.plant = RobotPlantSimulator(kinematics=self.kinematics)

        # Execution state
        self.robot_state = self.STATE_IDLE
        self.current_trajectory: Optional[JointTrajectory] = None
        self.traj_start_time: float = 0.0
        self.traj_elapsed_time: float = 0.0
        self.goal_q: Optional[np.ndarray] = None
        self.goal_pos: Optional[np.ndarray] = None

        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", qos_reliable)
        self.tool_pose_pub = self.create_publisher(PoseStamped, "/tool_pose", qos_reliable)
        self.status_pub = self.create_publisher(String, "/robot/trajectory_status", qos_reliable)
        self.metrics_pub = self.create_publisher(String, "/robot/tracking_metrics", qos_reliable)

        # Subscriptions
        self.safe_pose_sub = self.create_subscription(
            PoseStamped,
            "/safe_target_pose",
            self.on_safe_target_pose,
            qos_reliable,
        )
        self.estop_sub = self.create_subscription(
            Bool,
            "/safety/emergency_stop",
            self.on_emergency_stop,
            qos_reliable,
        )

        # Reset service
        self.reset_service = self.create_service(
            Trigger,
            "/robot/reset_home",
            self.on_reset_home_srv,
        )

        # 50 Hz simulation timer
        self.sim_timer = self.create_timer(self.dt, self.on_sim_tick)

        self.get_logger().info(
            f"SurgiTwin Robot Simulation Node initialized at {self.sim_rate_hz:.1f} Hz. "
            f"Home position: {np.round(self.plant.history[0].cartesian_pos if self.plant.history else self.kinematics.forward_kinematics(self.plant.q)[1], 3)}"
        )

    def on_safe_target_pose(self, msg: PoseStamped):
        """
        Handle incoming safe target pose from Phase 1 safety controller.
        """
        if self.robot_state == self.STATE_ESTOPPED:
            self.get_logger().warn("Rejected target pose: robot is in EMERGENCY STOP state")
            return

        target_p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)
        target_q = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ], dtype=np.float64)

        self.get_logger().info(f"Received safe target: ({target_p[0]:.3f}, {target_p[1]:.3f}, {target_p[2]:.3f})")
        self.robot_state = self.STATE_PLANNING
        self._publish_status()

        # Check if target orientation is non-trivial unit quaternion
        orient_norm = np.linalg.norm(target_q)
        has_orientation = orient_norm > 0.5 and not (target_q[0] == 0 and target_q[1] == 0 and target_q[2] == 0 and target_q[3] == 0)

        # Solve IK
        if has_orientation and abs(orient_norm - 1.0) < 0.1:
            ik_res = self.ik_solver.solve_pose(target_p, target_q, q_seed=self.plant.q)
            if not ik_res.success:
                # Fallback to position-only IK
                ik_res = self.ik_solver.solve_position(target_p, q_seed=self.plant.q)
        else:
            ik_res = self.ik_solver.solve_position(target_p, q_seed=self.plant.q)

        if not ik_res.success:
            self.get_logger().error(f"IK failed for target {target_p}: {ik_res.status_message}")
            self.robot_state = self.STATE_ERROR
            self._publish_status()
            return

        self.get_logger().info(
            f"IK solved in {ik_res.iterations} iterations. Residual error: {ik_res.position_error * 1000.0:.3f} mm"
        )

        # Plan trajectory
        try:
            traj = self.planner.plan(self.plant.q, ik_res.q, dt=self.dt)
            self.current_trajectory = traj
            self.traj_elapsed_time = 0.0
            self.goal_q = ik_res.q.copy()
            self.goal_pos = target_p.copy()
            self.robot_state = self.STATE_EXECUTING
            self._publish_status()
            self.get_logger().info(
                f"Trajectory planned: {len(traj.points)} steps, duration: {traj.duration:.2f} s"
            )
        except Exception as e:
            self.get_logger().error(f"Trajectory planning error: {e}")
            self.robot_state = self.STATE_ERROR
            self._publish_status()

    def on_emergency_stop(self, msg: Bool):
        """
        Handle safety subsystem emergency stop line.
        """
        if msg.data:
            if self.robot_state != self.STATE_ESTOPPED:
                self.get_logger().error("EMERGENCY STOP ASSERTED! Halting robot simulation.")
                self.robot_state = self.STATE_ESTOPPED
                self.plant.set_estop(True)
                self.current_trajectory = None
                self._publish_status()
        else:
            if self.robot_state == self.STATE_ESTOPPED:
                self.get_logger().info("E-stop cleared. Robot entering IDLE state.")
                self.plant.set_estop(False)
                self.robot_state = self.STATE_IDLE
                self._publish_status()

    def on_reset_home_srv(self, request, response):
        """Service callback to reset robot to home configuration."""
        self.plant.reset_state(self.kinematics.SURGICAL_HOME_Q)
        self.current_trajectory = None
        self.robot_state = self.STATE_IDLE
        self._publish_status()
        response.success = True
        response.message = "Robot reset to surgical home position."
        self.get_logger().info(response.message)
        return response

    def on_sim_tick(self):
        """
        50 Hz discrete simulation clock step.
        """
        # Step plant dynamics
        if self.robot_state == self.STATE_EXECUTING and self.current_trajectory is not None:
            self.traj_elapsed_time += self.dt
            sample_pt = self.current_trajectory.sample(self.traj_elapsed_time)

            self.plant.step(sample_pt.positions, sample_pt.velocities, dt=self.dt)

            if self.traj_elapsed_time >= self.current_trajectory.duration:
                # Settle at goal
                if self.goal_q is not None:
                    self.plant.step(self.goal_q, np.zeros(6), dt=self.dt)

                self.robot_state = self.STATE_COMPLETED
                self._publish_status()

                # Report performance metrics
                metrics = self.plant.get_metrics()
                metrics_msg = String()
                metrics_msg.data = json.dumps(metrics)
                self.metrics_pub.publish(metrics_msg)

                self.get_logger().info(
                    f"Trajectory completed! Final error: {metrics['final_error_mm']:.3f} mm, "
                    f"Max dynamic error: {metrics['max_error_mm']:.3f} mm, "
                    f"RMS error: {metrics['rms_error_mm']:.3f} mm"
                )
                self.current_trajectory = None
        elif self.robot_state == self.STATE_COMPLETED:
            # Settle holding current position
            if self.goal_q is not None:
                self.plant.step(self.goal_q, np.zeros(6), dt=self.dt)
            else:
                self.plant.step(self.plant.q, np.zeros(6), dt=self.dt)
        elif self.robot_state == self.STATE_ESTOPPED:
            self.plant.step(self.plant.q, np.zeros(6), dt=self.dt)
        else:
            # IDLE: hold current position
            self.plant.step(self.plant.q, np.zeros(6), dt=self.dt)

        # Publish 50 Hz ROS telemetry
        self._publish_joint_states()
        self._publish_tool_pose()

    def _publish_joint_states(self):
        """Publish sensor_msgs/JointState at 50 Hz."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = self.kinematics.JOINT_NAMES
        msg.position = self.plant.q.tolist()
        msg.velocity = self.plant.dq.tolist()
        msg.effort = (self.plant.history[-1].joint_efforts.tolist() if self.plant.history else [0.0] * 6)
        self.joint_state_pub.publish(msg)

    def _publish_tool_pose(self):
        """Publish geometry_msgs/PoseStamped for current end-effector tool pose."""
        _, pos, quat = self.kinematics.forward_kinematics(self.plant.q)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        msg.pose.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
        self.tool_pose_pub.publish(msg)

    def _publish_status(self):
        """Publish current state machine status."""
        msg = String()
        msg.data = self.robot_state
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SurgiTwinRobotSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
