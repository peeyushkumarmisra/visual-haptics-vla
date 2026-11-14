#!/usr/bin/env python3
"""
==============================================================================
 FILE: rt1_sim_bringup/scripts/gripper_force_node.py
==============================================================================
 Target:
   Simulated gripper "hardware layer". Turns ONE grip-force command F into
   two finger efforts, and adds a centring term that recreates the real
   OpenMANIPULATOR-X finger linkage (one motor drives both fingers
   symmetrically). DART has no mimic constraint, so without this the block
   drifts until one finger hits its stop (G2 sweep, 13 Nov).
   The expert FSM and the policy only ever command F.

 Physics / Math Implemented:
   Finger joint axes point outward, so closing = negative effort.
       u_L = -F - (K/2)(q_L - q_R) - (D/2)(v_L - v_R)
       u_R = -F + (K/2)(q_L - q_R) + (D/2)(v_L - v_R)
   Sum u_L + u_R = -2F: the squeeze stays exactly F per finger.
   The K, D terms act only on the finger DIFFERENCE (centring), like a
   stiff linkage between the fingers. Against a spring plate of stiffness k,
   each plate compresses x = F / k.

 Inputs & Outputs:
   Sub  /gripper/force_cmd                  std_msgs/Float64   F (N)
                                            F > 0 squeeze, F < 0 open
   Sub  /joint_states                       sensor_msgs/JointState
                                            gripper_left_joint, gripper_right_joint
                                            position (m), velocity (m/s)
   Pub  /gripper_effort_controller/commands std_msgs/Float64MultiArray
                                            data = [u_L, u_R] (N)
   Rate: one command per /joint_states message (controller manager, 100 Hz).
==============================================================================
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


class GripperForceNode(Node):

    def __init__(self):
        super().__init__('gripper_force_node')

        # --- 1. PARAMETERS ---
        # Centring stiffness: 200 N/m costs only 0.1 N per mm of offset,
        # small next to a 2 N squeeze, yet enough to hold the block centred.
        self.declare_parameter('centering_stiffness_n_per_m', 200.0)
        # Centring damping on the finger velocity difference. Near critical
        # for ~0.06 kg moving mass (2 * sqrt(200 * 0.06) ~ 7): stops the
        # centring loop from oscillating at the 100 Hz command rate.
        self.declare_parameter('centering_damping_ns_per_m', 5.0)
        # Hard cap on |F| and on each finger effort. Physical safety limit,
        # independent of the learned policy and the guardrail.
        self.declare_parameter('max_force_n', 5.0)
        # Start gently open so the fingers are never limp after launch.
        self.declare_parameter('initial_force_n', -1.0)
        self.declare_parameter('left_joint', 'gripper_left_joint')
        self.declare_parameter('right_joint', 'gripper_right_joint')

        self.centering_stiffness_n_per_m = self.get_parameter(
            'centering_stiffness_n_per_m').value
        self.centering_damping_ns_per_m = self.get_parameter(
            'centering_damping_ns_per_m').value
        self.max_force_n = self.get_parameter('max_force_n').value
        self.left_joint = self.get_parameter('left_joint').value
        self.right_joint = self.get_parameter('right_joint').value
        self.grip_force_cmd_n = self.clamp(
            self.get_parameter('initial_force_n').value, self.max_force_n)

        # --- 2. ROS INTERFACES ---
        self.effort_pub = self.create_publisher(
            Float64MultiArray, '/gripper_effort_controller/commands', 10)
        self.create_subscription(
            Float64, '/gripper/force_cmd', self.on_force_cmd, 10)
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, 10)

        self.get_logger().info(
            f'gripper_force_node ready: K={self.centering_stiffness_n_per_m} N/m, '
            f'D={self.centering_damping_ns_per_m} N*s/m, '
            f'|F|<={self.max_force_n} N, initial F={self.grip_force_cmd_n} N')

    # --- 3. HELPERS ---
    @staticmethod
    def clamp(value, limit):
        """Symmetric saturation to [-limit, +limit]."""
        return max(-limit, min(limit, value))

    # --- 4. FORCE COMMAND INPUT ---
    def on_force_cmd(self, msg):
        # Store only; efforts are recomputed on every joint-state update so
        # the centring term always uses fresh finger positions.
        self.grip_force_cmd_n = self.clamp(msg.data, self.max_force_n)

    # --- 5. CENTRING LAW, RUN AT THE JOINT-STATE RATE ---
    def on_joint_states(self, msg):
        try:
            i_left = msg.name.index(self.left_joint)
            i_right = msg.name.index(self.right_joint)
        except ValueError:
            return  # message without the finger joints

        left_pos_m = msg.position[i_left]
        right_pos_m = msg.position[i_right]
        # Velocities can be missing in some JointState publishers.
        has_vel = len(msg.velocity) > max(i_left, i_right)
        left_vel_m_s = msg.velocity[i_left] if has_vel else 0.0
        right_vel_m_s = msg.velocity[i_right] if has_vel else 0.0

        # Positive offset = left finger more open than right.
        finger_offset_m = left_pos_m - right_pos_m
        finger_offset_rate_m_s = left_vel_m_s - right_vel_m_s

        # Centring force: pushes the more-open finger closed and the other
        # open by the same amount, so the total squeeze is unchanged.
        centering_force_n = (
            0.5 * self.centering_stiffness_n_per_m * finger_offset_m
            + 0.5 * self.centering_damping_ns_per_m * finger_offset_rate_m_s)

        # Negative effort closes (finger axes point outward).
        left_effort_n = -self.grip_force_cmd_n - centering_force_n
        right_effort_n = -self.grip_force_cmd_n + centering_force_n

        out = Float64MultiArray()
        out.data = [self.clamp(left_effort_n, self.max_force_n),
                    self.clamp(right_effort_n, self.max_force_n)]
        self.effort_pub.publish(out)


def main():
    rclpy.init()
    node = GripperForceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()