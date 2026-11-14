#!/usr/bin/env python3
"""
==============================================================================
 FILE: rt1_sim_bringup/scripts/gripper_force_node.py
==============================================================================
 Target:
   Simulated gripper "hardware layer". Turns ONE grip-force command F into
   equal efforts on both finger joints. The expert FSM and the policy only
   ever command F, as on the real single-motor OpenMANIPULATOR-X gripper.

 Physics / Math Implemented:
   Finger joint axes point outward, so closing = negative effort:
       u_L = u_R = -F          F > 0 squeeze, F < 0 open
   Against a spring plate of stiffness k each plate compresses x = F / k
   (measured k*x/F = 0.995; the 0.5 % is DART's implicit finger damping,
   h*b/m = 0.001*0.1/0.02, see G2 log 14 Nov).
   No software centring: a 100 Hz centring loop on these fingers is unstable
   (needs K*T/b < 2), and equal finger damping already keeps closing
   symmetric (G2 log 13 Nov).

 Inputs & Outputs:
   Sub  /gripper/force_cmd                  std_msgs/Float64          F (N)
   Pub  /gripper_effort_controller/commands std_msgs/Float64MultiArray [u_L, u_R] (N)
   Rate: republished at 100 Hz so a dropped message never leaves the fingers
         without a command (the effort controller holds the last value).
==============================================================================
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class GripperForceNode(Node):

    def __init__(self):
        super().__init__('gripper_force_node')

        # --- 1. PARAMETERS ---
        # Hard cap on |F|: physical safety limit, independent of the learned
        # policy and of the physics guardrail.
        self.declare_parameter('max_force_n', 5.0)
        # Start gently open so the fingers are never limp after launch.
        self.declare_parameter('initial_force_n', -1.0)
        self.declare_parameter('publish_rate_hz', 100.0)

        self.max_force_n = self.get_parameter('max_force_n').value
        self.grip_force_cmd_n = self.clamp(
            self.get_parameter('initial_force_n').value, self.max_force_n)
        publish_period_s = 1.0 / self.get_parameter('publish_rate_hz').value

        # --- 2. ROS INTERFACES ---
        self.effort_pub = self.create_publisher(
            Float64MultiArray, '/gripper_effort_controller/commands', 10)
        self.create_subscription(
            Float64, '/gripper/force_cmd', self.on_force_cmd, 10)
        self.create_timer(publish_period_s, self.publish_efforts)

        self.get_logger().info(
            f'gripper_force_node ready: |F| <= {self.max_force_n} N, '
            f'initial F = {self.grip_force_cmd_n} N')

    # --- 3. HELPERS ---
    @staticmethod
    def clamp(value, limit):
        """Symmetric saturation to [-limit, +limit]."""
        return max(-limit, min(limit, value))

    # --- 4. FORCE COMMAND INPUT ---
    def on_force_cmd(self, msg):
        self.grip_force_cmd_n = self.clamp(msg.data, self.max_force_n)

    # --- 5. FORCE SPLIT (equal efforts, negative closes) ---
    def publish_efforts(self):
        finger_effort_n = -self.grip_force_cmd_n
        out = Float64MultiArray()
        out.data = [finger_effort_n, finger_effort_n]
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