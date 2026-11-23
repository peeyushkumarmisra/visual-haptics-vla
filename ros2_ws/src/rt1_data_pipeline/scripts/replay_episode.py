#!/usr/bin/env python3
# =============================================================================
# replay_episode.py
# -----------------------------------------------------------------------------
# Target:  G3 item 5: replay a logged episode OPEN-LOOP (no vision, no FSM) and
#          check the block still lands in the same bin. Proves the logged
#          joint/gripper sequence alone reproduces the sort.
# Math:    Joint positions at 5 Hz -> one JointTrajectory (spline through all
#          points); gripper force held piecewise-constant per 0.2 s step.
# I/O:     Param episode_dir. Pub /arm_controller/joint_trajectory,
#          /gripper/force_cmd. Uses gz CLI to respawn the block and read its pose.
#          Run with the sim up and the FSM idle (collection not running).
# =============================================================================

import re
import time
import json
import rclpy
import xacro
import subprocess
import numpy as np
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

BIN_CENTRES_M = {"blue_bin": (0.12, 0.20), "red_bin": (0.12, -0.20)}
BIN_HALF_SIZE_M = 0.07


def to_duration(seconds):
    return Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))



class Replayer(Node):
    def __init__(self):
        super().__init__("replay_episode")
        self.episode_dir = Path(self.declare_parameter("episode_dir", "").value)
        self.traj_pub    = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.grip_pub    = self.create_publisher(Float64, "/gripper/force_cmd", 10)

    def sim_sleep(self, seconds):
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds * 1e-9 < seconds:
            rclpy.spin_once(self, timeout_sec=0.01)

    def grip(self, force_n):
        self.grip_pub.publish(Float64(data=float(force_n)))

    def send_trajectory(self, positions_list, dt_s, first_time_s):
        trajectory = JointTrajectory(joint_names=["joint1", "joint2", "joint3", "joint4"])
        for index, positions in enumerate(positions_list):
            trajectory.points.append(JointTrajectoryPoint(
                positions=[float(v) for v in positions],
                time_from_start=to_duration(first_time_s + index * dt_s)))
        self.traj_pub.publish(trajectory)

    def respawn_block(self, episode):
        subprocess.run(["gz", "service", "-s", "/world/sorting_arena/remove",
                        "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
                        "--timeout", "3000", "--req", 'name: "mystery_block" type: MODEL'],
                       capture_output=True, timeout=10)
        time.sleep(1.0)
        block_xacro = str(Path(get_package_share_directory("rt1_sim_bringup"))/ "urdf" / "mystery_block.sdf.xacro")
        sdf = xacro.process_file(block_xacro, mappings={
            "name": "mystery_block", "mass": f"{episode['mass_kg']:.4f}",
            "k": f"{episode['stiffness_n_per_m']:.2f}",
            "c": f"{episode['damping_n_s_per_m']:.4f}", "mu": "0.8"}).toxml()
        subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-world", "sorting_arena",
                        "-name", "mystery_block", "-string", sdf,
                        "-x", "0.20", "-y", "0.0", "-z", "0.026"],
                       capture_output=True, timeout=30, check=True)

    def landed_bin(self):
        out = subprocess.run(["gz", "model", "-m", "mystery_block", "-p"],
                             capture_output=True, text=True, timeout=10).stdout
        match = re.search(r"\[\s*([-0-9.e+]+)\s+([-0-9.e+]+)\s+([-0-9.e+]+)\s*\]", out)
        if not match:
            return "unknown"
        x_m, y_m = float(match.group(1)), float(match.group(2))
        for name, (cx, cy) in BIN_CENTRES_M.items():
            if abs(x_m - cx) <= BIN_HALF_SIZE_M and abs(y_m - cy) <= BIN_HALF_SIZE_M:
                return name
        return "none"

    def run(self):
        meta = json.loads((self.episode_dir / "meta.json").read_text())
        steps = np.load(self.episode_dir / "steps.npz")
        joint_pos, grip_force = steps["joint_pos"], steps["grip_force_cmd"]
        dt_s = 1.0 / meta["rate_hz"]
        # --- 1. SAME BLOCK, SAME START POSE ---
        self.grip(-1.0)
        self.send_trajectory([joint_pos[0]], dt_s, 2.0)
        self.sim_sleep(2.5)
        self.respawn_block(meta["episode"])
        self.sim_sleep(1.5)
        # --- 2. OPEN-LOOP PLAYBACK ---
        self.send_trajectory(joint_pos[1:], dt_s, dt_s)
        start = self.get_clock().now()
        while True:
            elapsed_s = (self.get_clock().now() - start).nanoseconds * 1e-9
            index = int(elapsed_s / dt_s)
            if index >= len(grip_force):
                break
            self.grip(grip_force[index])
            rclpy.spin_once(self, timeout_sec=0.01)
        self.grip(-1.0)
        self.sim_sleep(2.0)
        # --- 3. VERDICT ---
        landed = self.landed_bin()
        target = meta["episode"]["target_bin"]
        verdict = "PASS" if landed == target else "FAIL"
        self.get_logger().info(f"{verdict}: {self.episode_dir.name} landed {landed}, target {target}")


def main():
    rclpy.init()
    node = Replayer()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()