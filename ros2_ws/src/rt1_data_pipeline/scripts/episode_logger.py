#!/usr/bin/env python3
# =============================================================================
# episode_logger.py
# -----------------------------------------------------------------------------
# Target:
#   Record each expert episode as a training sample for RT-1:
#   data/<split>/episode_00042/{frames/000.png, steps.npz, meta.json}
#
# Physics / Math Implemented:
#   - Sampling at 5 Hz on SIMULATION time (policy control rate; 6 frames = 1.2 s).
#   - Continuous actions a_t = [q(t+1) - q(t) (4 joints, rad), F(t+1) - F(t) (N)].
#     Stored continuous, not as bins, so bin limits can be retuned later (G3).
#   - Dropped frame: newest canvas older than max_image_age_s at a sample tick.
#
# Inputs & Outputs:
#   Sub : /stitched_canvas/image_raw, /joint_states, /probe/metrics,
#         /fsm/status (start = HOME, stop = DONE/FAILED), /episode/info (latched)
#   Out : files under <data_root>/<split>/ ; meta.json is written LAST
#         (the collector waits for it).
# =============================================================================

import cv2
import json
import math
import rclpy
import shutil
import numpy as np
from pathlib import Path
from rclpy.node import Node
from rclpy.time import Time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from rt1_interfaces.msg import EpisodeInfo, FsmStatus, ProbeMetrics
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4")
GRIPPER_JOINTS = ("gripper_left_joint", "gripper_right_joint")
TERMINAL_STATES = (FsmStatus.DONE, FsmStatus.FAILED)


def episode_info_to_dict(info):
    return {
        "episode_id": int(info.episode_id), "split": info.split, "seed": int(info.seed),
        "prompt_id": info.prompt_id, "prompt_text": info.prompt_text,
        "stiffness_class": info.stiffness_class, "mass_class": info.mass_class,
        "stiffness_n_per_m": info.stiffness_n_per_m,
        "damping_n_s_per_m": info.damping_n_s_per_m,
        "mass_kg": info.mass_kg, "target_bin": info.target_bin,
    }


class EpisodeLogger(Node):

    def __init__(self):
        super().__init__("episode_logger")
        # --- 1. PARAMETERS ---
        self.data_root = Path(self.declare_parameter("data_root", "/workspace/data").value)
        self.split = self.declare_parameter("split", "train").value
        rate_hz = float(self.declare_parameter("rate_hz", 5.0).value)
        self.max_image_age_s = float(self.declare_parameter("max_image_age_s", 0.25).value)
        # --- 2. LATEST-VALUE CACHES (filled by callbacks, sampled by the timer) ---
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_joints = {}
        self.latest_metrics = None
        self.latest_status = None
        self.episode_info = None
        # --- 3. RECORDING STATE ---
        self.recording = False
        self.current_episode_id = None
        self.last_logged_episode_id = None
        self.steps = []
        self.frames = []
        self.dropped_frames = 0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, "/stitched_canvas/image_raw", self.on_image, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(ProbeMetrics, "/probe/metrics", self.on_metrics, 10)
        self.create_subscription(FsmStatus, "/fsm/status", self.on_status, 10)
        self.create_subscription(EpisodeInfo, "/episode/info", self.on_info, latched)
        # Node clock = sim time when use_sim_time is true -> 5 Hz of SIM time.
        self.create_timer(1.0 / rate_hz, self.on_tick)
        self.get_logger().info(f"Logging to {self.data_root / self.split} at {rate_hz} Hz")

    # --- CALLBACKS ---------------------------------------------------------
    def on_image(self, msg):
        self.latest_image = msg

    def on_joints(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.latest_joints[name] = position

    def on_metrics(self, msg):
        self.latest_metrics = msg

    def on_info(self, msg):
        self.episode_info = msg

    def on_status(self, msg):
        self.latest_status = msg
        if (not self.recording and msg.state == FsmStatus.HOME and msg.episode_id != self.last_logged_episode_id):
            self.start_episode(msg.episode_id)
        elif self.recording and msg.state in TERMINAL_STATES:
            self.record_step()          # capture the terminal frame too
            self.finalize_episode(msg)

    def on_tick(self):
        if self.recording:
            self.record_step()

    # --- RECORDING ---------------------------------------------------------
    def start_episode(self, episode_id):
        self.recording = True
        self.current_episode_id = episode_id
        self.steps = []
        self.frames = []
        self.dropped_frames = 0
        self.get_logger().info(f"Recording episode {episode_id}")

    def record_step(self):
        if self.latest_image is None or self.latest_status is None:
            self.dropped_frames += 1
            return
        now = self.get_clock().now()
        image_age_s = (now - Time.from_msg(self.latest_image.header.stamp)).nanoseconds * 1e-9
        if image_age_s > self.max_image_age_s:
            # Stale canvas = camera or stitcher stalled; a duplicated frame would
            # teach the policy that the world froze, so skip and count it.
            self.dropped_frames += 1
            return
        frame_bgr = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8").copy()
        metrics = self.latest_metrics
        status = self.latest_status
        self.frames.append(frame_bgr)
        self.steps.append({
            "t_sim": now.nanoseconds * 1e-9,
            "joint_pos": [self.latest_joints.get(j, math.nan) for j in ARM_JOINTS],
            "gripper_pos": [self.latest_joints.get(j, math.nan) for j in GRIPPER_JOINTS],
            "grip_force_cmd": status.grip_force_cmd_n,
            "fsm_state": status.state,
            "eps_vis": metrics.eps_vis if metrics else math.nan,
            "dy_rel": metrics.dy_rel_px if metrics else math.nan,
            "d_px": metrics.d_px if metrics else math.nan,
            "probe_valid": bool(metrics.valid) if metrics else False,
        })

    def finalize_episode(self, status):
        self.recording = False
        self.last_logged_episode_id = self.current_episode_id
        episode_dir = self.data_root / self.split / f"episode_{self.current_episode_id:05d}"
        if episode_dir.exists():
            shutil.rmtree(episode_dir)
        (episode_dir / "frames").mkdir(parents=True)
        for index, frame in enumerate(self.frames):
            cv2.imwrite(str(episode_dir / "frames" / f"{index:03d}.png"), frame)
        num_steps = len(self.steps)
        joint_pos = np.array([s["joint_pos"] for s in self.steps], dtype=np.float64).reshape(-1, 4)
        grip_force = np.array([s["grip_force_cmd"] for s in self.steps], dtype=np.float64)
        fsm_state = np.array([s["fsm_state"] for s in self.steps], dtype=np.int8)
        # Action at step t = what changes between t and t+1. Last step has no next.
        action = np.zeros((num_steps, 5), dtype=np.float64)
        action_valid = np.zeros(num_steps, dtype=bool)
        if num_steps >= 2:
            action[:-1, :4] = joint_pos[1:] - joint_pos[:-1]
            action[:-1, 4] = grip_force[1:] - grip_force[:-1]
            action_valid[:-1] = True
        np.savez_compressed(
            episode_dir / "steps.npz",
            t_sim=np.array([s["t_sim"] for s in self.steps]),
            joint_pos=joint_pos,
            gripper_pos=np.array([s["gripper_pos"] for s in self.steps]).reshape(-1, 2),
            grip_force_cmd=grip_force,
            fsm_state=fsm_state,
            eps_vis=np.array([s["eps_vis"] for s in self.steps]),
            dy_rel=np.array([s["dy_rel"] for s in self.steps]),
            d_px=np.array([s["d_px"] for s in self.steps]),
            probe_valid=np.array([s["probe_valid"] for s in self.steps], dtype=bool),
            action=action,
            action_valid=action_valid)
        route_steps = np.flatnonzero(fsm_state == FsmStatus.ROUTE_BIN)
        flags = []
        if status.state == FsmStatus.FAILED:
            flags.append(f"fsm_failed:{status.failure_code}")
        if self.dropped_frames > 0:
            flags.append(f"dropped_frames:{self.dropped_frames}")
        if num_steps < 10:
            flags.append("too_short")
        info_ok = self.episode_info is not None and \
            self.episode_info.episode_id == self.current_episode_id
        if not info_ok:
            flags.append("missing_episode_info")
        meta = {
            "episode": episode_info_to_dict(self.episode_info) if info_ok else None,
            "fsm": {
                "final_state": status.state_name, "failure_code": status.failure_code,
                "decided_stiffness": status.decided_stiffness,
                "decided_mass": status.decided_mass, "decided_bin": status.decided_bin,
                "eps_vis_at_decision": status.eps_vis_at_decision,
                "regrasp_count": int(status.regrasp_count),
                "decision_matches_truth": bool(status.decision_matches_truth),
            },
            "num_steps": num_steps,
            "rate_hz": 5.0,
            "route_start_step": int(route_steps[0]) if route_steps.size else -1,
            "dropped_frames": self.dropped_frames,
            "flags": flags,
        }
        # Written LAST: its existence tells the collector the episode is complete.
        (episode_dir / "meta.json").write_text(json.dumps(meta, indent=1))
        self.get_logger().info(
            f"Saved episode {self.current_episode_id}: {num_steps} steps, flags={flags}")


def main():
    rclpy.init()
    node = EpisodeLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()