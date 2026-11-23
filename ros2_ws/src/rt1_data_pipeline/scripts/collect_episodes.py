#!/usr/bin/env python3
# =============================================================================
# collect_episodes.py
# -----------------------------------------------------------------------------
# Target:
#   Drive data collection: spawn -> run expert -> wait -> check where the block
#   landed -> write result.json + summary.csv -> repeat.
#
# Physics / Math Implemented:
#   - Success = FSM reached DONE AND block centre lies inside the ground-truth
#     target bin footprint (|x - bx| <= h and |y - by| <= h, h = 0.07 m).
#
# Inputs & Outputs:
#   Cli : /episode/next, /fsm/run_episode (std_srvs/Trigger)
#   Sub : /fsm/status, /episode/info (latched)
#   Out : <data_root>/<split>/episode_XXXXX/result.json, <split>/summary.csv
# =============================================================================

import re
import time
import csv
import json
import rclpy
import subprocess
from pathlib import Path
from rclpy.node import Node
from std_srvs.srv import Trigger
from rt1_interfaces.msg import EpisodeInfo, FsmStatus
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

TERMINAL_STATES = (FsmStatus.DONE, FsmStatus.FAILED)
CSV_COLUMNS = ["episode_id", "prompt_id", "stiffness_class", "mass_class", "stiffness_n_per_m",
               "mass_kg", "target_bin", "fsm_final", "failure_code", "decided_bin",
               "eps_vis_at_decision", "regrasp_count", "landed_bin", "success"]


class Collector(Node):
    def __init__(self):
        super().__init__("collect_episodes")
        self.split = self.declare_parameter("split", "train").value
        self.num_episodes = int(self.declare_parameter("num_episodes", 0).value)  # 0 = all
        self.data_root = Path(self.declare_parameter("data_root", "/workspace/data").value)
        self.episode_timeout_s = float(self.declare_parameter("episode_timeout_s", 180.0).value)
        self.bin_half_size_m = float(self.declare_parameter("bin_half_size_m", 0.07).value)
        self.model_name = self.declare_parameter("model_name", "mystery_block").value
        self.bin_centres_m = {"blue_bin": (0.12, 0.20), "red_bin": (0.12, -0.20)}
        self.next_client = self.create_client(Trigger, "/episode/next")
        self.run_client = self.create_client(Trigger, "/fsm/run_episode")
        self.latest_status = None
        self.latest_info = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(FsmStatus, "/fsm/status", self.on_status, 10)
        self.create_subscription(EpisodeInfo, "/episode/info", self.on_info, latched)

    def on_status(self, msg):
        self.latest_status = msg

    def on_info(self, msg):
        self.latest_info = msg

    # --- 1. SMALL BLOCKING HELPERS (wall-clock timeouts) ------------------------
    def call_trigger(self, client, timeout_s):
        if not client.wait_for_service(timeout_sec=10.0):
            return None
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        return future.result() if future.done() else None

    def wait_until(self, predicate, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def landed_bin(self):
        # gz CLI queries can come back EMPTY (not an error) when the server is
        # busy (same slowness as the 30 s spawn timeouts). Retry before giving up.
        for attempt in range(6):
            try:
                result = subprocess.run(["gz", "model", "-m", self.model_name, "-p"],
                                        capture_output=True, text=True, timeout=10)
                match = re.search(r"\[\s*([-0-9.e+]+)\s+([-0-9.e+]+)\s+([-0-9.e+]+)\s*\]",
                                  result.stdout)
            except subprocess.TimeoutExpired:
                match = None
            if match:
                break
            self.get_logger().warn(f"pose query empty (attempt {attempt + 1}/6), retrying")
            time.sleep(1.0)
        else:
            return "unknown", None
        x_m, y_m, z_m = (float(v) for v in match.groups())
        for bin_name, (centre_x, centre_y) in self.bin_centres_m.items():
            if abs(x_m - centre_x) <= self.bin_half_size_m and abs(y_m - centre_y) <= self.bin_half_size_m:
                return bin_name, (x_m, y_m, z_m)
        return "none", (x_m, y_m, z_m)

    # --- 2. MAIN LOOP -----------------------------------------------------------
    def run(self):
        split_dir = self.data_root / self.split
        split_dir.mkdir(parents=True, exist_ok=True)
        summary_path = split_dir / "summary.csv"
        if not summary_path.exists():
            with summary_path.open("w", newline="") as handle:
                csv.writer(handle).writerow(CSV_COLUMNS)
        attempted = successes = consecutive_errors = 0
        while self.num_episodes == 0 or attempted < self.num_episodes:
            reply = self.call_trigger(self.next_client, 120.0)
            if reply is None or not reply.success:
                message = reply.message if reply else "no reply from /episode/next"
                if "schedule complete" in message:
                    self.get_logger().info("Schedule complete.")
                    break
                consecutive_errors += 1
                self.get_logger().error(f"spawn error: {message[-120:]}")
                if consecutive_errors >= 3:
                    break
                continue
            consecutive_errors = 0
            episode_id = int(re.search(r"episode (\d+)", reply.message).group(1))
            self.wait_until(lambda: self.latest_info is not None
                            and self.latest_info.episode_id == episode_id, 5.0)
            info = self.latest_info
            self.wait_until(lambda: False, 0.5)   # let the block settle on the table
            reply = self.call_trigger(self.run_client, 10.0)
            if reply is None or not reply.success:
                self.get_logger().error(f"FSM did not start: {reply.message if reply else 'no reply'}")
                break
            # Wait until this episode is visibly RUNNING, so a stale DONE from the
            # previous episode can never be mistaken for this one's result.
            self.wait_until(
                lambda: self.latest_status is not None
                and self.latest_status.episode_id == episode_id
                and self.latest_status.state not in TERMINAL_STATES,
                10.0)
            finished = self.wait_until(
                lambda: self.latest_status is not None
                and self.latest_status.episode_id == episode_id
                and self.latest_status.state in TERMINAL_STATES,
                self.episode_timeout_s)
            status = self.latest_status
            if not finished or status.state == FsmStatus.FAILED:
                # The FSM parks the arm (3 s move); keep the next spawn clear of it.
                self.wait_until(lambda: False, 5.0)
            self.wait_until(lambda: False, 0.1)   # block comes to rest in the bin
            landed, position_m = self.landed_bin()
            episode_dir = split_dir / f"episode_{episode_id:05d}"
            meta_ready = self.wait_until(lambda: (episode_dir / "meta.json").exists(), 30.0)
            fsm_final = status.state_name if finished else "COLLECTOR_TIMEOUT"
            success = bool(finished and status.state == FsmStatus.DONE
                           and landed == info.target_bin)
            result = {"episode_id": episode_id, "fsm_final": fsm_final,
                      "failure_code": status.failure_code if finished else "",
                      "landed_bin": landed, "block_position_m": position_m,
                      "target_bin": info.target_bin, "success": success}
            if meta_ready:
                (episode_dir / "result.json").write_text(json.dumps(result, indent=1))
            with summary_path.open("a", newline="") as handle:
                csv.writer(handle).writerow([
                    episode_id, info.prompt_id, info.stiffness_class, info.mass_class,
                    info.stiffness_n_per_m, info.mass_kg, info.target_bin, fsm_final,
                    result["failure_code"], status.decided_bin, round(status.eps_vis_at_decision, 4),
                    status.regrasp_count, landed, int(success)])
            attempted += 1
            successes += int(success)
            self.get_logger().info(
                f"[{attempted}] ep {episode_id} {info.prompt_id} "
                f"{info.stiffness_class}/{info.mass_class} eps={status.eps_vis_at_decision:.3f} "
                f"-> {status.decided_bin or '-'} landed {landed} "
                f"{'OK' if success else 'FAIL ' + fsm_final}  (success {successes}/{attempted})")


def main():
    rclpy.init()
    node = Collector()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()