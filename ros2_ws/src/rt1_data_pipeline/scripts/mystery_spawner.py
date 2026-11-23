#!/usr/bin/env python3
# =============================================================================
# mystery_spawner.py
# -----------------------------------------------------------------------------
# Target:
#   Episode generator for data collection. On each /episode/next request it
#   removes the old mystery block, spawns a new one with hidden physical
#   properties (k, m) and publishes the episode's ground truth (prompt,
#   classes, correct bin) on a latched topic for the FSM and the logger.
#
# Physics / Math Implemented:
#   - Stiffness k [N/m] sampled LOG-uniformly: stiffness spans a decade, so
#     each factor of k is equally likely (uniform would almost never pick 300).
#   - Mass m [kg] sampled uniformly.
#   - Plate damping c = sqrt(k * m_plate), m_plate = 0.02 kg (zeta = 0.5,
#     the rule validated in Phase 1 G2).
#   - Train / eval ranges are DISJOINT: each eval band sits between the two
#     train bands of its class (tests unseen values by interpolation).
#   - Balanced design: every (prompt x stiffness class x mass class) cell gets
#     the same episode count; order shuffled with a saved seed.
#
# Inputs & Outputs:
#   Srv  : /episode/next   std_srvs/Trigger -> spawn the next scheduled block
#   Out  : /episode/info   rt1_interfaces/EpisodeInfo, transient-local (latched)
#   File : <schedule_dir>/<split>_seed<seed>.json  full pre-sampled schedule
#   Sim  : model "mystery_block" in world "sorting_arena" at (0.20, 0, 0.026) m
# =============================================================================

import json
import math
import time
import rclpy
import xacro
import random
import itertools
import subprocess
from pathlib import Path
from rclpy.node import Node
from std_srvs.srv import Trigger
from rt1_interfaces.msg import EpisodeInfo
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# --- 1. EXPERIMENT DESIGN TABLES ---------------------------------------------
PLATE_MASS_KG = 0.02          # must match mystery_block.sdf.xacro plate_mass
FRICTION_COEFF = 0.8          # block mu; slip threshold m = 2*mu*F/g

# (low, high) intervals per class and split. A value is drawn from the union.
STIFFNESS_RANGES_N_PER_M = {
    "train": {
        "soft":  [( 300.0,  400.0), ( 500.0,  650.0)],
        "stiff": [(2000.0, 3000.0), (4000.0, 5000.0)]
        },
    "eval": {
        "soft":  [( 400.0,  500.0)],
        "stiff": [(3000.0, 4000.0)]
        },
}
MASS_RANGES_KG = {
    "train": {
        "light": [(0.06, 0.12), (0.18, 0.24)],
        "heavy": [(0.40, 0.45), (0.50, 0.55)]
        },
    "eval": {
        "light": [(0.12, 0.18)],
        "heavy": [(0.45, 0.50)]
        },
}

# prompt_id -> (axis, class sent to `bin_for_class`, bin_for_class, text)
PROMPTS = {
    "P_STIFF_0": ("stiffness", "soft", "blue_bin", "sort by stiffness: put soft blocks in the blue bin"),
    "P_STIFF_1": ("stiffness", "soft",  "red_bin", "sort by stiffness: put soft blocks in the red bin"),
    "P_MASS_0" : ("mass", "light", "blue_bin", "sort by weight: put light blocks in the blue bin"),
    "P_MASS_1" : ("mass", "light",  "red_bin", "sort by weight: put light blocks in the red bin"),
}
STAGE_PROMPTS = {
    "A": ["P_STIFF_0", "P_STIFF_1"],
    "B": ["P_STIFF_0", "P_STIFF_1", "P_MASS_0", "P_MASS_1"]
}
OTHER_BIN = {"blue_bin": "red_bin", "red_bin": "blue_bin"}


# --- 2. SAMPLING HELPERS ------------------------------------------------------
def sample_from_intervals(rng, intervals, log_scale):
    """Draw one value from a union of intervals.
    Each interval is chosen with probability proportional to its length
    (in log space for log sampling), so the union behaves like ONE
    continuous distribution with a hole in it.
    """
    if log_scale:
        weights = [math.log(high) - math.log(low) for low, high in intervals]
    else:
        weights = [high - low for low, high in intervals]
    low, high = rng.choices(intervals, weights=weights, k=1)[0]
    if log_scale:
        return math.exp(rng.uniform(math.log(low), math.log(high)))
    return rng.uniform(low, high)


def target_bin_for(prompt_id, stiffness_class, mass_class):
    axis, special_class, special_bin, _ = PROMPTS[prompt_id]
    block_class = stiffness_class if axis == "stiffness" else mass_class
    return special_bin if block_class == special_class else OTHER_BIN[special_bin]


def build_schedule(split, stage, episodes_per_cell, seed):
    """Pre-sample EVERY episode up front -> the dataset is fixed by the seed."""
    rng = random.Random(seed)
    cells = list(itertools.product(STAGE_PROMPTS[stage], ("soft", "stiff"), ("light", "heavy")))
    cell_sequence = [cell for cell in cells for _ in range(episodes_per_cell)]
    rng.shuffle(cell_sequence)  # interleave cells so a crash mid-run stays balanced-ish
    schedule = []
    for episode_id, (prompt_id, stiffness_class, mass_class) in enumerate(cell_sequence):
        stiffness = sample_from_intervals(
            rng, STIFFNESS_RANGES_N_PER_M[split][stiffness_class], log_scale=True)
        mass = sample_from_intervals(rng, MASS_RANGES_KG[split][mass_class], log_scale=False)
        schedule.append({
            "episode_id": episode_id,
            "prompt_id": prompt_id,
            "prompt_text": PROMPTS[prompt_id][3],
            "stiffness_class": stiffness_class,
            "mass_class": mass_class,
            "stiffness_n_per_m": round(stiffness, 2),
            "damping_n_s_per_m": round(math.sqrt(stiffness * PLATE_MASS_KG), 4),
            "mass_kg": round(mass, 4),
            "target_bin": target_bin_for(prompt_id, stiffness_class, mass_class),
        })
    return schedule



# --- 3. ROS NODE --------------------------------------------------------------
class MysterySpawner(Node):
    def __init__(self):
        super().__init__("mystery_spawner")
        self.split       = self.declare_parameter("split", "train").value
        self.stage       = self.declare_parameter("stage", "A").value
        default_count    = 38 if self.split == "train" else 8
        self.episodes_per_cell = self.declare_parameter("episodes_per_cell", default_count).value
        default_seed     = 1234 if self.split == "train" else 5678
        self.seed        = self.declare_parameter("seed", default_seed).value
        self.world_name  = self.declare_parameter("world_name", "sorting_arena").value
        self.model_name  = self.declare_parameter("model_name", "mystery_block").value
        self.spawn_xyz_m = self.declare_parameter("spawn_xyz_m", [0.20, 0.0, 0.026]).value
        schedule_dir     = Path(self.declare_parameter( "schedule_dir", "/workspace/data/schedules").value)
        if self.split not in ("train", "eval") or self.stage not in STAGE_PROMPTS:
            raise ValueError(f"bad split/stage: {self.split}/{self.stage}")
        self.block_xacro_path = str(
            Path(get_package_share_directory("rt1_sim_bringup")) / "urdf" / "mystery_block.sdf.xacro")
        # Whole schedule decided now; save it so the dataset can be audited/reproduced.
        self.schedule = build_schedule(self.split, self.stage, self.episodes_per_cell, self.seed)
        self.next_index = 0
        schedule_dir.mkdir(parents=True, exist_ok=True)
        schedule_path = schedule_dir / f"{self.split}_stage{self.stage}_seed{self.seed}.json"
        schedule_path.write_text(json.dumps(
            {"split": self.split, "stage": self.stage, "seed": self.seed,
             "episodes_per_cell": self.episodes_per_cell, "episodes": self.schedule}, indent=1))
        # Latched: a logger that starts late still receives the current episode.
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.info_publisher = self.create_publisher(EpisodeInfo, "/episode/info", latched_qos)
        self.create_service(Trigger, "/episode/next", self.on_next_episode)
        self.get_logger().info(
            f"{len(self.schedule)} episodes ({self.split}, stage {self.stage}, seed {self.seed}) "
            f"saved to {schedule_path}")

    # --- 4. GAZEBO SPAWN / REMOVE (same mechanisms validated in Phase 1) ----
    def remove_block(self):
        # Ignore failure: on the first episode there is nothing to remove.
        subprocess.run(
            ["gz", "service", "-s", f"/world/{self.world_name}/remove",
             "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
             "--timeout", "3000", "--req", f'name: "{self.model_name}" type: MODEL'],
            capture_output=True, timeout=10)
        time.sleep(0.5)  # let the entity disappear before re-using its name

    def spawn_block(self, episode):
        block_sdf = xacro.process_file(
            self.block_xacro_path,
            mappings={"name": self.model_name,
                      "mass": f"{episode['mass_kg']:.4f}",
                      "k": f"{episode['stiffness_n_per_m']:.2f}",
                      "c": f"{episode['damping_n_s_per_m']:.4f}",
                      "mu": f"{FRICTION_COEFF}"}).toxml()
        x_m, y_m, z_m = self.spawn_xyz_m
        result = subprocess.run(
            ["ros2", "run", "ros_gz_sim", "create",
             "-world", self.world_name, "-name", self.model_name, "-string", block_sdf,
             "-x", str(x_m), "-y", str(y_m), "-z", str(z_m)],
            capture_output=True, text=True, timeout=60)
        if result.returncode != 0 or "successful" not in (result.stdout + result.stderr):
            raise RuntimeError(f"spawn failed: {result.stderr.strip()[-300:]}")

    # --- 5. SERVICE: NEXT EPISODE ---------------------------------------------
    def on_next_episode(self, request, response):
        del request  # Trigger has no fields
        if self.next_index >= len(self.schedule):
            response.success = False
            response.message = "schedule complete"
            return response
        episode = self.schedule[self.next_index]
        try:
            self.remove_block()
            self.spawn_block(episode)
        except (RuntimeError, subprocess.TimeoutExpired) as spawn_error:
            response.success = False
            response.message = str(spawn_error)
            self.get_logger().error(response.message)
            return response
        info = EpisodeInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = "world"
        info.episode_id = episode["episode_id"]
        info.split = self.split
        info.seed = self.seed
        info.prompt_id = episode["prompt_id"]
        info.prompt_text = episode["prompt_text"]
        info.stiffness_class = episode["stiffness_class"]
        info.mass_class = episode["mass_class"]
        info.stiffness_n_per_m = episode["stiffness_n_per_m"]
        info.damping_n_s_per_m = episode["damping_n_s_per_m"]
        info.mass_kg = episode["mass_kg"]
        info.target_bin = episode["target_bin"]
        self.info_publisher.publish(info)
        self.next_index += 1
        response.success = True
        response.message = (f"episode {info.episode_id}: {info.prompt_id} "
                            f"{info.stiffness_class}/{info.mass_class} "
                            f"k={info.stiffness_n_per_m} m={info.mass_kg} -> {info.target_bin}")
        self.get_logger().info(response.message)
        return response



def main():
    rclpy.init()
    node = MysterySpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()



if __name__ == "__main__":
    main()
    