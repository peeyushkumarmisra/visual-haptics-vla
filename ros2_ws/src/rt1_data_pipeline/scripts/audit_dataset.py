#!/usr/bin/env python3
# =============================================================================
# audit_dataset.py   (offline, no ROS needed)
# -----------------------------------------------------------------------------
# Target:  Gate G3: decide whether the collected dataset is fit for training.
# Checks:
#   1. Expert success >= 95 % of attempted episodes.
#   2. Action limits per dimension: [p1 - 10 % span, p99 + 10 % span] -> action_limits.json
#      (consumed by the 256-bin tokenizer in Phase 3.1).
#   3. eps_vis at the decision frame: max(stiff) < min(soft)  (no class overlap).
#   4. Usable episodes per (prompt x stiffness x mass) cell within 10 % of each other.
# Inputs & Outputs:
#   In : <data>/episode_*/meta.json, result.json, steps.npz
#   Out: <data>/action_limits.json, PASS/FAIL table on stdout, exit code 0 = all pass
# =============================================================================

import sys
import json
import argparse
import collections
import numpy as np
from pathlib import Path


ACTION_NAMES = ["d_joint1_rad", "d_joint2_rad", "d_joint3_rad", "d_joint4_rad", "d_grip_force_n"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default="/workspace/data/train")
    args = parser.parse_args()
    data_dir = Path(args.data)
    episodes = []
    for episode_dir in sorted(data_dir.glob("episode_*")):
        meta_path, result_path = episode_dir / "meta.json", episode_dir / "result.json"
        if meta_path.exists() and result_path.exists():
            episodes.append(
                (episode_dir, json.loads(meta_path.read_text()),json.loads(result_path.read_text()))
                )
    if not episodes:
        sys.exit(f"No complete episodes in {data_dir}")
    usable = [(d, m, r) for d, m, r in episodes if r["success"] and not m["flags"]]
    checks = []
    # --- 1. EXPERT SUCCESS ---
    success_rate = sum(r["success"] for _, _, r in episodes) / len(episodes)
    checks.append(("1 expert success >= 95%", success_rate >= 0.95,
                   f"{success_rate:.1%} of {len(episodes)} ({len(usable)} usable)"))
    # --- 2. ACTION LIMITS ---
    all_actions = []
    for episode_dir, _, _ in usable:
        steps = np.load(episode_dir / "steps.npz")
        all_actions.append(steps["action"][steps["action_valid"]])
    actions = np.concatenate(all_actions) if all_actions else np.zeros((0, 5))
    limits = {}
    for dim, name in enumerate(ACTION_NAMES):
        if actions.shape[0] == 0:
            break
        p1, p99 = np.percentile(actions[:, dim], [1, 99])
        span = p99 - p1
        limits[name] = {"min": float(p1 - 0.1 * span), "max": float(p99 + 0.1 * span),
                        "p1": float(p1), "p99": float(p99)
                        }
    (data_dir / "action_limits.json").write_text(json.dumps(limits, indent=1))
    checks.append(("2 action limits written", bool(limits),
                   ", ".join(f"{k}=[{v['min']:.4f},{v['max']:.4f}]" for k, v in limits.items())))
    # --- 3. eps_vis SEPARATION AT THE DECISION FRAME ---
    eps_by_class = collections.defaultdict(list)
    for _, meta, _ in usable:
        eps_by_class[meta["episode"]["stiffness_class"]].append(meta["fsm"]["eps_vis_at_decision"])
    if eps_by_class["soft"] and eps_by_class["stiff"]:
        margin = min(eps_by_class["soft"]) - max(eps_by_class["stiff"])
        checks.append(("3 eps_vis classes separate", margin > 0,
                       f"stiff max {max(eps_by_class['stiff']):.3f}, "
                       f"soft min {min(eps_by_class['soft']):.3f}, margin {margin:+.3f}"))
    else:
        checks.append(("3 eps_vis classes separate", False, "missing a class"))
    # --- 4. CELL BALANCE ---
    cells = collections.Counter(
        (m["episode"]["prompt_id"], m["episode"]["stiffness_class"], m["episode"]["mass_class"])
        for _, m, _ in usable)
    if cells:
        imbalance = (max(cells.values()) - min(cells.values())) / max(cells.values())
        checks.append(("4 cells balanced within 10%", imbalance <= 0.10,
                       f"{len(cells)} cells, min {min(cells.values())}, "
                       f"max {max(cells.values())}, imbalance {imbalance:.1%}"))
    print(f"\nGate G3 audit: {data_dir}\n" + "-" * 70)
    for name, passed, detail in checks:
        print(f"{'PASS' if passed else 'FAIL'}  {name:30s} {detail}")
    print("Item 5 (open-loop replay): run replay_episode.py on 3 episodes.\n")
    sys.exit(0 if all(passed for _, passed, _ in checks) else 1)


if __name__ == "__main__":
    main()