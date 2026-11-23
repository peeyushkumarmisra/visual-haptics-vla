#!/usr/bin/env python3
# =============================================================================
# verify_landing_from_frames.py
# -----------------------------------------------------------------------------
# Target:
#   Recover the landed bin for episodes whose live pose query failed
#   (landed_bin == "unknown") from the LAST recorded camera frame.
#
# Method:
#   1. Segment the purple block (HSV H 125-155, S >= 80, V >= 40, OpenCV units)
#      in the cam_top tile (rows 112..223 of the 224x224 canvas).
#   2. Calibrate on episodes whose landing WAS measured (blue_bin / red_bin):
#      centroid mean + spread per bin.
#   3. Classify each "unknown" episode by nearest bin centroid; accept only if
#      within max(4 * spread, 10 px). Otherwise leave it "unknown".
#   4. Self-test: classify every measured episode too and report the accuracy,
#      so the method is checked before it is trusted.
#
# Output:
#   summary.csv rewritten (backup summary_before_verify.csv); provenance in landing_verification.csv.
# =============================================================================
import argparse
import csv
import glob
import os
import shutil

import cv2
import numpy as np

TOP_TILE_ROWS = (112, 224)


def block_centroid(png_path):
    image = cv2.imread(png_path)
    if image is None:
        return None
    tile = image[TOP_TILE_ROWS[0]:TOP_TILE_ROWS[1]]
    hsv = cv2.cvtColor(tile, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (125, 80, 40), (155, 255, 255))
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
    if count < 2:
        return None
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[largest, cv2.CC_STAT_AREA] < 20:
        return None
    return centroids[largest]   # (x, y) in tile pixels


def last_frame(episode_dir):
    frames = sorted(glob.glob(os.path.join(episode_dir, "**", "*.png"), recursive=True))
    return frames[-1] if frames else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", required=True)
    parser.add_argument("--apply", action="store_true", help="rewrite summary.csv")
    args = parser.parse_args()

    summary_path = os.path.join(args.data, "summary.csv")
    with open(summary_path, newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames
        rows = list(reader)

    centroids = {}
    for row in rows:
        episode_dir = os.path.join(args.data, f"episode_{int(row['episode_id']):05d}")
        frame = last_frame(episode_dir)
        centroids[row["episode_id"]] = block_centroid(frame) if frame else None

    # --- calibrate on measured landings ---
    calib = {}
    for bin_name in ("blue_bin", "red_bin"):
        points = np.array([centroids[r["episode_id"]] for r in rows
                           if r["landed_bin"] == bin_name and centroids[r["episode_id"]] is not None])
        calib[bin_name] = (points.mean(axis=0), max(4.0 * points.std(axis=0).max(), 10.0), len(points))
        print(f"{bin_name}: n={len(points)} centre=({calib[bin_name][0][0]:.1f},"
              f"{calib[bin_name][0][1]:.1f}) px  accept radius {calib[bin_name][1]:.1f} px")

    def classify(point):
        if point is None:
            return "unknown"
        best, best_dist = "unknown", float("inf")
        for bin_name, (centre, radius, _) in calib.items():
            dist = float(np.linalg.norm(point - centre))
            if dist <= radius and dist < best_dist:
                best, best_dist = bin_name, dist
        return best

    # --- self-test on measured episodes ---
    measured = [r for r in rows if r["landed_bin"] in ("blue_bin", "red_bin")]
    agree = sum(classify(centroids[r["episode_id"]]) == r["landed_bin"] for r in measured)
    print(f"self-test: {agree}/{len(measured)} measured landings reproduced from frames")

    # --- recover unknowns ---
    log_rows = []
    for row in rows:
        if row["landed_bin"] != "unknown":
            continue
        inferred = classify(centroids[row["episode_id"]])
        ok = inferred == row["target_bin"]
        print(f"ep {row['episode_id']:>3}: target {row['target_bin']:<8} frame says {inferred:<8} "
              f"{'OK' if ok else 'FAIL'}")
        log_rows.append({"episode_id": row["episode_id"], "inferred_bin": inferred,
                         "target_bin": row["target_bin"], "success": int(ok)})
        if args.apply and inferred != "unknown":
            row["landed_bin"] = inferred
            row["success"] = "1" if ok else "0"

    if args.apply:
        shutil.copy(summary_path, os.path.join(args.data, "summary_before_verify.csv"))
        with open(summary_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        with open(os.path.join(args.data, "landing_verification.csv"), "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["episode_id", "inferred_bin", "target_bin", "success"])
            writer.writeheader()
            writer.writerows(log_rows)
        print("summary.csv updated (backup: summary_before_verify.csv)")


if __name__ == "__main__":
    main()