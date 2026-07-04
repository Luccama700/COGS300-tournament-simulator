"""
Training-data generation v2 — supersedes generate_line_data.py.

Differences from the old generator (each fixed a failure of the trained models):
  * The expert drives the REAL physics engine (physics.py) with the REAL six
    firmware commands, so every label is an action the robot can execute and
    the dynamics match the simulator the model is later evaluated in.
    (The old one used bespoke kinematics with continuous steering and then
    discretized — and its expert could never reach the goal, so 90% of every
    episode was STOP rows.)
  * Sensor models are IDENTICAL to drive/eval mode (same SensorSimulator and
    IRSensorSimulator, including dropouts) — no train/test sensor gap.
  * Features come from policy_runtime.FeatureBuilder — the same code used at
    inference — and contain no absolute pose / heading / time (the old
    memorization features).
  * Episodes are domain-randomized (motor strength/asymmetry, sensor noise,
    start pose, odometry error) so the model sees recovery states, not just
    one perfect trajectory.

Usage:
    python generate_data_v2.py \
        --track configs/tracks/COGS_300_Tournament_Track/COGS_300_Tournament_Track_v03.yaml \
        --episodes 150 --randomization mild --output data/bc_train.csv --workers 8
"""

import argparse
import csv
import json
import math
import os
from multiprocessing import Pool

from track import load_track
from robot_config import load_robot_config
from physics import load_physics_params
from expert_policy import build_route, make_follower, CMD_STOP
from policy_runtime import FeatureBuilder
from sim_env import SimEnv, RANDOMIZATION_PRESETS


def run_episode(env: SimEnv, route, info, fb: FeatureBuilder,
                seed: int, max_time_s: float = 150.0,
                stop_frames_cap: int = 6):
    """
    Run one expert episode. Returns (rows, metrics) where each row is
    [features..., command_label]. Rows use the feature vector observed at
    decision time t and the expert's command for time t.
    """
    obs = env.reset(seed=seed)
    follower = make_follower(route, info, decision_hz=env.decision_hz,
                             robot_cfg=env.robot_cfg)
    fb.reset()

    rows = []
    stop_frames = 0
    max_steps = int(max_time_s * env.decision_hz)
    reached = False
    for _ in range(max_steps):
        feats = fb.build(obs)
        cmd = follower.command(obs["true_x"], obs["true_y"], obs["true_heading"],
                               ir=obs["ir"])
        rows.append(list(map(float, feats)) + [int(cmd)])
        fb.observe_command(cmd)
        obs = env.step(cmd)

        if cmd == CMD_STOP:
            stop_frames += 1
            if stop_frames >= stop_frames_cap:
                break
        else:
            stop_frames = 0
        if env.dist_to_goal() < 5.0 and follower.done:
            reached = True
            break
    if env.dist_to_goal() < 5.0:
        reached = True

    metrics = {
        "seed": seed,
        "reached_goal": reached,
        "steps": len(rows),
        "time_s": round(obs["elapsed"], 2),
        "contact_frames": obs["collided_total"],
        "recoveries": follower._recover_count,
        "line_fallback_frames": follower.fallback_frames,
        "final_dist_to_goal": round(env.dist_to_goal(), 1),
    }
    return rows, metrics


def _worker(job):
    """Multiprocessing entry: build everything locally, run a chunk of episodes."""
    (track_path, robot_path, physics_path, randomization,
     decision_hz, seeds) = job
    track = load_track(track_path)
    robot = load_robot_config(robot_path)
    params, _ = load_physics_params(physics_path)
    route, info = build_route(track)
    env = SimEnv(track, robot, params, decision_hz=decision_hz,
                 randomization=randomization)
    fb = FeatureBuilder(n_us=len(robot.sensors), n_ir=max(2, len(robot.ir_sensors)))
    out = []
    for seed in seeds:
        out.append(run_episode(env, route, info, fb, seed))
    return out


def main():
    ap = argparse.ArgumentParser(description="Generate BC training data (v2)")
    ap.add_argument("--track", required=True)
    ap.add_argument("--robot", default="configs/robot-config.yaml")
    ap.add_argument("--physics", default="configs/physics.yaml")
    ap.add_argument("--output", default="data/bc_train.csv")
    ap.add_argument("--episodes", type=int, default=150)
    ap.add_argument("--randomization", default="mild",
                    choices=list(RANDOMIZATION_PRESETS.keys()))
    ap.add_argument("--decision-hz", type=float, default=10.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--workers", type=int, default=1)
    args = ap.parse_args()

    robot = load_robot_config(args.robot)
    fb = FeatureBuilder(n_us=len(robot.sensors), n_ir=max(2, len(robot.ir_sensors)))
    feat_names = fb.feature_names

    seeds = [args.seed + i for i in range(args.episodes)]
    chunks = max(1, args.workers)
    jobs = [(args.track, args.robot, args.physics, args.randomization,
             args.decision_hz, seeds[i::chunks]) for i in range(chunks)]

    if args.workers > 1:
        with Pool(args.workers) as pool:
            results = pool.map(_worker, jobs)
    else:
        results = [_worker(j) for j in jobs]

    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    meta_path = args.output.replace(".csv", "_meta.json")
    schema_path = args.output.replace(".csv", "_schema.json")

    all_metrics = []
    total_rows = 0
    episode_id = 0
    with open(args.output, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(feat_names + ["command_label", "episode_id"])
        for chunk in results:
            for rows, metrics in chunk:
                metrics["id"] = episode_id
                all_metrics.append(metrics)
                for row in rows:
                    writer.writerow([f"{v:.5f}" for v in row[:-1]]
                                    + [int(row[-1]), episode_id])
                total_rows += len(rows)
                episode_id += 1

    reached = sum(1 for m in all_metrics if m["reached_goal"])
    label_counts = {}
    for chunk in results:
        for rows, _ in chunk:
            for row in rows:
                label_counts[int(row[-1])] = label_counts.get(int(row[-1]), 0) + 1

    with open(meta_path, "w") as f:
        json.dump({"episodes": all_metrics,
                   "total_rows": total_rows,
                   "reached_goal_count": reached,
                   "label_counts": label_counts}, f, indent=2)
    with open(schema_path, "w") as f:
        json.dump({"feature_names": feat_names,
                   "n_us": len(robot.sensors),
                   "n_ir": max(2, len(robot.ir_sensors)),
                   "decision_hz": args.decision_hz,
                   "randomization": args.randomization,
                   "track": args.track}, f, indent=2)

    names = {0: "FWD", 1: "SL_L", 2: "SL_R", 3: "HD_L", 4: "HD_R", 5: "STOP"}
    dist = "  ".join(f"{names[k]}:{label_counts.get(k,0)}" for k in range(6))
    print(f"\n{total_rows} rows, {args.episodes} episodes ({reached} reached goal)")
    print(f"labels: {dist}")
    print(f"  data:   {args.output}\n  meta:   {meta_path}\n  schema: {schema_path}")


if __name__ == "__main__":
    main()
