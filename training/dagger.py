"""
DAgger — Dataset Aggregation (Ross et al. 2011) — for the maze robot.

Plain behavior cloning fails from covariate shift: the learner drifts a few
centimeters off the expert's states, sees inputs it was never trained on,
errs harder, and the errors compound until the robot spins or wedges. DAgger
fixes exactly this: roll out the CURRENT LEARNER (so we visit the learner's
own mistake states), label every visited state with the EXPERT's action, add
those rows to the dataset, retrain, repeat.

Each iteration mixes actions: expert with probability beta (decayed each
round), learner otherwise. Safeguards are DISABLED during rollouts so the
raw policy's true state distribution is what gets labeled.

Usage:
    python -m training.dagger \
        --track configs/tracks/COGS_300_Tournament_Track/COGS_300_Tournament_Track_v03.yaml \
        --base-data data/bc_train.csv --iters 3 --episodes-per-iter 40 \
        --out models/policy_dagger.npz
"""

import argparse
import csv
import json
import math
import os
import shutil
import subprocess
import sys

import numpy as np

from track import load_track
from robot_config import load_robot_config
from physics import load_physics_params
from expert_policy import build_route, RouteFollower, CMD_STOP
from policy_runtime import FeatureBuilder, MLPPolicy
from sim_env import SimEnv


def rollout_with_expert_labels(env, route, grid, model, fb: FeatureBuilder,
                               beta: float, seed: int, decision_hz: float,
                               max_time_s: float = 120.0):
    """
    Roll out the mixed policy; label every visited state with the expert.
    Feature flow per decision step (must match generate_data_v2 exactly):
        feats(t) built from obs(t) + history  →  row = (feats, expert_cmd)
        executed cmd = expert w.p. beta else learner  →  history update  →  env.step
    """
    obs = env.reset(seed=seed)
    follower = RouteFollower(route, grid=grid)
    fb.reset()
    rng = np.random.default_rng(seed)

    rows = []
    stall = 0
    stop_frames = 0
    reached = False
    for _ in range(int(max_time_s * decision_hz)):
        feats = fb.build(obs)
        expert_cmd = follower.command(obs["true_x"], obs["true_y"], obs["true_heading"])
        learner_cmd = int(np.argmax(model.predict_proba(feats)))
        cmd = expert_cmd if rng.random() < beta else learner_cmd

        rows.append(list(map(float, feats)) + [int(expert_cmd)])
        fb.observe_command(cmd)

        prev = (obs["true_x"], obs["true_y"])
        obs = env.step(cmd)

        if env.dist_to_goal() < 5.0:
            reached = True
            break
        if expert_cmd == CMD_STOP:
            stop_frames += 1
            if stop_frames > 5:
                break
        moved = math.hypot(obs["true_x"] - prev[0], obs["true_y"] - prev[1])
        if cmd != CMD_STOP and moved < 0.05:
            stall += 1
            if stall > int(6 * decision_hz):
                break
        else:
            stall = 0
    return rows, reached


def main():
    ap = argparse.ArgumentParser(description="DAgger training loop")
    ap.add_argument("--track", required=True)
    ap.add_argument("--robot", default="configs/robot-config.yaml")
    ap.add_argument("--physics", default="configs/physics.yaml")
    ap.add_argument("--base-data", required=True,
                    help="Initial BC dataset CSV (from generate_data_v2.py)")
    ap.add_argument("--iters", type=int, default=3)
    ap.add_argument("--episodes-per-iter", type=int, default=40)
    ap.add_argument("--beta0", type=float, default=0.3,
                    help="Initial expert-action mixing probability")
    ap.add_argument("--randomization", default="mild")
    ap.add_argument("--decision-hz", type=float, default=10.0)
    ap.add_argument("--out", default="models/policy_dagger.npz")
    ap.add_argument("--workdir", default="data/dagger")
    ap.add_argument("--seed", type=int, default=100000)
    args = ap.parse_args()

    os.makedirs(args.workdir, exist_ok=True)
    schema_src = args.base_data.replace(".csv", "_schema.json")

    track = load_track(args.track)
    robot = load_robot_config(args.robot)
    params, _ = load_physics_params(args.physics)
    route, info = build_route(track)
    grid = info["grid"]
    env = SimEnv(track, robot, params, decision_hz=args.decision_hz,
                 randomization=args.randomization)

    agg_csv = os.path.join(args.workdir, "aggregate.csv")
    shutil.copyfile(args.base_data, agg_csv)
    shutil.copyfile(schema_src, agg_csv.replace(".csv", "_schema.json"))
    with open(agg_csv) as f:
        next_episode_id = max(int(r.split(",")[-1]) for i, r in enumerate(f) if i > 0) + 1

    model_path = os.path.join(args.workdir, "policy_iter0.npz")
    print(f"== iter 0: train on base data ==")
    subprocess.run([sys.executable, "-m", "training.train",
                    "--data", agg_csv, "--out", model_path], check=True)

    fb = FeatureBuilder(n_us=len(robot.sensors), n_ir=max(2, len(robot.ir_sensors)))

    for it in range(1, args.iters + 1):
        beta = args.beta0 * (0.5 ** (it - 1))
        model = MLPPolicy.load(model_path)
        print(f"\n== iter {it}: rollouts (beta={beta:.2f}) ==")
        new_rows = []
        wins = 0
        for ep in range(args.episodes_per_iter):
            rows, reached = rollout_with_expert_labels(
                env, route, grid, model, fb, beta,
                seed=args.seed + it * 1000 + ep, decision_hz=args.decision_hz)
            wins += reached
            new_rows.append(rows)
        n_new = sum(len(r) for r in new_rows)
        print(f"  learner rollouts: {wins}/{args.episodes_per_iter} reached goal, "
              f"{n_new} new labeled states")

        with open(agg_csv, "a", newline="") as f:
            writer = csv.writer(f)
            for rows in new_rows:
                for row in rows:
                    writer.writerow([f"{v:.5f}" for v in row[:-1]]
                                    + [int(row[-1]), next_episode_id])
                next_episode_id += 1

        model_path = os.path.join(args.workdir, f"policy_iter{it}.npz")
        print(f"== iter {it}: retrain on aggregate ==")
        subprocess.run([sys.executable, "-m", "training.train",
                        "--data", agg_csv, "--out", model_path], check=True)

    shutil.copyfile(model_path, args.out)
    print(f"\nfinal model: {args.out}")


if __name__ == "__main__":
    main()
