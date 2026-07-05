"""
Closed-loop policy evaluation in the real physics simulator.

This is the metric that matters — per-frame label accuracy says almost
nothing about closed-loop driving (compounding errors dominate). Runs N
randomized episodes and reports success rate, time, wall contacts, spin
events, and where failures ended.

Evaluate a trained model:
    python evaluate_policy.py --track configs/tracks/.../v03.yaml \
        --policy models/policy.npz --episodes 20 --randomization mild

Evaluate the privileged expert (upper bound / sanity gate):
    python evaluate_policy.py --track ... --policy expert

Plot trajectories:
    ... --plot eval.png
"""

import argparse
import json
import math

import numpy as np

from track import load_track
from robot_config import load_robot_config
from physics import load_physics_params
from expert_policy import build_route, make_follower, plot_route, CMD_STOP
from policy_runtime import load_policy, PolicyRuntime
from sim_env import SimEnv, RANDOMIZATION_PRESETS


def evaluate(track_path: str, robot_path: str, physics_path: str,
             policy_path: str, episodes: int = 20, randomization: str = "mild",
             decision_hz: float = 10.0, max_time_s: float = 150.0,
             seed: int = 5000, safeguards: bool = True,
             collect_traj: bool = False, verbose: bool = True):
    track = load_track(track_path)
    robot = load_robot_config(robot_path)
    params, _ = load_physics_params(physics_path)
    env = SimEnv(track, robot, params, decision_hz=decision_hz,
                 randomization=randomization)

    expert_mode = policy_path == "expert"
    if expert_mode:
        route, info = build_route(track)
    else:
        model = load_policy(policy_path)
        runtime = PolicyRuntime(model, decision_hz=decision_hz, enabled=safeguards)

    stats = {"success": 0, "times": [], "contacts": [], "spins": 0,
             "stuck": 0, "timeout": 0, "lost": 0, "final_dists": [],
             "min_dists": [], "trajs": []}
    max_steps = int(max_time_s * decision_hz)

    for ep in range(episodes):
        obs = env.reset(seed=seed + ep)
        if expert_mode:
            actor = make_follower(route, info, decision_hz=decision_hz,
                                  robot_cfg=robot, physics_params=params)
        else:
            runtime.reset()
        traj = []
        stall = 0
        outcome = "timeout"
        min_goal_dist = env.dist_to_goal()
        for _ in range(max_steps):
            if expert_mode:
                cmd = actor.command(obs["true_x"], obs["true_y"], obs["true_heading"],
                                    ir=obs["ir"])
            else:
                cmd = runtime.step(obs)
            prev = (obs["true_x"], obs["true_y"])
            obs = env.step(cmd)
            if collect_traj:
                traj.append((obs["true_x"], obs["true_y"]))
            min_goal_dist = min(min_goal_dist, env.dist_to_goal())
            if env.dist_to_goal() < 5.0:
                outcome = "success"
                break
            if env.dist_to_goal() > 1500.0:        # left the arena entirely
                outcome = "lost"
                break
            moved = math.hypot(obs["true_x"] - prev[0], obs["true_y"] - prev[1])
            if cmd != CMD_STOP and moved < 0.05:
                stall += 1
                if stall > int(6 * decision_hz):   # 6s hard-stuck → give up
                    outcome = "stuck"
                    break
            else:
                stall = 0

        ok = outcome == "success"
        stats["success"] += ok
        stats["timeout"] += outcome == "timeout"
        stats["stuck"] += outcome == "stuck"
        stats["lost"] += outcome == "lost"
        if ok:
            stats["times"].append(obs["elapsed"])
        stats["contacts"].append(obs["collided_total"])
        stats["final_dists"].append(env.dist_to_goal())
        stats["min_dists"].append(min_goal_dist)
        if not expert_mode:
            stats["spins"] += runtime.spin_events
        if collect_traj:
            stats["trajs"].append((traj, ok))
        if verbose:
            extra = "" if expert_mode else f"  spins={runtime.spin_events}"
            print(f"  ep{ep:02d}: {outcome:8s} t={obs['elapsed']:6.1f}s "
                  f"dist_to_goal={env.dist_to_goal():6.1f} contacts={obs['collided_total']:5d}{extra}")

    n = episodes
    summary = {
        "success_rate": stats["success"] / n,
        "avg_time_s": float(np.mean(stats["times"])) if stats["times"] else None,
        "avg_contact_frames": float(np.mean(stats["contacts"])),
        "stuck": stats["stuck"], "timeout": stats["timeout"], "lost": stats["lost"],
        "spin_events_total": stats["spins"],
        "median_final_dist": float(np.median(stats["final_dists"])),
        "median_min_goal_dist": float(np.median(stats["min_dists"])),
    }
    return summary, stats, track


def main():
    ap = argparse.ArgumentParser(description="Closed-loop policy evaluation")
    ap.add_argument("--track", required=True)
    ap.add_argument("--robot", default="configs/robot-config.yaml")
    ap.add_argument("--physics", default="configs/physics.yaml")
    ap.add_argument("--policy", required=True, help="Path to .npz model, or 'expert'")
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--randomization", default="mild",
                    choices=list(RANDOMIZATION_PRESETS.keys()))
    ap.add_argument("--decision-hz", type=float, default=10.0)
    ap.add_argument("--max-time", type=float, default=150.0)
    ap.add_argument("--seed", type=int, default=5000)
    ap.add_argument("--no-safeguards", action="store_true",
                    help="Raw argmax output (no hysteresis/watchdog/reflex)")
    ap.add_argument("--plot", default=None, help="Save trajectory plot PNG")
    ap.add_argument("--json-out", default=None, help="Write summary JSON here")
    args = ap.parse_args()

    summary, stats, track = evaluate(
        args.track, args.robot, args.physics, args.policy,
        episodes=args.episodes, randomization=args.randomization,
        decision_hz=args.decision_hz, max_time_s=args.max_time,
        seed=args.seed, safeguards=not args.no_safeguards,
        collect_traj=args.plot is not None)

    print("\n== summary ==")
    for k, v in summary.items():
        print(f"  {k}: {v}")
    if args.plot:
        plot_route(track, None, None, args.plot, trajectories=stats["trajs"])
    if args.json_out:
        with open(args.json_out, "w") as f:
            json.dump(summary, f, indent=2)


if __name__ == "__main__":
    main()
