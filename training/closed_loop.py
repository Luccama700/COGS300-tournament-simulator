"""
Parallel closed-loop evaluation — the ONLY metric that selects checkpoints.

Validation accuracy provably does not rank driving policies in this project
(92% balanced val acc coexists with 0/20 closed loop — see
docs/lessons/closed-loop-is-the-only-score.md). This module makes closed-loop
scoring cheap enough to run inside training loops: it fans episodes out over
worker processes and returns success rate plus median closest-approach.

Seed discipline: selection/iteration seeds must stay disjoint from the final
claim seeds (31000+). Defaults here use 20000+ for selection.

Library use (training/train_gru.py, training/dagger*.py):
    from training.closed_loop import run_closed_loop
    result = run_closed_loop(policy_path, track, robot, physics,
                             episodes=10, seed=20000, workers=4)

CLI:
    python -m training.closed_loop --policy models/x.npz --episodes 10
"""

import argparse
import json
from multiprocessing import Pool

TRACK_DEFAULT = "configs/tracks/COGS_300_Tournament_Track/COGS_300_Tournament_Track_v03.yaml"
ROBOT_DEFAULT = "configs/robot-config-frontIR.yaml"
PHYS_DEFAULT = "configs/physics-slow.yaml"


def _eval_chunk(job):
    (track, robot, phys, policy_path, episodes, seed, randomization,
     max_time, safeguards) = job
    from evaluate_policy import evaluate
    summary, stats, _ = evaluate(
        track, robot, phys, policy_path, episodes=episodes,
        randomization=randomization, max_time_s=max_time, seed=seed,
        safeguards=safeguards, verbose=False)
    return {
        "success": int(round(summary["success_rate"] * episodes)),
        "episodes": episodes,
        "stuck": summary["stuck"], "timeout": summary["timeout"],
        "lost": summary["lost"],
        "spin_events": summary["spin_events_total"],
        "min_dists": stats["min_dists"],
        "times": stats["times"],
    }


def run_closed_loop(policy_path: str, track: str = TRACK_DEFAULT,
                    robot: str = ROBOT_DEFAULT, phys: str = PHYS_DEFAULT,
                    episodes: int = 10, seed: int = 20000,
                    randomization: str = "mild", max_time: float = 240.0,
                    safeguards: bool = True, workers: int = 4) -> dict:
    """Evaluate a saved policy over `episodes` seeds; return aggregate dict."""
    workers = max(1, min(workers, episodes))
    base, extra = divmod(episodes, workers)
    jobs = []
    s = seed
    for w in range(workers):
        n = base + (1 if w < extra else 0)
        if n == 0:
            continue
        jobs.append((track, robot, phys, policy_path, n, s, randomization,
                     max_time, safeguards))
        s += n
    if workers == 1:
        results = [_eval_chunk(jobs[0])]
    else:
        with Pool(len(jobs)) as pool:
            results = pool.map(_eval_chunk, jobs)

    min_dists = sorted(d for r in results for d in r["min_dists"])
    times = [t for r in results for t in r["times"]]
    n = sum(r["episodes"] for r in results)
    succ = sum(r["success"] for r in results)
    mid = min_dists[len(min_dists) // 2] if min_dists else float("nan")
    return {
        "episodes": n,
        "success": succ,
        "success_rate": succ / n,
        "median_min_goal_dist": mid,
        "stuck": sum(r["stuck"] for r in results),
        "timeout": sum(r["timeout"] for r in results),
        "lost": sum(r["lost"] for r in results),
        "spin_events": sum(r["spin_events"] for r in results),
        "avg_time_s": (sum(times) / len(times)) if times else None,
        "seed": seed,
        "randomization": randomization,
        "safeguards": safeguards,
    }


def score_key(result: dict):
    """Sort key: more successes first, then smaller closest-approach."""
    return (result["success"], -result["median_min_goal_dist"])


def main():
    ap = argparse.ArgumentParser(description="Parallel closed-loop eval")
    ap.add_argument("--policy", required=True)
    ap.add_argument("--track", default=TRACK_DEFAULT)
    ap.add_argument("--robot", default=ROBOT_DEFAULT)
    ap.add_argument("--physics", default=PHYS_DEFAULT)
    ap.add_argument("--episodes", type=int, default=10)
    ap.add_argument("--seed", type=int, default=20000)
    ap.add_argument("--randomization", default="mild")
    ap.add_argument("--max-time", type=float, default=240.0)
    ap.add_argument("--no-safeguards", action="store_true")
    ap.add_argument("--workers", type=int, default=4)
    args = ap.parse_args()

    res = run_closed_loop(args.policy, args.track, args.robot, args.physics,
                          episodes=args.episodes, seed=args.seed,
                          randomization=args.randomization,
                          max_time=args.max_time,
                          safeguards=not args.no_safeguards,
                          workers=args.workers)
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
