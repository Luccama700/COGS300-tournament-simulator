"""
DAgger for the GRU policy — with the three loop repairs the first DAgger
attempt paid for (docs/lessons/dagger-keep-last-collapse.md):

  1. GUARDS ON during rollouts. The deployed policy runs behind
     PolicyRuntime's guards; labeling a guards-off state distribution
     corrects for states deployment never visits. Rollouts here run the
     exact deployment stack, with expert actions mixed in at prob beta.
  2. KEEP-BEST, not keep-last. Every retrain is scored closed-loop
     (training/closed_loop.py) and the globally best checkpoint is the
     output. A late collapsed iteration cannot overwrite an earlier win.
  3. CONTROLLED base:recovery ratio. Recovery (rollout-labeled) rows are
     capped at a fraction of the aggregate so late iterations don't drown
     the clean demonstrations in flailing states.
  4. EXPERT-PREFIX coverage. A weak learner's rollouts never reach the late
     course, so DAgger never collects corrections there (measured: beta
     0.25 -> 11/50 rollouts complete; beta 0.125 -> 0/50, i.e. zero
     late-course coverage). For a fraction of episodes the expert drives to
     a random arc while the runtime OBSERVES (features + hidden state
     advanced on executed commands — hidden-state-consistent, see
     docs/lessons/segment-eval-needs-warm-hidden-state.md), then the mixed
     policy continues. Prefix rows are recorded too (plain BC rows), so
     training sequences stay contiguous from the episode start.

Also GRU-specific: the learner carries hidden state; rollouts reset it per
episode and step it exactly like deployment (PolicyRuntime.reset()/step()).

Usage:
    python -m training.dagger_gru --base-data data/train.csv \
        --init models/policy_gru_v1.npz --iters 4 --episodes-per-iter 50 \
        --out models/policy_gru_dagger.npz
"""

import argparse
import json
import math
import os
from multiprocessing import Pool

import numpy as np

from track import load_track
from robot_config import load_robot_config
from physics import load_physics_params
from expert_policy import build_route, make_follower, CMD_STOP
from policy_runtime import load_policy, PolicyRuntime
from sim_env import SimEnv
from training.closed_loop import (run_closed_loop, score_key,
                                  TRACK_DEFAULT, ROBOT_DEFAULT, PHYS_DEFAULT)
from training.train_gru import load_sequences, train_gru_core


def _rollout_chunk(job):
    """Roll out the guarded learner with beta-mixed expert actions; label
    every visited state with the expert. Returns per-episode (X, y, info)."""
    (track_path, robot_path, phys_path, model_path, seeds, beta,
     randomization, max_time, decision_hz, prefix_prob) = job
    track = load_track(track_path)
    robot = load_robot_config(robot_path)
    params, _ = load_physics_params(phys_path)
    route, info = build_route(track)
    env = SimEnv(track, robot, params, decision_hz=decision_hz,
                 randomization=randomization)
    runtime = PolicyRuntime(load_policy(model_path), decision_hz=decision_hz,
                            enabled=True)          # repair #1: guards ON

    out = []
    for seed in seeds:
        obs = env.reset(seed=seed)
        follower = make_follower(route, info, decision_hz=decision_hz,
                                 robot_cfg=robot, physics_params=params)
        runtime.reset()
        rng = np.random.default_rng(seed)
        # Repair #4: expert-driven prefix to a random arc for coverage
        handover = (rng.uniform(0.15, 0.9) * follower.total
                    if rng.random() < prefix_prob else 0.0)
        X, y = [], []
        stall = 0
        stop_frames = 0
        reached = False
        for _ in range(int(max_time * decision_hz)):
            # Deployment-exact inference path, with mixing injected between
            # the guard and the commit (PolicyRuntime.step inlined):
            feats = runtime.fb.build(obs)
            probs = runtime.policy.predict_proba(feats)
            guarded = runtime._guard(obs, probs)
            expert_cmd = follower.command(obs["true_x"], obs["true_y"],
                                          obs["true_heading"], ir=obs["ir"])
            if follower.progress < handover:
                exec_cmd = expert_cmd            # expert-driven prefix
            else:
                exec_cmd = expert_cmd if rng.random() < beta else guarded
            runtime._cmd = exec_cmd
            runtime.fb.observe_command(exec_cmd)

            X.append(feats)
            y.append(int(expert_cmd))

            prev = (obs["true_x"], obs["true_y"])
            obs = env.step(exec_cmd)
            if env.dist_to_goal() < 5.0:
                reached = True
                break
            if env.dist_to_goal() > 1500.0:
                break
            if expert_cmd == CMD_STOP:
                stop_frames += 1
                if stop_frames > 5:
                    break
            else:
                stop_frames = 0
            moved = math.hypot(obs["true_x"] - prev[0], obs["true_y"] - prev[1])
            if exec_cmd != CMD_STOP and moved < 0.05:
                stall += 1
                if stall > int(6 * decision_hz):
                    break
            else:
                stall = 0
        out.append((np.asarray(X, dtype=np.float32),
                    np.asarray(y, dtype=np.int64),
                    {"reached": reached, "spins": runtime.spin_events,
                     "steps": len(y)}))
    return out


def main():
    ap = argparse.ArgumentParser(description="DAgger loop for the GRU policy")
    ap.add_argument("--base-data", default="data/train.csv")
    ap.add_argument("--init", default=None,
                    help="Pretrained .npz to drive iter-1 rollouts "
                         "(skips the iter-0 retrain)")
    ap.add_argument("--iters", type=int, default=4)
    ap.add_argument("--episodes-per-iter", type=int, default=50)
    ap.add_argument("--beta0", type=float, default=0.2)
    ap.add_argument("--beta-decay", type=float, default=0.7)
    ap.add_argument("--prefix-prob", type=float, default=0.5,
                    help="Fraction of rollouts that start with an expert-"
                         "driven prefix to a random arc (repair #4)")
    ap.add_argument("--recovery-frac", type=float, default=0.4,
                    help="Max fraction of training rows from rollouts "
                         "(repair #3: base:recovery ratio)")
    ap.add_argument("--randomization", default="mild")
    ap.add_argument("--max-time", type=float, default=150.0)
    ap.add_argument("--decision-hz", type=float, default=10.0)
    ap.add_argument("--rollout-workers", type=int, default=4)
    ap.add_argument("--out", default="models/policy_gru_dagger.npz")
    ap.add_argument("--workdir", default="data/dagger_gru")
    ap.add_argument("--seed", type=int, default=100000)
    ap.add_argument("--hidden", type=int, default=96)
    ap.add_argument("--epochs", type=int, default=24)
    ap.add_argument("--select-episodes", type=int, default=10)
    ap.add_argument("--select-seed", type=int, default=20000)
    ap.add_argument("--track", default=TRACK_DEFAULT)
    ap.add_argument("--robot", default=ROBOT_DEFAULT)
    ap.add_argument("--physics", default=PHYS_DEFAULT)
    args = ap.parse_args()

    os.makedirs(args.workdir, exist_ok=True)
    with open(args.base_data.replace(".csv", "_schema.json")) as f:
        schema = json.load(f)

    print(f"loading base data {args.base_data} ...")
    base_eps, names = load_sequences(args.base_data)
    assert names == schema["feature_names"]
    base_rows = sum(len(y) for _, y in base_eps)
    print(f"  {len(base_eps)} base episodes, {base_rows} rows")

    rng = np.random.default_rng(args.seed)
    recovery_eps: list = []          # [(X, y)] across all iterations
    log: list = []

    model_path = args.init
    if model_path is None:
        print("== iter 0: train on base data ==")
        _, model_path, res0, _ = train_gru_core(
            base_eps, schema, os.path.join(args.workdir, "ckpt_it0"),
            hidden=args.hidden, epochs=args.epochs, seed=args.seed,
            select_episodes=args.select_episodes, select_seed=args.select_seed,
            track=args.track, robot=args.robot, phys=args.physics,
            tag=" it0 ")
        best_global = (score_key(res0), model_path, res0, 0)
    else:
        print(f"== iter 0: scoring init model {model_path} ==")
        res0 = run_closed_loop(model_path, args.track, args.robot,
                               args.physics, episodes=args.select_episodes,
                               seed=args.select_seed,
                               randomization=args.randomization)
        best_global = (score_key(res0), model_path, res0, 0)
    arc0 = (100.0 * res0["median_max_arc"] / res0["route_total"]
            if res0.get("route_total") else 0.0)
    print(f"  init closed-loop: {res0['success']}/{res0['episodes']} "
          f"arc={arc0:.0f}% median_min={res0['median_min_goal_dist']:.0f}cm")
    log.append({"iter": 0, "model": model_path, **res0})

    for it in range(1, args.iters + 1):
        beta = args.beta0 * (args.beta_decay ** (it - 1))
        print(f"\n== iter {it}: rollouts (beta={beta:.3f}, guards ON, "
              f"model={os.path.basename(model_path)}) ==", flush=True)
        seeds = [args.seed + it * 1000 + e for e in range(args.episodes_per_iter)]
        w = max(1, min(args.rollout_workers, len(seeds)))
        jobs = [(args.track, args.robot, args.physics, model_path,
                 seeds[i::w], beta, args.randomization, args.max_time,
                 args.decision_hz, args.prefix_prob) for i in range(w)]
        if w == 1:
            chunks = [_rollout_chunk(jobs[0])]
        else:
            with Pool(w) as pool:
                chunks = pool.map(_rollout_chunk, jobs)
        new = [ep for ch in chunks for ep in ch]
        wins = sum(1 for _, _, i in new if i["reached"])
        spins = sum(i["spins"] for _, _, i in new)
        n_new = sum(i["steps"] for _, _, i in new)
        print(f"  rollouts: {wins}/{len(new)} reached goal, {n_new} new rows, "
              f"{spins} spin events")
        recovery_eps.extend((X, y) for X, y, _ in new if len(y) > 0)

        # Repair #3: cap recovery rows at recovery_frac of the training mix
        budget = int(base_rows * args.recovery_frac / (1.0 - args.recovery_frac))
        pool_idx = rng.permutation(len(recovery_eps))
        picked, acc = [], 0
        for i in pool_idx:
            n = len(recovery_eps[i][1])
            if acc + n > budget and picked:
                continue
            picked.append(i)
            acc += n
        train_eps = base_eps + [recovery_eps[i] for i in picked]
        print(f"  training mix: {base_rows} base + {acc} recovery rows "
              f"({len(picked)}/{len(recovery_eps)} recovery episodes)")

        print(f"== iter {it}: retrain ==", flush=True)
        key, path, res, _ = train_gru_core(
            train_eps, schema, os.path.join(args.workdir, f"ckpt_it{it}"),
            hidden=args.hidden, epochs=args.epochs, seed=args.seed + it,
            select_episodes=args.select_episodes, select_seed=args.select_seed,
            track=args.track, robot=args.robot, phys=args.physics,
            tag=f" it{it} ")
        log.append({"iter": it, "beta": beta, "rollout_wins": wins,
                    "rollout_eps": len(new), "model": path, **res})
        # Repair #2: keep-best (the winner drives the NEXT rollouts too)
        if key > best_global[0]:
            best_global = (key, path, res, it)
            arcb = (100.0 * res["median_max_arc"] / res["route_total"]
                    if res.get("route_total") else 0.0)
            print(f"  ** new global best (iter {it}): "
                  f"{res['success']}/{res['episodes']} arc={arcb:.0f}% "
                  f"median_min={res['median_min_goal_dist']:.0f}cm")
        model_path = best_global[1]

    _, path, res, at = best_global
    data = np.load(path, allow_pickle=False)
    m = json.loads(str(data["meta_json"]))
    m["dagger"] = {"best_iter": at, "selection": {
        k: res[k] for k in ("success", "episodes", "median_min_goal_dist",
                            "seed", "randomization")}}
    np.savez(args.out, **{k: data[k] for k in data.files if k != "meta_json"},
             meta_json=json.dumps(m))
    with open(os.path.join(args.workdir, "dagger_log.json"), "w") as f:
        json.dump(log, f, indent=2)
    print(f"\nglobal best: iter {at} -- {res['success']}/{res['episodes']}, "
          f"median_min={res['median_min_goal_dist']:.0f}cm -> {args.out}")


if __name__ == "__main__":
    main()
