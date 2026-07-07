"""
Segment-start evaluation: spawn episodes mid-route to localize failures.

Full-course evals confound everything after the first weakness; starting
episodes at route arc-lengths (tape end, maze quarters) answers "WHERE does
it die" and gives fast iteration on the weak segment
(docs/AGENT_HANDOFF.md hypothesis #4).

Correctness trap handled here (docs/lessons/segment-eval-odometer-trap.md):
a teleported robot must also get a matching odometer. Training data at arc
s shows odom distance ~= f(s) (more than s: bang-bang weave and corrections
add arc length) and odom heading ~= true heading change since boot. Both
are measured from real expert episodes by calibrate() — including a sanity
check that odometry heading actually tracks true heading (sign convention).

Modes:
  # success per segment start (which section is weak?)  — teleport spawns.
  # VALID ONLY FOR FEEDFORWARD POLICIES: a recurrent policy spawned with
  # h=0 mid-course is off-manifold (docs/lessons/
  # segment-eval-needs-warm-hidden-state.md).
  python -m training.segment_eval --policy models/x.npz --episodes 6

  # recurrent-safe variant: the EXPERT drives to the segment arc while the
  # policy runtime observes (hidden state warmed on real history), then the
  # policy takes over.
  python -m training.segment_eval --policy models/x.npz --prefix --episodes 6

  # where do full-course runs die? (project failure points onto the route)
  python -m training.segment_eval --policy models/x.npz --localize --episodes 10
"""

import argparse
import math
from multiprocessing import Pool

import numpy as np

from track import load_track
from robot_config import load_robot_config
from physics import load_physics_params
from expert_policy import build_route, make_follower, CMD_STOP
from policy_runtime import load_policy, PolicyRuntime
from sim_env import SimEnv
from training.closed_loop import TRACK_DEFAULT, ROBOT_DEFAULT, PHYS_DEFAULT


# ── Route geometry ───────────────────────────────────────────────────────────

class RouteGeom:
    def __init__(self, route):
        self.pts = np.asarray(route, dtype=float)
        seg = self.pts[1:] - self.pts[:-1]
        self.seg_len = np.hypot(seg[:, 0], seg[:, 1])
        self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.total = float(self.cum[-1])

    def point_at(self, s: float):
        s = max(0.0, min(self.total, s))
        i = max(0, min(len(self.seg_len) - 1,
                       int(np.searchsorted(self.cum, s, side="right")) - 1))
        t = 0.0 if self.seg_len[i] < 1e-12 else (s - self.cum[i]) / self.seg_len[i]
        p = self.pts[i] + t * (self.pts[i + 1] - self.pts[i])
        return float(p[0]), float(p[1])

    def heading_at(self, s: float) -> float:
        """Tangent heading in the sim's screen convention (y inverted)."""
        a = self.point_at(max(0.0, s - 2.0))
        b = self.point_at(min(self.total, s + 6.0))
        return math.degrees(math.atan2(-(b[1] - a[1]), b[0] - a[0]))

    def project(self, x: float, y: float) -> float:
        """Global nearest-point arc length (post-hoc localization only)."""
        best_d, best_s = float("inf"), 0.0
        for i in range(len(self.seg_len)):
            x1, y1 = self.pts[i]
            dx, dy = self.pts[i + 1] - self.pts[i]
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / L2))
            px, py = x1 + t * dx, y1 + t * dy
            d = math.hypot(x - px, y - py)
            if d < best_d:
                best_d, best_s = d, self.cum[i] + t * self.seg_len[i]
        return best_s


def segment_starts(track, route, info, n_maze: int = 4):
    """Named spawn points: course start, tape end, and maze quarter points."""
    geom = RouteGeom(route)
    line_len = info.get("line_len") or 0.0
    segs = [("start", 0.0)]
    if line_len > 0:
        segs.append(("tape_end", min(line_len, geom.total - 10.0)))
    maze_len = geom.total - line_len
    for k in range(1, n_maze):
        segs.append((f"maze_{k}of{n_maze}", line_len + maze_len * k / n_maze))
    out = []
    for name, s in segs:
        x, y = geom.point_at(s)
        out.append({"name": name, "s": round(s, 1), "x": x, "y": y,
                    "heading": geom.heading_at(s)})
    return out, geom


# ── Odometry calibration from expert episodes ───────────────────────────────

def calibrate(track_path, robot_path, phys_path, arcs, episodes=3,
              randomization="mild", decision_hz=10.0):
    """
    Run expert episodes; for each requested arc s return the median odometer
    distance the robot shows when its route progress is ~s. Also verifies
    that odometry heading tracks true-heading change (sign convention guard).
    """
    track = load_track(track_path)
    robot = load_robot_config(robot_path)
    params, _ = load_physics_params(phys_path)
    route, info = build_route(track)
    env = SimEnv(track, robot, params, decision_hz=decision_hz,
                 randomization=randomization)
    samples = {round(s, 1): [] for s in arcs}
    heading_err = []
    for ep in range(episodes):
        obs = env.reset(seed=90000 + ep)
        follower = make_follower(route, info, decision_hz=decision_hz,
                                 robot_cfg=robot, physics_params=params)
        h0 = obs["true_heading"]
        for _ in range(int(200 * decision_hz)):
            cmd = follower.command(obs["true_x"], obs["true_y"],
                                   obs["true_heading"], ir=obs["ir"])
            obs = env.step(cmd)
            rel_true = (obs["true_heading"] - h0) % 360.0
            err = (obs["odom_heading"] - rel_true + 180.0) % 360.0 - 180.0
            heading_err.append(err)
            for s in samples:
                if abs(follower.progress - s) < 8.0:
                    samples[s].append(obs["distance"])
            if env.dist_to_goal() < 5.0:
                break
    med = {}
    for s, vals in samples.items():
        med[s] = float(np.median(vals)) if vals else s  # fallback: raw arc
    return med, float(np.median(np.abs(heading_err))), float(np.percentile(np.abs(heading_err), 95))


# ── Episode runner (policy or expert) from an arbitrary start ────────────────

def _run_episodes(job):
    (track_path, robot_path, phys_path, policy_path, seg, odom_dist,
     episodes, seed, randomization, max_time, safeguards, decision_hz,
     start_heading_ref) = job
    track = load_track(track_path)
    robot = load_robot_config(robot_path)
    params, _ = load_physics_params(phys_path)
    env = SimEnv(track, robot, params, decision_hz=decision_hz,
                 randomization=randomization)
    expert_mode = policy_path == "expert"
    if expert_mode:
        route, info = build_route(track)
    else:
        model = load_policy(policy_path)
        runtime = PolicyRuntime(model, decision_hz=decision_hz,
                                enabled=safeguards)
    rng = np.random.default_rng(seed * 7 + 13)

    res = {"success": 0, "outcomes": [], "min_dists": [], "end_pos": []}
    for ep in range(episodes):
        odom0 = (max(0.0, odom_dist * rng.normal(1.0, 0.02) + rng.normal(0, 3.0)),
                 ((seg["heading"] - start_heading_ref) + rng.normal(0, 4.0)))
        obs = env.reset(seed=seed + ep,
                        start_pose=(seg["x"], seg["y"], seg["heading"]),
                        odom_init=odom0 if seg["s"] > 1.0 else None)
        if expert_mode:
            actor = make_follower(route, info, decision_hz=decision_hz,
                                  robot_cfg=robot, physics_params=params)
            actor.progress = seg["s"]
            for ci, (cs, _, _) in enumerate(actor._corners):
                if cs < seg["s"] - 5.0:
                    actor._corners_done.add(ci)
        else:
            runtime.reset()
        stall = 0
        outcome = "timeout"
        min_d = env.dist_to_goal()
        for _ in range(int(max_time * decision_hz)):
            if expert_mode:
                cmd = actor.command(obs["true_x"], obs["true_y"],
                                    obs["true_heading"], ir=obs["ir"])
            else:
                cmd = runtime.step(obs)
            prev = (obs["true_x"], obs["true_y"])
            obs = env.step(cmd)
            min_d = min(min_d, env.dist_to_goal())
            if env.dist_to_goal() < 5.0:
                outcome = "success"
                break
            if env.dist_to_goal() > 1500.0:
                outcome = "lost"
                break
            moved = math.hypot(obs["true_x"] - prev[0], obs["true_y"] - prev[1])
            if cmd != CMD_STOP and moved < 0.05:
                stall += 1
                if stall > int(6 * decision_hz):
                    outcome = "stuck"
                    break
            else:
                stall = 0
        res["success"] += outcome == "success"
        res["outcomes"].append(outcome)
        res["min_dists"].append(min_d)
        res["end_pos"].append((obs["true_x"], obs["true_y"]))
    return seg["name"], res


def _run_prefix_episodes(job):
    """Expert drives to the segment arc with the policy runtime OBSERVING
    (features + hidden state advanced on the expert's executed commands);
    the policy takes over at the arc. History-consistent for recurrent nets."""
    (track_path, robot_path, phys_path, policy_path, seg, episodes, seed,
     randomization, max_time, safeguards, decision_hz) = job
    track = load_track(track_path)
    robot = load_robot_config(robot_path)
    params, _ = load_physics_params(phys_path)
    route, info = build_route(track)
    env = SimEnv(track, robot, params, decision_hz=decision_hz,
                 randomization=randomization)
    model = load_policy(policy_path)
    runtime = PolicyRuntime(model, decision_hz=decision_hz, enabled=safeguards)

    res = {"success": 0, "outcomes": [], "min_dists": [], "end_pos": [],
           "handover_fail": 0}
    for ep in range(episodes):
        obs = env.reset(seed=seed + ep)
        actor = make_follower(route, info, decision_hz=decision_hz,
                              robot_cfg=robot, physics_params=params)
        runtime.reset()
        # ── expert-driven prefix, runtime observing ──
        ok_prefix = True
        steps = 0
        while actor.progress < seg["s"] and seg["s"] > 1.0:
            feats = runtime.fb.build(obs)
            runtime.policy.predict_proba(feats)        # advance hidden state
            cmd = actor.command(obs["true_x"], obs["true_y"],
                                obs["true_heading"], ir=obs["ir"])
            runtime._cmd = cmd
            runtime.fb.observe_command(cmd)
            obs = env.step(cmd)
            steps += 1
            if steps > int(200 * decision_hz) or actor.done:
                ok_prefix = False
                break
        if not ok_prefix:
            res["handover_fail"] += 1
            res["outcomes"].append("prefix_fail")
            res["min_dists"].append(env.dist_to_goal())
            res["end_pos"].append((obs["true_x"], obs["true_y"]))
            continue
        # ── policy takes over ──
        stall = 0
        outcome = "timeout"
        min_d = env.dist_to_goal()
        for _ in range(int(max_time * decision_hz)):
            cmd = runtime.step(obs)
            prev = (obs["true_x"], obs["true_y"])
            obs = env.step(cmd)
            min_d = min(min_d, env.dist_to_goal())
            if env.dist_to_goal() < 5.0:
                outcome = "success"
                break
            if env.dist_to_goal() > 1500.0:
                outcome = "lost"
                break
            moved = math.hypot(obs["true_x"] - prev[0], obs["true_y"] - prev[1])
            if cmd != CMD_STOP and moved < 0.05:
                stall += 1
                if stall > int(6 * decision_hz):
                    outcome = "stuck"
                    break
            else:
                stall = 0
        res["success"] += outcome == "success"
        res["outcomes"].append(outcome)
        res["min_dists"].append(min_d)
        res["end_pos"].append((obs["true_x"], obs["true_y"]))
    return seg["name"], res


def main():
    ap = argparse.ArgumentParser(description="Segment-start evaluation")
    ap.add_argument("--policy", required=True, help=".npz path or 'expert'")
    ap.add_argument("--track", default=TRACK_DEFAULT)
    ap.add_argument("--robot", default=ROBOT_DEFAULT)
    ap.add_argument("--physics", default=PHYS_DEFAULT)
    ap.add_argument("--episodes", type=int, default=6)
    ap.add_argument("--seed", type=int, default=21000)
    ap.add_argument("--randomization", default="mild")
    ap.add_argument("--no-safeguards", action="store_true")
    ap.add_argument("--n-maze", type=int, default=4)
    ap.add_argument("--localize", action="store_true",
                    help="Full-course runs; report failure arc-lengths")
    ap.add_argument("--prefix", action="store_true",
                    help="Expert drives to the arc first (hidden-state-safe "
                         "for recurrent policies)")
    ap.add_argument("--workers", type=int, default=5)
    args = ap.parse_args()

    track = load_track(args.track)
    route, info = build_route(track)
    segs, geom = segment_starts(track, route, info, n_maze=args.n_maze)

    if args.localize:
        seg0 = segs[0]
        _, res = _run_episodes((args.track, args.robot, args.physics,
                                args.policy, seg0, 0.0, args.episodes,
                                args.seed, args.randomization, 240.0,
                                not args.no_safeguards, 10.0,
                                track.start_heading))
        print(f"\ncourse total: {geom.total:.0f}cm; tape ends ~{info.get('line_len', 0):.0f}cm")
        for i, (out, (ex, ey), md) in enumerate(zip(res["outcomes"],
                                                    res["end_pos"],
                                                    res["min_dists"])):
            s_end = geom.project(ex, ey)
            print(f"  ep{i:02d}: {out:8s} ended at arc {s_end:6.0f}cm "
                  f"({100 * s_end / geom.total:3.0f}%)  min_goal_dist={md:.0f}cm")
        return

    if args.prefix:
        jobs = [(args.track, args.robot, args.physics, args.policy, seg,
                 args.episodes, args.seed + 100 * i, args.randomization,
                 max(80.0, 260.0 * (geom.total - seg["s"]) / geom.total),
                 not args.no_safeguards, 10.0)
                for i, seg in enumerate(segs)]
        runner = _run_prefix_episodes
    else:
        print("calibrating segment odometry from expert episodes ...")
        med, h_med, h_p95 = calibrate(args.track, args.robot, args.physics,
                                      [s["s"] for s in segs],
                                      randomization=args.randomization)
        print(f"  odometry-vs-true heading |err|: median {h_med:.1f} deg, p95 {h_p95:.1f} deg"
              f"  (must be small or odom_init sign convention is wrong)")
        for s in segs:
            print(f"  {s['name']:12s} arc={s['s']:6.1f}  odom_dist={med[round(s['s'], 1)]:.0f}")
        jobs = [(args.track, args.robot, args.physics, args.policy, seg,
                 med[round(seg["s"], 1)], args.episodes,
                 args.seed + 100 * i, args.randomization,
                 max(80.0, 260.0 * (geom.total - seg["s"]) / geom.total),
                 not args.no_safeguards, 10.0, track.start_heading)
                for i, seg in enumerate(segs)]
        runner = _run_episodes
    if args.workers > 1:
        with Pool(min(args.workers, len(jobs))) as pool:
            results = pool.map(runner, jobs)
    else:
        results = [runner(j) for j in jobs]

    print(f"\n== segment results ({args.policy}, {args.randomization}, "
          f"guards={'off' if args.no_safeguards else 'on'}, "
          f"mode={'prefix' if args.prefix else 'teleport'}) ==")
    for name, res in results:
        n = len(res["outcomes"])
        md = float(np.median(res["min_dists"]))
        print(f"  {name:12s} {res['success']}/{n}  median_min_goal={md:6.0f}cm  "
              f"outcomes={','.join(res['outcomes'])}")


if __name__ == "__main__":
    main()
