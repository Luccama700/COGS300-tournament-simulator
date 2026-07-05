# Agent Handoff — Mission Brief for Continuing This Work

You are an autonomous coding agent working in this repository on the user's
Windows machine. Your mission: **train a policy that completes the COGS 300
tournament course in this simulator from sensors alone.** A previous agent
session built the entire infrastructure and got the policy to master the tape
section; the maze section remains open. This file is your complete context.

## Required reading, in order

1. `docs/PROJECT_GUIDE.md` — what everything is and why (15 min, non-optional)
2. `README.md` § "Machine learning pipeline (v2)" — commands and results table
3. Skim: `expert_policy.py` (the autopilot), `policy_runtime.py` (features +
   deployment stack), `sim_env.py` (observations), `training/dagger.py`

## Mission definition

- **Primary goal:** ≥80% closed-loop success over 30 episodes,
  `--randomization mild`, on the v03 track, front-IR robot, slow physics,
  `--max-time 240`, safeguards ON. Then confirm ≥50% on `heavy`.
- **Constraint:** the policy may use ONLY firmware observables (what
  `SimEnv` observations expose minus `true_*` fields). Never feed
  `true_x/true_y/true_heading` or sim internals to the model. That would
  make the sim numbers meaningless for the real robot.
- **Do not regress the expert:** if you touch `expert_policy.py`, `sim_env.py`,
  `physics.py`, or the track, re-run the expert gate and require ~15/15:
  `python evaluate_policy.py --policy expert --track $TRACK --robot $ROBOT
  --physics $PHYS --episodes 15 --randomization mild --max-time 240`
- A hybrid (model + hand-coded reflexes) is acceptable as a *fallback
  deliverable*, but the graded objective is the learned policy; the existing
  guards in `PolicyRuntime` are the agreed reflex budget. Don't quietly grow
  them into a hand-coded driver.

## Environment (verified working on this machine)

PowerShell. Set once per session:
```powershell
$TRACK = "configs/tracks/COGS_300_Tournament_Track/COGS_300_Tournament_Track_v03.yaml"
$ROBOT = "configs/robot-config-frontIR.yaml"
$PHYS  = "configs/physics-slow.yaml"
```
- `--workers 4` multiprocessing works. Timings on this machine: 200-episode
  datagen ≈ 3–5 min; training ≈ 1–2 min; 20-episode eval ≈ 3–4 min.
- `rebuild.ps1` runs the whole base pipeline.
- `visualize.py` opens a live window — use sparingly for spot checks; all
  decisions should be made from headless `evaluate_policy.py` numbers.
- You MAY `pip install torch` (CPU) for sequence models. Keep
  `policy_runtime.py` numpy-only: train in torch, export weights to .npz,
  implement the forward pass in numpy (a GRU forward is ~15 lines).

## Current state and key numbers (reproduce before changing anything)

| Thing | Number |
|---|---|
| Expert, full course | 15/15 mild, 15/15 none, 14–15/15 heavy, ~118s |
| Expert demonstrations | ~194/200 mild, ~85/100 heavy reach goal; "rescue" (pose-fallback) ~35 frames/ep mild |
| BC (96×96 MLP, 26 feats, ~334k rows) | 90–92% balanced val acc; **0/20 closed loop**; median closest-approach ~216cm; tape section: solved |
| DAgger (β 0.3→0.04, 3–4 iters × 50) | best rollout completions 9/50 (iter1) and 5/50 (iter2); late iterations regress (spin-event storms); 0/15 per checkpoint with guards |

Failure signature to beat: the policy rides the tape cleanly, transits to the
maze, then degrades — wanders out of the maze or wedges — over the ~1,200
decision horizon. Root cause analysis says: the expert is a hybrid state
machine; hand-made memory features cover the tape phase's hidden state but
not the maze phase's.

## Already tried — do NOT naively retry

- **Weaving expert** for tape observability → cloned into wandering (weave
  phase was hidden state). Superseded by bang-bang. Don't reintroduce.
- **Pure distance-triggered corner pivots** → uncomputable from noisy
  odometry (±2% scale). Superseded by fire-armed corners. Don't revert.
- **Heading features without signed odometry** → garbage after hard turns.
  `sim_env.py` now integrates signed wheel travel (documented firmware fix).
- **DAgger keep-last with unweighted aggregate** → late-iteration collapse.
  Fix the loop (see roadmap) instead of adding iterations.
- **Higher grid inflation** (>8cm) seals legitimate passages; corner
  clearance is handled by the densify+push pass, not inflation.
- Rear-IR configs: known-unstable plant; stay on `robot-config-frontIR.yaml`.

## Roadmap (priority order, with implementation notes)

1. **GRU policy.** Torch, hidden ≈ 64, input = the existing 26 features
   (keep `FeatureBuilder` as-is initially), truncated BPTT over ~50-step
   windows, class-weighted CE, batches of windows grouped by episode.
   Export → numpy forward in a new `GRUPolicy` class alongside `MLPPolicy`
   (`PolicyRuntime` needs a `reset()`-able hidden state — it already has a
   reset hook). Expect the biggest single gain here.
2. **Closed-loop checkpoint selection.** Add `--select-closed-loop N` to
   `training/train.py` (or a wrapper): every K epochs and at the end, run N
   fast eval episodes and keep the best checkpoint by (success, then median
   closest-approach). Use seeds disjoint from the final test seeds.
3. **DAgger repair:** (a) train on a fixed 60:40 sample of base:aggregated
   recovery rows each iteration; (b) per-iteration closed-loop selection as
   in #2; (c) try rollouts with guards ON (deployment distribution).
4. **Segment evals** (huge iteration-speed win): start episodes at route
   arc-lengths [tape-end, mouth, spiral-exit, corridor-junction,
   antechamber]. Implementation: after `env.reset()`, set
   `env.state.x/y/heading` to the route point/tangent at that arc length,
   and set `env._odom_dist` to that arc length ± noise (the `distance`
   feature must match what training data shows at that point — forgetting
   this invalidates the whole test). Report per-segment success; train on
   the weakest segment's states (datagen already covers them; you can also
   generate extra episodes that *start* at segment boundaries).
5. **Maze observability features:** `us_left − us_right` (centering error),
   front-distance delta, frames-since-any-wall<40cm. Add to FeatureBuilder
   (bump feature count; regenerate data — schema asserts will catch drift).
6. **Auxiliary mode head:** log the expert's mode (line/corner-armed/pivot/
   search/rescue/pursuit) into the CSV (extra column, excluded from
   features), train the net to predict it jointly (loss weight ~0.3).
7. **HistGradientBoosting baseline** (sklearn) on the same features — the
   expert is threshold logic; trees fit that shape. One experiment, cheap.
8. **PPO fine-tune** only after ≥20% BC/DAgger completion: reward = Δ(route
   progress) per step − small time penalty + terminal bonus; keep the
   randomization on.

## Operating protocol

- Keep a running log in `docs/EXPERIMENTS.md`: date, change, command, seed,
  numbers. One honest line per run beats a notebook of vibes.
- Fixed seed discipline: iterate with seeds 5000+, make final claims on
  seeds 31000+ (never trained/selected on).
- Iterate with 10–15 episode evals; only claim results from ≥30 episodes.
- Commit after every improvement with the numbers in the commit message.
  The remote branch is `claude/model-stability-improvements-3svftk` (PR #1).
- Long runs: prefer several 20–40 min runs over one monolith; checkpoint
  everything under `models/` (gitignored except force-added finals).
- If a result looks too good, suspect leakage first: true-pose in features,
  eval seeds overlapping training, or guards doing the driving (always also
  report a `--no-safeguards` eval for headline claims).

## Definition of done

30-episode mild eval ≥80% success (safeguards on, seeds 31000+), heavy ≥50%,
expert gate still green, `docs/EXPERIMENTS.md` updated, final model committed
as `models/policy_final.npz` with its training command reproducible from the
log. Then (stretch): a `--policy` option for `visualize.py` side-by-side
expert-vs-model, and a laptop UDP driver skeleton wiring `PolicyRuntime` to
the firmware packet format in `arduino/robot_firmware/robot_firmware.ino`.
