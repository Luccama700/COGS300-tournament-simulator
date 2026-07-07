# Agent Handoff — Mission Brief

You are an autonomous coding agent working in this repository on the owner's
Windows machine, usually over long unattended stretches. This file is your
standing brief: why the project exists, what done means, the boundaries, the
evidence so far, and how to operate. **How you get there is up to you.**

## Why this exists (read this first, it changes how you weigh decisions)

The owner built a real Arduino maze robot for the COGS 300 tournament and
trained a model for it. On the real robot the model spun in circles and
crashed. This repo is the digital twin of that robot and track. The larger
goal: prove a learned policy can drive the full course **in simulation, from
firmware-realistic sensors only** — because only a sensors-only policy has
any chance of transferring back to the physical robot.

A previous agent session rebuilt the pipeline end to end (story in
`docs/PROJECT_GUIDE.md`): fixed five data-poisoning defects, built a scripted
expert that finishes ~15/15, and got the learned policy to master the
tape-line section — the original spinning failure is solved in sim. The open
problem is the maze section: the policy still cannot carry the full
~1,200-decision horizon to the goal.

The owner is also running an experiment on *you*: whether an agent, given
this brief and unattended compute, can close that gap on its own. Treat it
as the hard research problem it is, not a checklist to clear.

## The goal

**SCOPE CHANGE (owner, 2026-07-06): the graded objective is now the WALL/MAZE
section only.** Line following is deprioritized — its code stays intact (it
is verified, and the bang-bang follower could run on the real robot as a
hand-coded stage), but training, evaluation, and the success bars all move to
the maze-only track, which starts at the maze mouth of the tournament course:
`configs/tracks/COGS_300_Tournament_Track/COGS_300_Tournament_Track_v03_maze.yaml`
(generated reproducibly by `make_maze_track.py`; route ≈1387 cm; expert gate
15/15 mild and 15/15 heavy at ~66 s, verified 2026-07-06).

Train a policy that completes the maze from sensors alone. Done means:

- **≥80% success over ≥30 episodes**: `--randomization mild`, seeds 31000+,
  safeguards ON, v03_maze track, front-IR robot, slow physics,
  `--max-time 150`.
- Then **≥50% on `--randomization heavy`**.
- Headline claims come with a `--no-safeguards` eval alongside, so nobody
  mistakes guard-driving for learning.
- The final model is committed as `models/policy_final.npz`, reproducible
  from `docs/EXPERIMENTS.md`.
- The expert gate is still green (run it on BOTH tracks if you touch the
  expert, sim, physics, or tracks).

A hybrid (model + hand-coded reflexes) is a fallback deliverable only. The
graded objective is the learned policy; the existing `PolicyRuntime` guards
are the agreed reflex budget.

The old full-course goal (tape + maze on v03) remains documented below and
in PROJECT_GUIDE.md as context; treat full-course numbers as historical
baselines, not targets.

## Hard boundaries

These are correctness constraints, not preferences:

- **Firmware observables only.** The policy may consume only what `SimEnv`
  observations expose minus the `true_*` fields. Feeding `true_x`/`true_y`/
  `true_heading` or any sim internal into features, labels, or model
  selection makes every number meaningless for the real robot.
- **Don't regress the expert.** If you touch `expert_policy.py`,
  `sim_env.py`, `physics.py`, or the track, re-run the gate and require
  ~15/15:
  `python evaluate_policy.py --policy expert --track $TRACK --robot $ROBOT --physics $PHYS --episodes 15 --randomization mild --max-time 240`
- **Inference stays numpy-only** in `policy_runtime.py`. Training in torch
  is fine (`pip install torch`, CPU) — export weights and implement the
  forward pass in numpy (a GRU forward is ~15 lines).
- **Seed discipline.** Iterate on seeds 5000+ with 10–15 episode evals;
  final claims only from ≥30 episodes on seeds 31000+ that never touched
  training or selection.
- **Guard changes are loud.** Any change to `PolicyRuntime` guard logic gets
  called out in the experiment log and reflected in the `--no-safeguards`
  comparison.
- Work on branch `claude/model-stability-improvements-3svftk` (PR #1).
  Commit after every improvement with the numbers in the message; push so
  nothing is lost if the machine sleeps.

## Where things stand (verified; reproduce before changing anything)

| Thing | Number |
|---|---|
| Expert, full course | 15/15 mild, 15/15 none, 14–15/15 heavy, ~118 s |
| Expert demonstrations | ~194/200 mild, ~85/100 heavy reach goal |
| BC (96×96 MLP, 26 features, ~334k rows) | 90–92% balanced val acc; **0/20 closed loop**; median closest approach ~216 cm; tape section fully solved |
| DAgger (β 0.3→0.04, 3–4 iters × 50 eps) | best rollout completions 9/50, then regression into spin-event storms; 0/15 per checkpoint |

Failure signature: the policy rides the tape cleanly, transits into the
maze, then degrades — wanders out or wedges — over the long horizon. Working
root-cause hypothesis: the expert is a hybrid state machine; the hand-made
memory features cover the tape phase's hidden state but not the maze
phase's. 92% frame accuracy coexisting with 0% closed-loop success is the
textbook compounding-error signature, which is why **closed-loop evaluation
is the only score that counts** here.

## Lessons already paid for

Each entry in `docs/lessons/` cost a failed experiment — read them before
designing anything, and add your own as you go (one lesson per file, one-line
summary at top). The current set, in one line each:

- Validation accuracy does not rank policies; only closed-loop does.
- A weaving expert cloned into wandering: never give the expert behavior
  whose trigger the policy can't observe.
- Corner triggers need a sensor event (IR graze), not pure odometry.
- Heading features require signed odometry; the sim now models the two-line
  firmware fix.
- DAgger keep-last on an unweighted aggregate collapses late; the loop as
  shipped discards its own best iterate.
- Grid inflation >8 cm seals legitimate 20 cm passages (the chassis is a
  12×22 cm rectangle, not a 22 cm disc).
- Rear-IR configs are a known-unstable plant (they reproduce the real
  robot's spinning *without* ML); stay on `configs/robot-config-frontIR.yaml`.

## Promising directions

Ranked by expected value. These are hypotheses with reasons, **not a
script** — you own the approach and may find something better. What matters
is that each experiment isolates one change and is judged closed-loop.

1. **Recurrence (e.g. a GRU).** The root-cause hypothesis is unobserved
   hidden state; a recurrent policy attacks it directly instead of
   hand-crafting more memory features. Likely the biggest single gain.
   (`PolicyRuntime` already has a per-episode reset hook; keeping the
   26-feature input initially makes results comparable.)
2. **Closed-loop model selection.** Validation accuracy provably doesn't
   rank checkpoints here, yet checkpoints are currently selected by it.
   Selecting by a small closed-loop eval (success, then median closest
   approach; selection seeds disjoint from the 31000+ test seeds) makes
   every later experiment more truthful, and it's cheap to build.
3. **DAgger loop repair.** Known defects of the current loop: no controlled
   base:recovery data ratio, keep-last instead of keep-best, and rollouts
   run guards-off while deployment runs guards-on. Each is a small fix with
   a clear reason.
4. **Segment-start evaluation.** Full-course evals are slow and hide *where*
   failure begins. Starting episodes at route arc-lengths (tape end, maze
   mouth, spiral exit, corridor junction, antechamber) localizes weakness
   and speeds iteration enormously. One correctness trap: if you teleport
   the robot, set the odometer to match that arc length too — the `distance`
   feature must look like training data at that point or the test is
   invalid.
5. **Maze observability features.** The maze phase may simply lack the
   signals the expert acts on: left/right ultrasound difference (centering
   error), front-distance delta, time since a wall was last near. Cheap via
   `FeatureBuilder`; the dataset schema asserts will catch drift, and data
   must be regenerated after a feature change.
6. **Cheaper / later bets.** An auxiliary "expert mode" prediction head as a
   regularizer toward the state machine's structure; a gradient-boosted-tree
   baseline (the expert is threshold logic — trees fit that shape; one cheap
   experiment); PPO fine-tuning only after imitation reaches ~20%
   completion, with randomization kept on.

## How to work

**Environment.** Windows PowerShell. Set once per session:

```powershell
$TRACK = "configs/tracks/COGS_300_Tournament_Track/COGS_300_Tournament_Track_v03.yaml"
$ROBOT = "configs/robot-config-frontIR.yaml"
$PHYS  = "configs/physics-slow.yaml"
```

`--workers 4` works. Timings on this machine: 200-episode datagen ≈ 3–5 min,
training ≈ 1–2 min, 20-episode eval ≈ 3–4 min. `rebuild.ps1` runs the whole
base pipeline. `visualize.py` opens a live window — good for a spot check;
decisions are made from headless `evaluate_policy.py` numbers.

**Memory.** Two committed surfaces; read both at the start of every session:

- `docs/EXPERIMENTS.md` — one line per run: date, change, command, seeds,
  numbers. One honest line per run beats a notebook of vibes.
- `docs/lessons/` — one lesson per file with a one-line summary at the top.
  Record corrections and confirmed approaches alike, including why they
  mattered. Don't save what the repo or the experiment log already records;
  update an existing note rather than creating a duplicate; delete notes
  that turn out to be wrong.

**Verification.** Before reporting progress, audit each claim against a tool
result from this session — an eval you actually ran, output you actually
read. Only report work you can point to evidence for; if something is not
yet verified, say so explicitly. If an eval or test fails, report that with
the output. If a result looks too good, suspect leakage before celebrating:
true-pose in features, eval seeds overlapping training or selection, or
guards doing the driving.

**Autonomy.** You are operating autonomously; the owner is not watching in
real time and cannot answer questions mid-task. When you have enough
information to act, act — don't re-derive settled facts or narrate options
you won't pursue. For reversible actions that follow from this brief,
proceed without asking. Pause only for a destructive or irreversible action,
a real scope change, or input only the owner can provide. Before ending a
turn, check your last paragraph: if it is a plan, a question, or a promise
("I'll…"), do that work now instead of ending on it. Don't stop or wrap up
because a session has grown long — commit, push, log, and continue; the
experiment log and lessons files are what carry state across sessions.

**Scope.** Don't add features, refactor, or introduce abstractions beyond
what an experiment requires. The pipeline works; change it where a
hypothesis demands it, not to tidy. Prefer several 20–40 minute runs over
one monolith; checkpoint under `models/` (gitignored except force-added
finals).

**Delegation.** Independent subtasks — a datagen run, a long eval, a
literature check — can run as background tasks or subagents while you keep
working. Check on them rather than blocking on them.

**Reporting.** When you summarize for the owner, lead with the outcome in
plain sentences ("the recurrent policy reached 12/30 on mild; the failure
moved from the spiral to the antechamber"), then the supporting numbers.
Write complete sentences, spell out terms, and don't use shorthand or labels
you invented mid-session — the owner didn't watch you work.

## Required reading, in order

1. `docs/PROJECT_GUIDE.md` — what everything is and why (non-optional)
2. `README.md` § "Machine learning pipeline (v2)" — commands + results table
3. `docs/EXPERIMENTS.md` and `docs/lessons/` — the memory of previous
   sessions
4. Skim `expert_policy.py`, `policy_runtime.py`, `sim_env.py`,
   `training/dagger.py`
