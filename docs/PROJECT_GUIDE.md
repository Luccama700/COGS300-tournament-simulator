# Project Guide — Teaching a Neural Network to Drive the COGS 300 Maze

This document explains the whole system: what we are building, every technique
in the pipeline, **why** each one is there (each earned its place by fixing a
failure we actually hit), and where the open problem stands. Read this first;
the README covers commands, this covers understanding.

---

## 1. The goal

The tournament robot is an Arduino R4 with 2 drive motors (L298N), 3 HC-SR04
ultrasonic sensors, 2 TCRT5000 IR line sensors, and single-channel wheel
encoders. It streams sensor packets over WiFi/UDP to a laptop; the laptop
replies with one of **six discrete motor commands** (forward, slight
left/right, hard left/right, stop). The course: follow a taped zigzag line,
enter a walled maze, navigate to a goal chamber.

**The goal of this repo:** train a neural network that, given only what the
robot can actually sense, outputs those commands and completes the course —
first in this simulator, then on hardware.

That last clause — *only what the robot can actually sense* — turns out to be
the entire difficulty, and most of this document is about it.

---

## 2. System architecture

```
                    ┌────────────────────────────────────────────┐
 real world:        │ Arduino firmware ──UDP──▶ laptop policy    │
                    │   (sensors, motors)       (this repo's     │
                    │                            PolicyRuntime)  │
                    └────────────────────────────────────────────┘
                                     ▲  same interface
                                     ▼
 simulation:        SimEnv (sim_env.py)
                      ├── PhysicsEngine (physics.py)      ← Arduino motor model,
                      │                                     oriented-rect collision
                      ├── SensorSimulator (sensor_model.py) ← HC-SR04 noise/dropout
                      ├── IRSensorSimulator (ir_model.py)   ← TCRT5000 reflectance
                      └── firmware-faithful observations    ← odometry, peak-hold IR

 expert:            expert_policy.py — plans a route and drives SimEnv perfectly
                      (only exists in sim; used to LABEL data, never deployed)

 learning:          generate_data_v2.py → training/train.py (MLP baseline)
                                          or training/train_gru.py (GRU)
                                        → evaluate_policy.py
                                        → training/dagger_gru.py
 deployment stack:  policy_runtime.py — features + MLP + safety guards
                      (imports nothing sim-specific; drop into the UDP relay)
```

Key invariant: **`policy_runtime.FeatureBuilder` is the single source of truth
for features.** Data generation, DAgger, evaluation, and (eventually) the real
laptop driver all call the same code, so train-time and run-time inputs can
never drift apart.

---

## 3. Why the original models spun (the five defects)

The first training attempts produced robots that spun in place and crashed.
Post-mortem found five independent causes — worth knowing because each is a
classic failure pattern:

1. **Poisoned labels.** The old data generator's expert could only follow the
   tape, but the goal is inside the maze — unreachable. After running out of
   waypoints it logged STOP for the rest of every episode: **90.8% of the
   dataset was STOP**, 0.4% was FORWARD. A model trained on that knows only
   how to turn and stop.
2. **Unsolvable world.** The digitized track was broken: the goal chamber was
   sealed by walls on all sides, and four phantom walls (double-drawn strokes)
   sealed the maze's internal passages. Connectivity analysis (flood-fill over
   an occupancy grid) found them all; `..._v03.yaml` is the repaired track.
3. **Wrong collision model.** The robot was simulated as a 22cm circle, but
   the maze has 20cm slits that the real 12cm-wide chassis drives through
   lengthwise. Physics now collides the true oriented rectangle.
4. **Train/test sensor mismatch.** Sim ultrasonic dropouts returned 0.0cm;
   the firmware returns 200 on timeout. The old data generator used a third,
   different noise model. A model that never saw a 0.0 in training panics at
   test time. One sensor model is now used everywhere.
5. **Memorization features.** Absolute x/y estimates, absolute heading, and
   elapsed time let the network memorize one trajectory instead of learning a
   sensor→action mapping. After the first disturbance those features are
   lies, the model's outputs become garbage, which causes bigger disturbances
   — a feedback loop that ends in spinning.

---

## 4. The core method: imitation learning

We use **behavior cloning (BC)**: run an expert in simulation, record
(observation → expert action) pairs, train a classifier to imitate. Six
commands → a 6-class classification problem. This is the simplest way to get
a policy — no reward engineering, no exploration — *if* you can build an
expert and *if* the expert's decisions are learnable from the observations.

### 4.1 The privileged expert

`expert_policy.py` is an autopilot that always completes the course
(15/15–20/20 across noise levels). It has three layers:

* **Route planning** (runs once per track):
  - A* over the tape-line graph (`track.py` builds nodes/edges from segments).
  - A* over an **occupancy grid** of the walls, inflated by 8cm (robot
    half-width + margin) so any grid path is physically drivable. Grid cells
    with no wall within 35cm cost 1.6× — this **wall-hug preference** keeps
    routes inside ultrasonic sensing range, because a sensor-driven policy
    cannot hold a course across featureless open floor (2% odometry error
    scatters a 300cm blind dash wider than the 60cm maze mouth).
  - Post-processing: densify the path and push points to ≥12.5cm from wall
    *endpoints* — the chassis nose swings ~12cm when cornering, and pure
    pursuit otherwise wedges on corner points.
* **Line phase — a sensor-driven bang-bang controller.** Steer toward
  whichever IR sensor sees the tape; drive straight when neither does;
  pivot hard at sharp corners. Corners **arm** inside a coarse distance
  window and **fire on the next IR contact** (the vertex graze), so the
  pivot timing is locked to an observable event.
* **Maze phase — pure pursuit** on the planned route with line-of-sight
  target clamping, plus a pivot-and-go unstick reflex for wall wedges.

Why so much engineering in the expert? Because of the single most important
lesson of this project:

> **You can only clone behavior whose triggers are visible in the
> observations.** The expert's job is not just to succeed — it is to succeed
> *for reasons the sensors can see*.

Three real bugs made this concrete:

| Incident | Hidden trigger | Fix |
|---|---|---|
| Model locked into HARD_LEFT at p=1.0 and spun forever | The expert released turns when its (privileged) heading aligned with the route — invisible to sensors | Added `turn_accum` / `heading_rate` features (rotation since the turn began, from odometry) so "the turn is done" became observable |
| Weaving expert (sinusoidal tape sweep) cloned into aimless wandering | The weave phase was keyed to route position — identical sensor states demanded opposite actions | Replaced with bang-bang, where every switch is caused by an IR event |
| Corner pivots fired at the right distance in training, never in deployment | Trigger was true route progress; the policy only sees odometry distance with ±2% scale error (±11cm at the far corners — wider than the trigger window) | Fire-armed corners: distance only *selects* the corner; the IR graze *times* it |

### 4.2 Features (`policy_runtime.FeatureBuilder`, 26 inputs)

Everything is computable on the real robot from the firmware packet:

| Feature group | What | Why |
|---|---|---|
| `us{0,1,2}_t{0,1}` | 3 ultrasonics, current + previous frame, /200 | Walls; the derivative (via history) gives approach rate |
| `ir{0,1}_t{0,1,2}` | 2 IR sensors, 3 frames, /1023 | Tape contacts; 3 frames because a crossing lasts 1–3 frames |
| `ir{0,1}_mem` | Exponentially decaying max of each IR | "Which side did I last see the tape, roughly when" — disambiguates centered-on-tape from lost (both read floor *now*) |
| `lastcmd_0..5` | One-hot of the executed command | Hysteresis context: hold a maneuver through sensor-quiet gaps |
| `turn_accum` | Signed odometry rotation while the current turn is held | The turn-release signal (see incident table) |
| `heading_rate` | Mean heading delta over 3 frames | Am I actually rotating (vs. commanded but wedged) |
| `heading_sin/cos` | Odometry heading, relative to start | Which leg of the fixed course I'm pointing along — requires the signed-encoder firmware fix |
| `speed` | Displacement speed | Wedge/stall detection |
| `distance` | Odometry arc length | Coarse course progress (which corner is next); deliberately noisy in training so the model can't over-trust it |

Deliberately **excluded**: absolute pose, elapsed time (memorization traps),
and anything the firmware can't produce.

### 4.3 Domain randomization

Every training episode randomizes motor strength (±10–25%), per-wheel
asymmetry, motor lag, sensor noise/dropout rates, start pose, and odometry
scale/drift (`sim_env.EnvRandomization`). Two reasons: the real robot's
parameters are unknown (sim2real robustness), and randomization forces the
expert to *demonstrate corrections*, giving the model recovery examples
instead of one sterile trajectory.

### 4.4 Training (`training/train.py`)

A 2-hidden-layer MLP (96×96, ~14k params) in pure numpy — softmax
cross-entropy, Adam, inverse-frequency **class weights** (FORWARD is ~54% of
labels; unweighted training under-learns the rare hard turns that matter
most), early stopping on **episode-level** validation splits (rows within an
episode are correlated; row-level splits leak and report fantasy accuracy).

### 4.5 Evaluation (`evaluate_policy.py`)

**Per-frame accuracy does not predict driving.** Our best model is 92%
balanced-accurate per frame and completes 0% of courses. The only meaningful
metric is closed-loop: run N randomized episodes, count completions, wedges,
spin events, closest approach to goal. This gap — great per-frame, dead
closed-loop — is **covariate shift / compounding error**: each small
disagreement puts the robot in a state slightly outside the training
distribution, where the next error is bigger. Over ~1,200 sequential
decisions, tiny per-step error compounds relentlessly.

### 4.6 DAgger (`training/dagger_gru.py`; the original `training/dagger.py` described here was removed 2026-07-06 after its defects were fixed in the GRU loop)

The textbook fix for compounding error (Ross et al. 2011): roll out the
*learner*, label every state it visits with the *expert's* action, add to the
dataset, retrain, repeat. The learner thereby gets labels exactly where it
goes wrong. Our implementation mixes actions (expert with probability β,
decayed 0.3 → 0.04 across iterations).

Observed behavior: mid-loop iterations improve (up to 9/50 and 5/50 rollout
completions — the only full completions any learned policy has achieved),
then late iterations **regress**: the aggregate becomes dominated by messy
recovery states, the class balance shifts toward corrections, and per-frame
early stopping selects for the wrong thing. Fixing this loop is next-step #2
below.

### 4.7 Safety guards (`policy_runtime.PolicyRuntime`)

Deployment wraps the network with reflexes using only firmware observables:
probability-margin command switching (anti-dither), low-confidence hold, a
**spin watchdog** (sustained one-direction rotation with no progress →
straight burst — the direct antidote to the original failure mode), and a
front-wall block. Guards are OFF during DAgger rollouts (the raw policy's
distribution is what needs labels) and ON at deployment/eval.

---

## 5. Findings that matter for the physical robot

These came out of the sim work but are about hardware; they apply regardless
of how the ML ends up:

1. **Move the IR sensors to the front.** With rear-mounted sensors even a
   hand-written controller cannot track the tape: a yaw correction swings the
   rear (sensors) the wrong way first, the robot walks 20–25cm off the tape
   before the sensors re-cross it, and the hard-turn search circle (~25cm) is
   smaller than the distance to the lost tape — it orbits forever. This
   *reproduces the real robot's spinning exactly, with no ML involved*.
   Front-mounted sensors make the control loop textbook-stable.
   (`configs/robot-config-frontIR.yaml` is the sim twin of the fix.)
2. **Sign the encoder ticks** by commanded wheel direction in
   `updateOdometry()`. Stock firmware counts direction-blind, so every hard
   turn corrupts heading by ~50% of the rotation and `distanceTraveled` grows
   while pivoting. Two lines; makes dead-reckoning usable.
3. **Peak-hold the IR readings** between packets (sample fast, send the max).
   A tape crossing lasts less than one packet at speed; single samples miss it.
4. **Lower BASE_SPEED to ~110** (from 150) and make the three HC-SR04 reads
   non-blocking so the loop runs ~10Hz. Slower + more decisions = everything
   about control and learning gets easier. Completing slowly beats DNF.

---

## 6. Current state (honest)

| What | Status |
|---|---|
| Simulator physics, sensors, firmware fidelity | Fixed and verified (OBB collision, dropout semantics, signed odometry, peak-hold) |
| Track | v03 repaired and solvable; entrance area is a reconstruction — verify against the real track |
| Expert autopilot | **15/15–20/20** full-course across none/mild/heavy randomization, ~118s |
| Cloned policy (BC, `models/policy_bc.npz`) | Masters the full tape section closed-loop (the part that used to spin); 0/20 full course — degrades in the maze over the long horizon |
| DAgger | Peaks mid-loop (5–9/50 completions during rollouts), regresses late |

The remaining gap is a real ML research-flavored problem: **cloning a hybrid
state machine with a feedforward network**. The expert has internal modes
(which corner is armed, search phase, sweep direction); we hand-crafted
features to expose some of that state, which is why the tape phase works.
The maze phase needs the same treatment — or a model that can carry state
itself.

---

## 7. Next steps, in priority order

1. **Recurrent policy (GRU/LSTM).** The principled fix for hidden state:
   let the network *learn* its own memory instead of hand-engineering it.
   Train on episode sequences with truncated BPTT (windows ~50 steps).
   Use PyTorch locally for training speed; export weights and write the
   ~15-line numpy GRU forward pass so `policy_runtime` stays
   dependency-light for the laptop.
2. **Closed-loop model selection.** Stop selecting checkpoints by per-frame
   accuracy (proven uncorrelated). After every training run and every DAgger
   iteration, run ~10 closed-loop episodes and keep the checkpoint with the
   best success/progress. Mechanical, cheap, high value.
3. **Fix DAgger's late-iteration regression:** sample the aggregate at a
   fixed base:recovery ratio (e.g., 60:40) instead of letting recovery data
   swamp the clean demonstrations; keep-best (closed-loop) instead of
   keep-last; consider rollouts with guards ON so the labeled distribution
   matches deployment.
4. **Segment-level evaluation and training.** Start episodes at route
   waypoints (tape end, maze mouth, spiral exit, corridor, antechamber) with
   odometry initialized to that arc length. Localizes failures ("dies in the
   spiral") instead of confounding whole-course runs, and enables curriculum
   training on the weakest segment. Big iteration-speed win.
5. **Maze-phase observability features**, same philosophy as `turn_accum`:
   corridor-centering error (us_left − us_right), front-distance derivative,
   time-since-wall-contact. The maze pursuit is currently the least
   observable behavior in the demonstrations.
6. **Auxiliary mode-prediction head.** The expert knows its internal mode
   (line/corner/search/transit/maze) at every step — log it and train the
   network to predict it as a second output. Free supervision that forces the
   hidden state to organize around the right structure.
7. **Gradient-boosted trees baseline.** The expert is literally a small
   program of thresholds; boosted trees fit sharp switching logic better
   than MLPs. One afternoon experiment; trivially portable inference.
8. **RL fine-tuning (last).** Once imitation gets a nonzero completion rate,
   fine-tune with PPO on progress-along-route reward in the randomized sim.
   Escapes the imitation ceiling; pointless before the policy can survive
   long enough to collect reward.

---

## 8. Glossary

- **Behavior cloning (BC):** supervised learning of expert actions from logged
  (observation, action) pairs.
- **Covariate shift / compounding error:** the learner's own mistakes take it
  into states absent from training data, where it errs harder. The reason
  per-frame accuracy ≠ driving ability.
- **DAgger:** Dataset Aggregation — iteratively label the learner's own
  visited states with expert actions and retrain.
- **Domain randomization:** varying sim parameters per episode so the policy
  works across (and is not over-fit to) one exact physics.
- **Privileged expert:** a controller using sim-only ground truth (true pose)
  to generate demonstrations. Must be designed so its *decisions* are
  inferable from sensors, or the demonstrations are unlearnable.
- **Partial observability:** the sensors do not reveal the full world state;
  handled with history features, memory features, or recurrent networks.
- **Pure pursuit:** steering controller that chases a point a fixed distance
  ahead on a reference path.
- **Bang-bang control:** switching between discrete actions on sensor
  thresholds — the classic sparse-sensor line-following strategy.
