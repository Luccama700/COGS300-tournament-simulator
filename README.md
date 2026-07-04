# COGS 300 Tournament — Maze Robot Simulator

A Pygame-based physics simulation for a maze-navigating Arduino robot. Drive your robot around a test arena to validate physics, tune sensor parameters, and draw tournament tracks — all before touching any hardware.

---

## Features

- **Realistic physics** — differential-drive kinematics, wall collision with sliding, speed/turn acceleration curves
- **HC-SR04 ultrasonic sensors** — multi-ray beam cone, distance-dependent Gaussian noise, dropout modeling
- **TCRT5000 IR line sensors** — analog reflectance simulation with soft edge transitions
- **Track editor** — draw walls and tape paths with your mouse, then immediately test-drive on them
- **Save/load tracks** — YAML format, shareable with teammates
- **Configurable robot** — swap chassis geometry and sensor layout via a YAML file (from the robot configurator app)

---

## Setup

**Requirements:** Python 3.11+

```bash
# 1. Clone the repo
git clone https://github.com/Luccama700/COGS300-tournament-simulator.git
cd COGS300-tournament-simulator

# 2. Create a virtual environment (recommended)
python -m venv .venv
source .venv/bin/activate      # macOS / Linux
.venv\Scripts\activate         # Windows

# 3. Install dependencies
pip install -r requirements.txt

# 4. Run
python run_test.py
```

---

## Running the sim

### Drive on the built-in test arena

```bash
python run_test.py
```

### Open the track editor

```bash
python run_test.py --editor
```

### Open the editor and load an existing track

```bash
python run_test.py --editor --track configs/track.yaml
```

### Drive directly on a saved track (no editor)

```bash
python run_test.py --track configs/track.yaml
```

### Use a different robot config or physics preset

```bash
python run_test.py --robot configs/my-robot.yaml --physics configs/physics.yaml
python run_test.py --noise noisy     # or: clean, default
python run_test.py --calibration calibration/my_profile.yaml
```

---

## Drive mode controls

| Key | Action |
|-----|--------|
| W / ↑ | Throttle forward |
| S / ↓ | Throttle reverse |
| A / ← | Steer left |
| D / → | Steer right |
| F | Toggle camera follow |
| R | Reset robot to start position |
| + / - | Zoom in / out |
| Q / Esc | Quit |

---

## Track editor controls

Open the editor with `python run_test.py --editor`.

### Switching modes

| Key | Action |
|-----|--------|
| Tab | Toggle between **Edit** and **Drive** mode |

### Edit mode — drawing

| Input | Action |
|-------|--------|
| Left-click | Place wall segment (click start, click end) |
| Shift + Left-click | Start a tape polyline; each subsequent click extends it |
| Right-click | Cancel wall in progress / finish tape polyline |
| Delete / Backspace | Toggle delete mode — click a segment to remove it |
| Z | Undo last placed element (or cancel in-progress drawing) |
| P | Place **start marker** at cursor position |
| E | Place **goal marker** at cursor position |

### Edit mode — view & file

| Key / Input | Action |
|-------------|--------|
| Scroll wheel | Zoom in / out (anchored under cursor) |
| Middle-mouse drag | Pan the view |
| G | Toggle grid snap on / off |
| S | Save track to YAML |
| L | Load track from YAML |
| Q / Esc | Cancel drawing in progress, or quit |

---

## File structure

```
run_test.py             ← main entry point (drive + editor launcher)
physics.py              ← physics engine (kinematics, oriented-rect collision, raycasting)
renderer.py             ← Pygame drawing (robot, walls, sensors, HUD)
robot_config.py         ← robot YAML loader and geometry helpers
sensor_model.py         ← HC-SR04 ultrasonic noise model
ir_model.py             ← TCRT5000 IR reflectance model
track.py                ← track data model (load/save YAML, line graph + A*)
track_editor.py         ← interactive track editor

# ── ML pipeline (v2) ──────────────────────────────────────────────
expert_policy.py        ← privileged expert: route planner + discrete-command follower
sim_env.py              ← closed-loop sim env with firmware-faithful observations
policy_runtime.py       ← feature builder + MLP inference + anti-spin safeguards
generate_data_v2.py     ← training data generation (supersedes generate_line_data.py)
evaluate_policy.py      ← closed-loop evaluation (success rate, spins, collisions)
training/
  train.py              ← numpy MLP trainer (class-balanced, episode-split val)
  dagger.py             ← DAgger loop (fixes compounding-error drift)
  dataset.py            ← CSV loading helpers
  filter_data.py        ← episode-quality filtering (legacy pipeline)

configs/
  robot-config.yaml     ← chassis geometry and sensor layout
  physics.yaml          ← physics and display parameters
  tracks/               ← saved tracks (v03 = repaired tournament track)

calibration/
  capture.py            ← serial capture tool for Arduino calibration data
  analyze.py            ← computes noise profiles from captured logs

arduino/
  hc_sr04_calibration/  ← Arduino sketch for HC-SR04 calibration
  robot_firmware/       ← robot firmware (WiFi AP + UDP sensor/command loop)
```

---

## Track file format (`configs/track.yaml`)

Tracks are plain YAML — easy to hand-edit or generate programmatically.

```yaml
track:
  name: "Tournament Track v1"
  walls:
    - [x1, y1, x2, y2]   # all values in cm
  line_paths:
    - x1: 0
      y1: 0
      x2: 100
      y2: 0
      width_cm: 5.0
  start:
    x: 0
    y: 0
    heading: 90.0         # degrees (0=east, 90=north/up)
  goal:
    x: 300
    y: 200
  bounds:
    width: 400
    height: 300
```

---

## Tuning physics (`configs/physics.yaml`)

Match these values to your real robot:

| Parameter | Description |
|-----------|-------------|
| `max_speed` | Top forward speed (cm/s) |
| `acceleration` | How fast it reaches top speed (cm/s²) |
| `max_turn_rate` | Max rotation speed (deg/s) |
| `wall_bounce` | 0 = dead stop on hit, 1 = full elastic bounce |
| `sensor_noise_std` | HC-SR04 Gaussian jitter (typically 1–3 cm) |
| `sensor_dropout_chance` | Probability of a bad reading per sensor per frame |

---

## Sensor models

### HC-SR04 ultrasonic (`sensor_model.py`)
Simulates a multi-ray beam cone with:
- Distance-dependent Gaussian noise
- Configurable dropout rate
- Calibration profile loading from real Arduino serial logs

### TCRT5000 IR reflectance (`ir_model.py`)
Returns a 0–1023 analog value matching real `analogRead()` output:
- **On tape** (~875): white tape in real life → high reflectance → high reading
- **Off tape** (~125): dark floor → low reflectance → low reading
- Soft 0.8 cm transition zone at tape edges (prevents flickering)

Threshold comparison on the Arduino: `if (analogRead(IR_PIN) > threshold)` detects tape.

---

## Machine learning pipeline (v2)

The original pipeline (`generate_line_data.py` → external training) produced
models that spun in place and crashed. Post-mortem of that pipeline found five
compounding causes, all fixed in v2:

| # | Old-pipeline defect | Consequence | v2 fix |
|---|---------------------|-------------|--------|
| 1 | The data-gen expert only followed the tape lines, but every track's goal is inside the walled maze — it could never finish, then logged STOP forever | **90.8% of training rows were STOP**, nearly all the rest were turns (0.4% FORWARD) | Hybrid expert: A* over the line network + A* over an inflated occupancy grid through the maze (`expert_policy.py`) |
| 2 | Expert drove custom continuous-steering kinematics, labels discretized afterwards | Labels unreachable by the firmware's 6 discrete commands; train/test dynamics mismatch | Expert emits the 6 firmware commands directly and drives the real `physics.py` engine |
| 3 | Features included absolute heading, est_x/est_y, elapsed time | Model memorized one trajectory; any deviation → garbage inputs → compounding errors → spinning | Sensor-only features + short history + IR line-memory + last command (`policy_runtime.FeatureBuilder`) |
| 4 | Sim ultrasonic dropout returned 0.0cm; firmware returns 200 on timeout; data-gen used a third noise model with no dropouts | Test-time inputs the model never saw during training | One sensor model everywhere; dropouts return max range like the firmware (`sensor_model.py`) |
| 5 | Single deterministic demonstration, no recovery states | No data for "slightly off the line" states → first error was fatal | Domain randomization (`sim_env.py`) + DAgger (`training/dagger.py`) |

Additionally, the bounding-circle collision model made the maze's 20cm slits
impassable for the 22cm circle even though the real 12cm-wide chassis fits;
`physics.py` now collides the true oriented rectangle (and models wedging).

### ⚠ Hardware finding: rear-mounted IR sensors cannot track the tape

The strongest result of this work is not about ML at all. With the IR
sensors at the **back** of the chassis (`robot-config.yaml`, mount_y +0.55),
even a hand-written controller with direct sensor access loses the tape and
circles hunting for it — the same "spinning like crazy" the real robot showed:

* A yaw correction swings the rear (where the sensors are) the *wrong way*
  first, so the robot translates 20-25cm off the tape before the sensors
  re-cross it (unstable lever arm).
* Once lost, a hard-turn search sweeps a ~25cm circle — smaller than the
  distance to the lost tape — so it orbits indefinitely.
* Measured in sim: a sensor-driven expert needs pose-based rescue ~27s per
  run with rear sensors, vs ~8s with the same sensors moved to the front.

`configs/robot-config-frontIR.yaml` is identical hardware with the two
TCRT5000s moved to the chassis front (mount_y −0.55). With front sensors the
classic 3-state follower (steer toward the hot sensor, straight when quiet,
one-shot pivot at sharp corners) completes the course 45/45 across all
randomization presets. **Recommendation: physically move the IR sensors to
the front of the robot.** The ML pipeline below uses the front-IR config;
it also runs with the rear config, but the resulting demonstrations lean on
privileged rescue and clone poorly.

Three firmware changes are required to match the sim (all small):
* Sample `analogRead(IR_*)` fast and report the **max since the last packet**
  (peak-hold) — at 28cm/s a tape crossing lasts well under one 10Hz packet
  and a single sample misses it. The sim models peak-hold.
* **Sign the encoder tick deltas by the commanded wheel direction** in
  `updateOdometry()` (`executeCommand` knows each wheel's direction). The
  single-channel encoders are direction-blind, so stock firmware corrupts
  heading by ~50% of every hard turn and inflates `distanceTraveled` while
  pivoting — which makes all dead-reckoning features (and the policy's
  heading input) unusable. The sim integrates signed wheel travel.
* Read the three HC-SR04s without the blocking `delay(60)` calls (staggered
  pings or echo-pin interrupts) so the control loop can run at ~10Hz instead
  of ~5Hz.

### Track repairs (v03)

Connectivity analysis showed the digitized tournament track was **unsolvable**:
the goal room was fully sealed, and four phantom cross-walls (double-drawn or
overshot strokes) sealed the SE room chain, the spiral mouth, the spiral exit,
and the top corridor. `COGS_300_Tournament_Track_v03.yaml` removes those and
opens a doorway into the goal antechamber. **The entrance-chicane geometry is a
reconstruction — check it against the physical track and re-digitize if it
differs** (v01/v02 are untouched for reference).

### Workflow

```bash
TRACK=configs/tracks/COGS_300_Tournament_Track/COGS_300_Tournament_Track_v03.yaml
ROBOT=configs/robot-config-frontIR.yaml   # see hardware finding above

# 0. Sanity gate: the expert must reach the goal reliably
python evaluate_policy.py --track $TRACK --robot $ROBOT --policy expert --episodes 20

# 1. Generate behavior-cloning data (expert demos in the real sim)
python generate_data_v2.py --track $TRACK --robot $ROBOT --episodes 200 \
    --randomization mild --output data/bc_mild.csv --workers 4
python generate_data_v2.py --track $TRACK --robot $ROBOT --episodes 100 \
    --randomization heavy --output data/bc_heavy.csv --workers 4
python -m training.merge_datasets --out data/bc_train.csv data/bc_mild.csv data/bc_heavy.csv

# 2. Train the command classifier
python -m training.train --data data/bc_train.csv --out models/policy_bc.npz

# 3. Closed-loop evaluation (the metric that matters)
python evaluate_policy.py --track $TRACK --robot $ROBOT --policy models/policy_bc.npz \
    --episodes 20 --randomization mild --plot eval_bc.png

# 4. DAgger — retrain on the learner's own mistake states
python -m training.dagger --track $TRACK --robot $ROBOT --base-data data/bc_train.csv \
    --iters 3 --episodes-per-iter 40 --out models/policy_dagger.npz

# 5. Final evaluation
python evaluate_policy.py --track $TRACK --robot $ROBOT \
    --policy models/policy_dagger.npz --episodes 30 --randomization mild
```

`policy_runtime.PolicyRuntime` is the deployable inference stack (features →
MLP → guards). Its safety guards use only firmware observables, so the same
class can drive the real robot from the laptop UDP relay:
probability-margin command switching (anti-dither), low-confidence hold,
a spin watchdog (sustained one-direction rotation → straight burst), and a
front-wall reflex. Guards can be disabled (`--no-safeguards`) to measure the
raw model.

### Current results (v03 track, front-IR config, BASE_SPEED 110)

| Policy | Full-course success (mild randomization) | Notes |
|--------|------------------------------------------|-------|
| Sensor-driven expert (`--policy expert`) | **15/15 – 20/20** across none/mild/heavy | Tape bang-bang + fire-armed corner pivots + wall-hug transit + maze pursuit; ~118s runs |
| Behavior cloning (96×96 MLP, 330k rows) | 0/20 | Masters the tape phase (locks through every zigzag vertex), degrades over the ~1200-decision horizon in the maze |
| + DAgger (mixed rollouts) | up to 5/50 during β=0.15 rollouts; 0/30 for late checkpoints | Mid-loop checkpoints peak, late iterations drown in recovery-state labels |

The honest summary: the *system* now works — simulator physics, track, expert
autopilot, data/train/eval/DAgger infrastructure — and the cloned policy
reliably solves the tape section, which is where the original robot spun and
crashed. Cloning the full 20+-decision-per-second, two-minute course into a
feedforward net remains open; the highest-leverage next steps are a recurrent
policy (the expert is a state machine; its hidden state defeats feedforward
nets), closed-loop checkpoint selection inside the DAgger loop, and recovery-
data downweighting so late iterations stop regressing.

Note on odometry: the sim integrates signed wheel travel for heading and
distance, which corresponds to the signed-tick firmware fix described above.
The raw `enc_l`/`enc_r` counters remain direction-blind like the hardware.

## Robot configurator app

The `robot-configurator-app/` folder contains a React app for visually designing the robot chassis and sensor layout. Export from the configurator and save to `configs/robot-config.yaml`.

---

## Requirements

```
pygame>=2.5.0
numpy>=1.24.0
pyyaml>=6.0
pyserial>=3.5
matplotlib>=3.7
```
