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
physics.py              ← physics engine (kinematics, collision, raycasting)
renderer.py             ← Pygame drawing (robot, walls, sensors, HUD)
robot_config.py         ← robot YAML loader and geometry helpers
sensor_model.py         ← HC-SR04 ultrasonic noise model
ir_model.py             ← TCRT5000 IR reflectance model
track.py                ← track data model (load/save YAML)
track_editor.py         ← interactive track editor

configs/
  robot-config.yaml     ← chassis geometry and sensor layout
  physics.yaml          ← physics and display parameters
  track.yaml            ← saved track (created by the editor)

calibration/
  capture.py            ← serial capture tool for Arduino calibration data
  analyze.py            ← computes noise profiles from captured logs

arduino/
  hc_sr04_calibration/  ← Arduino sketch for HC-SR04 calibration
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
