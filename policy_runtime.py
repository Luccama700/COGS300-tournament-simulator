"""
Policy runtime: feature construction, MLP inference, and safety guards.

This module is the single source of truth for how raw sensor packets become
model inputs — generate_data_v2.py uses FeatureBuilder to write training CSVs
and evaluate_policy.py / a future laptop UDP driver use PolicyRuntime for
inference, so train/test features can never drift apart.

Feature design notes (why these and not others):
  * NO absolute pose, NO absolute heading, NO elapsed time. The old pipeline
    fed est_x/est_y/heading and the model simply memorized the trajectory —
    one disturbance and every feature was a lie, errors compounded, and the
    robot spun. Everything here is available on the real robot and remains
    meaningful after a disturbance.
  * Short sensor history (2-3 frames) + a decayed "line memory" per IR
    sensor. Both IR sensors read floor (~120) whether the robot is centered
    on the line or completely lost — the memory of which sensor saw the line
    last is what disambiguates those states.
  * Last command one-hot: gives the network hysteresis context so it can
    hold a turn through the line's dead zone instead of dithering.
  * Turn-progress features (accumulated odometry heading change while the
    current turn is held, and recent heading rate). Without these the model
    cannot know when a turn is DONE — the expert releases a turn based on
    heading alignment, which sensors don't show — so "last_cmd=HARD_L +
    floor-only IR" gets predicted as HARD_L forever and the robot spins.
    This was measured, not hypothesized: the first BC model locked into
    hard-left at p=1.0 at the first zigzag vertex.
  * Odometry distance (noisy, drifting) as a soft progress signal — the
    tournament track is known in advance, so letting the model condition on
    rough progress is legitimate; domain randomization keeps it soft.

Safety guards at inference (all computable on the real robot):
  * Probability-margin command switching (anti-dither).
  * Confidence hold: below-threshold predictions keep the previous command.
  * Spin watchdog: large accumulated |heading change| with no distance
    progress forces a straight burst — the exact failure mode ("spinning
    like crazy") the old models exhibited.
  * Front-block reflex: never drive forward into a wall closer than ~6cm.
"""

import json
import math
from collections import deque

import numpy as np

CMD_FORWARD, CMD_SLIGHT_L, CMD_SLIGHT_R, CMD_HARD_L, CMD_HARD_R, CMD_STOP = range(6)
N_COMMANDS = 6

US_NORM = 200.0     # firmware caps ultrasonic readings at 200cm
IR_NORM = 1023.0
SPEED_NORM = 60.0
DIST_NORM = 500.0
IR_MEM_DECAY = 0.92   # per decision step (~0.8s half-life at 10 Hz)


class FeatureBuilder:
    """
    Streaming feature extractor. Feed it one observation per decision step;
    it maintains sensor history, IR line memory, and the last command.

    Observation dict fields used: "us" (list of 3), "ir" (list of 2),
    "speed" (cm/s), "distance" (cm). These match both SimEnv observations
    and the real firmware packet.
    """

    US_HIST = 2   # current + previous
    IR_HIST = 3   # current + two previous

    def __init__(self, n_us: int = 3, n_ir: int = 2):
        self.n_us = n_us
        self.n_ir = n_ir
        self.reset()

    def reset(self):
        self._us_hist: deque = deque(maxlen=self.US_HIST)
        self._ir_hist: deque = deque(maxlen=self.IR_HIST)
        self._ir_mem = [0.0] * self.n_ir
        self.last_cmd = CMD_STOP
        self._prev_odom_heading: float | None = None
        self._turn_accum = 0.0                       # deg since turn started
        self._rate_hist: deque = deque(maxlen=3)     # recent heading deltas

    @property
    def feature_names(self) -> list[str]:
        names = []
        for h in range(self.US_HIST):
            names += [f"us{i}_t{h}" for i in range(self.n_us)]
        for h in range(self.IR_HIST):
            names += [f"ir{i}_t{h}" for i in range(self.n_ir)]
        names += [f"ir{i}_mem" for i in range(self.n_ir)]
        names += [f"lastcmd_{c}" for c in range(N_COMMANDS)]
        names += ["turn_accum", "heading_rate", "heading_sin", "heading_cos",
                  "speed", "distance"]
        return names

    def build(self, obs: dict) -> np.ndarray:
        """Convert an observation into the feature vector (also updates state)."""
        us = [min(1.0, max(0.0, v / US_NORM)) for v in obs["us"][:self.n_us]]
        us += [1.0] * (self.n_us - len(us))          # missing sensor → max range
        ir = [min(1.0, max(0.0, v / IR_NORM)) for v in obs["ir"][:self.n_ir]]
        ir += [0.0] * (self.n_ir - len(ir))

        self._us_hist.appendleft(us)
        self._ir_hist.appendleft(ir)
        while len(self._us_hist) < self.US_HIST:
            self._us_hist.append(us)
        while len(self._ir_hist) < self.IR_HIST:
            self._ir_hist.append(ir)

        for i in range(self.n_ir):
            self._ir_mem[i] = max(self._ir_mem[i] * IR_MEM_DECAY, ir[i])

        # Turn-progress bookkeeping from odometry heading
        h = obs.get("odom_heading", 0.0)
        if self._prev_odom_heading is None:
            self._prev_odom_heading = h
        delta = (h - self._prev_odom_heading + 180.0) % 360.0 - 180.0
        self._prev_odom_heading = h
        self._rate_hist.append(delta)
        if self.last_cmd in (CMD_SLIGHT_L, CMD_SLIGHT_R, CMD_HARD_L, CMD_HARD_R):
            self._turn_accum += delta
        else:
            self._turn_accum = 0.0

        feats: list[float] = []
        for hh in range(self.US_HIST):
            feats += self._us_hist[hh]
        for hh in range(self.IR_HIST):
            feats += self._ir_hist[hh]
        feats += self._ir_mem
        onehot = [0.0] * N_COMMANDS
        onehot[self.last_cmd] = 1.0
        feats += onehot
        feats.append(max(-1.5, min(1.5, self._turn_accum / 180.0)))
        feats.append(max(-1.5, min(1.5, sum(self._rate_hist)
                                    / (len(self._rate_hist) * 30.0))))
        # Course-relative direction from (signed-encoder) odometry heading.
        # On a fixed tournament course this tells the policy which leg it is
        # on / which way it currently points — the signal that finally
        # disambiguates "IR fired on the left" between opposite maneuvers.
        # Requires the signed-tick firmware odometry (see sim_env.py note);
        # start-heading randomization keeps the model from over-trusting it.
        rad = math.radians(h)
        feats.append(math.sin(rad))
        feats.append(math.cos(rad))
        feats.append(min(2.0, obs.get("speed", 0.0) / SPEED_NORM))
        feats.append(min(4.0, obs.get("distance", 0.0) / DIST_NORM))
        return np.asarray(feats, dtype=np.float32)

    def observe_command(self, cmd: int):
        """Record the command actually executed (call after each decision)."""
        cmd = int(cmd)
        # Reset turn accumulation when the turn direction changes — the
        # feature means "how far have I rotated in THIS turn"
        turn_dir = {CMD_SLIGHT_L: 1, CMD_HARD_L: 1, CMD_SLIGHT_R: -1, CMD_HARD_R: -1}
        if turn_dir.get(cmd, 0) != turn_dir.get(self.last_cmd, 0):
            self._turn_accum = 0.0
        self.last_cmd = cmd


# ── Model ────────────────────────────────────────────────────────────────────

class MLPPolicy:
    """Small numpy MLP loaded from the .npz produced by training/train.py."""

    def __init__(self, weights: list[np.ndarray], biases: list[np.ndarray],
                 mean: np.ndarray, std: np.ndarray, meta: dict):
        self.weights = weights
        self.biases = biases
        self.mean = mean
        self.std = std
        self.meta = meta

    @classmethod
    def load(cls, path: str) -> "MLPPolicy":
        data = np.load(path, allow_pickle=False)
        n_layers = int(data["n_layers"])
        weights = [data[f"W{i}"] for i in range(n_layers)]
        biases = [data[f"b{i}"] for i in range(n_layers)]
        meta = json.loads(str(data["meta_json"]))
        return cls(weights, biases, data["mean"], data["std"], meta)

    def predict_proba(self, x: np.ndarray) -> np.ndarray:
        z = (x - self.mean) / self.std
        for W, b in zip(self.weights[:-1], self.biases[:-1]):
            z = np.maximum(0.0, z @ W + b)
        z = z @ self.weights[-1] + self.biases[-1]
        z -= z.max(axis=-1, keepdims=True)
        e = np.exp(z)
        return e / e.sum(axis=-1, keepdims=True)


# ── Runtime with safety guards ───────────────────────────────────────────────

class PolicyRuntime:
    """
    Full inference stack: features → MLP → guarded command.

    step(obs) returns the command to execute. All guards use only firmware
    observables (odometry heading/distance, ultrasonic), so this class can be
    dropped into the real laptop UDP driver unchanged.
    """

    # Note on spin detection: firmware encoders are single-channel
    # (direction-blind), so during a pivot the reported distanceTraveled
    # still increases and reported rotation is ~2x too small. The watchdog
    # therefore uses ONLY sustained same-direction heading accumulation,
    # with a threshold calibrated to the odometry's underestimation.
    def __init__(self, policy: MLPPolicy,
                 switch_margin: float = 0.08,
                 min_confidence: float = 0.35,
                 spin_window_s: float = 3.0,
                 spin_turn_deg: float = 250.0,
                 front_block_cm: float = 6.0,
                 decision_hz: float = 10.0,
                 enabled: bool = True):
        self.policy = policy
        self.fb = FeatureBuilder(n_us=policy.meta.get("n_us", 3),
                                 n_ir=policy.meta.get("n_ir", 2))
        self.switch_margin = switch_margin
        self.min_confidence = min_confidence
        self.spin_turn_deg = spin_turn_deg
        self.front_block_cm = front_block_cm
        self.enabled = enabled          # False → raw model output (for DAgger)
        self._window = int(spin_window_s * decision_hz)
        self.reset()

    def reset(self):
        self.fb.reset()
        self._cmd = CMD_STOP
        self._recover = 0
        self._prev_heading = None
        self._turn_accum: deque = deque(maxlen=max(2, self._window))
        self.spin_events = 0
        self.low_conf_holds = 0

    def step(self, obs: dict) -> int:
        feats = self.fb.build(obs)
        probs = self.policy.predict_proba(feats)
        cmd = self._guard(obs, probs) if self.enabled else int(np.argmax(probs))
        self._cmd = cmd
        self.fb.observe_command(cmd)
        return cmd

    # ------------------------------------------------------------------

    def _guard(self, obs: dict, probs: np.ndarray) -> int:
        # Spin watchdog bookkeeping (odometry heading only — works on hardware)
        heading = obs.get("odom_heading", 0.0)
        if self._prev_heading is None:
            self._prev_heading = heading
        delta = (heading - self._prev_heading + 180.0) % 360.0 - 180.0
        self._prev_heading = heading
        self._turn_accum.append(delta)

        if self._recover > 0:
            self._recover -= 1
            return CMD_FORWARD

        turn_total = abs(sum(self._turn_accum))
        if (len(self._turn_accum) == self._turn_accum.maxlen
                and turn_total > self.spin_turn_deg):
            # Sustained same-direction rotation — break the loop with a
            # straight burst (normal cornering alternates directions and
            # stays far below the threshold within one window)
            self.spin_events += 1
            self._turn_accum.clear()
            self._recover = 6
            return CMD_FORWARD

        best = int(np.argmax(probs))
        # Hysteresis: keep the current command unless the challenger clearly wins
        if best != self._cmd and probs[best] < probs[self._cmd] + self.switch_margin:
            best = self._cmd
        # Confidence hold: don't act on noise
        if probs[best] < self.min_confidence and self._cmd != CMD_STOP:
            self.low_conf_holds += 1
            best = self._cmd

        # Front-block reflex: forward into a near wall is never right
        us = obs.get("us", [])
        if us and best in (CMD_FORWARD, CMD_SLIGHT_L, CMD_SLIGHT_R):
            if 0.0 < us[0] < self.front_block_cm:
                left = us[1] if len(us) > 1 else 0.0
                right = us[2] if len(us) > 2 else 0.0
                best = CMD_HARD_L if left >= right else CMD_HARD_R
        return best
