"""
Closed-loop simulation environment for training and evaluating policies.

Wraps the real PhysicsEngine (physics.py) and produces observations shaped
like the packet the Arduino firmware actually streams over UDP:

    S: us_front, us_left, us_right, ir_left, ir_right,
       heading, speed, distance, elapsed, enc_l, enc_r

Crucially, `heading`/`distance` here are ENCODER ODOMETRY — relative to the
episode start, quantized to encoder ticks, and drifting — NOT ground truth.
The robot's true pose is exposed separately (`obs["true_x"]` etc.) and may
only be used by the privileged expert and by evaluation metrics, never as
a policy input.

One decision step = one motor command held for 1/decision_hz seconds while
the physics integrates at physics_hz. This mirrors the real system, where
the laptop sends a command and the firmware holds it until the next one.

Domain randomization (EnvRandomization) varies motor strength/asymmetry,
sensor noise, start pose, and odometry error per episode so a cloned policy
can't just memorize one exact trajectory.
"""

import math
import random
from dataclasses import dataclass, replace

from physics import PhysicsEngine, PhysicsParams, RobotState, Wall
from robot_config import RobotConfig
from sensor_model import SensorNoiseProfile, HC_SR04_DEFAULT
from track import TrackData

# Firmware encoder constant (robot_firmware.ino)
TICKS_PER_CM = 5.0


@dataclass
class EnvRandomization:
    """Per-episode domain randomization ranges."""
    start_pos_std: float = 0.0        # cm
    start_heading_std: float = 0.0    # deg
    speed_scale: tuple = (1.0, 1.0)   # multiplier on max_wheel_speed_cmps
    tau_scale: tuple = (1.0, 1.0)     # multiplier on motor_tau
    wheel_asym_std: float = 0.0       # per-wheel efficiency std (motors differ)
    us_noise_scale: tuple = (1.0, 1.0)
    ir_noise_scale: tuple = (1.0, 1.0)
    odom_heading_drift: float = 0.0   # deg noise per decision step
    odom_scale_std: float = 0.0       # encoder distance scale error (episode)


RANDOMIZATION_PRESETS: dict[str, EnvRandomization] = {
    "none": EnvRandomization(),
    "mild": EnvRandomization(
        start_pos_std=2.0,
        start_heading_std=6.0,
        speed_scale=(0.9, 1.1),
        tau_scale=(0.85, 1.25),
        wheel_asym_std=0.03,
        us_noise_scale=(0.8, 1.3),
        ir_noise_scale=(0.8, 1.3),
        odom_heading_drift=0.25,
        odom_scale_std=0.02,
    ),
    "heavy": EnvRandomization(
        start_pos_std=4.0,
        start_heading_std=12.0,
        speed_scale=(0.75, 1.25),
        tau_scale=(0.7, 1.5),
        wheel_asym_std=0.06,
        us_noise_scale=(0.6, 1.7),
        ir_noise_scale=(0.6, 1.7),
        odom_heading_drift=0.5,
        odom_scale_std=0.04,
    ),
}


class SimEnv:
    """
    Decision-rate simulation loop around PhysicsEngine.

    Usage:
        env = SimEnv(track, robot_cfg, params)
        obs = env.reset(seed=0)
        obs = env.step(cmd)      # cmd in 0..5
    """

    def __init__(self, track: TrackData, robot_cfg: RobotConfig,
                 params: PhysicsParams,
                 noise_profile: SensorNoiseProfile | None = None,
                 decision_hz: float = 10.0, physics_hz: float = 60.0,
                 randomization: EnvRandomization | str = "none"):
        self.track = track
        self.robot_cfg = robot_cfg
        self.base_params = params
        self.base_profile = noise_profile or HC_SR04_DEFAULT
        self.decision_hz = decision_hz
        self.physics_hz = physics_hz
        self.substeps = max(1, int(round(physics_hz / decision_hz)))
        self.dt = 1.0 / physics_hz
        if isinstance(randomization, str):
            randomization = RANDOMIZATION_PRESETS[randomization]
        self.rand = randomization
        self.rng = random.Random()

        self.walls = [Wall(*w) for w in track.walls]
        self.engine: PhysicsEngine | None = None
        self.state: RobotState | None = None

    # ------------------------------------------------------------------

    def reset(self, seed: int | None = None) -> dict:
        if seed is not None:
            self.rng.seed(seed)
        r, rng = self.rand, self.rng

        params = replace(
            self.base_params,
            max_wheel_speed_cmps=self.base_params.max_wheel_speed_cmps
                                 * rng.uniform(*r.speed_scale),
            motor_tau=self.base_params.motor_tau * rng.uniform(*r.tau_scale),
            left_wheel_eff=max(0.5, rng.gauss(1.0, r.wheel_asym_std)),
            right_wheel_eff=max(0.5, rng.gauss(1.0, r.wheel_asym_std)),
        )
        us_scale = rng.uniform(*r.us_noise_scale)
        profile = replace(
            self.base_profile,
            noise_base_std=self.base_profile.noise_base_std * us_scale,
            noise_per_cm=self.base_profile.noise_per_cm * us_scale,
            dropout_chance=min(0.3, self.base_profile.dropout_chance * us_scale),
        )
        self.engine = PhysicsEngine(self.robot_cfg, params, self.walls,
                                    noise_profile=profile,
                                    track_lines=self.track.line_paths)
        self.engine.ir_sim.NOISE_STD = 20.0 * rng.uniform(*r.ir_noise_scale)

        self.state = RobotState(
            x=self.track.start_x + rng.gauss(0, r.start_pos_std) if r.start_pos_std else self.track.start_x,
            y=self.track.start_y + rng.gauss(0, r.start_pos_std) if r.start_pos_std else self.track.start_y,
            heading=self.track.start_heading + (rng.gauss(0, r.start_heading_std) if r.start_heading_std else 0.0),
        )

        # Odometry state (what the firmware would compute from encoders)
        self._odom_heading = 0.0        # firmware heading starts at 0 at boot
        self._odom_dist = 0.0
        self._enc_l = 0
        self._enc_r = 0
        self._enc_scale = rng.gauss(1.0, r.odom_scale_std) if r.odom_scale_std else 1.0
        self._elapsed = 0.0
        self._collided_frames = 0

        # Populate initial sensor readings
        self.state.sensor_readings = self.engine._read_sensors(self.state)
        self.state.ir_readings = self.engine._read_ir_sensors(self.state)
        return self._obs(0.0, 0.0)

    # ------------------------------------------------------------------

    def step(self, cmd: int) -> dict:
        """Hold `cmd` for one decision period; return the new observation."""
        st = self.state
        x0, y0 = st.x, st.y
        wheel_l_cm = 0.0
        wheel_r_cm = 0.0
        collided = 0
        for _ in range(self.substeps):
            self.engine.step(st, cmd, self.dt)
            wheel_l_cm += abs(st.left_wheel_speed) * self.dt
            wheel_r_cm += abs(st.right_wheel_speed) * self.dt
            if st.wall_contact:
                collided += 1
        self._collided_frames += collided
        self._elapsed += self.substeps * self.dt

        # ── Encoder odometry (mirrors firmware updateOdometry exactly) ────
        # Single-channel encoders count ticks regardless of wheel DIRECTION,
        # so during hard turns (one wheel reversed) the firmware's heading
        # integration is systematically wrong — replicate that faithfully.
        self._enc_l += int(round(wheel_l_cm * TICKS_PER_CM * self._enc_scale))
        self._enc_r += int(round(wheel_r_cm * TICKS_PER_CM * self._enc_scale))
        d_center = (wheel_l_cm + wheel_r_cm) / 2.0
        self._odom_dist += d_center * self._enc_scale
        d_theta = (wheel_r_cm - wheel_l_cm) * self._enc_scale \
            / self.robot_cfg.chassis.wheelbase_cm
        drift = (self.rng.gauss(0, self.rand.odom_heading_drift)
                 if self.rand.odom_heading_drift else 0.0)
        self._odom_heading = (self._odom_heading + math.degrees(d_theta) + drift) % 360.0

        return self._obs(math.hypot(st.x - x0, st.y - y0) / (self.substeps * self.dt),
                         collided)

    # ------------------------------------------------------------------

    def _obs(self, speed_cms: float, collided: int) -> dict:
        st = self.state
        us = list(st.sensor_readings) if st.sensor_readings else []
        ir = list(st.ir_readings) if st.ir_readings else []
        return {
            # What the firmware sends (policy MAY use):
            "us": us,                       # cm, one per ultrasonic sensor
            "ir": ir,                       # 0-1023, one per IR sensor
            "odom_heading": self._odom_heading % 360.0,   # deg, relative to start
            "speed": speed_cms,             # cm/s
            "distance": self._odom_dist,    # cm since start
            "elapsed": self._elapsed,       # s
            "enc_l": self._enc_l,
            "enc_r": self._enc_r,
            # Privileged (expert + metrics ONLY — never a policy feature):
            "true_x": st.x,
            "true_y": st.y,
            "true_heading": st.heading,
            "collided": collided,           # physics frames in contact this step
            "collided_total": self._collided_frames,
        }

    # ------------------------------------------------------------------

    def dist_to_goal(self) -> float:
        return math.hypot(self.track.goal_x - self.state.x,
                          self.track.goal_y - self.state.y)


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0
