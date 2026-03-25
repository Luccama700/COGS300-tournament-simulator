"""
Physics engine for the maze robot simulation.

Handles:
- Differential-drive kinematics (2WD + caster)
- Servo-steer / 4WD kinematics (if configured)
- Wall collision detection and response
- Ultrasonic sensor raycasting with noise
"""

import math
from dataclasses import dataclass, field
import numpy as np
import yaml
from robot_config import RobotConfig
from sensor_model import SensorSimulator, SensorNoiseProfile, HC_SR04_DEFAULT
from ir_model import IRSensorSimulator, TrackLine


@dataclass
class PhysicsParams:
    # Motor model (matching Arduino firmware)
    max_wheel_speed_cmps: float = 60.0  # cm/s at PWM=255; BASE_SPEED=150 → ~35 cm/s
    motor_tau: float = 0.08             # motor inertia time constant (seconds)

    # Collision
    wall_bounce: float = 0.2
    wall_slide_friction: float = 0.5
    collision_margin: float = 1.0

    # Sensor noise (HC-SR04 simulation)
    sensor_noise_std: float = 2.0
    sensor_dropout_chance: float = 0.02
    sensor_dropout_value: float = 0.0
    sensor_min_range: float = 2.0

    # Legacy fields kept so old YAML files don't break on load
    max_speed: float = 80.0
    min_speed: float = -40.0
    acceleration: float = 120.0
    deceleration: float = 200.0
    idle_deceleration: float = 60.0
    max_turn_rate: float = 180.0
    turn_acceleration: float = 360.0
    turn_deceleration: float = 540.0


# Arduino firmware motor command table (mirrors executeCommand() in robot_firmware.ino)
_BASE_PWM = 150  # BASE_SPEED in firmware
MOTOR_COMMANDS: dict[int, tuple[int, int]] = {
    0: ( _BASE_PWM,                  _BASE_PWM),           # forward
    1: (int(_BASE_PWM * 0.6),        _BASE_PWM),           # slight left
    2: ( _BASE_PWM,                  int(_BASE_PWM * 0.6)),# slight right
    3: (-int(_BASE_PWM * 0.3),       _BASE_PWM),           # hard left
    4: ( _BASE_PWM,                  -int(_BASE_PWM * 0.3)),# hard right
    5: (0,                           0),                    # stop
}
COMMAND_NAMES = {0: "FORWARD", 1: "SLIGHT LEFT", 2: "SLIGHT RIGHT",
                 3: "HARD LEFT", 4: "HARD RIGHT", 5: "STOP"}


def load_physics_params(path: str) -> tuple[PhysicsParams, dict]:
    with open(path) as f:
        raw = yaml.safe_load(f)
    p = raw["physics"]
    params = PhysicsParams(**{k: v for k, v in p.items() if hasattr(PhysicsParams, k)})
    return params, raw.get("display", {})


# --- Wall definitions -------------------------------------------------------

@dataclass
class Wall:
    """A line segment wall from (x1,y1) to (x2,y2) in world cm."""
    x1: float
    y1: float
    x2: float
    y2: float


def make_box_walls(x: float, y: float, w: float, h: float) -> list[Wall]:
    """Create 4 walls forming a rectangle."""
    return [
        Wall(x, y, x + w, y),         # top
        Wall(x + w, y, x + w, y + h), # right
        Wall(x + w, y + h, x, y + h), # bottom
        Wall(x, y + h, x, y),         # left
    ]


# --- Robot state ------------------------------------------------------------

@dataclass
class RobotState:
    x: float = 0.0           # world position cm
    y: float = 0.0
    heading: float = 90.0    # degrees, 0=east, 90=north (screen up)
    speed: float = 0.0       # cm/s — center speed (derived from wheel speeds)

    # Per-wheel state (mirrors Arduino L298N + encoder model)
    left_pwm:  int   = 0     # commanded PWM (-255 to 255)
    right_pwm: int   = 0
    left_wheel_speed:  float = 0.0  # actual wheel speed cm/s (lagged by inertia)
    right_wheel_speed: float = 0.0

    # Current command (0-5, matches Arduino executeCommand() IDs)
    command: int = 5  # start stopped

    # Current sensor readings (updated by physics step)
    sensor_readings: list[float] = field(default_factory=list)
    ir_readings: list[float] = field(default_factory=list)


# --- Physics step -----------------------------------------------------------

class PhysicsEngine:
    def __init__(self, robot_cfg: RobotConfig, params: PhysicsParams, walls: list[Wall], noise_profile: SensorNoiseProfile | None = None, track_lines: list[TrackLine] | None = None):
        self.robot_cfg = robot_cfg
        self.params = params
        self.walls = walls
        self.sensor_sim = SensorSimulator(noise_profile or HC_SR04_DEFAULT)
        self.track_lines = track_lines or []
        self.ir_sim = IRSensorSimulator()

        # Precompute wall segment arrays for fast raycasting
        self._wall_starts = np.array([[w.x1, w.y1] for w in walls], dtype=np.float64)
        self._wall_ends = np.array([[w.x2, w.y2] for w in walls], dtype=np.float64)

    def update_walls(self, walls: list[Wall]):
        """Hot-swap walls (for track changes)."""
        self.walls = walls
        self._wall_starts = np.array([[w.x1, w.y1] for w in walls], dtype=np.float64)
        self._wall_ends = np.array([[w.x2, w.y2] for w in walls], dtype=np.float64)

    def step(self, state: RobotState, command: int, dt: float) -> RobotState:
        """
        Advance physics by dt seconds using Arduino motor commands.

        command: integer 0-5 matching Arduino executeCommand() IDs:
            0 = forward, 1 = slight left, 2 = slight right,
            3 = hard left, 4 = hard right, 5 = stop

        Replicates the exact differential-drive kinematics used in
        robot_firmware.ino updateOdometry() so sim behaviour matches hardware.
        """
        p = self.params
        state.command = command

        # --- Motor command → target PWM (mirrors Arduino executeCommand) ---
        left_pwm, right_pwm = MOTOR_COMMANDS.get(command, (0, 0))
        state.left_pwm  = left_pwm
        state.right_pwm = right_pwm

        # --- PWM → target wheel speed (cm/s, signed) ---
        target_left  = left_pwm  / 255.0 * p.max_wheel_speed_cmps
        target_right = right_pwm / 255.0 * p.max_wheel_speed_cmps

        # --- Motor inertia: first-order lag (τ = motor_tau seconds) ---
        alpha = 1.0 - math.exp(-dt / p.motor_tau) if p.motor_tau > 0 else 1.0
        state.left_wheel_speed  += (target_left  - state.left_wheel_speed)  * alpha
        state.right_wheel_speed += (target_right - state.right_wheel_speed) * alpha

        # --- Differential-drive kinematics (matches Arduino updateOdometry) ---
        d_left  = state.left_wheel_speed  * dt  # cm travelled by left wheel
        d_right = state.right_wheel_speed * dt  # cm travelled by right wheel
        d_center = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / self.robot_cfg.chassis.wheelbase_cm  # radians

        state.heading = (state.heading + math.degrees(d_theta)) % 360.0
        state.speed   = d_center / dt if dt > 0 else 0.0

        rad  = math.radians(state.heading)
        new_x = state.x + math.cos(rad) * d_center
        new_y = state.y - math.sin(rad) * d_center  # screen Y inverted

        # --- Collision ---
        margin = p.collision_margin + max(self.robot_cfg.half_length, self.robot_cfg.half_width)
        if self._check_collision(new_x, new_y, margin):
            # Try sliding along each axis independently
            if not self._check_collision(new_x, state.y, margin):
                state.x = new_x
                state.left_wheel_speed  *= p.wall_slide_friction
                state.right_wheel_speed *= p.wall_slide_friction
            elif not self._check_collision(state.x, new_y, margin):
                state.y = new_y
                state.left_wheel_speed  *= p.wall_slide_friction
                state.right_wheel_speed *= p.wall_slide_friction
            else:
                # Full stall — wheels bounce back slightly
                state.left_wheel_speed  *= -p.wall_bounce
                state.right_wheel_speed *= -p.wall_bounce
        else:
            state.x = new_x
            state.y = new_y

        # --- Sensor update ---
        state.sensor_readings = self._read_sensors(state)
        state.ir_readings     = self._read_ir_sensors(state)

        return state

    def _check_collision(self, x: float, y: float, radius: float) -> bool:
        """Check if a circle at (x,y) with given radius hits any wall."""
        if len(self.walls) == 0:
            return False
        point = np.array([x, y])
        ab = self._wall_ends - self._wall_starts
        ap = point - self._wall_starts
        ab_sq = np.sum(ab * ab, axis=1) + 1e-12
        t = np.clip(np.sum(ap * ab, axis=1) / ab_sq, 0.0, 1.0)
        closest = self._wall_starts + t[:, np.newaxis] * ab
        dists = np.sqrt(np.sum((point - closest) ** 2, axis=1))
        return bool(np.any(dists < radius))

    def _read_sensors(self, state: RobotState) -> list[float]:
        """Raycast all sensors and apply noise."""
        readings = []
        for sensor in self.robot_cfg.sensors:
            sx, sy = self.robot_cfg.sensor_world_position(sensor, state.x, state.y, state.heading)
            world_angle = self.robot_cfg.sensor_world_angle(sensor, state.heading)
            reading = self.sensor_sim.measure_with_cone(
                raycast_fn=self._raycast,
                origin_x=sx, origin_y=sy,
                center_angle_deg=world_angle,
                max_range=sensor.range_cm,
            )
            readings.append(round(reading, 1))
        return readings

    def _read_ir_sensors(self, state: RobotState) -> list[float]:
        """Read all IR sensors against the current track lines."""
        readings = []
        for ir in self.robot_cfg.ir_sensors:
            sx, sy = self.robot_cfg.ir_sensor_world_position(ir, state.x, state.y, state.heading)
            reading = self.ir_sim.measure(sx, sy, self.track_lines)
            readings.append(round(reading, 1))
        return readings

    def _raycast(self, ox: float, oy: float, angle_deg: float, max_dist: float) -> float:
        """Cast a ray, return distance to nearest wall."""
        if len(self.walls) == 0:
            return max_dist

        rad = math.radians(angle_deg)
        dx = math.cos(rad)
        dy = -math.sin(rad)
        direction = np.array([dx, dy])
        origin = np.array([ox, oy])

        best = max_dist

        seg_d = self._wall_ends - self._wall_starts  # (N, 2)
        oc = self._wall_starts - origin               # (N, 2)

        denom = direction[0] * seg_d[:, 1] - direction[1] * seg_d[:, 0]
        valid = np.abs(denom) > 1e-9

        t = np.full(len(self.walls), max_dist + 1)
        u = np.full(len(self.walls), -1.0)

        t[valid] = (oc[valid, 0] * seg_d[valid, 1] - oc[valid, 1] * seg_d[valid, 0]) / denom[valid]
        u[valid] = (oc[valid, 0] * direction[1] - oc[valid, 1] * direction[0]) / denom[valid]

        hits = valid & (t >= 0) & (u >= 0) & (u <= 1) & (t < best)
        if np.any(hits):
            best = float(np.min(t[hits]))

        return best

    def get_collision_radius(self) -> float:
        """Effective collision radius for the robot."""
        return max(self.robot_cfg.half_length, self.robot_cfg.half_width) + self.params.collision_margin
