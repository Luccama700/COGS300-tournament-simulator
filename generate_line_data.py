"""
Optimal line-following controller with data generation.

Uses A* to find the shortest path through the line network,
then a PID controller to follow each segment precisely.
Logs IR sensor readings + steering commands as training data.

Usage:
    python generate_line_data.py --track configs/track.yaml --robot configs/robot-config.yaml --output data/line_train.csv
    python generate_line_data.py --track configs/track.yaml --robot configs/robot-config.yaml --output data/line_train.csv --visualize
"""

import argparse
import csv
import json
import math
import os
import random
import sys
from dataclasses import dataclass, field
from typing import Optional

from track import load_track, build_line_graph, find_nearest_node, astar_line_path, LineSeg
from robot_config import load_robot_config, RobotConfig


@dataclass
class LineFollowState:
    x: float
    y: float
    heading: float          # degrees, 0=east, CCW positive
    speed: float = 0.0
    # Odometry (replicable from wheel encoders + clock on real Arduino)
    distance_traveled: float = 0.0  # cumulative cm since episode start
    elapsed_time: float = 0.0       # seconds since episode start
    estimated_x: float = 0.0        # dead-reckoning x (drifts over time)
    estimated_y: float = 0.0        # dead-reckoning y (drifts over time)
    # Internal: accumulated heading used for dead reckoning — starts at
    # actual heading and drifts each step to simulate encoder imprecision.
    # Not written to CSV; reset to actual heading at episode start.
    dr_heading: float = field(default=0.0, repr=False)


@dataclass
class SteeringCommand:
    """
    Normalized steering output.
    throttle: -1.0 to 1.0 (forward/reverse)
    steering: -1.0 to 1.0 (right/left)
    """
    throttle: float
    steering: float

    def to_label(self) -> int:
        """Discretize into command classes for classification."""
        # 0=forward, 1=slight_left, 2=slight_right, 3=hard_left, 4=hard_right, 5=stop
        if abs(self.throttle) < 0.05:
            return 5  # stop
        if abs(self.steering) < 0.1:
            return 0  # forward
        if self.steering > 0.4:
            return 3  # hard left
        if self.steering < -0.4:
            return 4  # hard right
        if self.steering > 0:
            return 1  # slight left
        return 2  # slight right


@dataclass
class Perturbation:
    """Controls how much noise is injected per episode and per step."""
    # Episode-level (applied once at episode start)
    start_pos_std: float = 0.0          # cm — Gaussian offset to starting position
    start_heading_std: float = 0.0      # degrees — Gaussian heading misalignment
    speed_range: tuple = (1.0, 1.0)     # uniform multiplier on base speed
    pid_gain_range: tuple = (1.0, 1.0)  # uniform multiplier on each PID gain
    # Per-step
    heading_drift_std: float = 0.0      # degrees/step — wheel slip / surface variation
    speed_jitter_std: float = 0.0       # fraction — per-step speed noise
    nudge_prob: float = 0.0             # probability of a positional bump per step
    nudge_std: float = 0.0             # cm — bump magnitude


# Perturbation presets
_PERTURBATIONS: dict[str, Perturbation] = {
    "none": Perturbation(),
    "mild": Perturbation(
        start_pos_std=3.0,
        start_heading_std=8.0,
        speed_range=(0.7, 1.3),
        pid_gain_range=(0.7, 1.3),
        heading_drift_std=0.5,
        speed_jitter_std=0.05,
        nudge_prob=0.01,
        nudge_std=2.0,
    ),
    "heavy": Perturbation(
        start_pos_std=6.0,
        start_heading_std=16.0,
        speed_range=(0.4, 1.6),
        pid_gain_range=(0.4, 1.6),
        heading_drift_std=1.0,
        speed_jitter_std=0.10,
        nudge_prob=0.02,
        nudge_std=4.0,
    ),
}


class PIDController:
    def __init__(self, kp: float = 0.8, ki: float = 0.01, kd: float = 0.3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error: float, dt: float) -> float:
        self.integral += error * dt
        self.integral = max(-50, min(50, self.integral))  # anti-windup
        derivative = (error - self.prev_error) / max(dt, 0.001)
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class OptimalLineFollower:
    """
    Generates optimal steering commands for line following.

    1. Builds a graph from the track's line segments
    2. Finds the shortest A* path from start to goal
    3. Follows each segment in the path using PID
    4. Logs sensor readings + commands at each step
    """

    def __init__(self, track_path: str, robot_path: str):
        self.track = load_track(track_path)
        self.robot_cfg = load_robot_config(robot_path)

        # Build line graph and find optimal path
        self.nodes, self.point_to_node = build_line_graph(self.track)
        start_node = find_nearest_node(self.nodes, self.track.start_x, self.track.start_y)
        goal_node = find_nearest_node(self.nodes, self.track.goal_x, self.track.goal_y)

        self.optimal_path = astar_line_path(self.nodes, start_node, goal_node)
        if not self.optimal_path:
            print("WARNING: No path found from start to goal!")
            self.waypoints = []
        else:
            # Convert node path to waypoint coordinates
            self.waypoints = [(self.nodes[nid].x, self.nodes[nid].y) for nid in self.optimal_path]
            print(f"Optimal path: {len(self.optimal_path)} nodes, {len(self.waypoints)} waypoints")

        self.pid = PIDController()
        self.current_waypoint_idx = 0
        self.speed = 40.0  # cm/s — moderate speed for line following

        # Base values — episode randomization jitters from these, never modifies them
        self._base_speed = self.speed
        self._base_kp = self.pid.kp
        self._base_ki = self.pid.ki
        self._base_kd = self.pid.kd

        # Active perturbation profile (set before each episode or viz run)
        self._perturbation: Perturbation = _PERTURBATIONS["none"]

    def simulate_ir_reading(self, sensor_x: float, sensor_y: float) -> float:
        """
        Simulate a TCRT5000 IR reading at a world position.
        Returns analog value 0-1023.
        High (~800-950) = on line, Low (~50-200) = off line.
        """
        import random
        on_line = self.track.is_on_line(sensor_x, sensor_y)
        if on_line:
            return min(1023, max(0, random.gauss(880, 30)))
        else:
            return min(1023, max(0, random.gauss(120, 25)))

    def simulate_ultrasonic_reading(self, sensor_x: float, sensor_y: float, angle_deg: float, max_range: float) -> float:
        """Simplified ultrasonic reading against track walls."""
        import random
        from physics import Wall

        best = max_range
        rad = math.radians(angle_deg)
        dx = math.cos(rad)
        dy = -math.sin(rad)

        for w in self.track.walls:
            # Ray-segment intersection
            wx1, wy1, wx2, wy2 = w
            sx, sy = wx2 - wx1, wy2 - wy1
            denom = dx * sy - dy * sx
            if abs(denom) < 1e-9:
                continue
            ox, oy = wx1 - sensor_x, wy1 - sensor_y
            t = (ox * sy - oy * sx) / denom
            u = (ox * dy - oy * dx) / denom
            if t >= 0 and 0 <= u <= 1 and t < best:
                best = t

        # Add noise
        if best < max_range:
            best += random.gauss(0, 0.5 + 0.008 * best)
            best = max(2.0, min(max_range, best))
        return best

    def get_ir_readings(self, state: LineFollowState) -> list[float]:
        """Get IR sensor readings for current robot state."""
        readings = []
        for ir in self.robot_cfg.ir_sensors if hasattr(self.robot_cfg, 'ir_sensors') else []:
            sx, sy = self.robot_cfg.ir_sensor_world_position(ir, state.x, state.y, state.heading)
            readings.append(self.simulate_ir_reading(sx, sy))

        # If no IR sensors in config, simulate default front-left and front-right
        if not readings:
            rad = math.radians(state.heading)
            cos_h, sin_h = math.cos(rad), math.sin(rad)
            hw = self.robot_cfg.half_width
            hl = self.robot_cfg.half_length

            # Front-left IR
            fwd, right = hl * 0.96, -hw * 0.7
            fl_x = state.x + fwd * cos_h + right * sin_h
            fl_y = state.y - fwd * sin_h + right * cos_h
            readings.append(self.simulate_ir_reading(fl_x, fl_y))

            # Front-right IR
            right = hw * 0.7
            fr_x = state.x + fwd * cos_h + right * sin_h
            fr_y = state.y - fwd * sin_h + right * cos_h
            readings.append(self.simulate_ir_reading(fr_x, fr_y))

        return readings

    def get_ultrasonic_readings(self, state: LineFollowState) -> list[float]:
        """Get ultrasonic sensor readings for current robot state."""
        readings = []
        for sensor in self.robot_cfg.sensors:
            sx, sy = self.robot_cfg.sensor_world_position(sensor, state.x, state.y, state.heading)
            angle = self.robot_cfg.sensor_world_angle(sensor, state.heading)
            readings.append(self.simulate_ultrasonic_reading(sx, sy, angle, sensor.range_cm))
        return readings

    def compute_steering(self, state: LineFollowState, dt: float) -> SteeringCommand:
        """
        Compute the optimal steering command.
        Uses waypoints from A* and PID to track the path.
        """
        if self.current_waypoint_idx >= len(self.waypoints):
            return SteeringCommand(0.0, 0.0)  # reached goal

        # Current target waypoint
        tx, ty = self.waypoints[self.current_waypoint_idx]
        dist_to_wp = math.hypot(tx - state.x, ty - state.y)

        # Advance waypoint if close enough
        while dist_to_wp < 8.0 and self.current_waypoint_idx < len(self.waypoints) - 1:
            self.current_waypoint_idx += 1
            tx, ty = self.waypoints[self.current_waypoint_idx]
            dist_to_wp = math.hypot(tx - state.x, ty - state.y)

        # At goal?
        if self.current_waypoint_idx >= len(self.waypoints) - 1 and dist_to_wp < 5.0:
            return SteeringCommand(0.0, 0.0)

        # Compute heading error
        target_angle = math.degrees(math.atan2(-(ty - state.y), tx - state.x))
        heading_error = (target_angle - state.heading + 180) % 360 - 180

        # PID steering
        steering = self.pid.update(heading_error, dt)
        steering = max(-1.0, min(1.0, steering / 45.0))  # normalize to [-1, 1]

        # Slow down for sharp turns
        throttle = 1.0
        if abs(heading_error) > 30:
            throttle = 0.5
        if abs(heading_error) > 60:
            throttle = 0.3

        return SteeringCommand(throttle, steering)

    def step(self, state: LineFollowState, dt: float) -> tuple[LineFollowState, SteeringCommand]:
        """Advance one simulation step. Returns updated state and command."""
        cmd = self.compute_steering(state, dt)
        p = self._perturbation

        # Heading change this frame (same for actual and dead-reckoning base)
        delta_heading = cmd.steering * 120.0 * dt

        # Apply command to actual state
        state.heading = (state.heading + delta_heading) % 360.0

        # Per-step speed jitter — simulates uneven motor output / surface friction
        speed_scale = random.gauss(1.0, p.speed_jitter_std) if p.speed_jitter_std else 1.0
        state.speed = cmd.throttle * self.speed * speed_scale

        rad = math.radians(state.heading)
        displacement = state.speed * dt
        state.x += math.cos(rad) * displacement
        state.y -= math.sin(rad) * displacement

        # Per-step heading drift — wheel slip / surface variation
        if p.heading_drift_std:
            drift = random.gauss(0, p.heading_drift_std)
            state.heading = (state.heading + drift) % 360.0

        # Occasional bump — surface imperfection / collision nudge
        if p.nudge_prob and random.random() < p.nudge_prob:
            state.x += random.gauss(0, p.nudge_std)
            state.y += random.gauss(0, p.nudge_std)

        # ── Odometry (wheel-encoder simulation) ──────────────────────────────
        state.elapsed_time += dt
        state.distance_traveled += abs(displacement)

        # Dead reckoning: same heading change as actual, plus small drift
        # (0.3° std per step — small errors compound over the run)
        state.dr_heading = (state.dr_heading + delta_heading + random.gauss(0, 0.3)) % 360.0
        dr_rad = math.radians(state.dr_heading)
        # 2% distance noise — simulates encoder slip / wheel radius uncertainty
        drifted_disp = displacement * random.gauss(1.0, 0.02)
        state.estimated_x += math.cos(dr_rad) * drifted_disp
        state.estimated_y -= math.sin(dr_rad) * drifted_disp

        return state, cmd

    def generate_data(
        self,
        dt: float = 1/60,
        max_steps: int = 20000,
        episode: int = 0,
        seed: Optional[int] = None,
        writer: Optional[csv.writer] = None,
        write_header: bool = True,
        rows_start: int = 0,
        output_path: Optional[str] = None,
    ) -> tuple[int, dict]:
        """
        Run one episode and write rows to `writer` (or a new file at `output_path`).
        Returns (rows_written, metrics_dict).
        """
        # Seed per-episode for reproducibility (seed=None → truly random)
        if seed is not None:
            random.seed(seed + episode)

        p = self._perturbation

        # ── Episode-level randomization ───────────────────────────────────────
        start_x = self.track.start_x + (random.gauss(0, p.start_pos_std) if p.start_pos_std else 0)
        start_y = self.track.start_y + (random.gauss(0, p.start_pos_std) if p.start_pos_std else 0)
        start_h = self.track.start_heading + (random.gauss(0, p.start_heading_std) if p.start_heading_std else 0)
        self.speed = self._base_speed * random.uniform(*p.speed_range)
        kp_mult, ki_mult, kd_mult = (random.uniform(*p.pid_gain_range) for _ in range(3))
        self.pid.kp = self._base_kp * kp_mult
        self.pid.ki = self._base_ki * ki_mult
        self.pid.kd = self._base_kd * kd_mult

        state = LineFollowState(
            x=start_x, y=start_y, heading=start_h,
            estimated_x=start_x, estimated_y=start_y, dr_heading=start_h,
        )

        n_ir = len(self.get_ir_readings(state))
        n_us = len(self.robot_cfg.sensors)

        # Header — full column order:
        #   ir_0..ir_N, us_0..us_N, heading, speed,
        #   distance_traveled, elapsed_time, est_x, est_y,
        #   throttle, steering, command_label, episode_id
        #
        # Normalization hints for training pipeline:
        #   distance_traveled : 0 – ~2000 cm
        #   elapsed_time      : 0 – ~60 s
        #   est_x, est_y      : ~-200 – 400 cm (approximate; drifts)
        header = [f"ir_{i}" for i in range(n_ir)]
        header += [f"us_{i}" for i in range(n_us)]
        header += ["heading", "speed",
                   "distance_traveled", "elapsed_time", "est_x", "est_y",
                   "throttle", "steering", "command_label", "episode_id"]

        # File management — use provided writer or open own file
        own_file = None
        if writer is None:
            path = output_path or "data/line_train.csv"
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            own_file = open(path, "w", newline="")
            writer = csv.writer(own_file)
            write_header = True

        if write_header:
            writer.writerow(header)

        # ── Per-episode metric accumulators ──────────────────────────────────
        count = 0
        log_interval = 3
        reached_goal = False
        off_line_frames = 0
        steering_magnitudes: list[float] = []
        max_deviation = 0.0

        try:
            for step_i in range(max_steps):
                ir_readings = self.get_ir_readings(state)
                us_readings = self.get_ultrasonic_readings(state)

                state, cmd = self.step(state, dt)

                # Quality metrics (measured every step, not just logged steps)
                _, nearest_dist = self.track.nearest_line_segment(state.x, state.y)
                if nearest_dist > self.track.line_paths[0].width_cm / 2 if self.track.line_paths else 5.0:
                    off_line_frames += 1
                max_deviation = max(max_deviation, nearest_dist)
                steering_magnitudes.append(abs(cmd.steering))

                if step_i % log_interval == 0:
                    row = [f"{r:.0f}" for r in ir_readings]
                    row += [f"{r:.1f}" for r in us_readings]
                    row += [f"{state.heading:.1f}", f"{state.speed:.1f}"]
                    row += [f"{state.distance_traveled:.1f}",
                            f"{state.elapsed_time:.2f}",
                            f"{state.estimated_x:.1f}",
                            f"{state.estimated_y:.1f}"]
                    row += [f"{cmd.throttle:.3f}", f"{cmd.steering:.3f}"]
                    row += [str(cmd.to_label()), str(episode)]
                    writer.writerow(row)
                    count += 1

                dist_to_goal = math.hypot(self.track.goal_x - state.x, self.track.goal_y - state.y)
                if dist_to_goal < 5.0:
                    reached_goal = True
                    print(f"  ep{episode}: reached goal at step {step_i}")
                    break

                if nearest_dist > 50.0:
                    print(f"  ep{episode}: too far from line at step {step_i}, stopping.")
                    break
        finally:
            if own_file:
                own_file.close()

        avg_steering = sum(steering_magnitudes) / len(steering_magnitudes) if steering_magnitudes else 0.0
        metrics = {
            "id": episode,
            "steps": step_i + 1,
            "time": round(state.elapsed_time, 2),
            "reached_goal": reached_goal,
            "off_line_frames": off_line_frames,
            "avg_steering": round(avg_steering, 4),
            "max_deviation": round(max_deviation, 2),
            "rows_start": rows_start,
            "rows_end": rows_start + count,
        }
        return count, metrics


def run_with_visualization(
    follower: OptimalLineFollower,
    dt: float = 1/60,
    perturbation: Optional[Perturbation] = None,
    seed: Optional[int] = None,
):
    """Run the optimal controller with Pygame visualization."""
    import pygame

    # Apply perturbation profile (episode 0, optional seed)
    p = perturbation or _PERTURBATIONS["none"]
    follower._perturbation = p
    if seed is not None:
        random.seed(seed)
    follower.speed = follower._base_speed * random.uniform(*p.speed_range)
    follower.pid.kp = follower._base_kp * random.uniform(*p.pid_gain_range)
    follower.pid.ki = follower._base_ki * random.uniform(*p.pid_gain_range)
    follower.pid.kd = follower._base_kd * random.uniform(*p.pid_gain_range)

    start_x = follower.track.start_x + (random.gauss(0, p.start_pos_std) if p.start_pos_std else 0)
    start_y = follower.track.start_y + (random.gauss(0, p.start_pos_std) if p.start_pos_std else 0)
    start_h = follower.track.start_heading + (random.gauss(0, p.start_heading_std) if p.start_heading_std else 0)
    perturb_label = next(k for k, v in _PERTURBATIONS.items() if v is p) if p in _PERTURBATIONS.values() else "custom"

    pygame.init()
    W, H = 1000, 700
    screen = pygame.display.set_mode((W, H), pygame.RESIZABLE)
    pygame.display.set_caption("Optimal Line Follower — Data Generation")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("menlo", 12)

    ppcm = 3.0
    cam_x, cam_y = W / 2, H / 2

    state = LineFollowState(
        x=start_x,
        y=start_y,
        heading=start_h,
        estimated_x=start_x,
        estimated_y=start_y,
        dr_heading=start_h,
    )

    step_count = 0
    running = True
    paused = False
    follow = True
    speed_mult = 1
    # Dead-reckoning trail: store every Nth estimated position for display
    dr_trail: list[tuple[float, float]] = []
    DR_TRAIL_INTERVAL = 6  # frames between trail samples

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_f:
                    follow = not follow
                elif event.key == pygame.K_EQUALS:
                    ppcm = min(10, ppcm + 0.5)
                elif event.key == pygame.K_MINUS:
                    ppcm = max(0.5, ppcm - 0.5)
                elif event.key == pygame.K_RIGHT:
                    speed_mult = min(10, speed_mult + 1)
                elif event.key == pygame.K_LEFT:
                    speed_mult = max(1, speed_mult - 1)

        if not paused:
            for _ in range(speed_mult):
                state, cmd = follower.step(state, dt)
                step_count += 1

                if step_count % DR_TRAIL_INTERVAL == 0:
                    dr_trail.append((state.estimated_x, state.estimated_y))

                dist_to_goal = math.hypot(follower.track.goal_x - state.x, follower.track.goal_y - state.y)
                if dist_to_goal < 5.0:
                    paused = True
                    break

        # Camera
        if follow:
            cam_x = W / 2 - state.x * ppcm
            cam_y = H / 2 - state.y * ppcm

        def w2s(wx, wy):
            return (int(cam_x + wx * ppcm), int(cam_y + wy * ppcm))

        # Draw
        screen.fill((245, 245, 240))

        # Line paths (black strips)
        for seg in follower.track.line_paths:
            p1 = w2s(seg.x1, seg.y1)
            p2 = w2s(seg.x2, seg.y2)
            width = max(2, int(seg.width_cm * ppcm))
            pygame.draw.line(screen, (40, 40, 40), p1, p2, width)

        # Walls
        for w in follower.track.walls:
            p1 = w2s(w[0], w[1])
            p2 = w2s(w[2], w[3])
            pygame.draw.line(screen, (60, 60, 65), p1, p2, max(2, int(ppcm * 0.8)))

        # Waypoints
        for i, (wx, wy) in enumerate(follower.waypoints):
            color = (100, 200, 100) if i <= follower.current_waypoint_idx else (200, 200, 100)
            p = w2s(wx, wy)
            pygame.draw.circle(screen, color, p, 3)

        # Start / goal
        sp = w2s(follower.track.start_x, follower.track.start_y)
        gp = w2s(follower.track.goal_x, follower.track.goal_y)
        pygame.draw.circle(screen, (50, 200, 80), sp, 8, 2)
        pygame.draw.circle(screen, (220, 80, 50), gp, 8, 2)

        # Dead-reckoning ghost trail (dotted — every other dot drawn)
        for i, (ex, ey) in enumerate(dr_trail):
            if i % 2 == 0:
                pygame.draw.circle(screen, (160, 100, 210), w2s(ex, ey), max(1, int(ppcm * 0.35)))
        # Ghost position marker
        if dr_trail:
            pygame.draw.circle(screen, (180, 120, 230), w2s(*dr_trail[-1]), max(3, int(ppcm * 0.6)), 1)

        # Robot
        rp = w2s(state.x, state.y)
        pygame.draw.circle(screen, (220, 70, 50), rp, max(4, int(6 * ppcm / 3)))
        rad = math.radians(state.heading)
        tip = (rp[0] + int(math.cos(rad) * 12), rp[1] - int(math.sin(rad) * 12))
        pygame.draw.line(screen, (255, 255, 255), rp, tip, 2)

        # IR sensor indicators
        ir_readings = follower.get_ir_readings(state)
        ir_str = "  ".join(f"IR{i}:{r:.0f}" for i, r in enumerate(ir_readings))

        # HUD
        dr_err = math.hypot(state.estimated_x - state.x, state.estimated_y - state.y)
        lines = [
            f"Step: {step_count}  Pos: ({state.x:.0f}, {state.y:.0f})  Heading: {state.heading:.0f}°",
            f"DR est: ({state.estimated_x:.0f}, {state.estimated_y:.0f})  drift: {dr_err:.1f}cm  t: {state.elapsed_time:.1f}s  dist: {state.distance_traveled:.0f}cm",
            f"WP: {follower.current_waypoint_idx}/{len(follower.waypoints)}  Speed: {speed_mult}x  Perturb: {perturb_label}  {'PAUSED' if paused else 'RUNNING'}",
            f"Sensors: {ir_str}",
            "Space=pause  F=follow  +/-=zoom  Left/Right=speed  Q=quit",
        ]
        hud_w = max(font.size(l)[0] for l in lines) + 20
        hud_h = len(lines) * 18 + 12
        hud_s = pygame.Surface((hud_w, hud_h), pygame.SRCALPHA)
        pygame.draw.rect(hud_s, (20, 20, 25, 200), (0, 0, hud_w, hud_h), border_radius=6)
        screen.blit(hud_s, (12, 12))
        for i, line in enumerate(lines):
            text = font.render(line, True, (200, 200, 195))
            screen.blit(text, (22, 18 + i * 18))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


def main():
    parser = argparse.ArgumentParser(description="Generate line-following training data")
    parser.add_argument("--track", default="configs/track.yaml", help="Track YAML file")
    parser.add_argument("--robot", default="configs/robot-config.yaml", help="Robot config YAML")
    parser.add_argument("--output", default="data/line_train.csv", help="Output CSV path")
    parser.add_argument("--episodes", type=int, default=1, help="Number of episodes to generate")
    parser.add_argument("--visualize", action="store_true", help="Show Pygame visualization")
    parser.add_argument("--perturbation", choices=["none", "mild", "heavy"], default="none",
                        help="Noise level: none=deterministic, mild=realistic variation, heavy=robust training")
    parser.add_argument("--seed", type=int, default=None,
                        help="Base random seed for reproducibility (episode N uses seed+N)")
    args = parser.parse_args()

    print(f"Loading track: {args.track}")
    print(f"Loading robot: {args.robot}")
    print(f"Perturbation: {args.perturbation}" + (f"  seed: {args.seed}" if args.seed is not None else ""))

    follower = OptimalLineFollower(args.track, args.robot)
    perturbation = _PERTURBATIONS[args.perturbation]
    follower._perturbation = perturbation

    if args.visualize:
        print("\nVisualization mode — watch the optimal controller drive the track.")
        run_with_visualization(follower, perturbation=perturbation, seed=args.seed)
    else:
        csv_path  = args.output
        meta_path = args.output.replace(".csv", "_meta.json")
        os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)

        all_metrics: list[dict] = []
        total_rows = 0

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            for ep in range(args.episodes):
                print(f"Episode {ep + 1}/{args.episodes}", end="  ", flush=True)
                follower.current_waypoint_idx = 0
                follower.pid.reset()
                rows, metrics = follower.generate_data(
                    episode=ep,
                    seed=args.seed,
                    writer=writer,
                    write_header=(ep == 0),
                    rows_start=total_rows,
                )
                total_rows += rows
                all_metrics.append(metrics)
                status = "✓" if metrics["reached_goal"] else "✗"
                print(f"{status}  steps={metrics['steps']}  off_line={metrics['off_line_frames']}  avg_steer={metrics['avg_steering']:.3f}")

        meta = {
            "episodes": all_metrics,
            "total_episodes": args.episodes,
            "total_rows": total_rows,
            "reached_goal_count": sum(1 for m in all_metrics if m["reached_goal"]),
        }
        with open(meta_path, "w") as f:
            json.dump(meta, f, indent=2)

        reached = meta["reached_goal_count"]
        print(f"\nDone! {total_rows} rows across {args.episodes} episodes ({reached} reached goal)")
        print(f"  Data:     {csv_path}")
        print(f"  Metadata: {meta_path}")


if __name__ == "__main__":
    main()
