"""
Physics test harness — drive the robot around with WASD in a Pygame window.

Usage:
    python run_test.py
    python run_test.py --robot configs/my-other-robot.yaml
    python run_test.py --physics configs/physics-fast.yaml
    python run_test.py --track configs/track.yaml
    python run_test.py --editor
    python run_test.py --editor --track configs/track.yaml

Controls:
    W/S     = throttle forward/reverse
    A/D     = steer left/right
    F       = toggle camera follow
    R       = reset robot position
    +/-     = zoom in/out
    Q/Esc   = quit
"""

import argparse
import sys
import pygame

from robot_config import load_robot_config
from physics import PhysicsEngine, PhysicsParams, RobotState, Wall, make_box_walls, load_physics_params
from sensor_model import load_noise_profile, HC_SR04_DEFAULT, HC_SR04_NOISY, HC_SR04_CLEAN
from ir_model import TrackLine, make_lane
from renderer import Renderer
from track import load_track
from track_editor import TrackEditor


def build_test_arena(size: float = 200.0) -> list[Wall]:
    """
    Simple test arena: outer walls + some internal obstacles.
    All dimensions in cm.
    """
    walls = []

    # Outer boundary
    walls.extend(make_box_walls(-size, -size, size * 2, size * 2))

    # Internal obstacles — some corridors and blocks to test sensor readings
    # Vertical wall
    walls.append(Wall(60, -80, 60, 40))
    # Horizontal wall
    walls.append(Wall(-80, 50, 30, 50))
    # L-shaped corner
    walls.append(Wall(-40, -60, -40, -20))
    walls.append(Wall(-40, -20, 0, -20))
    # Small box
    walls.extend(make_box_walls(100, -60, 40, 40))
    # Diagonal-ish corridor (two parallel walls)
    walls.append(Wall(-120, 80, -20, 120))
    walls.append(Wall(-120, 100, -20, 140))

    return walls


NOISE_PRESETS = {"default": HC_SR04_DEFAULT, "noisy": HC_SR04_NOISY, "clean": HC_SR04_CLEAN}


def build_test_track() -> list[TrackLine]:
    """
    S-curve dual-strip lane. Robot drives in the channel between the two strips.
    separation_cm=15 → inner gap of 13 cm (robot is 12 cm wide, just fits).
    """
    centerline = [
        ((-150,   0), ( -80,   0)),   # approach straight
        ((  -80,  0), ( -30, -50)),   # curve left
        ((  -30,-50), (  30, -50)),   # middle straight
        ((   30,-50), (  80,   0)),   # curve right
        ((   80,  0), ( 150,   0)),   # exit straight
    ]
    strips = []
    for (x1, y1), (x2, y2) in centerline:
        strips.extend(make_lane(x1, y1, x2, y2, strip_width_cm=2.0, separation_cm=15.0))
    return strips


def main():
    parser = argparse.ArgumentParser(description="Robot physics test harness")
    parser.add_argument("--robot",    default="configs/robot-config.yaml", help="Robot config YAML")
    parser.add_argument("--physics",  default="configs/physics.yaml",      help="Physics config YAML")
    parser.add_argument("--calibration", default=None, help="Sensor calibration YAML (from analyze.py)")
    parser.add_argument("--noise",    default="default", choices=["default", "noisy", "clean"],
                        help="Sensor noise preset (ignored if --calibration is set)")
    parser.add_argument("--track",   default=None, metavar="FILE",
                        help="Load a saved track YAML instead of the hardcoded test arena")
    parser.add_argument("--editor",  action="store_true",
                        help="Start in track editor mode")
    args = parser.parse_args()

    # Load configs
    print(f"Loading robot: {args.robot}")
    robot_cfg = load_robot_config(args.robot)
    print(f"  Chassis: {robot_cfg.chassis.length_cm}x{robot_cfg.chassis.width_cm}cm, drive={robot_cfg.drive_type}")
    print(f"  Sensors: {len(robot_cfg.sensors)} ({', '.join(s.name for s in robot_cfg.sensors)})")
    print(f"  IR sensors: {len(robot_cfg.ir_sensors)}" + (f" ({', '.join(s.name for s in robot_cfg.ir_sensors)})" if robot_cfg.ir_sensors else ""))

    print(f"Loading physics: {args.physics}")
    physics_params, display_cfg = load_physics_params(args.physics)

    # Load sensor noise profile
    noise_profile = None
    if args.calibration:
        print(f"Loading calibration: {args.calibration}")
        noise_profile = load_noise_profile(args.calibration)
        print(f"  Noise: base={noise_profile.noise_base_std:.2f}cm, slope={noise_profile.noise_per_cm:.4f}, dropout={noise_profile.dropout_chance:.1%}")
    else:
        noise_profile = NOISE_PRESETS[args.noise]
        print(f"  Sensor noise preset: {args.noise}")

    pygame.init()
    w = display_cfg.get("window_width",  1000)
    h = display_cfg.get("window_height",  700)
    screen = pygame.display.set_mode((w, h), pygame.RESIZABLE)
    clock  = pygame.time.Clock()

    # ── Editor mode ──────────────────────────────────────────────────────────
    if args.editor:
        pygame.display.set_caption("Robot Track Editor")
        track_path = args.track or "configs/track.yaml"
        editor = TrackEditor(
            robot_cfg, physics_params, display_cfg,
            track_path=track_path,
            noise_profile=noise_profile,
        )
        if args.track:
            editor.load_track_file(args.track)
        print(f"\nTrack editor ready. Tab=drive, S=save, L=load — track: {track_path}\n")
        editor.run(screen, clock)
        print("Done.")
        return

    # ── Normal drive mode ─────────────────────────────────────────────────────
    if args.track:
        print(f"Loading track: {args.track}")
        track_data  = load_track(args.track)
        walls       = track_data.walls
        track_lines = track_data.line_paths
        start_x, start_y, start_heading = (
            track_data.start_x, track_data.start_y, track_data.start_heading
        )
        pygame.display.set_caption(f"Robot Drive — {track_data.name}")
    else:
        walls       = build_test_arena()
        track_lines = build_test_track()
        start_x, start_y, start_heading = 0.0, 0.0, 90.0
        pygame.display.set_caption(f"Robot Physics Test — {args.robot}")

    engine = PhysicsEngine(robot_cfg, physics_params, walls,
                           noise_profile=noise_profile, track_lines=track_lines)
    state  = RobotState(x=start_x, y=start_y, heading=start_heading,
                        sensor_readings=[s.range_cm for s in robot_cfg.sensors])

    renderer   = Renderer(robot_cfg, display_cfg)
    follow_cam = True
    fps        = display_cfg.get("fps", 60)

    print("\nReady! WASD to drive, F=follow, R=reset, Q=quit\n")

    running = True
    while running:
        dt = clock.tick(fps) / 1000.0
        dt = min(dt, 0.05)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif event.key == pygame.K_f:
                    follow_cam = not follow_cam
                elif event.key == pygame.K_r:
                    state = RobotState(x=start_x, y=start_y, heading=start_heading,
                                       sensor_readings=[s.range_cm for s in robot_cfg.sensors])
                elif event.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    renderer.ppcm = min(20, renderer.ppcm + 1)
                elif event.key == pygame.K_MINUS:
                    renderer.ppcm = max(1, renderer.ppcm - 1)
            elif event.type == pygame.VIDEORESIZE:
                renderer.w = event.w
                renderer.h = event.h

        keys     = pygame.key.get_pressed()
        throttle = 0.0
        steering = 0.0
        if keys[pygame.K_w] or keys[pygame.K_UP]:
            throttle =  1.0
        elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
            throttle = -1.0
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            steering =  1.0
        elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            steering = -1.0
        if throttle != 0 and steering != 0:
            throttle *= 0.85

        engine.step(state, throttle, steering, dt)
        renderer.draw(screen, state, walls, follow=follow_cam, track_lines=track_lines)
        pygame.display.flip()

    pygame.quit()
    print("Done.")


if __name__ == "__main__":
    main()
