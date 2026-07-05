"""
Live visualizer — watch the expert or a trained policy drive the course.

This is the "does it actually work" test: it opens a Pygame window and drives
the robot around the real track with the same SimEnv used for training and
evaluation, so what you see is exactly what the closed-loop numbers measure.

Examples:
    # Watch the sensor-driven expert (the autopilot) drive:
    python visualize.py --policy expert \
        --track configs/tracks/COGS_300_Tournament_Track/COGS_300_Tournament_Track_v03.yaml \
        --robot configs/robot-config-frontIR.yaml --physics configs/physics-slow.yaml

    # Watch the trained model drive:
    python visualize.py --policy models/policy_bc.npz \
        --track configs/tracks/COGS_300_Tournament_Track/COGS_300_Tournament_Track_v03.yaml \
        --robot configs/robot-config-frontIR.yaml --physics configs/physics-slow.yaml

Controls:
    SPACE  pause / resume        F      toggle camera follow
    R      restart (next seed)   N      restart (next seed)
    ↑ / ↓  speed up / slow down  + / -  zoom in / out
    G      toggle guards (policy only)  Q / Esc  quit
"""

import argparse
import math
import sys

import pygame

from track import load_track
from robot_config import load_robot_config
from physics import load_physics_params, COMMAND_NAMES
from expert_policy import build_route, make_follower, CMD_STOP
from policy_runtime import load_policy, PolicyRuntime
from sim_env import SimEnv, RANDOMIZATION_PRESETS
from renderer import Renderer


def main():
    ap = argparse.ArgumentParser(description="Watch the expert or a policy drive")
    ap.add_argument("--track", required=True)
    ap.add_argument("--robot", default="configs/robot-config-frontIR.yaml")
    ap.add_argument("--physics", default="configs/physics-slow.yaml")
    ap.add_argument("--policy", required=True, help="'expert' or path to a .npz model")
    ap.add_argument("--randomization", default="mild",
                    choices=list(RANDOMIZATION_PRESETS.keys()))
    ap.add_argument("--decision-hz", type=float, default=10.0)
    ap.add_argument("--max-time", type=float, default=240.0)
    ap.add_argument("--seed", type=int, default=5000)
    ap.add_argument("--no-safeguards", action="store_true",
                    help="Policy only: run the raw model with no guards")
    args = ap.parse_args()

    track = load_track(args.track)
    robot = load_robot_config(args.robot)
    params, display_cfg = load_physics_params(args.physics)
    env = SimEnv(track, robot, params, decision_hz=args.decision_hz,
                 randomization=args.randomization)

    expert_mode = args.policy == "expert"
    route, info = build_route(track)          # for the drawn intent line
    model = None
    if not expert_mode:
        model = load_policy(args.policy)

    guards_on = not args.no_safeguards

    pygame.init()
    W, H = display_cfg.get("window_width", 1000), display_cfg.get("window_height", 700)
    screen = pygame.display.set_mode((W, H), pygame.RESIZABLE)
    title = "expert" if expert_mode else args.policy.split("/")[-1]
    pygame.display.set_caption(f"MazeBot — {title}")
    clock = pygame.time.Clock()
    renderer = Renderer(robot, display_cfg)
    renderer.w, renderer.h = W, H
    hud_font = pygame.font.SysFont("menlo", 14)
    big_font = pygame.font.SysFont("menlo", 26, bold=True)

    seed = args.seed

    def new_episode(s):
        obs = env.reset(seed=s)
        if expert_mode:
            actor = make_follower(route, info, decision_hz=args.decision_hz,
                                  robot_cfg=robot, physics_params=params)
        else:
            actor = PolicyRuntime(model, decision_hz=args.decision_hz, enabled=guards_on)
            actor.reset()
        return obs, actor

    obs, actor = new_episode(seed)
    step = 0
    max_steps = int(args.max_time * args.decision_hz)
    outcome = None                 # None=running, "success", "stuck", "lost", "timeout"
    stall = 0
    follow = True
    paused = False
    speed_mult = 1.0
    cmd = CMD_STOP

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.VIDEORESIZE:
                renderer.w, renderer.h = event.w, event.h
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_f:
                    follow = not follow
                elif event.key in (pygame.K_r, pygame.K_n):
                    seed += 1
                    obs, actor = new_episode(seed)
                    step, outcome, stall = 0, None, 0
                elif event.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    renderer.ppcm = min(20, renderer.ppcm + 1)
                elif event.key == pygame.K_MINUS:
                    renderer.ppcm = max(1, renderer.ppcm - 1)
                elif event.key == pygame.K_UP:
                    speed_mult = min(8.0, speed_mult * 1.5)
                elif event.key == pygame.K_DOWN:
                    speed_mult = max(0.25, speed_mult / 1.5)
                elif event.key == pygame.K_g and not expert_mode:
                    guards_on = not guards_on
                    actor.enabled = guards_on

        # ── Advance one decision ────────────────────────────────────────────
        if not paused and outcome is None:
            if expert_mode:
                cmd = actor.command(obs["true_x"], obs["true_y"], obs["true_heading"],
                                    ir=obs["ir"])
            else:
                cmd = actor.step(obs)
            prev = (obs["true_x"], obs["true_y"])
            obs = env.step(cmd)
            step += 1

            d_goal = env.dist_to_goal()
            moved = math.hypot(obs["true_x"] - prev[0], obs["true_y"] - prev[1])
            if d_goal < 5.0:
                outcome = "success"
            elif d_goal > 1500.0:
                outcome = "lost"
            elif cmd != CMD_STOP and moved < 0.05:
                stall += 1
                if stall > int(6 * args.decision_hz):
                    outcome = "stuck"
            else:
                stall = 0
            if step >= max_steps and outcome is None:
                outcome = "timeout"

        # ── Draw ────────────────────────────────────────────────────────────
        renderer.draw(screen, env.state, env.walls, follow=follow,
                      track_lines=track.line_paths)

        # goal marker
        gx, gy = renderer.world_to_screen(track.goal_x, track.goal_y)
        pygame.draw.circle(screen, (220, 60, 60), (gx, gy), 9, 3)
        pygame.draw.circle(screen, (220, 60, 60), (gx, gy), 2)

        # planned route (faint intent line)
        if len(route) > 1:
            pts = [renderer.world_to_screen(x, y) for x, y in route]
            pygame.draw.lines(screen, (150, 160, 200), False, pts, 1)

        # HUD
        d_goal = env.dist_to_goal()
        guard_txt = "" if expert_mode else f"   guards:{'on' if guards_on else 'OFF'}"
        lines = [
            f"{title}   seed {seed}   {'PAUSED' if paused else 'running'}{guard_txt}",
            f"cmd: {COMMAND_NAMES.get(cmd, '?')}    step {step}/{max_steps}"
            f"    t {obs['elapsed']:.1f}s   speed {speed_mult:.2f}x",
            f"dist to goal: {d_goal:6.0f} cm    contacts: {obs['collided_total']}",
            "SPACE pause  R next  arrows speed  +/- zoom  F follow"
            + ("  G guards" if not expert_mode else "") + "  Q quit",
        ]
        pad = 8
        hud_h = len(lines) * 18 + pad * 2
        hud_w = max(hud_font.size(l)[0] for l in lines) + pad * 2
        hud = pygame.Surface((hud_w, hud_h), pygame.SRCALPHA)
        pygame.draw.rect(hud, (18, 18, 24, 210), (0, 0, hud_w, hud_h), border_radius=6)
        screen.blit(hud, (10, 10))
        for i, ln in enumerate(lines):
            screen.blit(hud_font.render(ln, True, (215, 215, 210)), (10 + pad, 10 + pad + i * 18))

        # outcome banner
        if outcome:
            color = {"success": (60, 200, 90), "stuck": (220, 150, 40),
                     "lost": (220, 70, 70), "timeout": (200, 200, 80)}[outcome]
            msg = {"success": "REACHED GOAL", "stuck": "WEDGED",
                   "lost": "LOST", "timeout": "OUT OF TIME"}[outcome]
            banner = big_font.render(f"{msg}  —  R for next run", True, color)
            bx = (renderer.w - banner.get_width()) // 2
            bg = pygame.Surface((banner.get_width() + 24, banner.get_height() + 16), pygame.SRCALPHA)
            pygame.draw.rect(bg, (10, 10, 14, 230), bg.get_rect(), border_radius=8)
            screen.blit(bg, (bx - 12, 60))
            screen.blit(banner, (bx, 68))

        pygame.display.flip()
        clock.tick(args.decision_hz * speed_mult)

    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
