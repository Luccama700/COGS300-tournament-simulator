"""
Pygame renderer for the robot simulation.

Draws the robot chassis, sensor cones and readings, walls,
and a HUD with live telemetry. Completely separate from physics —
just takes state and draws it.
"""

import math
import pygame
from robot_config import RobotConfig
from physics import RobotState, Wall
from ir_model import TrackLine

SENSOR_COLORS = [
    (59, 139, 212), (216, 90, 48), (29, 158, 117),
    (212, 83, 126), (127, 119, 221), (99, 153, 34),
    (186, 117, 23), (226, 75, 74),
]


class Renderer:
    def __init__(self, robot_cfg: RobotConfig, display_cfg: dict):
        self.robot_cfg = robot_cfg
        self.dcfg = display_cfg
        self.ppcm = display_cfg.get("pixels_per_cm", 4)
        self.grid_spacing = display_cfg.get("grid_spacing", 50)
        self.bg_color = tuple(display_cfg.get("background_color", [245, 245, 240]))
        self.w = display_cfg.get("window_width", 1000)
        self.h = display_cfg.get("window_height", 700)

        pygame.font.init()
        self.font = pygame.font.SysFont("menlo", 12)
        self.font_big = pygame.font.SysFont("menlo", 14, bold=True)

        # Camera offset (world cm -> screen pixel)
        self.cam_x = self.w / 2
        self.cam_y = self.h / 2

    def world_to_screen(self, wx: float, wy: float) -> tuple[int, int]:
        return (
            int(self.cam_x + wx * self.ppcm),
            int(self.cam_y + wy * self.ppcm),
        )

    def cm_to_px(self, cm: float) -> float:
        return cm * self.ppcm

    def center_camera_on(self, wx: float, wy: float):
        self.cam_x = self.w / 2 - wx * self.ppcm
        self.cam_y = self.h / 2 - wy * self.ppcm

    def draw(self, surface: pygame.Surface, state: RobotState, walls: list[Wall], follow: bool = True, track_lines: list[TrackLine] | None = None):
        if follow:
            self.center_camera_on(state.x, state.y)

        surface.fill(self.bg_color)
        self._draw_grid(surface)
        if track_lines:
            self._draw_track_lines(surface, track_lines)
        self._draw_walls(surface, walls)
        self._draw_sensors(surface, state)
        self._draw_robot(surface, state)
        self._draw_ir_sensors(surface, state)
        self._draw_hud(surface, state)

    def _draw_track_lines(self, surface: pygame.Surface, track_lines: list[TrackLine]):
        for tl in track_lines:
            p1 = self.world_to_screen(tl.x1, tl.y1)
            p2 = self.world_to_screen(tl.x2, tl.y2)
            # Black tape — pixel width matches detection zone exactly
            tape_w = max(2, int(self.cm_to_px(tl.width_cm)))
            pygame.draw.line(surface, (30, 30, 30), p1, p2, tape_w)

    def _draw_grid(self, surface: pygame.Surface):
        gs = self.grid_spacing * self.ppcm
        # Offset grid relative to camera
        start_x = self.cam_x % gs
        start_y = self.cam_y % gs
        grid_color = (220, 218, 212)
        for x in range(int(start_x), self.w + 1, max(1, int(gs))):
            pygame.draw.line(surface, grid_color, (x, 0), (x, self.h), 1)
        for y in range(int(start_y), self.h + 1, max(1, int(gs))):
            pygame.draw.line(surface, grid_color, (0, y), (self.w, y), 1)

    def _draw_walls(self, surface: pygame.Surface, walls: list[Wall]):
        wall_color = (50, 50, 55)
        for wall in walls:
            p1 = self.world_to_screen(wall.x1, wall.y1)
            p2 = self.world_to_screen(wall.x2, wall.y2)
            pygame.draw.line(surface, wall_color, p1, p2, max(2, int(self.ppcm * 0.8)))

    def _draw_robot(self, surface: pygame.Surface, state: RobotState):
        cfg = self.robot_cfg
        cx, cy = self.world_to_screen(state.x, state.y)
        heading_rad = math.radians(state.heading)
        cos_h, sin_h = math.cos(heading_rad), math.sin(heading_rad)

        hw = self.cm_to_px(cfg.half_width)
        hl = self.cm_to_px(cfg.half_length)

        # Chassis corners as (forward, right) in pixels
        corners_local = [
            (hl, -hw),   # front-left
            (hl, hw),    # front-right
            (-hl, hw),   # rear-right
            (-hl, -hw),  # rear-left
        ]
        corners_screen = []
        for fwd, right in corners_local:
            sx = cx + fwd * cos_h + right * sin_h
            sy = cy - fwd * sin_h + right * cos_h
            corners_screen.append((sx, sy))

        # Chassis fill
        pygame.draw.polygon(surface, (180, 190, 205), corners_screen)
        pygame.draw.polygon(surface, (80, 85, 95), corners_screen, 2)

        # Front bumper accent (front edge)
        bumper_l = (
            (corners_screen[0][0] * 0.85 + corners_screen[1][0] * 0.15),
            (corners_screen[0][1] * 0.85 + corners_screen[1][1] * 0.15),
        )
        bumper_r = (
            (corners_screen[0][0] * 0.15 + corners_screen[1][0] * 0.85),
            (corners_screen[0][1] * 0.15 + corners_screen[1][1] * 0.85),
        )
        pygame.draw.line(surface, (59, 139, 212), bumper_l, bumper_r, 3)

        # Heading arrow
        arrow_tip_x = cx + cos_h * hl * 1.3
        arrow_tip_y = cy - sin_h * hl * 1.3
        pygame.draw.line(surface, (255, 255, 255), (cx, cy), (arrow_tip_x, arrow_tip_y), 2)

        # Center dot
        pygame.draw.circle(surface, (60, 60, 65), (cx, cy), 3)

        # Wheels
        self._draw_wheel(surface, state, -1, 1, hw, hl, cos_h, sin_h, cx, cy)  # rear left
        self._draw_wheel(surface, state, 1, 1, hw, hl, cos_h, sin_h, cx, cy)   # rear right
        if cfg.drive_type == "caster":
            # Front caster (+0.6*hl forward, 0 right)
            caster_fwd = hl * 0.6
            caster_sx = cx + caster_fwd * cos_h
            caster_sy = cy - caster_fwd * sin_h
            pygame.draw.circle(surface, (100, 100, 105), (int(caster_sx), int(caster_sy)), max(3, int(hw * 0.18)))
        else:
            self._draw_wheel(surface, state, -1, -1, hw, hl, cos_h, sin_h, cx, cy)
            self._draw_wheel(surface, state, 1, -1, hw, hl, cos_h, sin_h, cx, cy)

    def _draw_wheel(self, surface, state, side_x, side_y, hw, hl, cos_h, sin_h, cx, cy):
        """Draw a single wheel. side_x: -1=left, 1=right. side_y: -1=front, 1=rear."""
        ww = max(3, int(hw * 0.2))
        wh = max(8, int(hl * 0.28))
        right_offset = side_x * (hw + ww * 0.6)
        fwd_offset = -side_y * hl * 0.6  # side_y: +1=rear → negative forward
        wx = cx + fwd_offset * cos_h + right_offset * sin_h
        wy = cy - fwd_offset * sin_h + right_offset * cos_h

        # Wheel as a rotated rectangle — corners as (forward, right)
        corners = []
        for fwd, right in [(-wh/2, -ww/2), (wh/2, -ww/2), (wh/2, ww/2), (-wh/2, ww/2)]:
            rx = wx + fwd * cos_h + right * sin_h
            ry = wy - fwd * sin_h + right * cos_h
            corners.append((rx, ry))
        pygame.draw.polygon(surface, (35, 35, 38), corners)
        pygame.draw.polygon(surface, (70, 70, 75), corners, 1)

    def _draw_sensors(self, surface: pygame.Surface, state: RobotState):
        cfg = self.robot_cfg
        for i, sensor in enumerate(cfg.sensors):
            color = SENSOR_COLORS[i % len(SENSOR_COLORS)]
            sx, sy = cfg.sensor_world_position(sensor, state.x, state.y, state.heading)
            screen_s = self.world_to_screen(sx, sy)

            world_angle = cfg.sensor_world_angle(sensor, state.heading)
            angle_rad = math.radians(world_angle)

            reading = state.sensor_readings[i] if i < len(state.sensor_readings) else sensor.range_cm
            reading_px = self.cm_to_px(reading)
            range_px = self.cm_to_px(sensor.range_cm)

            # Cone (faded)
            half_sweep = math.radians(sensor.sweep_deg)
            cone_surface = pygame.Surface((self.w, self.h), pygame.SRCALPHA)
            n_segments = 12
            points = [screen_s]
            for j in range(n_segments + 1):
                a = angle_rad - half_sweep + (2 * half_sweep * j / n_segments)
                px = screen_s[0] + math.cos(a) * range_px
                py = screen_s[1] - math.sin(a) * range_px
                points.append((px, py))
            if len(points) >= 3:
                pygame.draw.polygon(cone_surface, (*color, 18), points)
                surface.blit(cone_surface, (0, 0))

            # Ray to reading distance
            end_x = screen_s[0] + math.cos(angle_rad) * reading_px
            end_y = screen_s[1] - math.sin(angle_rad) * reading_px

            # Color based on distance (green=far, red=close)
            ratio = min(reading / sensor.range_cm, 1.0) if sensor.range_cm > 0 else 0
            ray_color = (
                int(255 * (1 - ratio)),
                int(180 * ratio),
                50,
            )
            pygame.draw.line(surface, ray_color, screen_s, (end_x, end_y), 1)

            # Hit dot
            pygame.draw.circle(surface, ray_color, (int(end_x), int(end_y)), 3)

            # Reading label
            label = self.font.render(f"{reading:.0f}", True, color)
            surface.blit(label, (int(end_x) + 5, int(end_y) - 6))

    def _draw_ir_sensors(self, surface: pygame.Surface, state: RobotState):
        cfg = self.robot_cfg
        for i, ir in enumerate(cfg.ir_sensors):
            sx, sy = cfg.ir_sensor_world_position(ir, state.x, state.y, state.heading)
            screen_s = self.world_to_screen(sx, sy)

            reading = state.ir_readings[i] if i < len(state.ir_readings) else 0
            on_tape = reading > ir.threshold   # HIGH = on tape (white in real life)
            color = (255, 235, 50) if on_tape else (80, 55, 15)

            size = max(3, int(self.cm_to_px(0.9)))
            pts = [
                (screen_s[0],          screen_s[1] - size),
                (screen_s[0] + size,   screen_s[1]),
                (screen_s[0],          screen_s[1] + size),
                (screen_s[0] - size,   screen_s[1]),
            ]
            pygame.draw.polygon(surface, color, pts)
            pygame.draw.polygon(surface, (180, 180, 175), pts, 1)

    def _draw_hud(self, surface: pygame.Surface, state: RobotState):
        hud_x, hud_y = 12, 12
        lines = [
            f"Pos: ({state.x:.1f}, {state.y:.1f})  Heading: {state.heading:.1f}°",
            f"Speed: {state.speed:.1f} cm/s  Turn: {state.turn_rate:.1f}°/s",
        ]
        if state.sensor_readings:
            names = [s.name for s in self.robot_cfg.sensors]
            readings = "  ".join(f"{n}:{r:.0f}" for n, r in zip(names, state.sensor_readings))
            lines.append(f"Sensors: {readings}")
        if state.ir_readings and self.robot_cfg.ir_sensors:
            ir_str = "  ".join(f"{s.name}:{int(r)}" for s, r in zip(self.robot_cfg.ir_sensors, state.ir_readings))
            lines.append(f"IR: {ir_str}")

        lines.append("")
        lines.append("WASD=drive  F=follow  R=reset  Q=quit")

        # Background
        max_w = max(self.font.size(l)[0] for l in lines) + 20
        hud_h = len(lines) * 18 + 12
        hud_surface = pygame.Surface((max_w, hud_h), pygame.SRCALPHA)
        pygame.draw.rect(hud_surface, (20, 20, 25, 200), (0, 0, max_w, hud_h), border_radius=6)
        surface.blit(hud_surface, (hud_x, hud_y))

        for i, line in enumerate(lines):
            color = (200, 200, 195) if i < 3 else (140, 140, 135)
            text = self.font.render(line, True, color)
            surface.blit(text, (hud_x + 10, hud_y + 6 + i * 18))
