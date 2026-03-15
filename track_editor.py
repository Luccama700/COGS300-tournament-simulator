"""
Track editor for the maze robot simulation.

Tab toggles between two modes:
  Edit  — draw walls and line-path tape, place start/goal markers
  Drive — test-drive the robot on the track you just built

Edit-mode controls
  Left-click              start / finish a wall segment
  Shift + left-click      extend the line-path polyline
  Right-click             cancel wall in progress / finish polyline
  G                       toggle grid snap
  P                       place start marker at cursor
  E                       place goal marker at cursor
  Z                       undo last placed element (or cancel in-progress)
  Delete / Backspace      toggle delete mode (click a segment to remove it)
  S                       save track
  L                       load track
  Scroll                  zoom
  Middle-drag             pan
  Q / Esc                 cancel in-progress drawing, or quit

Drive-mode controls
  WASD / arrow keys       drive
  F                       toggle camera follow
  R                       reset robot to start position
  +/-                     zoom
  Tab                     back to edit mode
  Q / Esc                 quit
"""

import math
import pygame

from physics import Wall, PhysicsEngine, RobotState
from renderer import Renderer
from ir_model import TrackLine
from robot_config import RobotConfig
from physics import PhysicsParams
from track import TrackData, load_track, save_track


# ── Colours ────────────────────────────────────────────────────────────────
BG_COLOR             = (245, 245, 240)
GRID_COLOR           = (210, 208, 202)
WALL_COLOR           = (50,  50,  55)
WALL_PREVIEW_COLOR   = (110, 125, 160)
LINE_COLOR           = (30,  30,  30)
LINE_PREVIEW_COLOR   = (90,  90,  90)
LINE_VERTEX_COLOR    = (200, 175,  55)
HIGHLIGHT_COLOR      = (255, 175,  20)
START_COLOR          = (40,  185,  80)
GOAL_COLOR           = (220,  75,  40)
HUD_BG               = (20,   20,  25, 210)

DEFAULT_LINE_WIDTH   = 5.0   # cm
DEFAULT_TRACK_PATH   = "configs/track.yaml"
SNAP_VERTEX_PX       = 14    # pixel radius for vertex snap


# ── Helpers ────────────────────────────────────────────────────────────────

def _dist_to_seg(px, py, x1, y1, x2, y2) -> float:
    """Perpendicular distance from point to segment (world coords)."""
    dx, dy = x2 - x1, y2 - y1
    sq = dx*dx + dy*dy
    if sq < 1e-12:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1)*dx + (py - y1)*dy) / sq))
    return math.hypot(px - (x1 + t*dx), py - (y1 + t*dy))


# ── Editor ─────────────────────────────────────────────────────────────────

class TrackEditor:
    def __init__(
        self,
        robot_cfg: RobotConfig,
        physics_params: PhysicsParams,
        display_cfg: dict,
        track_path: str = DEFAULT_TRACK_PATH,
        noise_profile=None,
    ):
        self.robot_cfg      = robot_cfg
        self.physics_params = physics_params
        self.display_cfg    = display_cfg
        self.track_path     = track_path
        self.noise_profile  = noise_profile

        self.w = display_cfg.get("window_width",  1000)
        self.h = display_cfg.get("window_height",  700)

        # Editor camera (independent from the drive-mode renderer camera)
        self.ppcm  = display_cfg.get("pixels_per_cm", 4)
        self.cam_x = self.w / 2
        self.cam_y = self.h / 2

        # Mode
        self.mode = "edit"   # "edit" | "drive"

        # Edit state
        self.snap_enabled = True
        self.grid_size    = 10.0   # cm
        self.delete_mode  = False
        self.hover_elem   = None   # ("wall"|"line", index)

        # Drawing in progress
        self.wall_start   = None   # (wx, wy) | None
        self.line_points  = []     # [(wx, wy), ...] for current polyline

        # Track data
        self.track      = TrackData()
        self.undo_stack = []       # list of state snapshots

        # Drive state
        self.engine      = None
        self.drive_state = None
        self.follow_cam  = True

        # Panning
        self._pan_orig_mouse = None
        self._pan_orig_cam   = None

        # Status message (save / load feedback)
        self.status_msg   = ""
        self.status_timer = 0.0

        # Renderer (re-used in drive mode; also owns fonts)
        self.renderer = Renderer(robot_cfg, display_cfg)

        # Own fonts for edit mode
        pygame.font.init()
        self.font       = pygame.font.SysFont("menlo", 12)
        self.font_title = pygame.font.SysFont("menlo", 15, bold=True)

    # ── Coordinate helpers ──────────────────────────────────────────────────

    def _w2s(self, wx, wy) -> tuple[int, int]:
        return (int(self.cam_x + wx * self.ppcm),
                int(self.cam_y + wy * self.ppcm))

    def _s2w(self, sx, sy) -> tuple[float, float]:
        return ((sx - self.cam_x) / self.ppcm,
                (sy - self.cam_y) / self.ppcm)

    def _snap(self, wx, wy) -> tuple[float, float]:
        if not self.snap_enabled:
            return wx, wy
        g = self.grid_size
        return round(wx / g) * g, round(wy / g) * g

    def _snap_vertex(self, wx, wy) -> tuple[float, float]:
        """Snap to nearest existing vertex within SNAP_VERTEX_PX pixels."""
        r_cm = SNAP_VERTEX_PX / self.ppcm
        best_sq = r_cm * r_cm
        best = None
        for wall in self.track.walls:
            for vx, vy in [(wall.x1, wall.y1), (wall.x2, wall.y2)]:
                d = (wx-vx)**2 + (wy-vy)**2
                if d < best_sq:
                    best_sq, best = d, (vx, vy)
        for tl in self.track.line_paths:
            for vx, vy in [(tl.x1, tl.y1), (tl.x2, tl.y2)]:
                d = (wx-vx)**2 + (wy-vy)**2
                if d < best_sq:
                    best_sq, best = d, (vx, vy)
        return best if best else (wx, wy)

    def _snapped(self, wx_raw, wy_raw) -> tuple[float, float]:
        """Grid-snap then vertex-snap."""
        wx, wy = self._snap(wx_raw, wy_raw)
        return self._snap_vertex(wx, wy)

    # ── Zoom / pan ──────────────────────────────────────────────────────────

    def _zoom(self, screen_pos, factor):
        wx, wy = self._s2w(*screen_pos)
        self.ppcm = max(0.5, min(40.0, self.ppcm * factor))
        self.cam_x = screen_pos[0] - wx * self.ppcm
        self.cam_y = screen_pos[1] - wy * self.ppcm

    # ── Undo ───────────────────────────────────────────────────────────────

    def _push_undo(self):
        self.undo_stack.append((
            [Wall(w.x1, w.y1, w.x2, w.y2) for w in self.track.walls],
            [TrackLine(t.x1, t.y1, t.x2, t.y2, t.width_cm) for t in self.track.line_paths],
            (self.track.start_x, self.track.start_y, self.track.start_heading),
            (self.track.goal_x,  self.track.goal_y),
        ))

    def _undo(self):
        if not self.undo_stack:
            return
        walls, lps, start, goal = self.undo_stack.pop()
        self.track.walls      = walls
        self.track.line_paths = lps
        self.track.start_x, self.track.start_y, self.track.start_heading = start
        self.track.goal_x,  self.track.goal_y  = goal

    # ── Nearest-element lookup ──────────────────────────────────────────────

    def _find_nearest(self, wx, wy):
        """Return ("wall"|"line", index) of nearest segment, or None."""
        threshold = 10.0 / self.ppcm   # 10 px in world-cm
        best_d = threshold
        best   = None
        for i, w in enumerate(self.track.walls):
            d = _dist_to_seg(wx, wy, w.x1, w.y1, w.x2, w.y2)
            if d < best_d:
                best_d, best = d, ("wall", i)
        for i, tl in enumerate(self.track.line_paths):
            d = _dist_to_seg(wx, wy, tl.x1, tl.y1, tl.x2, tl.y2)
            if d < best_d:
                best_d, best = d, ("line", i)
        return best

    # ── Mode switching ──────────────────────────────────────────────────────

    def _enter_drive(self):
        self.engine = PhysicsEngine(
            self.robot_cfg, self.physics_params,
            self.track.walls,
            noise_profile=self.noise_profile,
            track_lines=self.track.line_paths,
        )
        self.drive_state = RobotState(
            x=self.track.start_x,
            y=self.track.start_y,
            heading=self.track.start_heading,
            sensor_readings=[s.range_cm for s in self.robot_cfg.sensors],
        )
        # Hand current camera to the renderer
        self.renderer.ppcm  = self.ppcm
        self.renderer.cam_x = self.cam_x
        self.renderer.cam_y = self.cam_y
        self.renderer.w     = self.w
        self.renderer.h     = self.h
        self.follow_cam = True
        self.mode = "drive"

    def _enter_edit(self):
        # Absorb any camera drift from drive mode
        self.cam_x = self.renderer.cam_x
        self.cam_y = self.renderer.cam_y
        self.ppcm  = self.renderer.ppcm
        self.wall_start  = None
        self.line_points = []
        self.delete_mode = False
        self.engine      = None
        self.drive_state = None
        self.mode = "edit"

    # ── Save / load ─────────────────────────────────────────────────────────

    def _save(self):
        try:
            save_track(self.track, self.track_path)
            self._status(f"Saved → {self.track_path}")
        except Exception as exc:
            self._status(f"Save failed: {exc}")

    def _load(self):
        try:
            self.track       = load_track(self.track_path)
            self.undo_stack  = []
            self.wall_start  = None
            self.line_points = []
            self._status(f"Loaded {self.track_path}")
        except FileNotFoundError:
            self._status(f"Not found: {self.track_path}")
        except Exception as exc:
            self._status(f"Load failed: {exc}")

    def load_track_file(self, path: str):
        """Load a track from a specific path (called from run_test.py)."""
        self.track_path = path
        self._load()

    def _status(self, msg: str, duration: float = 3.0):
        self.status_msg   = msg
        self.status_timer = duration

    # ── Main loop ───────────────────────────────────────────────────────────

    def run(self, screen: pygame.Surface, clock: pygame.time.Clock):
        fps     = self.display_cfg.get("fps", 60)
        running = True

        while running:
            dt = clock.tick(fps) / 1000.0
            dt = min(dt, 0.05)

            if self.status_timer > 0:
                self.status_timer -= dt

            mouse_px = pygame.mouse.get_pos()
            mx_raw, my_raw = self._s2w(*mouse_px)

            if self.mode == "edit" and self.delete_mode:
                self.hover_elem = self._find_nearest(mx_raw, my_raw)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif self.mode == "edit":
                    if not self._edit_event(event):
                        running = False

                else:  # drive
                    if not self._drive_event(event):
                        running = False

            # ── Draw ────────────────────────────────────────────────────────
            if self.mode == "edit":
                # Snapped world coords of current mouse position
                mx, my = self._snapped(mx_raw, my_raw)
                self._draw_edit(screen, mx, my)

            else:
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

                self.engine.step(self.drive_state, throttle, steering, dt)
                self.renderer.draw(
                    screen, self.drive_state, self.track.walls,
                    follow=self.follow_cam,
                    track_lines=self.track.line_paths,
                )
                self._draw_drive_overlay(screen)

            pygame.display.flip()

        pygame.quit()

    # ── Edit-mode event handler ─────────────────────────────────────────────

    def _edit_event(self, event) -> bool:
        """Return False to quit."""

        if event.type == pygame.KEYDOWN:
            k = event.key

            if k in (pygame.K_q, pygame.K_ESCAPE):
                if self.wall_start or self.line_points:
                    self.wall_start  = None
                    self.line_points = []
                else:
                    return False

            elif k == pygame.K_TAB:
                self._enter_drive()

            elif k == pygame.K_g:
                self.snap_enabled = not self.snap_enabled

            elif k == pygame.K_z:
                if self.wall_start or self.line_points:
                    self.wall_start  = None
                    self.line_points = []
                else:
                    self._undo()

            elif k in (pygame.K_DELETE, pygame.K_BACKSPACE):
                self.delete_mode = not self.delete_mode
                self.wall_start  = None
                self.line_points = []

            elif k == pygame.K_p:
                # Place start at cursor
                mx_raw, my_raw = self._s2w(*pygame.mouse.get_pos())
                mx, my = self._snapped(mx_raw, my_raw)
                self._push_undo()
                self.track.start_x, self.track.start_y = mx, my

            elif k == pygame.K_e:
                # Place goal at cursor
                mx_raw, my_raw = self._s2w(*pygame.mouse.get_pos())
                mx, my = self._snapped(mx_raw, my_raw)
                self._push_undo()
                self.track.goal_x, self.track.goal_y = mx, my

            elif k == pygame.K_s:
                self._save()

            elif k == pygame.K_l:
                self._load()

        elif event.type == pygame.MOUSEBUTTONDOWN:
            wx_raw, wy_raw = self._s2w(*event.pos)
            wx, wy = self._snapped(wx_raw, wy_raw)

            if event.button == 1:   # left click
                if self.delete_mode:
                    nearest = self._find_nearest(wx_raw, wy_raw)
                    if nearest:
                        self._push_undo()
                        kind, idx = nearest
                        if kind == "wall":
                            self.track.walls.pop(idx)
                        else:
                            self.track.line_paths.pop(idx)
                        self.hover_elem = None

                elif self.line_points:
                    # Polyline in progress — extend regardless of shift
                    x1, y1 = self.line_points[-1]
                    if abs(wx - x1) > 0.01 or abs(wy - y1) > 0.01:
                        self._push_undo()
                        self.track.line_paths.append(
                            TrackLine(x1, y1, wx, wy, DEFAULT_LINE_WIDTH)
                        )
                        self.line_points.append((wx, wy))

                elif pygame.key.get_mods() & pygame.KMOD_SHIFT:
                    # Start a new polyline
                    self.line_points = [(wx, wy)]

                else:
                    # Wall placement
                    if self.wall_start is None:
                        self.wall_start = (wx, wy)
                    else:
                        x1, y1 = self.wall_start
                        if abs(wx - x1) > 0.01 or abs(wy - y1) > 0.01:
                            self._push_undo()
                            self.track.walls.append(Wall(x1, y1, wx, wy))
                        self.wall_start = None

            elif event.button == 3:  # right click
                if self.line_points:
                    self.line_points = []   # finish polyline
                else:
                    self.wall_start = None  # cancel wall

            elif event.button == 2:  # middle click — start pan
                self._pan_orig_mouse = event.pos
                self._pan_orig_cam   = (self.cam_x, self.cam_y)

            elif event.button == 4:  # scroll up
                self._zoom(event.pos, 1.15)

            elif event.button == 5:  # scroll down
                self._zoom(event.pos, 1 / 1.15)

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 2:
                self._pan_orig_mouse = None
                self._pan_orig_cam   = None

        elif event.type == pygame.MOUSEMOTION:
            if self._pan_orig_mouse and self._pan_orig_cam:
                dx = event.pos[0] - self._pan_orig_mouse[0]
                dy = event.pos[1] - self._pan_orig_mouse[1]
                self.cam_x = self._pan_orig_cam[0] + dx
                self.cam_y = self._pan_orig_cam[1] + dy

        elif event.type == pygame.VIDEORESIZE:
            self.w = event.w
            self.h = event.h

        return True

    # ── Drive-mode event handler ────────────────────────────────────────────

    def _drive_event(self, event) -> bool:
        """Return False to quit."""
        if event.type == pygame.KEYDOWN:
            k = event.key
            if k in (pygame.K_q, pygame.K_ESCAPE):
                return False
            elif k == pygame.K_TAB:
                self._enter_edit()
            elif k == pygame.K_f:
                self.follow_cam = not self.follow_cam
            elif k == pygame.K_r:
                self.drive_state = RobotState(
                    x=self.track.start_x,
                    y=self.track.start_y,
                    heading=self.track.start_heading,
                    sensor_readings=[s.range_cm for s in self.robot_cfg.sensors],
                )
            elif k in (pygame.K_EQUALS, pygame.K_PLUS):
                self.renderer.ppcm = min(40, self.renderer.ppcm + 1)
            elif k == pygame.K_MINUS:
                self.renderer.ppcm = max(1, self.renderer.ppcm - 1)

        elif event.type == pygame.VIDEORESIZE:
            self.w = self.renderer.w = event.w
            self.h = self.renderer.h = event.h

        return True

    # ── Edit-mode drawing ───────────────────────────────────────────────────

    def _draw_edit(self, surface: pygame.Surface, mx: float, my: float):
        surface.fill(BG_COLOR)
        self._draw_grid(surface)
        self._draw_line_paths(surface)
        self._draw_walls(surface)
        self._draw_markers(surface)
        self._draw_preview(surface, mx, my)
        self._draw_edit_hud(surface, mx, my)

    def _draw_grid(self, surface: pygame.Surface):
        gs_px = self.grid_size * self.ppcm   # float pixels per grid cell
        if gs_px < 4:
            return

        # Compute each line from its integer world-grid index so there is
        # zero drift between grid lines and snapped element positions,
        # regardless of whether ppcm is a whole number.
        n_x = math.ceil(-self.cam_x / gs_px)
        while True:
            sx = int(self.cam_x + n_x * gs_px)
            if sx > self.w:
                break
            pygame.draw.line(surface, GRID_COLOR, (sx, 0), (sx, self.h), 1)
            n_x += 1

        n_y = math.ceil(-self.cam_y / gs_px)
        while True:
            sy = int(self.cam_y + n_y * gs_px)
            if sy > self.h:
                break
            pygame.draw.line(surface, GRID_COLOR, (0, sy), (self.w, sy), 1)
            n_y += 1

    def _draw_walls(self, surface: pygame.Surface):
        w_px = max(2, int(self.ppcm * 0.8))
        for i, wall in enumerate(self.track.walls):
            color = HIGHLIGHT_COLOR if self.hover_elem == ("wall", i) else WALL_COLOR
            p1 = self._w2s(wall.x1, wall.y1)
            p2 = self._w2s(wall.x2, wall.y2)
            pygame.draw.line(surface, color, p1, p2, w_px)
            pygame.draw.circle(surface, color, p1, max(2, w_px // 2))
            pygame.draw.circle(surface, color, p2, max(2, w_px // 2))

    def _draw_line_paths(self, surface: pygame.Surface):
        vertices = set()
        for i, tl in enumerate(self.track.line_paths):
            color  = HIGHLIGHT_COLOR if self.hover_elem == ("line", i) else LINE_COLOR
            tape_w = max(2, int(tl.width_cm * self.ppcm))
            p1 = self._w2s(tl.x1, tl.y1)
            p2 = self._w2s(tl.x2, tl.y2)
            pygame.draw.line(surface, color, p1, p2, tape_w)
            vertices.add((tl.x1, tl.y1))
            vertices.add((tl.x2, tl.y2))
        dot_r = max(3, int(self.ppcm * 0.45))
        for vx, vy in vertices:
            pygame.draw.circle(surface, LINE_VERTEX_COLOR, self._w2s(vx, vy), dot_r)

    def _draw_markers(self, surface: pygame.Surface):
        # Start — green circle + heading arrow + "S" label
        sp = self._w2s(self.track.start_x, self.track.start_y)
        r  = max(8, int(self.ppcm * 1.2))
        pygame.draw.circle(surface, START_COLOR, sp, r, 2)
        rad = math.radians(self.track.start_heading)
        arrow_len = max(12, int(self.ppcm * 2.2))
        tip = (int(sp[0] + math.cos(rad) * arrow_len),
               int(sp[1] - math.sin(rad) * arrow_len))
        pygame.draw.line(surface, START_COLOR, sp, tip, 2)
        lbl = self.font.render("S", True, START_COLOR)
        surface.blit(lbl, (sp[0] + r + 2, sp[1] - 7))

        # Goal — orange circle + X + "G" label
        if self.track.goal_x is not None:
            gp = self._w2s(self.track.goal_x, self.track.goal_y)
            pygame.draw.circle(surface, GOAL_COLOR, gp, r, 2)
            xr = max(4, r - 4)
            pygame.draw.line(surface, GOAL_COLOR, (gp[0]-xr, gp[1]-xr), (gp[0]+xr, gp[1]+xr), 2)
            pygame.draw.line(surface, GOAL_COLOR, (gp[0]+xr, gp[1]-xr), (gp[0]-xr, gp[1]+xr), 2)
            lbl = self.font.render("G", True, GOAL_COLOR)
            surface.blit(lbl, (gp[0] + r + 2, gp[1] - 7))

    def _draw_preview(self, surface: pygame.Surface, mx: float, my: float):
        if self.delete_mode:
            return

        cursor_s = self._w2s(mx, my)

        if self.line_points:
            # Line-path polyline preview
            last_s = self._w2s(*self.line_points[-1])
            tape_w = max(2, int(DEFAULT_LINE_WIDTH * self.ppcm))
            pygame.draw.line(surface, LINE_PREVIEW_COLOR, last_s, cursor_s, tape_w)
            # Placed vertices
            dot_r = max(3, int(self.ppcm * 0.45))
            for vx, vy in self.line_points:
                pygame.draw.circle(surface, LINE_VERTEX_COLOR, self._w2s(vx, vy), dot_r)

        elif self.wall_start:
            # Wall preview
            start_s = self._w2s(*self.wall_start)
            w_px = max(2, int(self.ppcm * 0.8))
            pygame.draw.line(surface, WALL_PREVIEW_COLOR, start_s, cursor_s, w_px)
            pygame.draw.circle(surface, WALL_PREVIEW_COLOR, start_s, 3)

        # Snap-point indicator
        pygame.draw.circle(surface, (100, 120, 200), cursor_s, 4, 1)

    def _draw_edit_hud(self, surface: pygame.Surface, mx: float, my: float):
        mods = pygame.key.get_mods()
        shift = bool(mods & pygame.KMOD_SHIFT)

        if self.delete_mode:
            tool_label = "DELETE — click segment to remove"
            tool_color = (225, 80, 45)
        elif self.line_points:
            tool_label = f"Line Path — {len(self.line_points)} pts  (RMB to finish)"
            tool_color = (220, 200, 55)
        elif shift:
            tool_label = "Line Path — click to start"
            tool_color = (220, 200, 55)
        else:
            tool_label = "Wall"
            tool_color = (160, 185, 225)

        snap_str = f"{'ON' if self.snap_enabled else 'OFF'}  grid={self.grid_size}cm"

        rows = [
            ("EDIT MODE",                                    (175, 220, 255), True),
            (f"TOOL: {tool_label}",                          tool_color,      False),
            (f"Snap: {snap_str}  zoom={self.ppcm:.1f}px/cm", (155, 155, 150), False),
            (f"Cursor: ({mx:.1f}, {my:.1f}) cm",             (140, 140, 135), False),
            (f"Walls: {len(self.track.walls)}   "
             f"Line segs: {len(self.track.line_paths)}",     (140, 140, 135), False),
            ("",                                             None,            False),
            ("LClick=Wall  Shift+LClick=Line poly",          (105, 105, 100), False),
            ("RClick=Cancel/Finish  Del=Delete mode",        (105, 105, 100), False),
            ("P=Start  E=Goal  Z=Undo  G=Snap",              (105, 105, 100), False),
            ("S=Save  L=Load  Tab=Drive  Scroll/MMB=View",   (105, 105, 100), False),
        ]
        if self.status_timer > 0:
            rows.append((self.status_msg, (95, 210, 95), False))

        self._render_hud(surface, 12, 12, rows)

    # ── Drive-mode overlay ──────────────────────────────────────────────────

    def _draw_drive_overlay(self, surface: pygame.Surface):
        rows = [
            ("DRIVE MODE",                              (175, 255, 175), True),
            ("WASD=drive  F=follow  R=reset",           (140, 140, 135), False),
            ("Tab=edit  +/-=zoom  Q/Esc=quit",          (140, 140, 135), False),
        ]
        if self.status_timer > 0:
            rows.append((self.status_msg, (95, 210, 95), False))

        max_w = max(
            (self.font_title if bold else self.font).size(t)[0]
            for t, _, bold in rows if t
        ) + 20
        hud_h = len(rows) * 17 + 12
        hud_x = self.w - max_w - 12
        self._render_hud(surface, hud_x, 12, rows)

    # ── HUD utility ─────────────────────────────────────────────────────────

    def _render_hud(self, surface, hud_x, hud_y, rows):
        visible = [(t, c, b) for t, c, b in rows if t and c]
        if not visible:
            return
        max_w  = max((self.font_title if b else self.font).size(t)[0] for t, _, b in visible) + 20
        hud_h  = len(rows) * 17 + 12
        bg     = pygame.Surface((max_w, hud_h), pygame.SRCALPHA)
        pygame.draw.rect(bg, HUD_BG, (0, 0, max_w, hud_h), border_radius=6)
        surface.blit(bg, (hud_x, hud_y))
        for i, (text, color, bold) in enumerate(rows):
            if not text or not color:
                continue
            font   = self.font_title if bold else self.font
            rendered = font.render(text, True, color)
            surface.blit(rendered, (hud_x + 10, hud_y + 6 + i * 17))
