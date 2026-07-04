"""
Privileged expert policy for generating training data.

The old pipeline's expert (generate_line_data.py) had two fatal flaws:
  1. It only followed the tape-line network — but every tournament track's goal
     sits OFF the lines, inside the walled maze, so it never reached the goal
     and logged thousands of STOP rows per episode (90%+ of the dataset).
  2. It drove a simplified kinematics model with continuous steering, then
     discretized the labels afterwards — dynamics the real firmware (and the
     real physics engine in physics.py) can't reproduce.

This expert fixes both:
  * Route = A* over the tape-line network (line-following phase) chained with
    A* over an inflated occupancy grid through the walls to the goal
    (maze phase).
  * It emits the SAME six discrete commands the Arduino firmware executes
    (physics.MOTOR_COMMANDS) and is meant to drive the real PhysicsEngine,
    so the labels are realizable by construction.

The expert reads the robot's TRUE pose — privileged information that exists
only in simulation. The learned policy sees only sensors; the expert is used
to produce labels (behavior cloning + DAgger), never at inference time.

CLI (sanity check / visualization):
    python expert_policy.py --track configs/tracks/.../track.yaml --plot route.png
"""

import argparse
import heapq
import math

import numpy as np

from track import TrackData, load_track, build_line_graph, find_nearest_node, astar_line_path

# Command ids — must match physics.MOTOR_COMMANDS / firmware executeCommand()
CMD_FORWARD, CMD_SLIGHT_L, CMD_SLIGHT_R, CMD_HARD_L, CMD_HARD_R, CMD_STOP = range(6)


def wrap_deg(a: float) -> float:
    """Wrap angle to [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def polyline_length(pts) -> float:
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))


# ── Occupancy-grid planner (maze phase) ──────────────────────────────────────

class GridPlanner:
    """
    8-connected A* over an occupancy grid built from the track walls,
    inflated by the robot's collision radius so any grid path is drivable.
    """

    # Default inflation = robot half-width (6) + collision margin (1) + buffer
    # for the swing of the chassis nose in turns. The maze's 20cm slits are
    # dead-end side pockets, so the solution route tolerates this inflation;
    # pass a smaller inflate_cm if a track ever routes through a 20cm slit.
    def __init__(self, track: TrackData, cell_cm: float = 2.0,
                 inflate_cm: float = 9.5, pad_cm: float = 40.0):
        xs = [track.start_x, track.goal_x]
        ys = [track.start_y, track.goal_y]
        for w in track.walls:
            xs += [w[0], w[2]]
            ys += [w[1], w[3]]
        for s in track.line_paths:
            xs += [s.x1, s.x2]
            ys += [s.y1, s.y2]

        self.cell = cell_cm
        self.x0 = min(xs) - pad_cm
        self.y0 = min(ys) - pad_cm
        self.nx = int((max(xs) + pad_cm - self.x0) / cell_cm) + 1
        self.ny = int((max(ys) + pad_cm - self.y0) / cell_cm) + 1
        self.blocked = np.zeros((self.ny, self.nx), dtype=bool)

        # Disk stencil: all cell offsets whose center lies within inflate_cm
        r_cells = int(math.ceil(inflate_cm / cell_cm))
        offs = []
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                if math.hypot(dx, dy) * cell_cm <= inflate_cm:
                    offs.append((dy, dx))
        self._disk = np.array(offs, dtype=int)

        for w in track.walls:
            self._stamp_wall(*w)

    def _stamp_wall(self, x1, y1, x2, y2):
        length = math.hypot(x2 - x1, y2 - y1)
        n = max(2, int(length / (self.cell * 0.5)) + 1)
        for t in np.linspace(0.0, 1.0, n):
            cx, cy = self.world_to_cell(x1 + t * (x2 - x1), y1 + t * (y2 - y1))
            cells = self._disk + (cy, cx)
            valid = ((cells[:, 0] >= 0) & (cells[:, 0] < self.ny) &
                     (cells[:, 1] >= 0) & (cells[:, 1] < self.nx))
            cells = cells[valid]
            self.blocked[cells[:, 0], cells[:, 1]] = True

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (int(round((x - self.x0) / self.cell)),
                int(round((y - self.y0) / self.cell)))

    def cell_to_world(self, cx: int, cy: int) -> tuple[float, float]:
        return (self.x0 + cx * self.cell, self.y0 + cy * self.cell)

    def _nearest_free(self, cx: int, cy: int, max_r: int = 6) -> tuple[int, int] | None:
        """Snap a blocked cell to the nearest free cell (spiral search)."""
        if 0 <= cy < self.ny and 0 <= cx < self.nx and not self.blocked[cy, cx]:
            return cx, cy
        for r in range(1, max_r + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    nxc, nyc = cx + dx, cy + dy
                    if 0 <= nyc < self.ny and 0 <= nxc < self.nx and not self.blocked[nyc, nxc]:
                        return nxc, nyc
        return None

    def line_of_sight(self, x1, y1, x2, y2) -> bool:
        """True if the straight segment stays in free cells (for smoothing)."""
        length = math.hypot(x2 - x1, y2 - y1)
        n = max(2, int(length / (self.cell * 0.5)) + 1)
        for t in np.linspace(0.0, 1.0, n):
            cx, cy = self.world_to_cell(x1 + t * (x2 - x1), y1 + t * (y2 - y1))
            if not (0 <= cy < self.ny and 0 <= cx < self.nx) or self.blocked[cy, cx]:
                return False
        return True

    def plan(self, sx, sy, gx, gy) -> list[tuple[float, float]] | None:
        """A* from world (sx,sy) to (gx,gy). Returns smoothed world waypoints."""
        start = self._nearest_free(*self.world_to_cell(sx, sy))
        goal = self._nearest_free(*self.world_to_cell(gx, gy))
        if start is None or goal is None:
            return None

        SQRT2 = math.sqrt(2.0)
        nbrs = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]

        def h(c):
            dx, dy = abs(c[0] - goal[0]), abs(c[1] - goal[1])
            return max(dx, dy) + (SQRT2 - 1.0) * min(dx, dy)

        open_set = [(h(start), 0.0, start)]
        g = {start: 0.0}
        came: dict = {}
        closed = set()

        while open_set:
            _, gc, cur = heapq.heappop(open_set)
            if cur == goal:
                cells = [cur]
                while cur in came:
                    cur = came[cur]
                    cells.append(cur)
                cells.reverse()
                pts = [(sx, sy)] + [self.cell_to_world(cx, cy) for cx, cy in cells] + [(gx, gy)]
                return self._smooth(pts)
            if cur in closed:
                continue
            closed.add(cur)
            for dx, dy, cost in nbrs:
                nxt = (cur[0] + dx, cur[1] + dy)
                if not (0 <= nxt[1] < self.ny and 0 <= nxt[0] < self.nx):
                    continue
                if self.blocked[nxt[1], nxt[0]]:
                    continue
                # Forbid diagonal corner-cutting between two blocked cells
                if dx and dy and (self.blocked[cur[1], cur[0] + dx] or
                                  self.blocked[cur[1] + dy, cur[0]]):
                    continue
                ng = gc + cost
                if ng < g.get(nxt, float("inf")):
                    g[nxt] = ng
                    came[nxt] = cur
                    heapq.heappush(open_set, (ng + h(nxt), ng, nxt))
        return None

    def _smooth(self, pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
        """Greedy line-of-sight shortcutting."""
        if len(pts) <= 2:
            return pts
        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self.line_of_sight(*pts[i], *pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        return out


# ── Hybrid route builder ─────────────────────────────────────────────────────

def build_route(track: TrackData, grid: GridPlanner | None = None,
                n_exit_candidates: int = 4) -> tuple[list[tuple[float, float]], dict]:
    """
    Build the full start→goal route:
      line-network A* (follow the tape) + occupancy-grid A* (maze to the goal).

    The exit node (where we leave the line network) is chosen to minimize
    total route length over a few candidates near the goal.

    Returns (waypoints, info). Raises RuntimeError if no route exists.
    """
    grid = grid or GridPlanner(track)
    gx, gy = track.goal_x, track.goal_y
    info: dict = {"grid": grid}

    if not track.line_paths:
        tail = grid.plan(track.start_x, track.start_y, gx, gy)
        if tail is None:
            raise RuntimeError("GridPlanner found no path from start to goal")
        info["exit_node"] = None
        return _push_from_corners(_dedupe(tail), track, grid), info

    nodes, _ = build_line_graph(track)
    start_id = find_nearest_node(nodes, track.start_x, track.start_y)

    # Score every node as a potential exit point. The tournament robot should
    # drive the line course to its end before heading into the maze, so we
    # penalize skipped line length twice as heavily as extra grid distance —
    # a short diagonal across open floor must not beat finishing the line.
    line_lens: dict[int, float] = {}
    line_paths_by_node: dict[int, list] = {}
    for cand in nodes:
        ids = astar_line_path(nodes, start_id, cand.id)
        if ids:
            wps = [(nodes[i].x, nodes[i].y) for i in ids]
            line_paths_by_node[cand.id] = wps
            line_lens[cand.id] = polyline_length(wps)
    if not line_lens:
        raise RuntimeError("Line network unreachable from start")
    max_line = max(line_lens.values())

    best = None
    for cand in nodes:
        if cand.id not in line_paths_by_node:
            continue
        line_wps = line_paths_by_node[cand.id]
        if math.hypot(cand.x - gx, cand.y - gy) < 8.0:
            tail = [(gx, gy)]
        else:
            tail = grid.plan(cand.x, cand.y, gx, gy)
            if tail is None:
                continue
            tail = tail[1:]  # first tail point == exit node
        tail_len = polyline_length([line_wps[-1]] + tail)
        # Penalize a sharp turn at the line→maze transition. The firmware
        # command set cannot reverse or pivot in place, so a >90° switchback
        # (e.g. exiting the dead-end entry spur) wedges the chassis on nearby
        # wall corners. Exits where the tail continues roughly straight on
        # are strongly preferred.
        turn_pen = 0.0
        if len(line_wps) >= 2 and tail:
            a = math.atan2(line_wps[-1][1] - line_wps[-2][1],
                           line_wps[-1][0] - line_wps[-2][0])
            b = math.atan2(tail[0][1] - line_wps[-1][1],
                           tail[0][0] - line_wps[-1][0])
            turn = abs(wrap_deg(math.degrees(b - a)))
            turn_pen = 3.0 * max(0.0, turn - 60.0)
        score = tail_len + 2.0 * (max_line - line_lens[cand.id]) + turn_pen
        if best is None or score < best[0]:
            best = (score, line_wps, tail, cand.id)

    if best is None:
        raise RuntimeError("No line+grid route found from start to goal")

    _, line_wps, tail, exit_id = best
    info["exit_node"] = exit_id
    info["line_len"] = round(polyline_length(line_wps), 1)
    route = [(track.start_x, track.start_y)] + line_wps + tail
    route = _push_from_corners(_dedupe(route), track, grid)
    return route, info


def _dedupe(pts, eps: float = 1.0):
    out = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > eps:
            out.append(p)
    return out


def _push_from_corners(route, track: TrackData, grid: GridPlanner,
                       corner_clear_cm: float = 12.5):
    """
    Push route points away from nearby wall ENDPOINTS (convex corners).

    Straight corridor sections only need half-width clearance and the grid's
    9.5cm inflation is fine — but when the route TURNS around a wall corner,
    the chassis nose swings ~12cm from the center and wedges on the corner
    point (observed at the entry fin, the spiral mouth, and the old
    antechamber chicane). Inflating the whole grid that much would seal the
    track's legitimate ~30cm passages, so widen clearance only at corners.
    """
    corners = []
    for w in track.walls:
        corners.append((w[0], w[1]))
        corners.append((w[2], w[3]))

    # Densify first: the offending close pass is usually mid-segment on the
    # smoothed polyline, not at an existing vertex.
    dense: list[tuple[float, float]] = [tuple(route[0])]
    for a, b in zip(route, route[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(seg / 5.0))
        for k in range(1, n + 1):
            t = k / n
            dense.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    out = dense
    for i in range(1, len(out) - 1):
        x, y = out[i]
        for cx, cy in corners:
            d = math.hypot(x - cx, y - cy)
            if 1e-6 < d < corner_clear_cm:
                nx_, ny_ = (x - cx) / d, (y - cy) / d
                cand = (cx + nx_ * corner_clear_cm, cy + ny_ * corner_clear_cm)
                # keep the adjusted point only if the route stays drivable
                if (grid.line_of_sight(*out[i - 1], *cand)
                        and grid.line_of_sight(*cand, *out[i + 1])):
                    out[i] = cand
                    x, y = cand
    return out


# ── Discrete-command route follower ──────────────────────────────────────────

class RouteFollower:
    """
    Pure-pursuit tracker that outputs the six discrete firmware commands.

    Keeps a monotonic arc-length progress along the route and projects the
    robot onto a WINDOW around it — a global nearest-point projection would
    jump across the zigzag folds of the line course.

    Hysteresis on the command thresholds prevents chattering between classes
    (a major source of oscillation in the old labels).
    """

    def __init__(self, route: list[tuple[float, float]],
                 grid: "GridPlanner | None" = None,
                 goal_radius: float = 5.0,
                 lookahead_cm: float = 12.0,
                 slight_deg: float = 12.0,
                 hard_deg: float = 50.0,
                 hyst_deg: float = 5.0,
                 line_mode_until_cm: float = 0.0,
                 ir_threshold: float = 400.0,
                 lost_escalate_s: float = 2.5,
                 fallback_after_s: float = 6.5,   # room for the 2-phase search
                 decision_hz: float = 10.0,
                 ir_front: bool = False,
                 hard_turn_dps: float = 188.0):   # hard-turn yaw rate (physics-dependent)
        """
        line_mode_until_cm: while route progress is below this, the expert
        follows the tape with a SENSOR-DRIVEN bang-bang controller instead of
        privileged pure pursuit. Rationale (measured, not theoretical): the
        IR sensors sit 12.4cm apart around a 2.5cm tape, so a pose-based
        expert glides with both sensors on floor 80% of the time — a cloned
        policy gets no tracking signal and wanders (first BC models orbited
        the tape without locking on). Bang-bang makes every steering switch
        a function of the OBSERVATION stream (IR firing side, last command,
        rotation since the turn started), so the behavior is clonable by
        construction. With rear-mounted sensors the correct polarity is
        "turn toward the side that fired": yawing left sweeps the rear
        (and its sensors) right across the tape, and vice versa.
        The privileged pursuit remains as a rare rescue fallback and for the
        maze phase, where ultrasonics observe the walls directly.
        """
        self.grid = grid  # for line-of-sight target clamping (prevents corner cutting)
        self.line_mode_until = line_mode_until_cm
        self.ir_threshold = ir_threshold
        self.ir_front = ir_front
        self._lost_escalate = max(1, int(lost_escalate_s * decision_hz))
        self._fallback_after = max(2, int(fallback_after_s * decision_hz))
        self._hard_deg_per_frame = max(4.0, hard_turn_dps / decision_hz)
        pts = np.asarray(route, dtype=float)
        seg = pts[1:] - pts[:-1]
        seg_len = np.hypot(seg[:, 0], seg[:, 1])
        keep = seg_len > 1e-9
        self.pts = np.vstack([pts[0], pts[1:][keep]])
        seg = self.pts[1:] - self.pts[:-1]
        self.seg_len = np.hypot(seg[:, 0], seg[:, 1])
        self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.total = float(self.cum[-1])

        self.goal = tuple(self.pts[-1])
        self.goal_radius = goal_radius
        self.lookahead = lookahead_cm
        self.slight = slight_deg
        self.hard = hard_deg
        self.hyst = hyst_deg

        # Line-phase corner table: (arc_length, turn_is_left) for every route
        # vertex sharper than 45°. Corners this sharp cannot be caught
        # reactively at speed (the tape ends up behind the sensor line), so
        # the expert turns pre-emptively at the vertex. The cloned policy
        # learns the same timing from its odometry-distance feature.
        self._corners: list[tuple[float, bool, float]] = []  # (arc_s, left, deg)
        for i in range(1, len(self.pts) - 1):
            if self.cum[i] > self.line_mode_until:
                break
            v1 = self.pts[i] - self.pts[i - 1]
            v2 = self.pts[i + 1] - self.pts[i]
            a1 = math.degrees(math.atan2(-v1[1], v1[0]))
            a2 = math.degrees(math.atan2(-v2[1], v2[0]))
            turn = wrap_deg(a2 - a1)
            if abs(turn) > 45.0:
                self._corners.append((float(self.cum[i]), turn > 0, abs(turn)))
        self.reset()

    def reset(self):
        self.progress = 0.0
        self.last_cmd = CMD_FORWARD
        self.done = False
        self.offset_dist = 0.0  # distance from route at last projection
        # Unstick reflex state
        self._pose_hist: list[tuple[float, float]] = []
        self._recovery_seq: list[int] = []   # queued recovery commands
        self._recover_count = 0              # consecutive recoveries (escalation)
        self._last_recover_progress = -1e9
        # Bang-bang line-following state
        self._sweep = CMD_SLIGHT_L           # current sweep direction command
        self._since_fire = 0                 # decisions since an IR fired
        self._last_fire_left = True
        self._streak = 0                     # consecutive same-side hot frames
        self._corners_done: set[int] = set() # one-shot corner pivots consumed
        self._corner_seq = 0                 # frames left in committed pivot
        self._corner_left = True
        self.fallback_frames = 0             # pose-rescue usage (data quality metric)

    # -- geometry helpers ------------------------------------------------

    def _project(self, x: float, y: float,
                 behind: float = 20.0, ahead: float = 50.0):
        """Advance monotonic progress to the closest route point in a window."""
        lo_s = max(0.0, self.progress - behind)
        hi_s = min(self.total, self.progress + ahead)
        i0 = max(0, int(np.searchsorted(self.cum, lo_s, side="right")) - 1)
        i1 = min(len(self.seg_len), int(np.searchsorted(self.cum, hi_s, side="left")))

        best_d, best_s = float("inf"), self.progress
        for i in range(i0, max(i1, i0 + 1)):
            x1, y1 = self.pts[i]
            dx, dy = self.pts[i + 1] - self.pts[i]
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / L2))
            px, py = x1 + t * dx, y1 + t * dy
            d = math.hypot(x - px, y - py)
            if d < best_d:
                best_d = d
                best_s = self.cum[i] + t * self.seg_len[i]
        self.offset_dist = best_d
        self.progress = max(self.progress, best_s)

    def _point_at(self, s: float) -> tuple[float, float]:
        s = max(0.0, min(self.total, s))
        i = max(0, min(len(self.seg_len) - 1, int(np.searchsorted(self.cum, s, side="right")) - 1))
        t = 0.0 if self.seg_len[i] < 1e-12 else (s - self.cum[i]) / self.seg_len[i]
        p = self.pts[i] + t * (self.pts[i + 1] - self.pts[i])
        return float(p[0]), float(p[1])

    # -- policy ----------------------------------------------------------

    def command(self, x: float, y: float, heading_deg: float,
                ir: list[float] | None = None) -> int:
        """
        Expert action. Pose (x, y, heading) is privileged sim-only state;
        `ir` are the actual IR sensor readings ([left, right], 0-1023) and
        drive the bang-bang tape follower during the line phase.
        """
        if self.done:
            return CMD_STOP
        if math.hypot(self.goal[0] - x, self.goal[1] - y) < self.goal_radius:
            self.done = True
            return CMD_STOP

        self._project(x, y)

        # ── Line phase: sensor-driven bang-bang (see __init__ docstring) ──
        if (ir is not None and len(ir) >= 2
                and self.progress < self.line_mode_until
                and self.offset_dist < 30.0):
            cmd = self._bangbang(ir, self.progress)
            if cmd is not None:
                self.last_cmd = cmd
                return cmd
            # else: lost too long — fall through to pose-based rescue

        # Pure-pursuit target, clamped to line-of-sight through the inflated
        # grid. Without this the lookahead chord cuts across wall corners and
        # the chassis wedges on them (the discrete command set cannot reverse).
        if self.progress < self.line_mode_until:
            self.fallback_frames += 1
        s = self.progress + self.lookahead
        tx, ty = self._point_at(s)
        if self.grid is not None:
            while s > self.progress + 5.0 and not self.grid.line_of_sight(x, y, tx, ty):
                s -= 3.0
                tx, ty = self._point_at(s)
        if math.hypot(tx - x, ty - y) > 6.0:
            bearing = math.degrees(math.atan2(-(ty - y), tx - x))  # screen y inverted
        else:
            # Target degenerated to (nearly) our own position — we're at a
            # switchback or tight corner. Bearing-to-point is unstable here
            # (its sign flips with millimeter jitter → hard-L/R livelock), so
            # align with the route's tangent direction ahead instead.
            ax, ay = self._point_at(s + 8.0)
            bearing = math.degrees(math.atan2(-(ay - ty), ax - tx))
        err = wrap_deg(bearing - heading_deg)

        # Unstick reflex: if the pose hasn't moved for ~1s while commanding
        # motion, the chassis is wedged on a wall corner. Recovery = pivot
        # hard until the nose swings free, then drive FORWARD a few beats to
        # clear the corner before pure pursuit resumes — resuming immediately
        # just steers back into the identical wedge. Escalate (longer pivot,
        # alternating direction) if the same spot keeps wedging.
        self._pose_hist.append((x, y))
        if len(self._pose_hist) > 10:
            self._pose_hist.pop(0)
        if self._recovery_seq:
            cmd = self._recovery_seq.pop(0)
            self.last_cmd = cmd
            return cmd
        if (len(self._pose_hist) == 10
                and math.hypot(x - self._pose_hist[0][0], y - self._pose_hist[0][1]) < 1.0):
            if self.progress - self._last_recover_progress > 15.0:
                self._recover_count = 0          # new wedge site
            self._last_recover_progress = self.progress
            self._recover_count += 1
            toward = CMD_HARD_L if err > 0 else CMD_HARD_R
            away = CMD_HARD_R if toward == CMD_HARD_L else CMD_HARD_L
            pivot = toward if self._recover_count % 2 == 1 else away
            n_pivot = 5 + 2 * min(self._recover_count, 5)
            self._recovery_seq = [pivot] * n_pivot + [CMD_FORWARD] * 4
            self._pose_hist.clear()
            cmd = self._recovery_seq.pop(0)
            self.last_cmd = cmd
            return cmd

        cmd = self._classify(err)
        self.last_cmd = cmd
        return cmd

    def _bangbang(self, ir: list[float], progress: float) -> int | None:
        """
        Tape follower on raw IR. Returns None when lost long enough that the
        pose-based rescue should take over (rare; tracked in fallback_frames).

        Front-mounted sensors (stable feedback): steer toward the hot sensor,
        drive STRAIGHT while the tape sits quietly between the sensors —
        the classic 3-state line follower. Corners sharper than 45° are taken
        pre-emptively from the corner table (see __init__).
        Rear-mounted sensors (unstable lever arm): hold the sweep until the
        OPPOSITE sensor fires; driving straight when quiet would just carry
        the accumulated yaw error away from the tape.
        """
        left_hot = ir[0] > self.ir_threshold
        right_hot = ir[1] > self.ir_threshold
        if left_hot or right_hot:
            # Tape found — always beats a committed corner pivot (finishing
            # the pivot blind once the tape is visible caused hard-turn
            # circling deadlocks pinned by monotonic progress).
            self._corner_seq = 0
            fire_left = left_hot and (not right_hot or ir[0] >= ir[1])
            if fire_left == self._last_fire_left and self._since_fire == 0:
                self._streak += 1
            else:
                self._streak = 1
            self._last_fire_left = fire_left
            self._since_fire = 0
            if self.ir_front and self._streak >= 2:
                # Same side persistently hot → the tape is bending away
                # (corner), not a passing graze — turn hard to stay with it.
                self._sweep = CMD_HARD_L if fire_left else CMD_HARD_R
            else:
                self._sweep = CMD_SLIGHT_L if fire_left else CMD_SLIGHT_R
            return self._sweep

        # Committed corner pivot in progress (quiet sensors)
        if self._corner_seq > 0:
            self._corner_seq -= 1
            self._since_fire = 0
            return CMD_HARD_L if self._corner_left else CMD_HARD_R

        # Pre-emptive one-shot corner pivot (front mode): sharper than 45°
        # the tape leaves the sensors' reach almost instantly at speed, so
        # turn ~the vertex angle open-loop, sized by the corner table.
        if self.ir_front:
            for idx, (s_k, turn_left, deg) in enumerate(self._corners):
                if idx not in self._corners_done and -2.0 <= s_k - progress <= 6.0:
                    self._corners_done.add(idx)
                    self._corner_left = turn_left
                    self._corner_seq = max(1, int(deg / self._hard_deg_per_frame)) - 1
                    self._since_fire = 0
                    return CMD_HARD_L if turn_left else CMD_HARD_R

        self._since_fire += 1
        if self._since_fire > self._fallback_after:
            return None  # genuinely lost — pose rescue
        if self._since_fire > self._lost_escalate:
            # Tape gone quiet (vertex overshoot / drift). Two-phase search:
            # first arc hard toward the side that last saw the tape (~1.2s,
            # covers the common case), then sweep back the OTHER way twice as
            # long — a widening scan instead of circling one way forever.
            phase = self._since_fire - self._lost_escalate
            first = CMD_HARD_L if self._last_fire_left else CMD_HARD_R
            second = CMD_HARD_R if self._last_fire_left else CMD_HARD_L
            return first if phase <= self._lost_escalate else second
        if self.ir_front:
            return CMD_FORWARD   # tape between the sensors — hold course
        return self._sweep       # rear sensors: keep sweeping to the far edge

    def _classify(self, err: float) -> int:
        """Map heading error to a discrete command, with hysteresis."""
        a = abs(err)
        left = err > 0
        t_slight, t_hard = self.slight, self.hard

        if self.last_cmd == CMD_FORWARD:
            t_slight += self.hyst           # stickier straight
        elif self.last_cmd in (CMD_SLIGHT_L, CMD_SLIGHT_R):
            t_slight -= self.hyst           # stickier slight turn
            t_hard += self.hyst
        elif self.last_cmd in (CMD_HARD_L, CMD_HARD_R):
            t_hard -= self.hyst             # stickier hard turn

        if a < t_slight:
            return CMD_FORWARD
        if a < t_hard:
            return CMD_SLIGHT_L if left else CMD_SLIGHT_R
        return CMD_HARD_L if left else CMD_HARD_R


def make_follower(route, info: dict, decision_hz: float = 10.0,
                  robot_cfg=None, physics_params=None) -> RouteFollower:
    """
    Standard follower construction for data generation / DAgger / evaluation.
    Enables sensor-driven bang-bang tape following for the line phase, so all
    pipeline stages demonstrate the same (observable) behavior. IR mounting
    (front vs rear) is read from the robot config — it changes the stable
    control law (see RouteFollower._bangbang). Hard-turn rate is derived from
    the physics params so corner pivots stay angle-accurate at any BASE_SPEED.
    """
    line_len = info.get("line_len") or 0.0
    ir_front = False
    if robot_cfg is not None and robot_cfg.ir_sensors:
        ir_front = all(s.mount_y < 0 for s in robot_cfg.ir_sensors)
    hard_dps = 188.0
    if physics_params is not None and robot_cfg is not None:
        v = physics_params.max_wheel_speed_cmps / 255.0
        w_fast = physics_params.base_pwm * v
        w_slow = -int(physics_params.base_pwm * 0.3) * v
        hard_dps = math.degrees((w_fast - w_slow) / robot_cfg.chassis.wheelbase_cm)
    return RouteFollower(route, grid=info.get("grid"),
                         line_mode_until_cm=max(0.0, line_len - 20.0),
                         decision_hz=decision_hz, ir_front=ir_front,
                         hard_turn_dps=hard_dps)


# ── CLI: plan a route and plot it ────────────────────────────────────────────

def plot_route(track: TrackData, route, grid: GridPlanner, out_path: str,
               trajectories: list | None = None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 12))
    for w in track.walls:
        ax.plot([w[0], w[2]], [w[1], w[3]], color="#333", lw=2)
    for s in track.line_paths:
        ax.plot([s.x1, s.x2], [s.y1, s.y2], color="#999", lw=4, alpha=0.6)
    if route:
        r = np.asarray(route)
        ax.plot(r[:, 0], r[:, 1], "b.-", lw=1.2, ms=4, label="expert route")
    if trajectories:
        for tr, ok in trajectories:
            t = np.asarray(tr)
            ax.plot(t[:, 0], t[:, 1], lw=0.9, alpha=0.8,
                    color="#2a9d2a" if ok else "#d43a3a")
    ax.plot(track.start_x, track.start_y, "go", ms=10, label="start")
    ax.plot(track.goal_x, track.goal_y, "r*", ms=16, label="goal")
    ax.set_aspect("equal")
    ax.invert_yaxis()  # screen coords: negative y is up
    ax.legend(loc="lower right", fontsize=8)
    ax.set_title(track.name)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    print(f"Plot saved: {out_path}")


def main():
    ap = argparse.ArgumentParser(description="Plan and visualize the expert route")
    ap.add_argument("--track", required=True)
    ap.add_argument("--plot", default=None, help="Save route plot to this PNG")
    args = ap.parse_args()

    track = load_track(args.track)
    grid = GridPlanner(track)
    route, info = build_route(track, grid)
    print(f"Route: {len(route)} waypoints, {polyline_length(route):.0f} cm total "
          f"(exit node {info.get('exit_node')})")
    if args.plot:
        plot_route(track, route, grid, args.plot)


if __name__ == "__main__":
    main()
