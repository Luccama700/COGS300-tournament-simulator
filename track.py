"""
Track data model.

Loads and saves track YAML files. Builds a graph representation
of the line path network for A* pathfinding through junctions.
"""

import math
from dataclasses import dataclass, field
from pathlib import Path
import yaml


@dataclass
class LineSeg:
    x1: float
    y1: float
    x2: float
    y2: float
    width_cm: float = 2.5

    @property
    def length(self) -> float:
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)

    def point_distance(self, px: float, py: float) -> float:
        """Shortest distance from point to this line segment."""
        dx, dy = self.x2 - self.x1, self.y2 - self.y1
        if dx == 0 and dy == 0:
            return math.hypot(px - self.x1, py - self.y1)
        t = max(0, min(1, ((px - self.x1) * dx + (py - self.y1) * dy) / (dx * dx + dy * dy)))
        proj_x = self.x1 + t * dx
        proj_y = self.y1 + t * dy
        return math.hypot(px - proj_x, py - proj_y)

    def point_on_line(self, px: float, py: float) -> bool:
        """Check if point is within width_cm/2 of this segment."""
        return self.point_distance(px, py) <= self.width_cm / 2

    def lerp(self, t: float) -> tuple[float, float]:
        """Interpolate along segment. t=0 -> start, t=1 -> end."""
        return (self.x1 + t * (self.x2 - self.x1), self.y1 + t * (self.y2 - self.y1))

    def angle(self) -> float:
        """Angle of segment in degrees (0=east, CCW positive)."""
        return math.degrees(math.atan2(-(self.y2 - self.y1), self.x2 - self.x1))


@dataclass
class TrackData:
    name: str = "Unnamed Track"
    walls: list[list[float]] = field(default_factory=list)
    line_paths: list[LineSeg] = field(default_factory=list)
    start_x: float = 0.0
    start_y: float = 0.0
    start_heading: float = 0.0
    goal_x: float = 0.0
    goal_y: float = 0.0
    bounds_w: float = 400.0
    bounds_h: float = 300.0

    def is_on_line(self, px: float, py: float) -> bool:
        """Check if a point is on any line segment."""
        for seg in self.line_paths:
            if seg.point_on_line(px, py):
                return True
        return False

    def nearest_line_segment(self, px: float, py: float) -> tuple[LineSeg | None, float]:
        """Find the nearest line segment to a point. Returns (segment, distance)."""
        best_seg = None
        best_dist = float("inf")
        for seg in self.line_paths:
            d = seg.point_distance(px, py)
            if d < best_dist:
                best_dist = d
                best_seg = seg
        return best_seg, best_dist


def load_track(path: str | Path) -> TrackData:
    """Load a track from YAML."""
    with open(path) as f:
        raw = yaml.safe_load(f)

    t = raw.get("track", {})
    track = TrackData(
        name=t.get("name", "Unnamed"),
        bounds_w=t.get("bounds", {}).get("width", 400),
        bounds_h=t.get("bounds", {}).get("height", 300),
    )

    # Walls
    for w in t.get("walls", []):
        if isinstance(w, list) and len(w) == 4:
            track.walls.append(w)

    # Line paths
    for lp in t.get("line_paths", []):
        track.line_paths.append(LineSeg(
            x1=lp["x1"], y1=lp["y1"],
            x2=lp["x2"], y2=lp["y2"],
            width_cm=lp.get("width_cm", 2.5),
        ))

    # Start
    start = t.get("start", {})
    track.start_x = start.get("x", 0)
    track.start_y = start.get("y", 0)
    track.start_heading = start.get("heading", 0)

    # Goal
    goal = t.get("goal", {})
    track.goal_x = goal.get("x", 0)
    track.goal_y = goal.get("y", 0)

    return track


def save_track(track: TrackData, path: str | Path) -> None:
    """Save a TrackData to YAML."""
    import os
    path = Path(path)
    os.makedirs(path.parent, exist_ok=True)
    data = {
        "track": {
            "name": track.name,
            "bounds": {"width": track.bounds_w, "height": track.bounds_h},
            "walls": [list(w) for w in track.walls],
            "line_paths": [
                {"x1": s.x1, "y1": s.y1, "x2": s.x2, "y2": s.y2, "width_cm": s.width_cm}
                for s in track.line_paths
            ],
            "start":  {"x": track.start_x,  "y": track.start_y,  "heading": track.start_heading},
            "goal":   {"x": track.goal_x,   "y": track.goal_y},
        }
    }
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


# --- Line graph for A* pathfinding ---

@dataclass
class LineNode:
    """A point where line segments meet (junction) or terminate (endpoint)."""
    x: float
    y: float
    id: int = 0
    neighbors: list[tuple[int, float]] = field(default_factory=list)  # (node_id, distance)


def build_line_graph(track: TrackData, merge_radius: float = 3.0) -> tuple[list[LineNode], dict[int, list[int]]]:
    """
    Build a graph from line segments for A* pathfinding.

    Endpoints of segments that are within merge_radius of each other
    are treated as the same node (junction). Also handles T-junctions
    where an endpoint lands on the middle of another segment by splitting
    that segment into two.
    """
    # First pass: detect T-junctions (endpoint near middle of another segment)
    # and split those segments
    segments = list(track.line_paths)
    splits_needed = True
    max_iterations = 5  # prevent infinite loops

    while splits_needed and max_iterations > 0:
        splits_needed = False
        max_iterations -= 1
        new_segments = []

        for seg in segments:
            split_point = None

            # Check if any endpoint from OTHER segments lands on this segment's middle
            for other in segments:
                if other is seg:
                    continue
                for px, py in [(other.x1, other.y1), (other.x2, other.y2)]:
                    # Skip if this point is near our endpoints (that's a normal connection)
                    if math.hypot(px - seg.x1, py - seg.y1) <= merge_radius:
                        continue
                    if math.hypot(px - seg.x2, py - seg.y2) <= merge_radius:
                        continue

                    # Check if point is on this segment
                    dist = seg.point_distance(px, py)
                    if dist <= merge_radius:
                        split_point = (px, py)
                        break
                if split_point:
                    break

            if split_point:
                # Split this segment into two at the split point
                sx, sy = split_point
                new_segments.append(LineSeg(seg.x1, seg.y1, sx, sy, seg.width_cm))
                new_segments.append(LineSeg(sx, sy, seg.x2, seg.y2, seg.width_cm))
                splits_needed = True
            else:
                new_segments.append(seg)

        segments = new_segments

    # Collect all endpoints from (possibly split) segments
    raw_points: list[tuple[float, float, int, str]] = []
    for i, seg in enumerate(segments):
        raw_points.append((seg.x1, seg.y1, i, "start"))
        raw_points.append((seg.x2, seg.y2, i, "end"))

    # Merge nearby points into nodes
    nodes: list[LineNode] = []
    point_to_node: dict[int, int] = {}

    for pi, (px, py, si, end) in enumerate(raw_points):
        merged = False
        for node in nodes:
            if math.hypot(px - node.x, py - node.y) <= merge_radius:
                point_to_node[pi] = node.id
                merged = True
                break
        if not merged:
            node = LineNode(x=px, y=py, id=len(nodes))
            nodes.append(node)
            point_to_node[pi] = node.id

    # Build edges: each segment connects its start node to its end node
    seen_edges: set[tuple[int, int]] = set()
    for i, seg in enumerate(track.line_paths):
        start_pi = i * 2
        end_pi = i * 2 + 1
        n1 = point_to_node[start_pi]
        n2 = point_to_node[end_pi]
        if n1 == n2:
            continue
        edge = (min(n1, n2), max(n1, n2))
        if edge not in seen_edges:
            seen_edges.add(edge)
            dist = seg.length
            nodes[n1].neighbors.append((n2, dist))
            nodes[n2].neighbors.append((n1, dist))

    return nodes, point_to_node


def find_nearest_node(nodes: list[LineNode], x: float, y: float) -> int:
    """Find the node closest to a world position."""
    best_id = 0
    best_dist = float("inf")
    for node in nodes:
        d = math.hypot(x - node.x, y - node.y)
        if d < best_dist:
            best_dist = d
            best_id = node.id
    return best_id


def astar_line_path(nodes: list[LineNode], start_id: int, goal_id: int) -> list[int]:
    """
    A* pathfinding through the line graph.
    Returns list of node IDs from start to goal, or empty list if no path.
    """
    import heapq

    def heuristic(a: int, b: int) -> float:
        na, nb = nodes[a], nodes[b]
        return math.hypot(na.x - nb.x, na.y - nb.y)

    open_set: list[tuple[float, int]] = [(0.0, start_id)]
    came_from: dict[int, int] = {}
    g_score: dict[int, float] = {start_id: 0.0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal_id:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return list(reversed(path))

        for neighbor_id, dist in nodes[current].neighbors:
            tentative_g = g_score[current] + dist
            if tentative_g < g_score.get(neighbor_id, float("inf")):
                came_from[neighbor_id] = current
                g_score[neighbor_id] = tentative_g
                f = tentative_g + heuristic(neighbor_id, goal_id)
                heapq.heappush(open_set, (f, neighbor_id))

    return []  # no path found
