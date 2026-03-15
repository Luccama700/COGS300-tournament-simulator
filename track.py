"""
Track data model — load/save tournament tracks as YAML.

A track contains walls (maze boundaries), line_paths (tape for IR following),
a start pose, an optional goal marker, and bounding dimensions.
"""

from dataclasses import dataclass, field
from pathlib import Path
import yaml

from physics import Wall
from ir_model import TrackLine


@dataclass
class TrackData:
    name: str = "New Track"
    walls: list[Wall] = field(default_factory=list)
    line_paths: list[TrackLine] = field(default_factory=list)
    start_x: float = 0.0
    start_y: float = 0.0
    start_heading: float = 90.0
    goal_x: float | None = None
    goal_y: float | None = None
    bounds_w: float = 400.0
    bounds_h: float = 300.0


def load_track(path: str | Path) -> TrackData:
    with open(path) as f:
        raw = yaml.safe_load(f)
    t = raw["track"]

    walls = [Wall(*w) for w in t.get("walls", [])]

    line_paths = []
    for lp in t.get("line_paths", []):
        line_paths.append(TrackLine(
            x1=lp["x1"], y1=lp["y1"], x2=lp["x2"], y2=lp["y2"],
            width_cm=lp.get("width_cm", 5.0),
        ))

    start = t.get("start") or {}
    goal  = t.get("goal")  or {}
    bounds = t.get("bounds") or {}

    return TrackData(
        name=t.get("name", "Track"),
        walls=walls,
        line_paths=line_paths,
        start_x=start.get("x", 0.0),
        start_y=start.get("y", 0.0),
        start_heading=start.get("heading", 90.0),
        goal_x=goal.get("x") if goal else None,
        goal_y=goal.get("y") if goal else None,
        bounds_w=bounds.get("width", 400.0),
        bounds_h=bounds.get("height", 300.0),
    )


def save_track(track: TrackData, path: str | Path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)

    track_dict: dict = {
        "name": track.name,
        "walls": [
            [round(w.x1, 2), round(w.y1, 2), round(w.x2, 2), round(w.y2, 2)]
            for w in track.walls
        ],
        "line_paths": [
            {"x1": round(tl.x1, 2), "y1": round(tl.y1, 2),
             "x2": round(tl.x2, 2), "y2": round(tl.y2, 2),
             "width_cm": tl.width_cm}
            for tl in track.line_paths
        ],
        "start": {
            "x": round(track.start_x, 2),
            "y": round(track.start_y, 2),
            "heading": round(track.start_heading, 1),
        },
        "bounds": {"width": track.bounds_w, "height": track.bounds_h},
    }
    if track.goal_x is not None:
        track_dict["goal"] = {
            "x": round(track.goal_x, 2),
            "y": round(track.goal_y, 2),
        }

    with open(path, "w") as f:
        yaml.dump({"track": track_dict}, f, default_flow_style=False, sort_keys=False)
