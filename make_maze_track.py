"""
Derive the maze-only track from a full tournament track.

Owner decision 2026-07-04: the graded objective is now the WALL/MAZE section
only (line following deprioritized, code kept). This script produces that
track reproducibly instead of hand-editing YAML: same walls, same goal, no
line paths, and the start pose placed at the maze mouth — the exact point
where the full-course expert route leaves the tape network — heading along
the route tangent.

Usage:
    python make_maze_track.py \
        --track configs/tracks/COGS_300_Tournament_Track/COGS_300_Tournament_Track_v03.yaml \
        --out   configs/tracks/COGS_300_Tournament_Track/COGS_300_Tournament_Track_v03_maze.yaml
"""

import argparse
import math

import numpy as np
import yaml

from track import load_track
from expert_policy import build_route


def main():
    ap = argparse.ArgumentParser(description="Make maze-only track from full track")
    ap.add_argument("--track", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--start-back-cm", type=float, default=0.0,
                    help="Place the start this far BEFORE the tape-end arc "
                         "(along the route) if the mouth needs approach room")
    args = ap.parse_args()

    track = load_track(args.track)
    route, info = build_route(track)
    line_len = info.get("line_len") or 0.0
    if line_len <= 0:
        raise SystemExit("Track has no line phase — it is already maze-only?")

    pts = np.asarray(route, dtype=float)
    seg = pts[1:] - pts[:-1]
    seg_len = np.hypot(seg[:, 0], seg[:, 1])
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    s0 = max(0.0, line_len - args.start_back_cm)

    def point_at(s):
        i = max(0, min(len(seg_len) - 1, int(np.searchsorted(cum, s, side="right")) - 1))
        t = 0.0 if seg_len[i] < 1e-12 else (s - cum[i]) / seg_len[i]
        p = pts[i] + t * (pts[i + 1] - pts[i])
        return float(p[0]), float(p[1])

    x, y = point_at(s0)
    xa, ya = point_at(max(0.0, s0 - 2.0))
    xb, yb = point_at(min(float(cum[-1]), s0 + 8.0))
    heading = math.degrees(math.atan2(-(yb - ya), xb - xa))  # screen-y convention

    out = {
        "track": {
            "name": track.name + " [maze-only: starts at maze mouth]",
            "bounds": {"width": track.bounds_w, "height": track.bounds_h},
            "walls": [[float(v) for v in w] for w in track.walls],
            "line_paths": [],
            "start": {"x": round(x, 2), "y": round(y, 2),
                      "heading": round(heading, 2)},
            "goal": {"x": float(track.goal_x), "y": float(track.goal_y)},
        }
    }
    with open(args.out, "w") as f:
        yaml.safe_dump(out, f, sort_keys=False)
    print(f"maze track written: {args.out}")
    print(f"  start=({x:.1f}, {y:.1f}) heading={heading:.1f} deg "
          f"(tape-end arc {line_len:.1f} cm of full route {cum[-1]:.1f} cm)")
    print(f"  goal=({track.goal_x}, {track.goal_y})")


if __name__ == "__main__":
    main()
