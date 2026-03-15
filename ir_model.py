"""
IR line sensor simulator for TCRT5000.

Models the TCRT5000 reflective IR sensor reading a white line on a black surface.
Sensor returns a high analog value (~800-950) over white and low (~50-200) over black.
"""

import math
import random
from dataclasses import dataclass


@dataclass
class TrackLine:
    """A white line segment on the track surface (world cm)."""
    x1: float
    y1: float
    x2: float
    y2: float
    width_cm: float = 6.0


def make_lane(
    x1: float, y1: float, x2: float, y2: float,
    strip_width_cm: float = 2.0,
    separation_cm: float = 15.0,
) -> list[TrackLine]:
    """
    Return two parallel TrackLine strips centred on the given segment.
    The robot drives in the channel between them.

    separation_cm: center-to-center distance between the two strips.
    strip_width_cm: width of each individual strip.
    Inner gap = separation_cm - strip_width_cm.
    """
    dx, dy = x2 - x1, y2 - y1
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return []
    # Unit perpendicular (90° CCW rotation)
    px, py = -dy / length, dx / length
    half = separation_cm / 2
    return [
        TrackLine(x1 - px * half, y1 - py * half, x2 - px * half, y2 - py * half, strip_width_cm),
        TrackLine(x1 + px * half, y1 + py * half, x2 + px * half, y2 + py * half, strip_width_cm),
    ]


def _dist_to_segment(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    """Minimum distance from point (px, py) to line segment (x1,y1)-(x2,y2)."""
    dx = x2 - x1
    dy = y2 - y1
    seg_sq = dx * dx + dy * dy
    if seg_sq < 1e-12:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / seg_sq))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


class IRSensorSimulator:
    """Simulates TCRT5000 analog IR reflective sensor readings."""

    # Real life: white tape on dark floor. TCRT5000 returns HIGH over white tape.
    # Sim shows black tape visually, but the sensor values match the real hardware.
    ON_TAPE_BASE  = 875.0   # white tape  → high reflection → high reading
    OFF_TAPE_BASE = 125.0   # dark floor  → low reflection  → low reading
    NOISE_STD = 20.0
    TRANSITION_CM = 0.8  # soft blend zone on each side of the tape edge

    def measure(self, sensor_x: float, sensor_y: float, track_lines: list[TrackLine]) -> float:
        """
        Return analog reading 0-1023.
        High (~800-950) = on tape (white in real life).
        Low  (~50-200)  = off tape (dark floor).
        Uses a soft transition zone at tape edges to prevent flickering.
        """
        # Find closest line and its half-width
        min_dist = float('inf')
        half_w = 0.0
        for tl in track_lines:
            d = _dist_to_segment(sensor_x, sensor_y, tl.x1, tl.y1, tl.x2, tl.y2)
            if d < min_dist:
                min_dist = d
                half_w = tl.width_cm / 2

        # Linear blend over TRANSITION_CM on each side of the edge
        inner = half_w - self.TRANSITION_CM
        outer = half_w + self.TRANSITION_CM
        if min_dist <= inner:
            t = 1.0
        elif min_dist >= outer:
            t = 0.0
        else:
            t = 1.0 - (min_dist - inner) / (2 * self.TRANSITION_CM)

        base = t * self.ON_TAPE_BASE + (1.0 - t) * self.OFF_TAPE_BASE  # t=1 on tape, t=0 off tape
        return max(0.0, min(1023.0, base + random.gauss(0, self.NOISE_STD)))
