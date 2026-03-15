"""
Realistic HC-SR04 sensor simulation model.

Replaces the basic gaussian noise in physics.py with:
- Distance-dependent noise (gets noisier further out)
- Measurement bias
- Dropout modeling (timeouts, missed echoes)
- Multi-ray beam cone (optional, for angled wall detection)
- Sequential trigger timing
- Calibration profile loading

Can run with defaults (good HC-SR04 approximation) or with
a measured calibration profile from the analyze tool.
"""

import math
import random
from dataclasses import dataclass, field
from pathlib import Path
import numpy as np
import yaml


@dataclass
class SensorNoiseProfile:
    """Noise model parameters. Can be loaded from calibration data or use defaults."""

    # Distance-dependent noise: std = noise_base_std + noise_per_cm * distance
    noise_base_std: float = 0.5       # cm — baseline noise at 0cm
    noise_per_cm: float = 0.008       # cm of noise per cm of distance

    # Systematic bias
    bias_cm: float = 0.0              # constant offset added to readings

    # Dropouts
    dropout_chance: float = 0.02      # probability of failed reading per measurement
    dropout_value: float = 0.0        # what failed reading returns

    # Range limits
    min_range_cm: float = 2.0         # dead zone — objects closer than this return min_range
    max_reliable_cm: float = 200.0    # readings beyond this are very noisy

    # Beam cone (multi-ray)
    beam_half_angle_deg: float = 15.0  # HC-SR04 effective beam width
    beam_rays: int = 5                 # number of rays to cast within the cone
    beam_use_minimum: bool = True      # True=return shortest ray (closest object), False=return mean

    # Sequential timing
    measurement_time_ms: float = 30.0  # time for one measurement cycle

    @property
    def noise_std_at(self) -> callable:
        """Returns a function: distance_cm -> noise_std_cm"""
        base = self.noise_base_std
        slope = self.noise_per_cm
        return lambda d: base + slope * d


def load_noise_profile(path: str | Path) -> SensorNoiseProfile:
    """Load noise profile from a calibration YAML file."""
    with open(path) as f:
        raw = yaml.safe_load(f)

    sn = raw.get("sensor_noise", {})
    return SensorNoiseProfile(
        noise_base_std=sn.get("noise_base_std", 0.5),
        noise_per_cm=sn.get("noise_per_cm", 0.008),
        bias_cm=sn.get("bias_cm", 0.0),
        dropout_chance=sn.get("dropout_chance", 0.02),
        dropout_value=sn.get("dropout_value", 0.0),
        min_range_cm=sn.get("min_range_cm", 2.0),
        max_reliable_cm=sn.get("max_reliable_cm", 200.0),
    )


# Default profiles for common scenarios
HC_SR04_DEFAULT = SensorNoiseProfile(
    noise_base_std=0.5,
    noise_per_cm=0.008,
    bias_cm=0.0,
    dropout_chance=0.02,
    dropout_value=0.0,
    min_range_cm=2.0,
    max_reliable_cm=200.0,
    beam_half_angle_deg=10.0,   # was 15.0 — effective angle for flat walls in corridors
    beam_rays=3,                 # was 5 — fewer rays, less edge overdetection
    beam_use_minimum=True,
    measurement_time_ms=30.0,
)

HC_SR04_NOISY = SensorNoiseProfile(
    noise_base_std=1.5,
    noise_per_cm=0.015,
    dropout_chance=0.05,
)

HC_SR04_CLEAN = SensorNoiseProfile(
    noise_base_std=0.2,
    noise_per_cm=0.003,
    dropout_chance=0.01,
)


class SensorSimulator:
    """
    Simulates HC-SR04 ultrasonic sensor readings with realistic noise.

    Usage:
        sim = SensorSimulator(profile=HC_SR04_DEFAULT)
        reading = sim.measure(true_distance_cm=42.5)
    """

    def __init__(self, profile: SensorNoiseProfile | None = None):
        self.profile = profile or HC_SR04_DEFAULT

    def measure(self, true_distance: float) -> float:
        """
        Simulate a single sensor measurement.

        Args:
            true_distance: actual distance to nearest object in cm
                          (from raycasting). Use -1 or > max_range for no object.

        Returns:
            Simulated sensor reading in cm.
            Returns dropout_value for failed readings.
        """
        p = self.profile

        # No object in range
        if true_distance < 0 or true_distance > p.max_reliable_cm:
            # Far objects sometimes still register with high noise
            if true_distance > 0 and true_distance < p.max_reliable_cm * 1.3:
                # Increasing dropout chance beyond reliable range
                excess = (true_distance - p.max_reliable_cm) / (p.max_reliable_cm * 0.3)
                if random.random() < 0.5 + 0.5 * excess:
                    return p.dropout_value
                # If it does register, very noisy
                noise = random.gauss(0, p.noise_base_std + p.noise_per_cm * true_distance * 3)
                return max(p.min_range_cm, true_distance + p.bias_cm + noise)
            return p.dropout_value

        # Object in dead zone
        if true_distance < p.min_range_cm:
            # Objects very close return garbage — sometimes min_range, sometimes dropout
            if random.random() < 0.3:
                return p.dropout_value
            return p.min_range_cm + random.gauss(0, 0.5)

        # Random dropout
        if random.random() < p.dropout_chance:
            return p.dropout_value

        # Normal measurement with distance-dependent noise
        noise_std = p.noise_base_std + p.noise_per_cm * true_distance
        noise = random.gauss(0, noise_std)

        reading = true_distance + p.bias_cm + noise

        # Clamp to valid range
        reading = max(p.min_range_cm, min(p.max_reliable_cm, reading))

        return reading

    def measure_with_cone(
        self,
        raycast_fn,
        origin_x: float, origin_y: float,
        center_angle_deg: float,
        max_range: float,
    ) -> float:
        """
        Simulate a measurement using multiple rays within the beam cone.

        Args:
            raycast_fn: function(ox, oy, angle_deg, max_dist) -> distance
            origin_x, origin_y: sensor position in world coords
            center_angle_deg: beam center direction in world degrees
            max_range: max sensor range in cm

        Returns:
            Simulated reading accounting for beam width.
        """
        p = self.profile
        n = p.beam_rays
        half = p.beam_half_angle_deg

        if n <= 1:
            # Single ray mode
            true_dist = raycast_fn(origin_x, origin_y, center_angle_deg, max_range)
            return self.measure(true_dist)

        # Cast multiple rays across the cone
        distances = []
        for i in range(n):
            # Spread rays evenly across the cone
            t = (i / (n - 1)) * 2 - 1 if n > 1 else 0  # -1 to +1
            angle = center_angle_deg + t * half
            dist = raycast_fn(origin_x, origin_y, angle, max_range)
            if dist < max_range:
                distances.append(dist)

        if not distances:
            return self.measure(max_range)

        if p.beam_use_minimum:
            # HC-SR04 tends to detect the closest object in its cone
            true_dist = min(distances)
        else:
            true_dist = sum(distances) / len(distances)

        return self.measure(true_dist)
