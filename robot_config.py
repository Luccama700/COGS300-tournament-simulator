"""
Robot configuration loader.

Reads a robot YAML file and produces a RobotConfig dataclass
with chassis geometry and sensor definitions. Swap the YAML file
to change the robot — no code changes needed.
"""

from dataclasses import dataclass, field
from pathlib import Path
import yaml


@dataclass
class SensorConfig:
    name: str
    mount_x: float      # normalized [-0.5, 0.5] relative to chassis center
    mount_y: float       # normalized [-0.5, 0.5] (negative = front)
    angle: float         # degrees, 0=forward, positive=clockwise
    range_cm: float      # max detection range
    sweep_deg: float     # half-cone width in degrees


@dataclass
class IRSensorConfig:
    name: str
    mount_x: float      # normalized [-0.5, 0.5] relative to chassis center
    mount_y: float      # normalized [-0.5, 0.5] (negative = front)
    threshold: int      # analog value 0-1023; above threshold = line detected


@dataclass
class ChassisConfig:
    length_cm: float
    width_cm: float
    wheelbase_cm: float


@dataclass
class RobotConfig:
    chassis: ChassisConfig
    drive_type: str
    sensors: list[SensorConfig] = field(default_factory=list)
    ir_sensors: list[IRSensorConfig] = field(default_factory=list)

    @property
    def half_length(self) -> float:
        return self.chassis.length_cm / 2

    @property
    def half_width(self) -> float:
        return self.chassis.width_cm / 2

    def sensor_world_position(self, sensor: SensorConfig, robot_x: float, robot_y: float, robot_heading: float) -> tuple[float, float]:
        """
        Get sensor mount position in world coordinates given robot pose.
        heading is in degrees, 0 = east, increases counter-clockwise.

        Config mount convention:
          mount_x: -0.5=left, +0.5=right (relative to chassis width)
          mount_y: -0.5=front, +0.5=rear (relative to chassis length)

        Robot-local frame: forward=heading direction, right=90° clockwise from heading.
        """
        import math
        # Convert mount to robot-local cm
        # local_fwd = distance forward from center (negative mount_y = front = positive forward)
        # local_right = distance right from center (positive mount_x = right)
        local_fwd = -sensor.mount_y * self.chassis.length_cm
        local_right = sensor.mount_x * self.chassis.width_cm

        # Rotate to world frame
        # Forward direction = heading angle
        # Right direction = heading - 90°
        rad = math.radians(robot_heading)
        cos_h, sin_h = math.cos(rad), math.sin(rad)

        # Forward component: (cos_h, -sin_h) in world (x, y_screen)
        # Right component: (sin_h, cos_h) in world — this is heading rotated -90°
        #   cos(h-90) = sin(h), -sin(h-90) = cos(h)
        world_x = robot_x + local_fwd * cos_h + local_right * sin_h
        world_y = robot_y - local_fwd * sin_h + local_right * cos_h
        return world_x, world_y

    def ir_sensor_world_position(self, ir_sensor: IRSensorConfig, robot_x: float, robot_y: float, robot_heading: float) -> tuple[float, float]:
        """Get IR sensor mount position in world coordinates. Same math as sensor_world_position."""
        import math
        local_fwd = -ir_sensor.mount_y * self.chassis.length_cm
        local_right = ir_sensor.mount_x * self.chassis.width_cm
        rad = math.radians(robot_heading)
        cos_h, sin_h = math.cos(rad), math.sin(rad)
        world_x = robot_x + local_fwd * cos_h + local_right * sin_h
        world_y = robot_y - local_fwd * sin_h + local_right * cos_h
        return world_x, world_y

    def sensor_world_angle(self, sensor: SensorConfig, robot_heading: float) -> float:
        """Get sensor beam direction in world degrees.
        Config convention: 0=forward, positive=clockwise(right), negative=CCW(left).
        World convention: 0=east, positive=CCW.
        Clockwise in config = negative in world, so subtract.
        """
        return robot_heading - sensor.angle


def load_robot_config(path: str | Path) -> RobotConfig:
    """Load robot config from a YAML file."""
    with open(path) as f:
        raw = yaml.safe_load(f)

    r = raw["robot"]
    ch = r["chassis"]
    chassis = ChassisConfig(
        length_cm=ch["length_cm"],
        width_cm=ch["width_cm"],
        wheelbase_cm=ch["wheelbase_cm"],
    )

    sensors = []
    for s in r.get("sensors", []):
        sensors.append(SensorConfig(
            name=s["name"],
            mount_x=s["mount"][0],
            mount_y=s["mount"][1],
            angle=s["angle"],
            range_cm=s["range"],
            sweep_deg=s["sweep"],
        ))

    ir_sensors = []
    for s in r.get("ir_sensors", []):
        ir_sensors.append(IRSensorConfig(
            name=s["name"],
            mount_x=s["mount"][0],
            mount_y=s["mount"][1],
            threshold=s.get("threshold", 500),
        ))

    return RobotConfig(
        chassis=chassis,
        drive_type=r["drive_type"],
        sensors=sensors,
        ir_sensors=ir_sensors,
    )
