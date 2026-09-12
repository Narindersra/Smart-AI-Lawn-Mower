from dataclasses import dataclass
from typing import List


@dataclass
class RobotPose:
    x: float
    z: float
    heading: float


@dataclass
class Waypoint:
    x: float
    z: float


@dataclass
class Path:
    waypoints: List[Waypoint]


@dataclass
class MotionCommand:
    linear_velocity: float
    angular_velocity: float