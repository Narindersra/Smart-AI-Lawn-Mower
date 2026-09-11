from dataclasses import dataclass


@dataclass
class RobotPose:
    """Current robot pose."""

    x: float
    z: float
    heading: float


@dataclass
class Waypoint:
    """Navigation target waypoint."""

    x: float
    z: float


@dataclass
class MotionCommand:
    """Differential-drive wheel velocity command."""

    left_velocity: float
    right_velocity: float