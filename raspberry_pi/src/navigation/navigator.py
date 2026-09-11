import math

from .navigation_types import RobotPose, Waypoint, MotionCommand


class Navigator:
    """Basic waypoint navigation for a differential-drive robot."""

    def __init__(
        self,
        max_velocity=2.0,
        max_turn_velocity=1.0,
        position_tolerance=0.15,
        heading_tolerance=0.10,
    ):
        self.max_velocity = max_velocity
        self.max_turn_velocity = max_turn_velocity
        self.position_tolerance = position_tolerance
        self.heading_tolerance = heading_tolerance

        self.target = None
        self.finished = False

    def set_waypoint(self, waypoint):
        """Set a new navigation target."""

        self.target = waypoint
        self.finished = False

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def update(self, pose):
        """
        Calculate wheel velocities required to reach the waypoint.
        """

        if self.target is None:
            return MotionCommand(0.0, 0.0)

        if self.finished:
            return MotionCommand(0.0, 0.0)

        dx = self.target.x - pose.x
        dz = self.target.z - pose.z

        distance = math.sqrt(dx * dx + dz * dz)

        # ----------------------------------------------------
        # TARGET REACHED
        # ----------------------------------------------------

        if distance <= self.position_tolerance:
            self.finished = True
            return MotionCommand(0.0, 0.0)

        # ----------------------------------------------------
        # DESIRED HEADING
        # ----------------------------------------------------

        desired_heading = math.atan2(dz, dx)

        heading_error = self.normalize_angle(
            desired_heading - pose.heading
        )

        # ----------------------------------------------------
        # TURN IN PLACE IF HEADING ERROR IS LARGE
        # ----------------------------------------------------

        if abs(heading_error) > self.heading_tolerance:

            turn = self.max_turn_velocity

            if heading_error > 0:
                return MotionCommand(
                    -turn,
                    turn
                )

            return MotionCommand(
                turn,
                -turn
            )

        # ----------------------------------------------------
        # FORWARD MOTION
        # ----------------------------------------------------

        velocity = self.max_velocity

        # Slow down when approaching target.
        if distance < 0.5:
            velocity = self.max_velocity * (distance / 0.5)

        # Keep velocity within safe limits.
        velocity = max(0.0, min(velocity, self.max_velocity))

        return MotionCommand(
            velocity,
            velocity
        )

    def is_finished(self):
        """Return True when the current waypoint has been reached."""

        return self.finished
