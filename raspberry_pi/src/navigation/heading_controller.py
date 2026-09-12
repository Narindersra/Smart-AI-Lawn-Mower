import math

from .navigation_types import (
    RobotPose,
    Waypoint,
    MotionCommand,
)


class HeadingController:
    """
    Controls the robot heading toward the current waypoint.

    Project coordinate convention:

        Front = -X
        Rear  = +X
        Left  = +Y
        Right = -Y
        Up    = +Z

    Navigation ground plane:

        X-Z

    The Webots IMU yaw convention and the project's
    differential-drive angular convention require the
    angular control sign to be inverted.
    """

    def __init__(
        self,
        max_angular_speed=0.8,
        heading_tolerance=0.08,
        heading_kp=1.5,
    ):
        if max_angular_speed <= 0:
            raise ValueError(
                "max_angular_speed must be greater than zero."
            )

        if heading_tolerance < 0:
            raise ValueError(
                "heading_tolerance must not be negative."
            )

        if heading_kp <= 0:
            raise ValueError(
                "heading_kp must be greater than zero."
            )

        self.max_angular_speed = float(
            abs(max_angular_speed)
        )

        self.heading_tolerance = float(
            abs(heading_tolerance)
        )

        self.heading_kp = float(
            abs(heading_kp)
        )

    # ============================================================
    # ANGLE NORMALIZATION
    # ============================================================

    @staticmethod
    def normalize_angle(angle):
        """
        Normalize an angle to the range:

            [-pi, +pi]
        """

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # ============================================================
    # TARGET HEADING
    # ============================================================

    @staticmethod
    def calculate_target_heading(
        pose,
        waypoint,
    ):
        """
        Calculate the heading required to point the robot
        directly toward the waypoint.

        Project convention:

            heading = 0
            -> robot faces -X

        Therefore the forward direction is:

            fx = -cos(heading)
            fy = +sin(heading)

        Navigation z maps to world Y, so:

            dx = waypoint.x - pose.x
            dz = waypoint.z - pose.z   (= world dy)

        The required heading is:

            atan2(dz, -dx)
        """

        dx = waypoint.x - pose.x
        dz = waypoint.z - pose.z

        return math.atan2(
            dz,
            -dx,
        )

    # ============================================================
    # HEADING ERROR
    # ============================================================

    def calculate_heading_error(
        self,
        pose,
        waypoint,
    ):
        """
        Calculate the shortest signed angular error between
        the robot's current heading and the target heading.
        """

        target_heading = (
            self.calculate_target_heading(
                pose,
                waypoint,
            )
        )

        error = (
            target_heading
            - pose.heading
        )

        return self.normalize_angle(error)

    # ============================================================
    # ALIGNMENT
    # ============================================================

    def is_aligned(
        self,
        pose,
        waypoint,
    ):
        """
        Return True when the robot is sufficiently aligned
        with the waypoint.
        """

        error = self.calculate_heading_error(
            pose,
            waypoint,
        )

        return (
            abs(error)
            <= self.heading_tolerance
        )

    # ============================================================
    # ANGULAR VELOCITY
    # ============================================================

    def calculate_angular_velocity(
        self,
        heading_error,
    ):
        """
        Convert heading error into angular velocity.

        IMPORTANT:

        The Webots IMU yaw direction and the project's
        differential-drive angular convention require the
        control sign to be inverted.

            angular_velocity =
                -Kp * heading_error

        The result is limited to the configured maximum.
        """

        error = self.normalize_angle(
            heading_error
        )

        # Small deadband to prevent chatter while allowing fine tracking
        if abs(error) <= 0.001:
            return 0.0

        # Proportional angular control:
        angular_velocity = (
            -self.heading_kp * error
        )

        # Saturation.
        angular_velocity = max(
            -self.max_angular_speed,
            min(
                self.max_angular_speed,
                angular_velocity,
            ),
        )

        return angular_velocity

    # ============================================================
    # UPDATE
    # ============================================================

    def update(
        self,
        pose,
        waypoint,
    ):
        """
        Generate a rotation command that aligns the robot
        with the current waypoint.

        Linear velocity remains zero while aligning.
        """

        heading_error = (
            self.calculate_heading_error(
                pose,
                waypoint,
            )
        )

        angular_velocity = (
            self.calculate_angular_velocity(
                heading_error
            )
        )

        # No rotation required.
        if angular_velocity == 0.0:
            return MotionCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
            )

        # Rotate in place.
        return MotionCommand(
            linear_velocity=0.0,
            angular_velocity=angular_velocity,
        )