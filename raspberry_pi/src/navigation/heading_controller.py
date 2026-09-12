import math

from .navigation_types import (
    RobotPose,
    Waypoint,
    MotionCommand,
)


class HeadingController:

    def __init__(
        self,
        max_angular_speed=0.8,
        heading_tolerance=0.08,
        heading_kp=1.5,
    ):
        self.max_angular_speed = abs(
            max_angular_speed
        )

        self.heading_tolerance = abs(
            heading_tolerance
        )

        self.heading_kp = abs(
            heading_kp
        )

    @staticmethod
    def normalize_angle(angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    @staticmethod
    def calculate_target_heading(
        pose,
        waypoint,
    ):
        dx = waypoint.x - pose.x
        dz = waypoint.z - pose.z

        # Robot heading 0 points toward -X.
        #
        # Forward vector:
        #   Fx = -cos(theta)
        #   Fz =  sin(theta)
        #
        # Therefore target heading:
        #   theta = atan2(dz, -dx)

        return math.atan2(
            dz,
            -dx,
        )

    def calculate_heading_error(
        self,
        pose,
        waypoint,
    ):

        target_heading = (
            self.calculate_target_heading(
                pose,
                waypoint,
            )
        )

        return self.normalize_angle(
            target_heading
            - pose.heading
        )

    def is_aligned(
        self,
        pose,
        waypoint,
    ):

        error = (
            self.calculate_heading_error(
                pose,
                waypoint,
            )
        )

        return (
            abs(error)
            <= self.heading_tolerance
        )

    def calculate_angular_velocity(
        self,
        heading_error,
    ):

        error = self.normalize_angle(
            heading_error
        )

        if abs(error) <= self.heading_tolerance:
            return 0.0

        # Proportional angular control:
        #
        # omega = Kp * error

        angular_velocity = (
            self.heading_kp
            * error
        )

        angular_velocity = max(
            -self.max_angular_speed,
            min(
                self.max_angular_speed,
                angular_velocity,
            ),
        )

        return angular_velocity

    def update(
        self,
        pose,
        waypoint,
    ):

        error = (
            self.calculate_heading_error(
                pose,
                waypoint,
            )
        )

        angular_velocity = (
            self.calculate_angular_velocity(
                error
            )
        )

        if angular_velocity == 0.0:

            return MotionCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
            )

        return MotionCommand(
            linear_velocity=0.0,
            angular_velocity=angular_velocity,
        )