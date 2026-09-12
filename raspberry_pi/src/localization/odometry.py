import math


class Odometry:
    """Differential-drive wheel odometry."""

    def __init__(
        self,
        left_encoder,
        right_encoder,
        wheel_radius=0.08,
        wheel_track=0.44,
    ):
        self.left_encoder = left_encoder
        self.right_encoder = right_encoder

        self.wheel_radius = wheel_radius
        self.wheel_track = wheel_track

        self.initial_left_encoder = None
        self.initial_right_encoder = None

        self.previous_left_distance = 0.0
        self.previous_right_distance = 0.0

        self.x = 0.0
        self.z = 0.0
        self.heading = 0.0

    def initialize(self):
        """Capture initial encoder positions."""

        self.initial_left_encoder = self.left_encoder.getValue()
        self.initial_right_encoder = self.right_encoder.getValue()

    def update(self):
        """Update odometry from wheel encoder positions."""

        left_position = self.left_encoder.getValue()
        right_position = self.right_encoder.getValue()

        left_distance = (
            left_position - self.initial_left_encoder
        ) * self.wheel_radius

        right_distance = (
            right_position - self.initial_right_encoder
        ) * self.wheel_radius

        left_distance_delta = (
            left_distance - self.previous_left_distance
        )

        right_distance_delta = (
            right_distance - self.previous_right_distance
        )

        heading_delta = (
            right_distance_delta - left_distance_delta
        ) / self.wheel_track

        distance_delta = (
            left_distance_delta + right_distance_delta
        ) / 2.0

        self.heading += heading_delta

        self.x -= distance_delta * math.cos(self.heading)
        self.z -= distance_delta * math.sin(self.heading)

        self.previous_left_distance = left_distance
        self.previous_right_distance = right_distance

        return {
            "x": self.x,
            "z": self.z,
            "heading": self.heading,
            "left_encoder": left_position,
            "right_encoder": right_position,
            "left_distance": left_distance,
            "right_distance": right_distance,
            "average_distance": (
                left_distance + right_distance
            ) / 2.0,
        }