class DifferentialDriveController:

    def __init__(
        self,
        wheel_radius=0.10,
        wheel_track=0.44,
        max_wheel_velocity=10.0,
    ):
        if wheel_radius <= 0:
            raise ValueError(
                "wheel_radius must be greater than zero."
            )

        if wheel_track <= 0:
            raise ValueError(
                "wheel_track must be greater than zero."
            )

        if max_wheel_velocity <= 0:
            raise ValueError(
                "max_wheel_velocity must be greater than zero."
            )

        self.wheel_radius = wheel_radius
        self.wheel_track = wheel_track
        self.max_wheel_velocity = max_wheel_velocity

    def calculate_wheel_velocities(
        self,
        linear_velocity,
        angular_velocity,
    ):
        half_track = self.wheel_track / 2.0

        # Standard differential-drive kinematics.
        left_linear = (
            linear_velocity
            - angular_velocity * half_track
        )

        right_linear = (
            linear_velocity
            + angular_velocity * half_track
        )

        left = (
            -left_linear
            / self.wheel_radius
        )

        right = (
            -right_linear
            / self.wheel_radius
        )

        left = max(
            -self.max_wheel_velocity,
            min(self.max_wheel_velocity, left),
        )

        right = max(
            -self.max_wheel_velocity,
            min(self.max_wheel_velocity, right),
        )

        return left, right

    def forward(self, linear_velocity):
        return self.calculate_wheel_velocities(
            linear_velocity,
            0.0,
        )

    def rotate(self, angular_velocity):
        return self.calculate_wheel_velocities(
            0.0,
            angular_velocity,
        )

    @staticmethod
    def stop():
        return 0.0, 0.0