class SpeedController:

    def __init__(
        self,
        max_speed=0.5,
        min_speed=0.10,
        slowdown_distance=1.0,
        position_tolerance=0.15,
    ):
        self.max_speed = abs(max_speed)
        self.min_speed = abs(min_speed)

        self.position_tolerance = abs(
            position_tolerance
        )

        self.slowdown_distance = max(
            abs(slowdown_distance),
            self.position_tolerance,
        )

    def calculate_speed(self, distance):

        distance = abs(distance)

        if distance <= self.position_tolerance:
            return 0.0

        if distance >= self.slowdown_distance:
            return self.max_speed

        usable_range = (
            self.slowdown_distance
            - self.position_tolerance
        )

        progress = (
            distance - self.position_tolerance
        ) / usable_range

        speed = (
            self.min_speed
            + progress
            * (
                self.max_speed
                - self.min_speed
            )
        )

        return min(
            self.max_speed,
            max(self.min_speed, speed),
        )

    def is_stopped(self, distance):
        return (
            abs(distance)
            <= self.position_tolerance
        )