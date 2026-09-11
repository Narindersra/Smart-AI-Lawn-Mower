import math


class PositionEstimator:
    """Estimate robot pose from localization sensor data."""

    def __init__(self):
        self.initial_yaw = None

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def update(self, gps_data, imu_data):
        current_yaw = imu_data["yaw"]

        if self.initial_yaw is None:
            self.initial_yaw = current_yaw

        heading = self.normalize_angle(
            current_yaw - self.initial_yaw
        )

        return {
            "x": gps_data["x"],
            "z": gps_data["z"],
            "heading": heading,
        }