class IMU:
    """Webots IMU interface."""

    def __init__(self, imu_device):
        self.imu = imu_device

    def update(self):
        """Read current roll, pitch and yaw."""

        orientation = self.imu.getRollPitchYaw()

        return {
            "roll": orientation[0],
            "pitch": orientation[1],
            "yaw": orientation[2],
        }