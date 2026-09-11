class LocalizationManager:
    """Coordinates all localization components."""

    def __init__(self, gps, imu, odometry, position_estimator):
        self.gps = gps
        self.imu = imu
        self.odometry = odometry
        self.position_estimator = position_estimator

    def initialize(self):
        self.odometry.initialize()

    def update(self):
        gps_data = self.gps.update()
        imu_data = self.imu.update()
        odometry_data = self.odometry.update()

        estimated_pose = self.position_estimator.update(
            gps_data,
            imu_data
        )

        return {
            "gps": gps_data,
            "imu": imu_data,
            "odometry": odometry_data,
            "pose": estimated_pose,
        }