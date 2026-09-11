class GPS:
    """Webots GPS interface."""

    def __init__(self, gps_device):
        self.gps = gps_device

        self.initial_x = None
        self.initial_z = None

    def update(self):
        """Read the current GPS position."""

        position = self.gps.getValues()

        x = position[0]
        y = position[1]
        z = position[2]

        if self.initial_x is None:
            self.initial_x = x
            self.initial_z = z

        return {
            "x": x,
            "y": y,
            "z": z,
            "delta_x": x - self.initial_x,
            "delta_z": z - self.initial_z,
        }