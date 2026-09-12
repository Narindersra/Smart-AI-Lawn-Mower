class GPS:
    """Webots GPS interface."""

    def __init__(self, gps_device):
        self.gps = gps_device

        self.initial_x = None
        self.initial_z = None

    def update(self):
        """Read the current GPS position.

        Webots world coordinate system:
            position[0] = world X  (horizontal)
            position[1] = world Y  (horizontal)
            position[2] = world Z  (vertical / height)

        The navigation system uses (x, z) as the ground-plane
        pair.  We map world Y → "z" so that all downstream
        code receives the correct horizontal coordinate.
        """

        position = self.gps.getValues()

        x = position[0]
        z = position[1]   # world Y → navigation z

        if self.initial_x is None:
            self.initial_x = x
            self.initial_z = z

        return {
            "x": x,
            "y": position[2],   # world Z = height
            "z": z,             # world Y = second horizontal axis
            "delta_x": x - self.initial_x,
            "delta_z": z - self.initial_z,
        }