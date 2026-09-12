class Geofence:

    def __init__(
        self,
        min_x,
        max_x,
        min_z,
        max_z,
        safety_margin=0.25,
    ):
        if min_x >= max_x:
            raise ValueError(
                "min_x must be smaller than max_x."
            )

        if min_z >= max_z:
            raise ValueError(
                "min_z must be smaller than max_z."
            )

        if safety_margin < 0:
            raise ValueError(
                "safety_margin cannot be negative."
            )

        self.min_x = min_x
        self.max_x = max_x
        self.min_z = min_z
        self.max_z = max_z
        self.safety_margin = safety_margin

    def contains(self, x, z):
        return (
            self.min_x <= x <= self.max_x
            and
            self.min_z <= z <= self.max_z
        )

    def contains_safe(self, x, z):
        return (
            self.min_x + self.safety_margin
            <= x
            <=
            self.max_x - self.safety_margin
            and
            self.min_z + self.safety_margin
            <= z
            <=
            self.max_z - self.safety_margin
        )

    def get_safe_bounds(self):
        return (
            self.min_x + self.safety_margin,
            self.max_x - self.safety_margin,
            self.min_z + self.safety_margin,
            self.max_z - self.safety_margin,
        )

    def is_path_inside(
        self,
        path_points,
        safe=False,
    ):
        checker = (
            self.contains_safe
            if safe
            else self.contains
        )

        return all(
            checker(x, z)
            for x, z in path_points
        )