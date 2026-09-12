import math

from .navigation_types import Path, Waypoint


class CoveragePlanner:
    def __init__(
        self,
        cutting_width=0.20,
        overlap=0.10,
        turn_clearance=0.0,
    ):
        if cutting_width <= 0:
            raise ValueError("cutting_width must be greater than zero.")

        if not 0.0 <= overlap < 1.0:
            raise ValueError("overlap must be in the range [0, 1).")

        if turn_clearance < 0:
            raise ValueError("turn_clearance must not be negative.")

        self.cutting_width = cutting_width
        self.overlap = overlap
        self.turn_clearance = turn_clearance

    @property
    def lane_spacing(self):
        return self.cutting_width * (1.0 - self.overlap)

    def _calculate_lane_x_bounds(self, min_x, max_x):
        """
        Keep lane endpoints inside the geofence so the robot has
        sufficient clearance for turning.
        """
        if min_x >= max_x:
            raise ValueError("Invalid X bounds.")

        return (
            min_x + self.turn_clearance,
            max_x - self.turn_clearance,
        )

    def _generate_lane_positions(self, min_z, max_z, start_z):
        """
        Generate lane center positions beginning from the robot's
        starting lane and progressing across the safe Z range.
        """

        spacing = self.lane_spacing

        if min_z >= max_z:
            raise ValueError("Invalid Z bounds.")

        if not min_z <= start_z <= max_z:
            raise ValueError("start_z is outside the safe coverage area.")

        lanes = [start_z]

        # Generate lanes toward +Z.
        z = start_z + spacing

        while z <= max_z + 1e-9:
            lanes.append(min(z, max_z))
            z += spacing

        # Generate lanes toward -Z.
        z = start_z - spacing
        lower_lanes = []

        while z >= min_z - 1e-9:
            lower_lanes.append(max(z, min_z))
            z -= spacing

        # Put lanes below the starting lane first.
        lower_lanes.reverse()

        return lower_lanes + lanes

    def create_coverage_path(
        self,
        start_x,
        start_z,
        min_x,
        max_x,
        min_z,
        max_z,
    ):
        """
        Generate a complete boustrophedon coverage path.

        Coordinate convention:
            X = front/rear ground direction
            Z = lateral ground direction

        The path alternates direction on every lane.
        """

        lane_min_x, lane_max_x = self._calculate_lane_x_bounds(
            min_x,
            max_x,
        )

        lane_positions = self._generate_lane_positions(
            min_z,
            max_z,
            start_z,
        )

        waypoints = []

        # Determine which side of the lawn the robot starts from.
        start_from_min_x = abs(start_x - lane_min_x) <= abs(
            start_x - lane_max_x
        )

        for lane_index, lane_z in enumerate(lane_positions):

            if lane_index % 2 == 0:
                if start_from_min_x:
                    lane_start_x = lane_min_x
                    lane_end_x = lane_max_x
                else:
                    lane_start_x = lane_max_x
                    lane_end_x = lane_min_x
            else:
                if start_from_min_x:
                    lane_start_x = lane_max_x
                    lane_end_x = lane_min_x
                else:
                    lane_start_x = lane_min_x
                    lane_end_x = lane_max_x

            # First lane: only add the endpoint.
            if lane_index == 0:
                if math.hypot(
                    lane_end_x - start_x,
                    lane_z - start_z,
                ) > 1e-9:
                    waypoints.append(
                        Waypoint(
                            x=lane_end_x,
                            z=lane_z,
                        )
                    )
                continue

            # Turn waypoint:
            # move laterally to the next lane while remaining
            # at the current X endpoint.
            previous_waypoint = waypoints[-1]

            waypoints.append(
                Waypoint(
                    x=previous_waypoint.x,
                    z=lane_z,
                )
            )

            # Drive the complete next lane.
            waypoints.append(
                Waypoint(
                    x=lane_end_x,
                    z=lane_z,
                )
            )

        return Path(waypoints=waypoints)

    def get_lane_spacing(self):
        return self.lane_spacing

    def validate_path(self, path):
        if path is None or not path.waypoints:
            return False

        for waypoint in path.waypoints:
            if not (
                math.isfinite(waypoint.x)
                and math.isfinite(waypoint.z)
            ):
                return False

        return True