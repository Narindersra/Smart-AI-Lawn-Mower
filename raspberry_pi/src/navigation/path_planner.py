import math

from .navigation_types import Path, Waypoint


class PathPlanner:

    def __init__(
        self,
        waypoint_spacing=0.50,
    ):
        if waypoint_spacing <= 0:
            raise ValueError(
                "waypoint_spacing must be greater than zero."
            )

        self.waypoint_spacing = (
            waypoint_spacing
        )

    def create_straight_path(
        self,
        start_x,
        start_z,
        goal_x,
        goal_z,
    ):
        dx = goal_x - start_x
        dz = goal_z - start_z

        distance = math.hypot(
            dx,
            dz,
        )

        if distance <= 0.0:
            return Path(
                waypoints=[]
            )

        # Number of path segments required so that
        # no segment is longer than waypoint_spacing.
        segments = max(
            1,
            math.ceil(
                distance
                / self.waypoint_spacing
            ),
        )

        waypoints = []

        # Do not add the current robot position.
        # Only generate future points.
        for index in range(
            1,
            segments + 1,
        ):
            ratio = (
                index
                / segments
            )

            waypoint_x = (
                start_x
                + dx * ratio
            )

            waypoint_z = (
                start_z
                + dz * ratio
            )

            waypoints.append(
                Waypoint(
                    x=waypoint_x,
                    z=waypoint_z,
                )
            )

        return Path(
            waypoints=waypoints
        )

    def validate_path(
        self,
        path,
    ):
        if path is None:
            return False

        if not path.waypoints:
            return False

        for waypoint in path.waypoints:

            if not (
                math.isfinite(
                    waypoint.x
                )
                and
                math.isfinite(
                    waypoint.z
                )
            ):
                return False

        return True