import math

from .navigation_types import Path, Waypoint


class CoveragePlanner:
    """
    Coverage planner for the Smart AI Lawn Mower.

    Project navigation coordinate convention:
        X-Z = ground plane

    Mowing direction:
        X axis

    Lane-to-lane direction:
        Z axis

    Coverage pattern:
        Boustrophedon / zig-zag

    Waypoint structure:

        Lane 1:
            WP0 = lane end

        Transition:
            WP1 = next lane start

        Lane 2:
            WP2 = lane end

        Transition:
            WP3 = next lane start

        Lane 3:
            WP4 = lane end

        ...

    Navigator interprets consecutive waypoints having:
        same X
        different Z

    as a lane transition.
    """

    def __init__(
        self,
        cutting_width=0.20,
        overlap=0.10,
        body_length=0.50,
        body_width=0.40,
    ):
        if cutting_width <= 0:
            raise ValueError(
                "cutting_width must be greater than zero."
            )

        if not 0.0 <= overlap < 1.0:
            raise ValueError(
                "overlap must be in the range [0, 1)."
            )

        if body_length <= 0:
            raise ValueError(
                "body_length must be greater than zero."
            )

        if body_width <= 0:
            raise ValueError(
                "body_width must be greater than zero."
            )

        self.cutting_width = float(cutting_width)
        self.overlap = float(overlap)

        self.body_length = float(body_length)
        self.body_width = float(body_width)

    # ============================================================
    # DERIVED PARAMETERS
    # ============================================================

    @property
    def lane_spacing(self):
        """
        Distance between adjacent mowing lanes.

            lane_spacing =
                cutting_width * (1 - overlap)

        Current configuration:

            0.20 * (1 - 0.10)
            = 0.18 m
        """

        return (
            self.cutting_width
            * (1.0 - self.overlap)
        )

    @property
    def turning_clearance(self):
        """
        Conservative clearance for an in-place rotation.

        The robot rotates about its navigation reference point.

        Body dimensions:

            length = 0.50 m
            width  = 0.40 m

        Half dimensions:

            0.25 m
            0.20 m

        Conservative rotational clearance:

            sqrt(
                0.25^2 +
                0.20^2
            )

        This is approximately:

            0.320 m
        """

        half_length = (
            self.body_length / 2.0
        )

        half_width = (
            self.body_width / 2.0
        )

        return math.hypot(
            half_length,
            half_width,
        )

    @property
    def turn_radius(self):
        """
        Retained for API compatibility.

        The navigation system no longer models the lane
        transition as a semicircular U-turn.

        The physical transition is:

            90 degree turn
            ->
            lane spacing
            ->
            90 degree turn

        Therefore no semicircular turn radius is added
        to the boundary clearance.
        """

        return 0.0

    @property
    def total_turn_clearance(self):
        """
        Clearance used for lane-end X coordinates.

        Since the transition uses two in-place 90 degree
        rotations instead of a semicircular U-turn:

            total_turn_clearance
                = turning_clearance
        """

        return self.turning_clearance

    # ============================================================
    # VALIDATION
    # ============================================================

    @staticmethod
    def _validate_bounds(
        min_x,
        max_x,
        min_z,
        max_z,
    ):
        if min_x >= max_x:
            raise ValueError(
                "min_x must be smaller than max_x."
            )

        if min_z >= max_z:
            raise ValueError(
                "min_z must be smaller than max_z."
            )

    # ============================================================
    # TURNING BOUNDS
    # ============================================================

    def _calculate_turning_bounds(
        self,
        min_x,
        max_x,
    ):
        """
        Calculate safe X positions for lane endpoints.

        The endpoint must leave enough space for the mower
        body during the in-place 90 degree rotations.

        No additional semicircular U-turn radius is added.
        """

        total_clearance = (
            self.total_turn_clearance
        )

        turn_min_x = (
            min_x
            + total_clearance
        )

        turn_max_x = (
            max_x
            - total_clearance
        )

        if turn_min_x >= turn_max_x:
            raise ValueError(
                "Coverage area is too narrow for the mower "
                "body clearance."
            )

        return (
            turn_min_x,
            turn_max_x,
        )

    # ============================================================
    # LOWER LANES
    # ============================================================

    def _generate_lower_lanes(
        self,
        start_z,
        min_z,
    ):
        """
        Generate lane positions below the starting lane.

        Lane spacing is exactly:

            cutting_width * (1 - overlap)
        """

        lanes = []

        spacing = self.lane_spacing

        z = start_z - spacing

        while z > min_z:
            lanes.append(float(z))
            z -= spacing

        # Include the lower boundary as the final lane if
        # it is not already represented.
        if (
            not lanes
            or abs(lanes[-1] - min_z) > 1e-9
        ):
            lanes.append(float(min_z))

        return lanes

    # ============================================================
    # UPPER LANES
    # ============================================================

    def _generate_upper_lanes(
        self,
        start_z,
        max_z,
    ):
        """
        Generate lane positions above the starting lane.

        Lane spacing is exactly:

            cutting_width * (1 - overlap)
        """

        lanes = []

        spacing = self.lane_spacing

        z = start_z + spacing

        while z < max_z:
            lanes.append(float(z))
            z += spacing

        # Include the upper boundary as the final lane if
        # it is not already represented.
        if (
            not lanes
            or abs(lanes[-1] - max_z) > 1e-9
        ):
            lanes.append(float(max_z))

        return lanes

    # ============================================================
    # COVERAGE PATH
    # ============================================================

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
        Generate the complete boustrophedon coverage path.

        The path is represented using lane endpoints.

        Example:

            Lane 1:
                START -----------------> WP0

            Transition:
                                      |
                                      | 0.18 m
                                      v
                                     WP1

            Lane 2:
                WP1 <------------------ WP2

            Transition:
                                      |
                                      | 0.18 m
                                      v
                                     WP3

            Lane 3:
                WP3 -----------------> WP4

        The Navigator is responsible for executing the physical
        lane transition:

            90 degree rotation
            ->
            one lane spacing
            ->
            90 degree rotation

        The planner only defines the geometric waypoint positions.
        """

        self._validate_bounds(
            min_x,
            max_x,
            min_z,
            max_z,
        )

        if not (
            min_z <= start_z <= max_z
        ):
            raise ValueError(
                "start_z is outside the coverage area."
            )

        # --------------------------------------------------------
        # Safe X positions for lane endpoints.
        # --------------------------------------------------------

        (
            turn_min_x,
            turn_max_x,
        ) = self._calculate_turning_bounds(
            min_x,
            max_x,
        )

        # --------------------------------------------------------
        # Generate all lane positions.
        # --------------------------------------------------------

        lower_lanes = (
            self._generate_lower_lanes(
                start_z,
                min_z,
            )
        )

        upper_lanes = (
            self._generate_upper_lanes(
                start_z,
                max_z,
            )
        )

        waypoints = []

        # ========================================================
        # FIRST LANE
        # ========================================================

        # Determine which X endpoint is closest to the current
        # robot position.

        distance_to_min = abs(
            start_x - turn_min_x
        )

        distance_to_max = abs(
            turn_max_x - start_x
        )

        if distance_to_min >= distance_to_max:
            first_end_x = turn_min_x
        else:
            first_end_x = turn_max_x

        # --------------------------------------------------------
        # First lane endpoint.
        #
        # This is WP0.
        # --------------------------------------------------------

        if abs(
            first_end_x - start_x
        ) > 1e-9:

            waypoints.append(
                Waypoint(
                    x=first_end_x,
                    z=start_z,
                )
            )

        current_x = first_end_x

        # ========================================================
        # LOWER (-Z) LANES
        # ========================================================

        for lane_z in lower_lanes:

            # ----------------------------------------------------
            # Next lane START.
            #
            # Same X as previous lane END.
            #
            # Example:
            #
            #     WP0 (-9.34, 0.26)
            #     WP1 (-9.34, 0.08)
            #
            # Navigator performs:
            #
            #     90 degree turn
            #     0.18 m lane shift
            #     90 degree turn
            # ----------------------------------------------------

            waypoints.append(
                Waypoint(
                    x=current_x,
                    z=lane_z,
                )
            )

            # ----------------------------------------------------
            # Alternate mowing direction.
            # ----------------------------------------------------

            if abs(
                current_x - turn_min_x
            ) <= 1e-9:

                next_end_x = turn_max_x

            else:

                next_end_x = turn_min_x

            # ----------------------------------------------------
            # Next lane END.
            # ----------------------------------------------------

            waypoints.append(
                Waypoint(
                    x=next_end_x,
                    z=lane_z,
                )
            )

            current_x = next_end_x

        # ========================================================
        # UPPER (+Z) LANES
        # ========================================================

        for lane_z in upper_lanes:

            # ----------------------------------------------------
            # Next lane START.
            # ----------------------------------------------------

            waypoints.append(
                Waypoint(
                    x=current_x,
                    z=lane_z,
                )
            )

            # ----------------------------------------------------
            # Alternate mowing direction.
            # ----------------------------------------------------

            if abs(
                current_x - turn_min_x
            ) <= 1e-9:

                next_end_x = turn_max_x

            else:

                next_end_x = turn_min_x

            # ----------------------------------------------------
            # Next lane END.
            # ----------------------------------------------------

            waypoints.append(
                Waypoint(
                    x=next_end_x,
                    z=lane_z,
                )
            )

            current_x = next_end_x

        return Path(
            waypoints=waypoints
        )

    # ============================================================
    # ACCESSORS
    # ============================================================

    def get_lane_spacing(self):
        return self.lane_spacing

    def get_turning_clearance(self):
        return self.turning_clearance

    def get_turn_radius(self):
        return self.turn_radius

    def get_total_turn_clearance(self):
        return self.total_turn_clearance

    # ============================================================
    # PATH VALIDATION
    # ============================================================

    def validate_path(
        self,
        path,
        min_x=None,
        max_x=None,
        min_z=None,
        max_z=None,
    ):
        """
        Validate generated waypoint coordinates.
        """

        if path is None:
            return False

        if not path.waypoints:
            return False

        for waypoint in path.waypoints:

            # ----------------------------------------------------
            # Numerical validity.
            # ----------------------------------------------------

            if not (
                math.isfinite(waypoint.x)
                and math.isfinite(waypoint.z)
            ):
                return False

            # ----------------------------------------------------
            # X bounds.
            # ----------------------------------------------------

            if (
                min_x is not None
                and waypoint.x < min_x
            ):
                return False

            if (
                max_x is not None
                and waypoint.x > max_x
            ):
                return False

            # ----------------------------------------------------
            # Z bounds.
            # ----------------------------------------------------

            if (
                min_z is not None
                and waypoint.z < min_z
            ):
                return False

            if (
                max_z is not None
                and waypoint.z > max_z
            ):
                return False

        return True