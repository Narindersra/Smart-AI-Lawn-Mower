import math

from .navigation_types import (
    RobotPose,
    Waypoint,
    Path,
    MotionCommand,
)

from .heading_controller import HeadingController
from .speed_controller import SpeedController
from .navigation_state_machine import (
    NavigationState,
    NavigationStateMachine,
)


class Navigator:
    """
    High-level navigation controller.

    Normal waypoint:
        ALIGNING -> DRIVING

    Coverage lane transition:

        WP0
          |
          | 90 degree turn
          v
        WP1
          |
          | STOP
          | 90 degree turn
          v
        WP2
          |
          | straight driving
          v
        WP3

    Lane transitions are NOT curved U-turns.

    The mower performs:

        1. Stop at lane-end waypoint.
        2. Rotate exactly 90 degrees in place.
        3. Drive one lane spacing to the next waypoint.
        4. Stop at the next lane waypoint.
        5. Rotate another 90 degrees in place.
        6. Continue straight on the next mowing lane.
    """

    def __init__(
        self,
        differential_drive,
        heading_controller=None,
        speed_controller=None,
        position_tolerance=0.15,
    ):

        self.differential_drive = differential_drive

        self.heading_controller = (
            heading_controller
            if heading_controller is not None
            else HeadingController()
        )

        self.speed_controller = (
            speed_controller
            if speed_controller is not None
            else SpeedController(
                position_tolerance=position_tolerance
            )
        )

        self.position_tolerance = abs(
            position_tolerance
        )

        self.state_machine = NavigationStateMachine()

        self.path = None
        self.current_waypoint_index = 0

        # ========================================================
        # LANE TRANSITION CONTROL
        # ========================================================

        # Possible phases:
        #
        # None
        # FIRST_TURN
        # LANE_SHIFT
        # STOP_AT_LANE_START
        # SECOND_TURN
        #
        self.lane_transition_phase = None

        self.turn_target_heading = None

        self.lane_heading = None
        self.shift_heading = None
        self.next_lane_heading = None
        self.first_turn_delta = 0.0
        self.second_turn_delta = 0.0

        self.start_waypoint = None

    # ============================================================
    # PATH
    # ============================================================

    def set_path(self, path, start_pose=None):

        self.path = path

        self.current_waypoint_index = 0

        self.lane_transition_phase = None
        self.turn_target_heading = None

        self.lane_heading = None
        self.shift_heading = None
        self.next_lane_heading = None
        self.first_turn_delta = 0.0
        self.second_turn_delta = 0.0

        if start_pose is not None:
            self.start_waypoint = Waypoint(
                x=start_pose.x,
                z=start_pose.z,
            )

    def start(self, start_pose=None):

        if (
            self.path is None
            or not self.path.waypoints
        ):

            self.state_machine.path_complete()
            return

        self.current_waypoint_index = 0

        self.lane_transition_phase = None
        self.turn_target_heading = None

        self.lane_heading = None
        self.shift_heading = None
        self.next_lane_heading = None
        self.first_turn_delta = 0.0
        self.second_turn_delta = 0.0

        if start_pose is not None:
            self.start_waypoint = Waypoint(
                x=start_pose.x,
                z=start_pose.z,
            )

        self.state_machine.start()

    # ============================================================
    # STATE
    # ============================================================

    def get_state(self):

        return self.state_machine.get_state()

    def is_complete(self):

        return (
            self.state_machine.get_state()
            == NavigationState.PATH_COMPLETE
        )

    # ============================================================
    # WAYPOINT
    # ============================================================

    def get_current_waypoint_index(self):

        return self.current_waypoint_index

    def get_current_waypoint(self):

        if self.path is None:
            return None

        if (
            self.current_waypoint_index
            >= len(self.path.waypoints)
        ):
            return None

        return self.path.waypoints[
            self.current_waypoint_index
        ]

    # ============================================================
    # DISTANCE
    # ============================================================

    @staticmethod
    def calculate_distance(
        pose,
        waypoint,
    ):

        dx = waypoint.x - pose.x
        dz = waypoint.z - pose.z

        return math.hypot(
            dx,
            dz,
        )

    # ============================================================
    # LANE TRANSITION DETECTION
    # ============================================================

    def _is_lane_transition(self):

        if self.path is None:
            return False

        current = self.get_current_waypoint()

        if current is None:
            return False

        next_index = (
            self.current_waypoint_index + 1
        )

        if next_index >= len(
            self.path.waypoints
        ):
            return False

        next_waypoint = (
            self.path.waypoints[next_index]
        )

        dx = abs(
            next_waypoint.x - current.x
        )

        dz = abs(
            next_waypoint.z - current.z
        )

        # Same X and different Z means:
        #
        # current waypoint = lane end
        # next waypoint    = next lane start
        #
        return (
            dx <= 1e-6
            and dz > 1e-6
        )

    # ============================================================
    # ADVANCE
    # ============================================================

    def _advance_waypoint(self):

        self.current_waypoint_index += 1

        if (
            self.path is None
            or self.current_waypoint_index
            >= len(self.path.waypoints)
        ):

            self.state_machine.path_complete()

            return False

        self.state_machine.start()

        return True

    # ============================================================
    # TARGET HEADING
    # ============================================================

    @staticmethod
    def _target_heading(
        pose,
        waypoint,
    ):

        dx = waypoint.x - pose.x
        dz = waypoint.z - pose.z

        return math.atan2(
            dz,
            -dx,
        )

    # ============================================================
    # PATH-BASED VECTOR HEADING CALCULATION
    # ============================================================

    @staticmethod
    def _vector_heading(p_from, p_to):
        """
        Calculate heading of planned path segment:
            f_world = (-cos theta, +sin theta)
            dx = p_to.x - p_from.x
            dz = p_to.z - p_from.z  (= world dy)
            theta = atan2(dz, -dx)
        """
        dx = p_to.x - p_from.x
        dz = p_to.z - p_from.z

        return math.atan2(
            dz,
            -dx,
        )

    def _get_transition_points(self):
        """
        Retrieve planned waypoints for lane transition:
            A: previous lane point (or start_waypoint for WP0)
            B: current lane endpoint
            C: intermediate lane-shift waypoint
            D: next lane mowing endpoint
        """
        if self.path is None or not self.path.waypoints:
            return None, None, None, None

        current_index = self.current_waypoint_index
        if current_index >= len(self.path.waypoints):
            return None, None, None, None

        B = self.path.waypoints[current_index]

        if current_index > 0:
            A = self.path.waypoints[current_index - 1]
        else:
            A = self.start_waypoint

        if current_index + 1 < len(self.path.waypoints):
            C = self.path.waypoints[current_index + 1]
        else:
            C = None

        if current_index + 2 < len(self.path.waypoints):
            D = self.path.waypoints[current_index + 2]
        else:
            D = None

        return A, B, C, D

    def _calculate_lane_heading(
        self,
        pose,
    ):
        """
        Determine the planned heading of the lane that has just been
        completed (from A to B).
        """
        A, B, _, _ = self._get_transition_points()

        if A is not None and B is not None:
            return self._vector_heading(A, B)

        return pose.heading

    def get_target_heading(self):
        """
        Return the currently active target heading (planned segment or turn target).
        """
        if self.turn_target_heading is not None:
            return self.turn_target_heading
        if self.lane_transition_phase == "LANE_SHIFT" and self.shift_heading is not None:
            return self.shift_heading
        A, B, _, _ = self._get_transition_points()
        if A is not None and B is not None:
            return self._vector_heading(A, B)
        return None

    # ============================================================
    # START LANE TRANSITION
    # ============================================================

    def _start_lane_turn(
        self,
        pose,
        waypoint,
    ):
        """
        Start the FIRST 90-degree turn at a lane-end waypoint (B).

        The target heading is calculated from the planned path segments:
            D_current = B - A
            D_shift   = C - B
            theta_shift = atan2(+D_shift.z, -D_shift.x)
            Delta_theta_1 = normalize_angle(theta_shift - theta_current)

        This is strictly derived from the planned path segments and does NOT
        use the instantaneous robot GPS position to avoid diagonal error.
        """
        if not self._is_lane_transition():
            return

        A, B, C, D = self._get_transition_points()

        if B is None or C is None:
            return

        if A is not None:
            self.lane_heading = self._vector_heading(A, B)
        else:
            self.lane_heading = pose.heading

        self.shift_heading = self._vector_heading(B, C)

        # Expected rotation magnitude |Δθ1| ≈ π/2
        self.first_turn_delta = (
            self.heading_controller.normalize_angle(
                self.shift_heading - self.lane_heading
            )
        )

        self.turn_target_heading = self.shift_heading

        self.lane_transition_phase = (
            "FIRST_TURN"
        )

        self.state_machine.start_turning()

    # ============================================================
    # START SECOND TURN
    # ============================================================

    def _start_second_lane_turn(self):
        """
        Start the SECOND 90-degree turn at intermediate waypoint C.

        The target heading is calculated from the next planned mowing lane:
            D_next = D - C
            theta_next = atan2(+D_next.z, -D_next.x)
            Delta_theta_2 = normalize_angle(theta_next - theta_shift)
        """
        current_index = self.current_waypoint_index

        if (
            self.path is None
            or current_index >= len(self.path.waypoints)
        ):
            return

        C = self.path.waypoints[current_index]

        if current_index + 1 < len(self.path.waypoints):
            D = self.path.waypoints[current_index + 1]
            self.next_lane_heading = self._vector_heading(C, D)
        else:
            self.next_lane_heading = (
                self.heading_controller.normalize_angle(
                    self.lane_heading + math.pi
                )
            )

        if self.shift_heading is not None:
            self.second_turn_delta = (
                self.heading_controller.normalize_angle(
                    self.next_lane_heading - self.shift_heading
                )
            )
        else:
            self.second_turn_delta = math.pi / 2.0

        self.turn_target_heading = self.next_lane_heading

        self.lane_transition_phase = (
            "SECOND_TURN"
        )

        self.state_machine.start_turning()

    # ============================================================
    # UPDATE TURN
    # ============================================================

    def _update_turn(
        self,
        pose,
    ):
        """
        Execute a PURE in-place rotation.

        Linear velocity is ALWAYS zero during turning.
        """

        if self.turn_target_heading is None:

            return MotionCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
            )

        error = (
            self.heading_controller.normalize_angle(
                self.turn_target_heading
                - pose.heading
            )
        )

        # --------------------------------------------------------
        # TURN COMPLETE
        # --------------------------------------------------------

        if abs(error) <= (
            self.heading_controller.heading_tolerance
        ):

            completed_phase = (
                self.lane_transition_phase
            )

            # ====================================================
            # FIRST TURN COMPLETE
            # ====================================================

            if completed_phase == "FIRST_TURN":

                self.turn_target_heading = None

                # Move from WP0 -> WP1.
                #
                # WP1 is now the active waypoint.
                #
                if not self._advance_waypoint():

                    return MotionCommand(
                        linear_velocity=0.0,
                        angular_velocity=0.0,
                    )

                self.lane_transition_phase = (
                    "LANE_SHIFT"
                )

                self.state_machine.start_driving()

                # One cycle of zero command after rotation.
                return MotionCommand(
                    linear_velocity=0.0,
                    angular_velocity=0.0,
                )

            # ====================================================
            # SECOND TURN COMPLETE
            # ====================================================

            if completed_phase == "SECOND_TURN":

                self.turn_target_heading = None

                self.lane_transition_phase = None

                self.lane_heading = None
                self.shift_heading = None
                self.next_lane_heading = None

                # Move from WP1 -> WP2.
                if not self._advance_waypoint():

                    return MotionCommand(
                        linear_velocity=0.0,
                        angular_velocity=0.0,
                    )

                self.state_machine.start()

                # One zero-command cycle before driving.
                return MotionCommand(
                    linear_velocity=0.0,
                    angular_velocity=0.0,
                )

        # --------------------------------------------------------
        # PURE ROTATION
        # --------------------------------------------------------

        angular_velocity = (
            self.heading_controller
            .calculate_angular_velocity(
                error
            )
        )

        return MotionCommand(
            linear_velocity=0.0,
            angular_velocity=angular_velocity,
        )

    # ============================================================
    # MAIN UPDATE
    # ============================================================

    def update(
        self,
        pose,
    ):

        if self.start_waypoint is None:
            self.start_waypoint = Waypoint(
                x=pose.x,
                z=pose.z,
            )

        # --------------------------------------------------------
        # No path
        # --------------------------------------------------------

        if self.path is None:

            self.state_machine.set_error()

            return MotionCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
            )

        # --------------------------------------------------------
        # Complete
        # --------------------------------------------------------

        if self.is_complete():

            return MotionCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
            )

        # --------------------------------------------------------
        # Current waypoint
        # --------------------------------------------------------

        waypoint = (
            self.get_current_waypoint()
        )

        if waypoint is None:

            self.state_machine.path_complete()

            return MotionCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
            )

        # ========================================================
        # ACTIVE TURN
        # ========================================================

        if (
            self.state_machine.get_state()
            == NavigationState.TURNING
        ):

            return self._update_turn(
                pose
            )

        # ========================================================
        # LANE SHIFT
        # ========================================================

        if (
            self.lane_transition_phase
            == "LANE_SHIFT"
        ):

            distance = (
                self.calculate_distance(
                    pose,
                    waypoint,
                )
            )

            # ----------------------------------------------------
            # WP1 reached: purely position-based
            # ----------------------------------------------------

            if distance <= self.position_tolerance:

                # IMPORTANT:
                #
                # Do NOT advance waypoint here.
                #
                # WP1 must remain the active waypoint while
                # the robot is stopped and the second turn starts.
                # ------------------------------------------------

                self.state_machine.waypoint_reached()

                self.lane_transition_phase = (
                    "STOP_AT_LANE_START"
                )

                # Complete stop at WP1.
                return MotionCommand(
                    linear_velocity=0.0,
                    angular_velocity=0.0,
                )

            # ----------------------------------------------------
            # Continue straight to WP1.
            # ----------------------------------------------------

            self.state_machine.start_driving()

            speed = (
                self.speed_controller.calculate_speed(
                    distance
                )
            )

            if speed <= 0.0:

                return MotionCommand(
                    linear_velocity=0.0,
                    angular_velocity=0.0,
                )

            shift_error = (
                self.heading_controller.normalize_angle(
                    self.shift_heading - pose.heading
                )
                if self.shift_heading is not None
                else 0.0
            )

            omega = (
                self.heading_controller.calculate_angular_velocity(
                    shift_error
                )
            )

            return MotionCommand(
                linear_velocity=speed,
                angular_velocity=omega,
            )

        # ========================================================
        # STOP AT WP1
        # ========================================================

        if (
            self.lane_transition_phase
            == "STOP_AT_LANE_START"
        ):

            # ----------------------------------------------------
            # THIS IS THE IMPORTANT WP1 STOP.
            #
            # Keep both wheel commands zero for this complete
            # control cycle before starting the second turn.
            # ----------------------------------------------------

            self._start_second_lane_turn()

            return MotionCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
            )

        # ========================================================
        # ACTIVE SECOND TURN
        # ========================================================

        if (
            self.lane_transition_phase
            == "SECOND_TURN"
        ):

            return self._update_turn(
                pose
            )

        # ========================================================
        # STOP AT WP0 (OR LANE END)
        # ========================================================

        if (
            self.lane_transition_phase
            == "STOP_AT_LANE_END"
        ):

            # Physical stop complete; initiate first 90-degree turn
            self._start_lane_turn(
                pose,
                waypoint,
            )

            return MotionCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
            )

        # ========================================================
        # DISTANCE TO NORMAL WAYPOINT
        # ========================================================

        distance = (
            self.calculate_distance(
                pose,
                waypoint,
            )
        )

        # ========================================================
        # WAYPOINT REACHED
        # ========================================================

        if distance <= (
            self.position_tolerance
        ):

            # ----------------------------------------------------
            # If the NEXT waypoint is a lane-transition point,
            # first execute a COMPLETE STOP before turning.
            # ----------------------------------------------------

            if self._is_lane_transition():

                self.state_machine.waypoint_reached()

                self.lane_transition_phase = (
                    "STOP_AT_LANE_END"
                )

                # COMPLETE STOP: zero linear and angular velocities
                return MotionCommand(
                    linear_velocity=0.0,
                    angular_velocity=0.0,
                )

            # ----------------------------------------------------
            # Normal waypoint.
            # ----------------------------------------------------

            self.state_machine.waypoint_reached()

            if not self._advance_waypoint():

                return MotionCommand(
                    linear_velocity=0.0,
                    angular_velocity=0.0,
                )

            waypoint = (
                self.get_current_waypoint()
            )

            if waypoint is None:

                return MotionCommand(
                    linear_velocity=0.0,
                    angular_velocity=0.0,
                )

            return MotionCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
            )

        # ========================================================
        # NORMAL HEADING AND DRIVING
        # ========================================================

        # Use planned lane-segment heading (A -> B) to avoid singularity near waypoint
        A, B, _, _ = self._get_transition_points()
        if A is not None and B is not None:
            target_heading = self._vector_heading(A, B)
            heading_error = self.heading_controller.normalize_angle(
                target_heading - pose.heading
            )
        else:
            heading_error = self.heading_controller.calculate_heading_error(
                pose,
                waypoint,
            )

        self.state_machine.start_driving()

        speed = (
            self.speed_controller.calculate_speed(
                distance
            )
        )

        if speed <= 0.0:

            return MotionCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
            )

        omega = (
            self.heading_controller
            .calculate_angular_velocity(
                heading_error
            )
        )

        return MotionCommand(
            linear_velocity=speed,
            angular_velocity=omega,
        )