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

        self.state_machine = (
            NavigationStateMachine()
        )

        self.path = None
        self.current_waypoint_index = 0

    def set_path(self, path):
        self.path = path
        self.current_waypoint_index = 0

    def start(self):
        if (
            self.path is None
            or not self.path.waypoints
        ):
            self.state_machine.path_complete()
            return

        self.current_waypoint_index = 0
        self.state_machine.start()

    def get_state(self):
        return self.state_machine.get_state()

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

    def is_complete(self):
        return (
            self.state_machine.get_state()
            == NavigationState.PATH_COMPLETE
        )

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

    def update(self, pose):

        # ----------------------------------------------------
        # NO PATH
        # ----------------------------------------------------

        if self.path is None:
            self.state_machine.set_error()

            return MotionCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
            )

        # ----------------------------------------------------
        # PATH COMPLETE
        # ----------------------------------------------------

        if self.is_complete():
            return MotionCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
            )

        # ----------------------------------------------------
        # CURRENT WAYPOINT
        # ----------------------------------------------------

        waypoint = self.get_current_waypoint()

        if waypoint is None:
            self.state_machine.path_complete()

            return MotionCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
            )

        # ----------------------------------------------------
        # DISTANCE TO WAYPOINT
        # ----------------------------------------------------

        distance = self.calculate_distance(
            pose,
            waypoint,
        )

        # ----------------------------------------------------
        # WAYPOINT REACHED
        # ----------------------------------------------------

        if distance <= self.position_tolerance:

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

            distance = self.calculate_distance(
                pose,
                waypoint,
            )

        # ----------------------------------------------------
        # HEADING ERROR
        # ----------------------------------------------------

        heading_error = (
            self.heading_controller
            .calculate_heading_error(
                pose,
                waypoint,
            )
        )

        # ----------------------------------------------------
        # ALIGNMENT
        # ----------------------------------------------------

        if abs(heading_error) > (
            self.heading_controller.heading_tolerance
        ):

            self.state_machine.start()

            return self.heading_controller.update(
                pose,
                waypoint,
            )

        # ----------------------------------------------------
        # ALIGNED → DRIVE
        # ----------------------------------------------------

        self.state_machine.start_driving()

        speed = (
            self.speed_controller.calculate_speed(
                distance
            )
        )

        # ----------------------------------------------------
        # SAFETY STOP
        # ----------------------------------------------------

        if speed <= 0.0:

            self.state_machine.waypoint_reached()

            if not self._advance_waypoint():

                return MotionCommand(
                    linear_velocity=0.0,
                    angular_velocity=0.0,
                )

            return MotionCommand(
                linear_velocity=0.0,
                angular_velocity=0.0,
            )

        # ----------------------------------------------------
        # DRIVE STRAIGHT TOWARD WAYPOINT
        # ----------------------------------------------------

        return MotionCommand(
            linear_velocity=speed,
            angular_velocity=0.0,
        )