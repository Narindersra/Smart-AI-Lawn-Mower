import math
import os
import sys
from pathlib import Path

from controller import Robot

# ============================================================
# PROJECT PATH
# ============================================================

CONTROLLER_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CONTROLLER_DIR.parents[2]

RASPBERRY_PI_SRC = PROJECT_ROOT / "raspberry_pi" / "src"

if str(RASPBERRY_PI_SRC) not in sys.path:
    sys.path.insert(0, str(RASPBERRY_PI_SRC))


# ============================================================
# LOCALIZATION
# ============================================================

from localization.gps import GPS
from localization.imu import IMU
from localization.odometry import Odometry
from localization.position_estimator import PositionEstimator


# ============================================================
# NAVIGATION
# ============================================================

from navigation.navigation_types import RobotPose
from navigation.heading_controller import HeadingController
from navigation.speed_controller import SpeedController
from navigation.differential_drive import DifferentialDriveController
from navigation.navigator import Navigator
from navigation.coverage_planner import CoveragePlanner
from navigation.geofence import Geofence
from navigation.navigation_state_machine import NavigationState


# ============================================================
# ROBOT
# ============================================================

robot = Robot()

TIME_STEP = int(robot.getBasicTimeStep())


# ============================================================
# PHYSICAL PARAMETERS
# ============================================================
#
# These values come from the actual LawnMower PROTO.
#
# Body:
#   X length = 0.50 m
#   Y width  = 0.40 m
#
# Drive wheels:
#   radius = 0.10 m
#   track  = 0.44 m
#
# Coordinate convention:
#
#   Front = -X
#   Rear  = +X
#   Left  = +Y
#   Right = -Y
#   Up    = +Z
#
# Webots ground plane used by localization/navigation:
#   X-Z
#
# Therefore navigation pose is:
#   x
#   z
#   heading
# ============================================================

BODY_LENGTH = 0.50
BODY_WIDTH = 0.40

WHEEL_RADIUS = 0.10
WHEEL_TRACK = 0.44

MAX_WHEEL_VELOCITY = 10.0


# ============================================================
# NAVIGATION PARAMETERS
# ============================================================

POSITION_TOLERANCE = 0.01

HEADING_TOLERANCE = 0.008

MAX_LINEAR_SPEED = 0.50
MIN_LINEAR_SPEED = 0.10
SLOWDOWN_DISTANCE = 1.00

MAX_ANGULAR_SPEED = 0.80
HEADING_KP = 1.50


# ============================================================
# COVERAGE PARAMETERS
# ============================================================
#
# Effective cutting width is a physical/design parameter.
#
# Lane spacing is calculated:
#
#     spacing = cutting_width * (1 - overlap)
#
# 0.30 * 1.00 = 0.300 m
# ============================================================

CUTTING_WIDTH = 0.30
COVERAGE_OVERLAP = 0.0


# ============================================================
# GEOFENCE
# ============================================================
#
# These represent the already established safe lawn-center
# limits used by the project.
#
# The controller does NOT move the starting position.
# ============================================================

GEOFENCE_MIN_X = -10.0
GEOFENCE_MAX_X = 10.0
GEOFENCE_MIN_Z = -10.0
GEOFENCE_MAX_Z = 10.0

GEOFENCE_MARGIN = 0.25


# ============================================================
# WEBOTS DEVICES
# ============================================================

wheel1 = robot.getDevice("wheel1")
wheel2 = robot.getDevice("wheel2")

encoder1 = robot.getDevice("encoder1")
encoder2 = robot.getDevice("encoder2")

gps_device = robot.getDevice("gps")
imu_device = robot.getDevice("imu")


# ============================================================
# MOTOR CONTROL MODE
# ============================================================
#
# RotationalMotor defaults to POSITION control.
#
# Differential drive requires velocity control because the
# wheel commands can be positive OR negative.
#
# This is mandatory.
# ============================================================

wheel1.setPosition(float("inf"))
wheel2.setPosition(float("inf"))

wheel1.setVelocity(0.0)
wheel2.setVelocity(0.0)


# ============================================================
# SENSOR INITIALIZATION
# ============================================================

encoder1.enable(TIME_STEP)
encoder2.enable(TIME_STEP)

gps_device.enable(TIME_STEP)
imu_device.enable(TIME_STEP)


# ============================================================
# LOCALIZATION OBJECTS
# ============================================================

gps = GPS(gps_device)

imu = IMU(imu_device)

odometry = Odometry(
    encoder1,
    encoder2,
    wheel_radius=WHEEL_RADIUS,
    wheel_track=WHEEL_TRACK,
)

position_estimator = PositionEstimator()


# ============================================================
# DRIVE CONTROLLER
# ============================================================

drive_controller = DifferentialDriveController(
    wheel_radius=WHEEL_RADIUS,
    wheel_track=WHEEL_TRACK,
    max_wheel_velocity=MAX_WHEEL_VELOCITY,
)


# ============================================================
# HEADING CONTROLLER
# ============================================================

heading_controller = HeadingController(
    max_angular_speed=MAX_ANGULAR_SPEED,
    heading_tolerance=HEADING_TOLERANCE,
    heading_kp=HEADING_KP,
)


# ============================================================
# SPEED CONTROLLER
# ============================================================

speed_controller = SpeedController(
    max_speed=MAX_LINEAR_SPEED,
    min_speed=MIN_LINEAR_SPEED,
    slowdown_distance=SLOWDOWN_DISTANCE,
    position_tolerance=POSITION_TOLERANCE,
)


# ============================================================
# NAVIGATOR
# ============================================================

navigator = Navigator(
    differential_drive=drive_controller,
    heading_controller=heading_controller,
    speed_controller=speed_controller,
    position_tolerance=POSITION_TOLERANCE,
)


# ============================================================
# GEOFENCE
# ============================================================

geofence = Geofence(
    min_x=GEOFENCE_MIN_X,
    max_x=GEOFENCE_MAX_X,
    min_z=GEOFENCE_MIN_Z,
    max_z=GEOFENCE_MAX_Z,
    safety_margin=GEOFENCE_MARGIN,
)


# ============================================================
# COVERAGE PLANNER
# ============================================================

coverage_planner = CoveragePlanner(
    cutting_width=CUTTING_WIDTH,
    overlap=COVERAGE_OVERLAP,
    body_length=BODY_LENGTH,
    body_width=BODY_WIDTH,
)


# ============================================================
# INITIALIZATION
# ============================================================

initialized = False
mission_started = False
last_print_time = -1.0
prev_state = None
prev_phase = None

path = None


# ============================================================
# HELPERS
# ============================================================

def stop_motors():
    """
    Immediately command both drive wheels to zero.
    """

    wheel1.setVelocity(0.0)
    wheel2.setVelocity(0.0)


def calculate_start_pose():
    """
    Read the initial localization state.

    The project uses Webots GPS X/Z as the ground-plane
    position and IMU yaw as the robot heading.
    """

    gps_data = gps.update()
    imu_data = imu.update()

    pose_data = position_estimator.update(
        gps_data,
        imu_data,
    )

    return RobotPose(
        x=pose_data["x"],
        z=pose_data["z"],
        heading=pose_data["heading"],
    )


def build_coverage_path(start_pose):
    """
    Build the complete mowing path from the actual robot
    starting position.
    """

    safe_min_x, safe_max_x, safe_min_z, safe_max_z = (
        geofence.get_safe_bounds()
    )

    coverage_path = coverage_planner.create_coverage_path(
        start_x=start_pose.x,
        start_z=start_pose.z,
        min_x=safe_min_x,
        max_x=safe_max_x,
        min_z=safe_min_z,
        max_z=safe_max_z,
    )

    if not coverage_planner.validate_path(
        coverage_path,
        min_x=safe_min_x,
        max_x=safe_max_x,
        min_z=safe_min_z,
        max_z=safe_max_z,
    ):
        raise RuntimeError(
            "Generated coverage path failed validation."
        )

    if not geofence.is_path_inside(
        [
            (waypoint.x, waypoint.z)
            for waypoint in coverage_path.waypoints
        ],
        safe=True,
    ):
        raise RuntimeError(
            "Generated coverage path leaves the safe geofence."
        )

    return coverage_path


def apply_motion_command(command):
    """
    Convert high-level navigation command:

        linear velocity [m/s]
        angular velocity [rad/s]

    into actual Webots wheel angular velocities [rad/s].
    """

    left_velocity, right_velocity = (
        drive_controller.calculate_wheel_velocities(
            command.linear_velocity,
            command.angular_velocity,
        )
    )

    wheel1.setVelocity(left_velocity)
    wheel2.setVelocity(right_velocity)

    return left_velocity, right_velocity


# ============================================================
# WAIT FOR SENSOR INITIALIZATION
# ============================================================

while robot.step(TIME_STEP) != -1:

    # --------------------------------------------------------
    # Get current pose.
    # --------------------------------------------------------

    pose = calculate_start_pose()

    # --------------------------------------------------------
    # Initialize mission exactly once.
    # --------------------------------------------------------

    if not initialized:

        start_pose = pose

        print("")
        print("==============================================")
        print("SMART AI LAWN MOWER")
        print("NAVIGATION / COVERAGE CONTROLLER")
        print("==============================================")

        print(
            "Start Pose: "
            f"X={start_pose.x:.3f} "
            f"Z={start_pose.z:.3f} "
            f"Heading={start_pose.heading:.3f}"
        )

        print(
            "Body: "
            f"{BODY_LENGTH:.3f}m x "
            f"{BODY_WIDTH:.3f}m"
        )

        print(
            "Drive: "
            f"Wheel Radius={WHEEL_RADIUS:.3f}m "
            f"Track={WHEEL_TRACK:.3f}m"
        )

        print(
            "Coverage: "
            f"Cutting Width={CUTTING_WIDTH:.3f}m "
            f"Overlap={COVERAGE_OVERLAP * 100:.1f}%"
        )

        print(
            "Lane Spacing: "
            f"{coverage_planner.lane_spacing:.3f}m"
        )

        print(
            "Turning Clearance: "
            f"{coverage_planner.turning_clearance:.3f}m"
        )

        safe_min_x, safe_max_x, safe_min_z, safe_max_z = (
            geofence.get_safe_bounds()
        )
        navigator.configure_coverage(
            min_x=safe_min_x,
            max_x=safe_max_x,
            min_z=safe_min_z,
            max_z=safe_max_z,
            blade_width=CUTTING_WIDTH,
            overlap=COVERAGE_OVERLAP,
            clearance=coverage_planner.total_turn_clearance,
        )
        navigator.start(
            start_pose=start_pose,
        )

        initialized = True
        mission_started = True

    # ========================================================
    # NAVIGATION UPDATE
    # ========================================================

    if mission_started:

        max_test_lanes = int(os.environ.get("WEBOTS_TEST_LANES", "0"))
        if max_test_lanes > 0 and navigator.get_lane_index() >= max_test_lanes:
            navigator.state_machine.set_state(NavigationState.COVERAGE_COMPLETE)

        state = navigator.get_state()

        # ----------------------------------------------------
        # Complete
        # ----------------------------------------------------

        if state in (
            NavigationState.COVERAGE_COMPLETE,
            NavigationState.PATH_COMPLETE,
        ):

            stop_motors()

            if mission_started:

                print("")
                print("==============================================")
                print("COVERAGE COMPLETE")
                print(
                    f"Final X: {pose.x:.3f}"
                )
                print(
                    f"Final Z: {pose.z:.3f}"
                )
                print(
                    f"Final Heading: "
                    f"{pose.heading:.3f}"
                )
                print("Motors: STOPPED")
                print("==============================================")
                print("")

                mission_started = False

            break

        # ----------------------------------------------------
        # Error
        # ----------------------------------------------------

        if state == NavigationState.ERROR:

            stop_motors()

            print("")
            print("==============================================")
            print("NAVIGATION ERROR")
            print("Motors: STOPPED")
            print("==============================================")
            print("")

            mission_started = False

            break

        # ----------------------------------------------------
        # Update navigator.
        # ----------------------------------------------------

        command = navigator.update(pose)

        # ----------------------------------------------------
        # Apply command to physical motors.
        # ----------------------------------------------------

        left_velocity, right_velocity = (
            apply_motion_command(command)
        )

        # ----------------------------------------------------
        # Print discrete transition events immediately
        # ----------------------------------------------------
        for event in navigator.pop_events():
            print(f"[EVENT] {event}")

        # ====================================================
        # STATUS OUTPUT
        # ====================================================

        current_time = robot.getTime()
        current_state = navigator.get_state()

        state_changed = (current_state != prev_state)
        is_transition = (current_state != NavigationState.DRIVE_LANE)
        print_interval = 0.2 if is_transition else 1.0

        if (
            last_print_time < 0
            or state_changed
            or current_time - last_print_time >= print_interval
        ):
            prev_state = current_state

            lane_dir = navigator.get_lane_direction_str()
            lane_idx = navigator.get_lane_index()
            turn_phase = navigator.get_turn_phase()

            status_line = (
                f"STATE={current_state.name} | "
                f"X={pose.x:.3f} | "
                f"Z={pose.z:.3f} | "
                f"HEADING={pose.heading:.3f} | "
                f"LANE_DIRECTION={lane_dir} | "
                f"LANE_INDEX={lane_idx} | "
                f"TURN_PHASE={turn_phase}"
            )

            print(status_line)

            last_print_time = current_time


# ============================================================
# SAFETY STOP
# ============================================================

stop_motors()