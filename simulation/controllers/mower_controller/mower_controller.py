from pathlib import Path
import sys
import math

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
from localization.localization_manager import LocalizationManager


# ============================================================
# NAVIGATION
# ============================================================

from navigation.navigation_types import RobotPose
from navigation.path_planner import PathPlanner
from navigation.geofence import Geofence
from navigation.navigator import Navigator
from navigation.heading_controller import HeadingController
from navigation.speed_controller import SpeedController
from navigation.differential_drive import (
    DifferentialDriveController,
)


# ============================================================
# ROBOT
# ============================================================

robot = Robot()
timestep = int(robot.getBasicTimeStep())


# ============================================================
# MOTORS
#
# Verified Webots physics:
# Negative wheel velocity = robot forward (-X)
# ============================================================

left_motor = robot.getDevice("wheel1")
right_motor = robot.getDevice("wheel2")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


# ============================================================
# ENCODERS
# ============================================================

encoder_left = robot.getDevice("encoder1")
encoder_right = robot.getDevice("encoder2")

encoder_left.enable(timestep)
encoder_right.enable(timestep)


# ============================================================
# GPS
# ============================================================

gps_device = robot.getDevice("gps")
gps_device.enable(timestep)

gps = GPS(gps_device)


# ============================================================
# IMU
# ============================================================

imu_device = robot.getDevice("imu")
imu_device.enable(timestep)

imu = IMU(imu_device)


# ============================================================
# ODOMETRY
#
# Actual robot physics:
# wheel radius = 0.10 m
# wheel track  = 0.44 m
# ============================================================

odometry = Odometry(
    encoder_left,
    encoder_right,
    wheel_radius=0.10,
    wheel_track=0.44,
)


# ============================================================
# POSITION ESTIMATOR
# ============================================================

position_estimator = PositionEstimator()


# ============================================================
# LOCALIZATION MANAGER
# ============================================================

localization_manager = LocalizationManager(
    gps,
    imu,
    odometry,
    position_estimator,
)

localization_manager.initialize()


# ============================================================
# NAVIGATION PARAMETERS
# ============================================================

POSITION_TOLERANCE = 0.15
HEADING_TOLERANCE = 0.08

WAYPOINT_SPACING = 0.50

MAX_LINEAR_SPEED = 0.50
MIN_LINEAR_SPEED = 0.10
SLOWDOWN_DISTANCE = 1.00

MAX_ANGULAR_SPEED = 0.8
HEADING_KP = 1.5

WHEEL_RADIUS = 0.10
WHEEL_TRACK = 0.44

# PROTO RotationalMotor maxVelocity = 10 rad/s
MAX_WHEEL_VELOCITY = 10.0


# ============================================================
# GEOFENCE
# ============================================================

geofence = Geofence(
    min_x=-10.0,
    max_x=10.0,
    min_z=-10.0,
    max_z=10.0,
    safety_margin=0.25,
)


# ============================================================
# PATH PLANNER
# ============================================================

path_planner = PathPlanner(
    waypoint_spacing=WAYPOINT_SPACING,
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
# DIFFERENTIAL DRIVE
# ============================================================

drive_controller = DifferentialDriveController(
    wheel_radius=WHEEL_RADIUS,
    wheel_track=WHEEL_TRACK,
    max_wheel_velocity=MAX_WHEEL_VELOCITY,
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
# NAVIGATION INITIALIZATION FLAG
# ============================================================

navigation_initialized = False


# ============================================================
# DEBUG
# ============================================================

DEBUG_INTERVAL = 0.5
last_debug_time = 0.0


def print_status(
    pose,
    state,
    waypoint_index,
    distance,
    heading_error,
    left_velocity,
    right_velocity,
):
    global last_debug_time

    current_time = robot.getTime()

    if current_time - last_debug_time < DEBUG_INTERVAL:
        return

    last_debug_time = current_time

    print(
        f"STATE={state} | "
        f"WP={waypoint_index} | "
        f"X={pose.x:.3f} | "
        f"Z={pose.z:.3f} | "
        f"H={pose.heading:.3f} | "
        f"D={distance:.3f} | "
        f"HE={heading_error:.3f} | "
        f"L={left_velocity:.3f} | "
        f"R={right_velocity:.3f}"
    )


def calculate_forward_boundary_waypoint(pose, geofence):
    """
    Calculate the point where the robot's forward direction reaches
    the safe geofence boundary, then move that point inward by the
    robot's half-diagonal so the robot has clearance to rotate.
    """

    theta = pose.heading

    # Robot forward direction in Webots ground plane.
    # heading = 0 -> forward = -X
    fx = -math.cos(theta)
    fz = math.sin(theta)

    safe_min_x, safe_max_x, safe_min_z, safe_max_z = geofence.get_safe_bounds()

    distances = []

    # Intersection with X boundaries
    if abs(fx) > 1e-9:
        if fx > 0:
            distance = (safe_max_x - pose.x) / fx
        else:
            distance = (safe_min_x - pose.x) / fx

        if distance > 0:
            distances.append(distance)

    # Intersection with Z boundaries
    if abs(fz) > 1e-9:
        if fz > 0:
            distance = (safe_max_z - pose.z) / fz
        else:
            distance = (safe_min_z - pose.z) / fz

        if distance > 0:
            distances.append(distance)

    if not distances:
        raise RuntimeError("Unable to calculate forward boundary waypoint.")

    # Distance from robot center to the safe boundary.
    boundary_distance = min(distances)

    # Derive robot rotational clearance from actual body dimensions.
    body_length = 0.50
    body_width = 0.40

    rotation_clearance = math.hypot(
        body_length / 2.0,
        body_width / 2.0
    )

    # Keep the waypoint inside the boundary by the required clearance.
    waypoint_distance = max(
        0.0,
        boundary_distance - rotation_clearance
    )

    goal_x = pose.x + fx * waypoint_distance
    goal_z = pose.z + fz * waypoint_distance

    return goal_x, goal_z

# ============================================================
# MOTOR STOP
# ============================================================

def stop_motors():
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


# ============================================================
# APPLY NAVIGATION COMMAND
# ============================================================

def apply_motion_command(command):

    left_velocity, right_velocity = (
        drive_controller.calculate_wheel_velocities(
            command.linear_velocity,
            command.angular_velocity,
        )
    )

    left_motor.setVelocity(left_velocity)
    right_motor.setVelocity(right_velocity)

    return left_velocity, right_velocity


# ============================================================
# STARTUP
# ============================================================

print()
print("==============================================")
print(" Smart AI Lawn Mower")
print(" Basic Navigation Controller")
print("==============================================")
print("Localization: READY")
print("Navigation:   INITIALIZING")
print("==============================================")


# ============================================================
# MAIN LOOP
# ============================================================

while robot.step(timestep) != -1:

    # --------------------------------------------------------
    # UPDATE LOCALIZATION
    # --------------------------------------------------------

    localization_data = localization_manager.update()

    pose_data = localization_data["pose"]

    pose = RobotPose(
        x=pose_data["x"],
        z=pose_data["z"],
        heading=pose_data["heading"],
    )


    # --------------------------------------------------------
    # INITIALIZE NAVIGATION ONCE
    # --------------------------------------------------------

    if not navigation_initialized:

        start_x = pose.x
        start_z = pose.z

        # ----------------------------------------------------
        # BASIC NAVIGATION TEST
        #
        # Robot heading 0 points toward -X.
        # Therefore goal is placed 2 m directly in front.
        # ----------------------------------------------------

        goal_x, goal_z = (
            calculate_forward_boundary_waypoint(
                pose,
                geofence,
            )
        )


        # ----------------------------------------------------
        # START POSITION CHECK
        # ----------------------------------------------------

        if not geofence.contains(
            start_x,
            start_z,
        ):
            stop_motors()

            print(
                "ERROR: Robot starting position is "
                "outside the geofence."
            )

            print(
                f"X={start_x:.3f}, "
                f"Z={start_z:.3f}"
            )

            break


        # ----------------------------------------------------
        # CREATE STRAIGHT PATH
        # ----------------------------------------------------

        navigation_path = (
            path_planner.create_straight_path(
                start_x=start_x,
                start_z=start_z,
                goal_x=goal_x,
                goal_z=goal_z,
            )
        )


        # ----------------------------------------------------
        # VALIDATE PATH
        # ----------------------------------------------------

        if not path_planner.validate_path(
            navigation_path
        ):
            stop_motors()

            print(
                "ERROR: Generated navigation path "
                "is invalid."
            )

            break


        # ----------------------------------------------------
        # GEOFENCE PATH CHECK
        # ----------------------------------------------------

        path_points = [
            (waypoint.x, waypoint.z)
            for waypoint in navigation_path.waypoints
        ]

        if not geofence.is_path_inside(
            path_points,
            safe=True,
        ):
            stop_motors()

            print(
                "ERROR: Generated path exceeds "
                "the safe geofence."
            )

            break


        # ----------------------------------------------------
        # GIVE PATH TO NAVIGATOR
        # ----------------------------------------------------

        navigator.set_path(
            navigation_path
        )

        navigator.start()

        navigation_initialized = True


        # ----------------------------------------------------
        # STARTUP INFORMATION
        # ----------------------------------------------------

        print(
            "Navigation: READY"
        )

        print(
            f"Start -> X={start_x:.3f}, "
            f"Z={start_z:.3f}"
        )

        print(
            f"Goal  -> X={goal_x:.3f}, "
            f"Z={goal_z:.3f}"
        )

        print(
            "Goal is 2.0 m directly in front "
            "of the robot."
        )


    # --------------------------------------------------------
    # RUNTIME GEOFENCE
    # --------------------------------------------------------

    if not geofence.contains(
        pose.x,
        pose.z,
    ):
        stop_motors()

        print(
            "GEOFENCE STOP | "
            f"X={pose.x:.3f}, "
            f"Z={pose.z:.3f}"
        )

        break


    # --------------------------------------------------------
    # NAVIGATION UPDATE
    # --------------------------------------------------------

    command = navigator.update(
        pose
    )


    # --------------------------------------------------------
    # NAVIGATION COMPLETE
    # --------------------------------------------------------

    if navigator.is_complete():

        stop_motors()

        print()
        print("==============================================")
        print(" NAVIGATION COMPLETE")
        print("==============================================")
        print(
            f"Final X       : {pose.x:.3f}"
        )
        print(
            f"Final Z       : {pose.z:.3f}"
        )
        print(
            f"Final Heading : {pose.heading:.3f}"
        )
        print(
            "Motors        : STOPPED"
        )
        print("==============================================")

        break


    # --------------------------------------------------------
    # APPLY COMMAND TO PHYSICAL MOTORS
    # --------------------------------------------------------

    (
        left_velocity,
        right_velocity,
    ) = apply_motion_command(
        command
    )


    # --------------------------------------------------------
    # CURRENT WAYPOINT DEBUG DATA
    # --------------------------------------------------------

    current_waypoint = (
        navigator.get_current_waypoint()
    )

    if current_waypoint is not None:

        distance = (
            navigator.calculate_distance(
                pose,
                current_waypoint,
            )
        )

        heading_error = (
            navigator.heading_controller
            .calculate_heading_error(
                pose,
                current_waypoint,
            )
        )

    else:

        distance = 0.0
        heading_error = 0.0


    # --------------------------------------------------------
    # STATUS
    # --------------------------------------------------------

    print_status(
        pose=pose,
        state=navigator.get_state().name,
        waypoint_index=(
            navigator.get_current_waypoint_index()
        ),
        distance=distance,
        heading_error=heading_error,
        left_velocity=left_velocity,
        right_velocity=right_velocity,
    )


# ============================================================
# FINAL SAFETY STOP
# ============================================================

stop_motors()

print()
print("Controller stopped.")