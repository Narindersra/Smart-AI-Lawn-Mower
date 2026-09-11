from controller import Robot
from pathlib import Path
import sys


# ============================================================
# PROJECT PATH
# ============================================================

CONTROLLER_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CONTROLLER_DIR.parents[2]
RASPBERRY_PI_SRC = PROJECT_ROOT / "raspberry_pi" / "src"

if str(RASPBERRY_PI_SRC) not in sys.path:
    sys.path.insert(0, str(RASPBERRY_PI_SRC))


# ============================================================
# LOCALIZATION IMPORTS
# ============================================================

from localization.gps import GPS
from localization.imu import IMU
from localization.odometry import Odometry
from localization.position_estimator import PositionEstimator
from localization.localization_manager import LocalizationManager


# ============================================================
# ROBOT INITIALIZATION
# ============================================================

robot = Robot()

TIME_STEP = 16


# ============================================================
# MOTORS
# ============================================================

left_motor = robot.getDevice("left_wheel_motor")
right_motor = robot.getDevice("right_wheel_motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

# Keep robot stationary during localization validation.
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


# ============================================================
# GPS
# ============================================================

gps_device = robot.getDevice("gps")
gps_device.enable(TIME_STEP)


# ============================================================
# IMU
# ============================================================

imu_device = robot.getDevice("imu")
imu_device.enable(TIME_STEP)


# ============================================================
# WHEEL ENCODERS
# ============================================================

left_encoder = robot.getDevice("left_wheel_encoder")
right_encoder = robot.getDevice("right_wheel_encoder")

left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)


# ============================================================
# WAIT FOR SENSOR INITIALIZATION
# ============================================================

if robot.step(TIME_STEP) == -1:
    sys.exit()


# ============================================================
# LOCALIZATION COMPONENTS
# ============================================================

gps = GPS(gps_device)

imu = IMU(imu_device)

odometry = Odometry(
    left_encoder,
    right_encoder,
    wheel_radius=0.08,
    wheel_track=0.44
)

position_estimator = PositionEstimator()


# ============================================================
# LOCALIZATION MANAGER
# ============================================================

localization_manager = LocalizationManager(
    gps,
    imu,
    odometry,
    position_estimator
)

localization_manager.initialize()


# ============================================================
# MAIN CONTROL LOOP
# ============================================================

last_print_time = 0.0

while robot.step(TIME_STEP) != -1:

    current_time = robot.getTime()

    # --------------------------------------------------------
    # Update localization
    # --------------------------------------------------------

    localization = localization_manager.update()

    # --------------------------------------------------------
    # Print localization state once per second
    # --------------------------------------------------------

    if current_time - last_print_time >= 1.0:

        gps_data = localization["gps"]
        imu_data = localization["imu"]
        odometry_data = localization["odometry"]
        pose = localization["pose"]

        print(
            f"GPS -> "
            f"X: {gps_data['x']:.3f}, "
            f"Z: {gps_data['z']:.3f} | "

            f"IMU -> "
            f"Yaw: {imu_data['yaw']:.3f} | "

            f"Odometry -> "
            f"X: {odometry_data['x']:.3f}, "
            f"Z: {odometry_data['z']:.3f}, "
            f"Heading: {odometry_data['heading']:.3f} | "

            f"Pose -> "
            f"X: {pose['x']:.3f}, "
            f"Z: {pose['z']:.3f}, "
            f"Heading: {pose['heading']:.3f}"
        )

        last_print_time = current_time

