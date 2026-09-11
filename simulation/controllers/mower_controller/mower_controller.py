from controller import Robot
import math


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

# Turn test
left_motor.setVelocity(2.0)
right_motor.setVelocity(2.0)


# ============================================================
# GPS
# ============================================================

gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

robot_x = 0.0
robot_y = 0.0
robot_z = 0.0

initial_gps_x = None
initial_gps_z = None


# ============================================================
# WHEEL ENCODERS
# ============================================================

left_encoder = robot.getDevice("left_wheel_encoder")
right_encoder = robot.getDevice("right_wheel_encoder")

left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)


# Wait for first simulation step so encoder values are initialized
robot.step(TIME_STEP)

initial_left_encoder = left_encoder.getValue()
initial_right_encoder = right_encoder.getValue()


# ============================================================
# ODOMETRY
# ============================================================

odom_x = 0.0
odom_z = 0.0
odom_heading = 0.0

previous_left_distance = 0.0
previous_right_distance = 0.0

wheel_radius = 0.08
wheel_track = 0.44


# ============================================================
# IMU
# ============================================================

imu = robot.getDevice("imu")
imu.enable(TIME_STEP)

# Wait for IMU data
robot.step(TIME_STEP)

orientation = imu.getRollPitchYaw()
initial_yaw = orientation[2]


# ============================================================
# MAIN LOOP
# ============================================================

last_print_time = 0.0


while robot.step(TIME_STEP) != -1:

    current_time = robot.getTime()

    # Print approximately every 1 second
    if current_time - last_print_time >= 1.0:

        # ====================================================
        # GPS DATA
        # ====================================================

        position = gps.getValues()

        robot_x = position[0]
        robot_y = position[1]
        robot_z = position[2]

        if initial_gps_x is None:
            initial_gps_x = robot_x
            initial_gps_z = robot_z

        gps_delta_x = robot_x - initial_gps_x
        gps_delta_z = robot_z - initial_gps_z


        # ====================================================
        # ENCODER DATA
        # ====================================================

        left_position = left_encoder.getValue()
        right_position = right_encoder.getValue()

        left_distance = (
            left_position - initial_left_encoder
        ) * wheel_radius

        right_distance = (
            right_position - initial_right_encoder
        ) * wheel_radius

        average_distance = (
            left_distance + right_distance
        ) / 2.0


        # ====================================================
        # ENCODER DISTANCE DELTA
        # ====================================================

        left_distance_delta = (
            left_distance - previous_left_distance
        )

        right_distance_delta = (
            right_distance - previous_right_distance
        )


        # ====================================================
        # DIFFERENTIAL DRIVE ODOMETRY
        # ====================================================

        heading_delta_encoder = (
            right_distance_delta - left_distance_delta
        ) / wheel_track

        distance_delta = (
            left_distance_delta + right_distance_delta
        ) / 2.0

        odom_heading += heading_delta_encoder

        odom_x += (
            distance_delta * math.cos(odom_heading)
        )

        odom_z += (
            distance_delta * math.sin(odom_heading)
        )


        # Save current distances for next iteration
        previous_left_distance = left_distance
        previous_right_distance = right_distance


        # ====================================================
        # IMU DATA
        # ====================================================

        orientation = imu.getRollPitchYaw()

        roll = orientation[0]
        pitch = orientation[1]
        yaw = orientation[2]



        # ====================================================
        # CONSOLE OUTPUT
        # ====================================================

        print(
            f"Robot Position -> "
            f"X: {robot_x:.3f}, "
            f"Y: {robot_y:.3f}, "
            f"Z: {robot_z:.3f} | "

            f"Encoders -> "
            f"L: {left_position:.3f}, "
            f"R: {right_position:.3f} | "

            f"Distance -> "
            f"L: {left_distance:.4f} m, "
            f"R: {right_distance:.4f} m | "
            f"Avg Distance: {average_distance:.4f} m | "

            f"IMU -> "
            f"Roll: {roll:.3f}, "
            f"Pitch: {pitch:.3f}, "
            f"Yaw: {yaw:.3f} | "

            f"Odometry -> "
            f"X: {odom_x:.4f}, "
            f"Z: {odom_z:.4f}, "
            f"Heading: {odom_heading:.4f} rad | "

            f"GPS Delta -> "
            f"X: {gps_delta_x:.4f}, "
            f"Z: {gps_delta_z:.4f}"
        )

        last_print_time = current_time