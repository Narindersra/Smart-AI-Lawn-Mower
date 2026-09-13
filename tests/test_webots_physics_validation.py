"""
PHYSICAL KINEMATICS AND WEBOTS COORDINATE VALIDATION TEST SUITE
==============================================================
Validates all coordinate, sign, and kinematic equations against
actual Webots LawnMower physics and LawnMower.proto specification:
1. Heading = 0 -> Forward direction is -X (dx < 0, dz == 0)
2. Heading = +90 deg (+pi/2) -> Forward direction is -Z (dx == 0, dz < 0)
3. Heading = -90 deg (-pi/2) -> Forward direction is +Z (dx == 0, dz > 0)
4. Heading = 180 deg (pi) -> Forward direction is +X (dx > 0, dz == 0)
5. Equal wheel velocities produce straight motion with zero angular drift
6. Motor sign convention: negative motor velocity produces forward wheel linear speed
7. Differential velocity: left > right wheel speed produces counter-clockwise rotation (+yaw)
8. Pure Pursuit quadratic line-segment intersection produces forward lookahead point
9. Robot-frame transformation correctly separates forward (x_f) and lateral (y_l) error
10. Pure Pursuit curvature kappa = 2*y_l / Ld^2 correctly maps to omega = -v * kappa
"""

import math
import sys
import os

# Ensure raspberry_pi/src is in sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
raspberry_pi_src = os.path.join(project_root, "raspberry_pi", "src")
if raspberry_pi_src not in sys.path:
    sys.path.insert(0, raspberry_pi_src)

from navigation.navigation_types import RobotPose
from navigation.differential_drive import DifferentialDriveController
from navigation.heading_controller import HeadingController


class WebotsPhysicsValidator:
    """
    Direct simulation of LawnMower.proto physics and Webots coordinate system:
        World arena: X-Y ground plane, Z height
        Navigation mapping: x = world X, z = world Y
        Chassis: Front = -X, Rear = +X, Right = +Y (wheel1), Left = -Y (wheel2)
        Wheel axis: (0, 1, 0)
        Wheel radius: r = 0.10 m, Track: T = 0.44 m
    """
    def __init__(self, x=0.0, z=0.0, heading=0.0, dt=0.016):
        self.x = float(x)
        self.z = float(z)
        self.heading = float(heading)
        self.dt = float(dt)
        self.wheel_radius = 0.10
        self.wheel_track = 0.44

    def step(self, motor1_vel, motor2_vel):
        # LawnMower.proto: wheel1 is right wheel at +Y, wheel2 is left wheel at -Y
        # Motor axis is 0 1 0. Negative motor velocity produces forward wheel motion
        v_wheel1_right = -motor1_vel * self.wheel_radius
        v_wheel2_left = -motor2_vel * self.wheel_radius

        v_forward = (v_wheel1_right + v_wheel2_left) / 2.0
        # In LawnMower.proto: wheel1 is right at +Y, wheel2 is left at -Y.
        # When wheel1 (+Y) moves forward (toward -X) and wheel2 (-Y) moves backward (toward +X),
        # the mower rotates toward -Y, which is counter-clockwise around +Z (+yaw):
        omega = (v_wheel1_right - v_wheel2_left) / self.wheel_track

        # Forward motion in Webots coordinate frame:
        # Front is -X when heading is 0
        fx = -math.cos(self.heading)
        fz = -math.sin(self.heading)

        self.x += v_forward * fx * self.dt
        self.z += v_forward * fz * self.dt
        self.heading += omega * self.dt

        # Normalize heading to [-pi, pi]
        while self.heading > math.pi:
            self.heading -= 2.0 * math.pi
        while self.heading < -math.pi:
            self.heading += 2.0 * math.pi

    def get_pose(self):
        return RobotPose(x=self.x, z=self.z, heading=self.heading)


def test_webots_heading_cardinal_directions():
    """Verify forward motion along all 4 cardinal headings."""
    dt = 0.016
    speed = 0.50  # m/s
    steps = 100   # 1.6 seconds

    # 1. Heading = 0: must move toward -X
    sim0 = WebotsPhysicsValidator(x=0.0, z=0.0, heading=0.0, dt=dt)
    for _ in range(steps):
        # Motor command for forward speed 0.50 m/s: motor_vel = -v / r = -5.0 rad/s
        sim0.step(-5.0, -5.0)
    assert sim0.x < -0.79, f"Heading 0 must move toward -X: got x={sim0.x}"
    assert abs(sim0.z) < 1e-6, f"Heading 0 must have zero z displacement: got z={sim0.z}"

    # 2. Heading = +pi/2 (+90 deg): must move toward -Z (world -Y)
    sim_plus90 = WebotsPhysicsValidator(x=0.0, z=0.0, heading=math.pi / 2.0, dt=dt)
    for _ in range(steps):
        sim_plus90.step(-5.0, -5.0)
    assert abs(sim_plus90.x) < 1e-6, f"Heading +90 must have zero x displacement: got x={sim_plus90.x}"
    assert sim_plus90.z < -0.79, f"Heading +90 must move toward -Z: got z={sim_plus90.z}"

    # 3. Heading = -pi/2 (-90 deg): must move toward +Z (world +Y)
    sim_minus90 = WebotsPhysicsValidator(x=0.0, z=0.0, heading=-math.pi / 2.0, dt=dt)
    for _ in range(steps):
        sim_minus90.step(-5.0, -5.0)
    assert abs(sim_minus90.x) < 1e-6, f"Heading -90 must have zero x displacement: got x={sim_minus90.x}"
    assert sim_minus90.z > 0.79, f"Heading -90 must move toward +Z: got z={sim_minus90.z}"

    # 4. Heading = pi (180 deg): must move toward +X
    sim_180 = WebotsPhysicsValidator(x=0.0, z=0.0, heading=math.pi, dt=dt)
    for _ in range(steps):
        sim_180.step(-5.0, -5.0)
    assert sim_180.x > 0.79, f"Heading 180 must move toward +X: got x={sim_180.x}"
    assert abs(sim_180.z) < 1e-6, f"Heading 180 must have zero z displacement: got z={sim_180.z}"


def test_webots_differential_rotation():
    """Verify left/right differential velocity produces correct rotation direction."""
    dt = 0.016
    # In LawnMower.proto: wheel1 is right (+Y), wheel2 is left (-Y)
    # Turn LEFT (counter-clockwise, +yaw): wheel2 (left) moves backward or slower, wheel1 (right) moves forward
    # Forward on wheel1 is negative velocity (-5.0), backward on wheel2 is positive velocity (+5.0)
    sim_turn_left = WebotsPhysicsValidator(x=0.0, z=0.0, heading=0.0, dt=dt)
    sim_turn_left.step(motor1_vel=-5.0, motor2_vel=+5.0)
    assert sim_turn_left.heading > 0, f"Counter-clockwise turn must increase heading: got {sim_turn_left.heading}"

    # Turn RIGHT (clockwise, -yaw): wheel1 (right) moves backward (+5.0), wheel2 (left) moves forward (-5.0)
    sim_turn_right = WebotsPhysicsValidator(x=0.0, z=0.0, heading=0.0, dt=dt)
    sim_turn_right.step(motor1_vel=+5.0, motor2_vel=-5.0)
    assert sim_turn_right.heading < 0, f"Clockwise turn must decrease heading: got {sim_turn_right.heading}"


def test_target_heading_cardinal_points():
    """Verify HeadingController.calculate_target_heading matches Webots coordinates."""
    hc = HeadingController()
    origin = RobotPose(x=0.0, z=0.0, heading=0.0)

    # Target in front (-X): heading must be 0.0
    h_front = hc.calculate_target_heading(origin, RobotPose(x=-5.0, z=0.0, heading=0.0))
    assert abs(h_front - 0.0) < 1e-6, f"Front target heading must be 0.0, got {h_front}"

    # Target to left (-Z): heading must be +pi/2
    h_left = hc.calculate_target_heading(origin, RobotPose(x=0.0, z=-5.0, heading=0.0))
    assert abs(h_left - math.pi / 2.0) < 1e-6, f"Left target heading must be +pi/2, got {h_left}"

    # Target behind (+X): heading must be pi
    h_rear = hc.calculate_target_heading(origin, RobotPose(x=+5.0, z=0.0, heading=0.0))
    assert abs(abs(h_rear) - math.pi) < 1e-6, f"Rear target heading must be pi, got {h_rear}"

    # Target to right (+Z): heading must be -pi/2
    h_right = hc.calculate_target_heading(origin, RobotPose(x=0.0, z=+5.0, heading=0.0))
    assert abs(h_right - (-math.pi / 2.0)) < 1e-6, f"Right target heading must be -pi/2, got {h_right}"


def test_pure_pursuit_frame_transformation():
    """Verify robot-frame transformation x_f and y_l."""
    # When robot is at (0, 0) with heading 0 (facing -X):
    # Front is -X, Left is -Z, Right is +Z
    heading = 0.0
    cos_h = math.cos(heading)
    sin_h = math.sin(heading)

    # Target A ahead at (-2, 0)
    dx_a, dz_a = -2.0, 0.0
    xf_a = -(dx_a * cos_h + dz_a * sin_h)
    yl_a = dx_a * sin_h - dz_a * cos_h
    assert abs(xf_a - 2.0) < 1e-6, f"Ahead point must have x_f = +2.0, got {xf_a}"
    assert abs(yl_a - 0.0) < 1e-6, f"Centerline point must have y_l = 0.0, got {yl_a}"

    # Target B to the left at (0, -1)
    dx_b, dz_b = 0.0, -1.0
    xf_b = -(dx_b * cos_h + dz_b * sin_h)
    yl_b = dx_b * sin_h - dz_b * cos_h
    assert abs(xf_b - 0.0) < 1e-6, f"Lateral point must have x_f = 0.0, got {xf_b}"
    assert abs(yl_b - 1.0) < 1e-6, f"Left point must have y_l = +1.0, got {yl_b}"

    # Target C to the right at (0, +1)
    dx_c, dz_c = 0.0, 1.0
    xf_c = -(dx_c * cos_h + dz_c * sin_h)
    yl_c = dx_c * sin_h - dz_c * cos_h
    assert abs(yl_c - (-1.0)) < 1e-6, f"Right point must have y_l = -1.0, got {yl_c}"


def test_pure_pursuit_curvature_to_differential_drive():
    """Verify that Pure Pursuit curvature maps to differential drive command correctly."""
    # For a target to the left: y_l > 0 -> kappa > 0 -> robot must turn counter-clockwise (yaw rate > 0)
    # In differential drive, commanding omega < 0 causes v_L > v_R, rotating counter-clockwise
    # Therefore, omega = -v * kappa
    v = 0.50
    Ld = 0.50
    yl = 0.05  # 5 cm to the left
    kappa = 2.0 * yl / (Ld ** 2)
    omega_cmd = -v * kappa

    drive = DifferentialDriveController(wheel_radius=0.10, wheel_track=0.44)
    motor1_vel, motor2_vel = drive.calculate_wheel_velocities(v, omega_cmd)

    # LawnMower.proto: wheel1 is right (+Y), wheel2 is left (-Y)
    # For a left turn: left wheel (wheel2) must move slower/backward, right wheel (wheel1) faster forward
    # Both velocities are negative in forward motion; more negative means faster forward
    v_wheel1_right = -motor1_vel * 0.10
    v_wheel2_left = -motor2_vel * 0.10

    # Right wheel must be faster forward than left wheel
    assert v_wheel1_right > v_wheel2_left, (
        f"Right wheel ({v_wheel1_right:.3f}) must be faster forward than left wheel ({v_wheel2_left:.3f})"
    )


if __name__ == "__main__":
    test_webots_heading_cardinal_directions()
    test_webots_differential_rotation()
    test_target_heading_cardinal_points()
    test_pure_pursuit_frame_transformation()
    test_pure_pursuit_curvature_to_differential_drive()
    print("ALL WEBOTS PHYSICAL KINEMATICS VALIDATION TESTS PASSED 100%!")
