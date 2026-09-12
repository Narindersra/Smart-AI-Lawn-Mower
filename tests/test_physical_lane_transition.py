import math
import sys
import os

# Add raspberry_pi/src to python path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
raspberry_pi_src = os.path.join(project_root, "raspberry_pi", "src")
if raspberry_pi_src not in sys.path:
    sys.path.insert(0, raspberry_pi_src)

from navigation.navigation_types import RobotPose, Waypoint
from navigation.navigator import Navigator
from navigation.differential_drive import DifferentialDriveController
from navigation.heading_controller import HeadingController
from navigation.speed_controller import SpeedController
from navigation.coverage_planner import CoveragePlanner
from navigation.geofence import Geofence
from navigation.navigation_state_machine import NavigationState


class PhysicalRobotSimulator:
    """
    Simulates the exact physical differential-drive robot kinematics
    matching Webots LawnMower at basicTimeStep = 16 ms:
        Body: 0.50m x 0.40m
        Drive: r = 0.10m, T = 0.44m
        Front = -X, Rear = +X
        Ground Plane = X-Z (mapped from Webots world X-Y)
    """
    def __init__(self, x=9.50, z=-9.50, heading=0.0, dt=0.016):
        self.x = float(x)
        self.z = float(z)
        self.heading = float(heading)
        self.dt = float(dt)
        self.wheel_radius = 0.10
        self.wheel_track = 0.44

    def step(self, left_motor_vel, right_motor_vel):
        # Convert motor angular velocities (rad/s) to surface linear speeds
        # In LawnMower.proto, axis is 0 1 0. DifferentialDriveController produces:
        # left = -v_left / r, right = -v_right / r.
        # Invert to get linear wheel speeds:
        v_left = -left_motor_vel * self.wheel_radius
        v_right = -right_motor_vel * self.wheel_radius

        v = (v_left + v_right) / 2.0
        # In Webots, left motor at +Y and right motor at -Y;
        # left forward and right backward produces positive yaw around +Z:
        omega = (v_left - v_right) / self.wheel_track

        # Kinematics in project coordinate frame:
        # Robot heading 0 -> facing -X
        # fx = -cos(heading), fz = +sin(heading)
        self.x -= v * math.cos(self.heading) * self.dt
        self.z += v * math.sin(self.heading) * self.dt
        self.heading += omega * self.dt

        # Normalize heading to [-pi, pi]
        while self.heading > math.pi:
            self.heading -= 2.0 * math.pi
        while self.heading < -math.pi:
            self.heading += 2.0 * math.pi

    def get_pose(self):
        return RobotPose(x=self.x, z=self.z, heading=self.heading)


def run_physical_verification():
    print("============================================================")
    print("SMART AI LAWN MOWER — PHYSICAL VERIFICATION TEST SUITE")
    print("============================================================")

    # Initialize geofence & coverage path
    geofence = Geofence(-10.0, 10.0, -10.0, 10.0, safety_margin=0.25)
    smin_x, smax_x, smin_z, smax_z = geofence.get_safe_bounds()
    planner = CoveragePlanner(
        cutting_width=0.20,
        overlap=0.10,
        body_length=0.50,
        body_width=0.40
    )

    sim = PhysicalRobotSimulator(x=9.50, z=-9.50, heading=0.0, dt=0.016)
    start_pose = sim.get_pose()

    # TEST 1: START POSE
    print("\n--- TEST 1: START POSE ---")
    print(f"Initial X: {start_pose.x:.3f}, Z: {start_pose.z:.3f}, H: {start_pose.heading:.3f}")
    assert abs(start_pose.x - 9.50) < 1e-3, "Start X failed"
    assert abs(start_pose.z - (-9.50)) < 1e-3, "Start Z failed"
    print("TEST 1 PASSED: Start pose exactly matches physical world placement (9.50, -9.50).")

    coverage_path = planner.create_coverage_path(
        start_x=start_pose.x,
        start_z=start_pose.z,
        min_x=smin_x,
        max_x=smax_x,
        min_z=smin_z,
        max_z=smax_z
    )

    drive_ctrl = DifferentialDriveController(wheel_radius=0.10, wheel_track=0.44)
    heading_ctrl = HeadingController(max_angular_speed=0.80, heading_tolerance=0.08, heading_kp=1.50)
    speed_ctrl = SpeedController(max_speed=0.50, min_speed=0.10, slowdown_distance=1.0, position_tolerance=0.05)
    nav = Navigator(drive_ctrl, heading_ctrl, speed_ctrl, position_tolerance=0.05)

    nav.set_path(coverage_path, start_pose=start_pose)
    nav.start(start_pose=start_pose)

    print(f"Generated {len(coverage_path.waypoints)} waypoints.")
    print(f"WP0: ({coverage_path.waypoints[0].x:.3f}, {coverage_path.waypoints[0].z:.3f})")
    print(f"WP1: ({coverage_path.waypoints[1].x:.3f}, {coverage_path.waypoints[1].z:.3f})")
    print(f"WP2: ({coverage_path.waypoints[2].x:.3f}, {coverage_path.waypoints[2].z:.3f})")

    # TEST 2: STRAIGHT MOTION ON LANE 1 TO WP0
    print("\n--- TEST 2: STRAIGHT MOTION ON LANE 1 ---")
    step_count = 0
    max_steps = 500000
    reached_wp0 = False
    
    # Track metrics
    wp0_reached_pose = None
    wp0_stopped_verified = False
    first_turn_completed_pose = None
    wp1_reached_pose = None
    wp1_stopped_verified = False
    second_turn_completed_pose = None
    wp2_reached_pose = None

    transitions_logged = []
    current_phase = None

    while step_count < max_steps and not nav.is_complete():
        pose = sim.get_pose()
        cmd = nav.update(pose)
        l_vel, r_vel = drive_ctrl.calculate_wheel_velocities(cmd.linear_velocity, cmd.angular_velocity)
        
        phase = nav.lane_transition_phase
        wp_idx = nav.get_current_waypoint_index()

        # Log phase changes
        if phase != current_phase:
            print(f"Step {step_count:05d} (t={step_count*0.016:6.2f}s) | PHASE CHANGE: {current_phase} -> {phase} | WP={wp_idx} | X={pose.x:.3f}, Z={pose.z:.3f}, H={pose.heading:.3f}")
            current_phase = phase

        # Detect WP0 reached (transition into STOP_AT_LANE_END or FIRST_TURN)
        if wp_idx == 0 and phase in ("STOP_AT_LANE_END", "FIRST_TURN") and not reached_wp0:
            reached_wp0 = True
            wp0_reached_pose = RobotPose(x=pose.x, z=pose.z, heading=pose.heading)
            print(f"WP0 Reached at step {step_count}: X={pose.x:.3f}, Z={pose.z:.3f}, H={pose.heading:.3f}")
            if abs(cmd.linear_velocity) < 1e-4 and abs(cmd.angular_velocity) < 1e-4 and abs(l_vel) < 1e-4 and abs(r_vel) < 1e-4:
                wp0_stopped_verified = True
                print("WP0 Physical Stop VERIFIED: v=0, w=0, left=0, right=0")

        # Detect First Turn Complete (transition into LANE_SHIFT)
        if wp_idx == 1 and phase == "LANE_SHIFT" and first_turn_completed_pose is None:
            first_turn_completed_pose = RobotPose(x=pose.x, z=pose.z, heading=pose.heading)
            print(f"First 90° Turn Complete: X={pose.x:.3f}, Z={pose.z:.3f}, H={pose.heading:.3f}")

        # Detect WP1 Reached (transition into STOP_AT_LANE_START)
        if wp_idx == 1 and phase in ("STOP_AT_LANE_START", "SECOND_TURN") and wp1_reached_pose is None:
            wp1_reached_pose = RobotPose(x=pose.x, z=pose.z, heading=pose.heading)
            print(f"WP1 Reached: X={pose.x:.3f}, Z={pose.z:.3f}, H={pose.heading:.3f}")
            if abs(cmd.linear_velocity) < 1e-4 and abs(cmd.angular_velocity) < 1e-4 and abs(l_vel) < 1e-4 and abs(r_vel) < 1e-4:
                wp1_stopped_verified = True
                print("WP1 Physical Stop VERIFIED: v=0, w=0, left=0, right=0")

        # Detect Second Turn Complete (transition into Lane 2 mowing, wp_idx becomes 2)
        if wp_idx == 2 and phase is None and second_turn_completed_pose is None:
            second_turn_completed_pose = RobotPose(x=pose.x, z=pose.z, heading=pose.heading)
            print(f"Second 90 deg Turn Complete: X={pose.x:.3f}, Z={pose.z:.3f}, H={pose.heading:.3f}")

        # Detect WP2 Reached
        if wp_idx == 2 and phase in ("STOP_AT_LANE_END", "FIRST_TURN") and wp2_reached_pose is None:
            wp2_reached_pose = RobotPose(x=pose.x, z=pose.z, heading=pose.heading)
            print(f"WP2 Reached: X={pose.x:.3f}, Z={pose.z:.3f}, H={pose.heading:.3f}")

        # Record multiple transitions
        if phase in ("STOP_AT_LANE_END", "FIRST_TURN") and wp_idx not in transitions_logged:
            transitions_logged.append(wp_idx)

        # Apply physics step
        sim.step(l_vel, r_vel)
        step_count += 1

        # Stop after testing 5 lane transitions to evaluate results
        if len(transitions_logged) >= 5 and wp2_reached_pose is not None:
            break

    # ============================================================
    # PHYSICAL VERIFICATION TEST REPORT (TEST 1 - TEST 7)
    # ============================================================
    print("\n============================================================")
    print("PHYSICAL VERIFICATION TEST REPORT (TEST 1 - TEST 7)")
    print("============================================================")

    # ------------------------------------------------------------
    # TEST 1: WP0 ARRIVAL AND COMPLETE STOP
    # ------------------------------------------------------------
    print("\n--- TEST 1: WP0 ARRIVAL AND COMPLETE STOP ---")
    wp0_target = coverage_path.waypoints[0]
    wp0_dist = math.hypot(wp0_reached_pose.x - wp0_target.x, wp0_reached_pose.z - wp0_target.z)
    print(f"Planned WP0 Target:      X={wp0_target.x:.3f} m, Z={wp0_target.z:.3f} m")
    print(f"Actual WP0 Reached:      X={wp0_reached_pose.x:.3f} m, Z={wp0_reached_pose.z:.3f} m, Heading={wp0_reached_pose.heading:.4f} rad")
    print(f"Distance to WP0:         {wp0_dist:.4f} m (Criterion: <= 0.050 m)")
    print(f"Physical Stop at WP0:    Verified (v=0.000 m/s, w=0.000 rad/s, left=0.000 rad/s, right=0.000 rad/s)")
    assert wp0_dist <= 0.050, f"TEST 1 FAILED: Distance {wp0_dist} > 0.050 m"
    assert wp0_stopped_verified, "TEST 1 FAILED: WP0 physical stop not verified"
    print("TEST 1 RESULT: PASS")

    # ------------------------------------------------------------
    # TEST 2: WP0 FIRST TURN
    # ------------------------------------------------------------
    print("\n--- TEST 2: WP0 FIRST TURN ---")
    shift_target_h = -math.pi / 2.0  # -1.5708 rad (-90.00 deg)
    achieved_h1 = first_turn_completed_pose.heading
    signed_turn1 = heading_ctrl.normalize_angle(first_turn_completed_pose.heading - wp0_reached_pose.heading)
    turn1_angle = abs(signed_turn1)
    turn1_dir = "LEFT (Counter-Clockwise / toward -Y / WP1)" if signed_turn1 < 0 else "RIGHT (Clockwise / toward +Y)"
    h_error1 = abs(heading_ctrl.normalize_angle(shift_target_h - achieved_h1))
    trans_displacement1 = math.hypot(
        first_turn_completed_pose.x - wp0_reached_pose.x,
        first_turn_completed_pose.z - wp0_reached_pose.z
    )
    print(f"Expected Physical Direction: LEFT toward WP1 (-Y)")
    print(f"Actual Physical Direction:   {turn1_dir}")
    print(f"Expected Target Heading:     {shift_target_h:.4f} rad (-90.00 deg)")
    print(f"Actual Achieved Heading:     {achieved_h1:.4f} rad ({math.degrees(achieved_h1):.2f} deg)")
    print(f"Actual Turn Angle:           {turn1_angle:.4f} rad ({math.degrees(turn1_angle):.2f} deg)")
    print(f"Heading Error:               {h_error1:.4f} rad ({math.degrees(h_error1):.2f} deg) (Criterion: <= 0.080 rad)")
    print(f"In-place Translation:        {trans_displacement1:.4f} m (Criterion: < 0.020 m)")
    assert signed_turn1 < 0, f"TEST 2 FAILED: Turn direction is {turn1_dir}, expected LEFT (< 0)"
    assert h_error1 <= 0.080, f"TEST 2 FAILED: Heading error {h_error1} > 0.080 rad"
    assert trans_displacement1 < 0.020, f"TEST 2 FAILED: In-place translation {trans_displacement1} >= 0.020 m"
    print("TEST 2 RESULT: PASS (Numerical tolerance AND physical turn direction verified)")

    # ------------------------------------------------------------
    # TEST 3: LANE SHIFT (WP0 -> WP1)
    # ------------------------------------------------------------
    print("\n--- TEST 3: LANE SHIFT (WP0 -> WP1) ---")
    wp1_target = coverage_path.waypoints[1]
    planned_shift_dx = wp1_target.x - wp0_target.x  # 0.000 m
    planned_shift_dz = wp1_target.z - wp0_target.z  # -0.180 m
    planned_shift_dist = math.hypot(planned_shift_dx, planned_shift_dz)
    actual_shift_dx = wp1_reached_pose.x - first_turn_completed_pose.x
    actual_shift_dz = wp1_reached_pose.z - first_turn_completed_pose.z
    actual_shift_dist = math.hypot(actual_shift_dx, actual_shift_dz)
    shift_err = abs(actual_shift_dist - planned_shift_dist)
    drift_dx = abs(actual_shift_dx)
    shift_dir = "NEGATIVE lateral direction (-Z)" if actual_shift_dz < 0 else "POSITIVE lateral direction (+Z)"
    print(f"Expected Direction:      Negative lateral direction (-Z / toward -Y in Webots)")
    print(f"Actual Direction:        {shift_dir}")
    print(f"Planned Displacement:    dZ={planned_shift_dz:.4f} m (0.180 m toward negative lateral axis)")
    print(f"Actual Displacement:     dX={actual_shift_dx:.4f} m, dZ={actual_shift_dz:.4f} m, Total={actual_shift_dist:.4f} m")
    print(f"Shift Distance Error:    {shift_err:.4f} m ({shift_err*100:.2f} cm) (Criterion: < 0.050 m)")
    print(f"Lateral Drift Error dX:  {drift_dx:.4f} m ({drift_dx*100:.2f} cm) (Criterion: < 0.020 m)")
    assert actual_shift_dz < 0, f"TEST 3 FAILED: Mower moved in positive lateral direction (dZ={actual_shift_dz:.4f})"
    assert drift_dx < 0.020, f"TEST 3 FAILED: Lateral drift {drift_dx} >= 0.020 m"
    assert shift_err < 0.050, f"TEST 3 FAILED: Shift distance error {shift_err} >= 0.050 m"
    print("TEST 3 RESULT: PASS (Planned 0.180m vs actual displacement verified toward -Z)")

    # ------------------------------------------------------------
    # TEST 4: STOP AT WP1
    # ------------------------------------------------------------
    print("\n--- TEST 4: STOP AT WP1 ---")
    wp1_dist = math.hypot(wp1_reached_pose.x - wp1_target.x, wp1_reached_pose.z - wp1_target.z)
    print(f"Planned WP1 Target:      X={wp1_target.x:.3f} m, Z={wp1_target.z:.3f} m")
    print(f"Actual WP1 Reached:      X={wp1_reached_pose.x:.3f} m, Z={wp1_reached_pose.z:.3f} m, Heading={wp1_reached_pose.heading:.4f} rad")
    print(f"Distance to WP1:         {wp1_dist:.4f} m (Criterion: <= 0.050 m)")
    print(f"Physical Stop at WP1:    Verified (v=0.000 m/s, w=0.000 rad/s, left=0.000 rad/s, right=0.000 rad/s)")
    assert wp1_dist <= 0.050, f"TEST 4 FAILED: Distance {wp1_dist} > 0.050 m"
    assert wp1_stopped_verified, "TEST 4 FAILED: Robot did not execute complete physical stop at WP1"
    print("TEST 4 RESULT: PASS")

    # ------------------------------------------------------------
    # TEST 5: SECOND TURN AT WP1
    # ------------------------------------------------------------
    print("\n--- TEST 5: SECOND TURN AT WP1 ---")
    target_lane2_h = math.pi  # +3.1416 rad (facing +X)
    achieved_h2 = second_turn_completed_pose.heading
    signed_turn2 = heading_ctrl.normalize_angle(second_turn_completed_pose.heading - first_turn_completed_pose.heading)
    turn2_angle = abs(signed_turn2)
    turn2_dir = "LEFT (Counter-Clockwise / toward +X / WP2)" if signed_turn2 < 0 else "RIGHT (Clockwise)"
    h_error2 = abs(heading_ctrl.normalize_angle(target_lane2_h - achieved_h2))
    trans_displacement2 = math.hypot(
        second_turn_completed_pose.x - wp1_reached_pose.x,
        second_turn_completed_pose.z - wp1_reached_pose.z
    )
    # Check that robot physically points toward +X: forward vector fx = -cos(H) > 0.99
    fx_pointing = -math.cos(achieved_h2)
    print(f"Expected Direction:      Physically point toward WP2 (+X)")
    print(f"Actual Direction:        {turn2_dir} -> forward vector fx={fx_pointing:.4f} (faces +X)")
    print(f"Planned Target Heading:  {target_lane2_h:.4f} rad (+180.00 deg / facing +X)")
    print(f"Actual Achieved Heading: {achieved_h2:.4f} rad ({math.degrees(achieved_h2):.2f} deg)")
    print(f"Actual Turn Angle:       {turn2_angle:.4f} rad ({math.degrees(turn2_angle):.2f} deg)")
    print(f"Heading Error:           {h_error2:.4f} rad ({math.degrees(h_error2):.2f} deg) (Criterion: <= 0.080 rad)")
    print(f"In-place Translation:    {trans_displacement2:.4f} m (Criterion: < 0.020 m)")
    assert fx_pointing > 0.99, f"TEST 5 FAILED: Mower does not face +X (fx={fx_pointing})"
    assert h_error2 <= 0.080, f"TEST 5 FAILED: Heading error {h_error2} > 0.080 rad"
    assert trans_displacement2 < 0.020, f"TEST 5 FAILED: Translation {trans_displacement2} >= 0.020 m"
    print("TEST 5 RESULT: PASS (Numerical tolerance AND physical pointing toward WP2 / +X verified)")

    # ------------------------------------------------------------
    # TEST 6: WP1 -> WP2 STRAIGHT DRIVE
    # ------------------------------------------------------------
    print("\n--- TEST 6: WP1 -> WP2 STRAIGHT DRIVE (LANE 2) ---")
    wp2_target = coverage_path.waypoints[2]
    planned_lane2_dx = wp2_target.x - wp1_target.x  # +18.860 m
    actual_lane2_dx = wp2_reached_pose.x - second_turn_completed_pose.x
    lane2_drift_from_line = abs(wp2_reached_pose.z - wp2_target.z)
    wp2_dist = math.hypot(wp2_reached_pose.x - wp2_target.x, wp2_reached_pose.z - wp2_target.z)
    print(f"Planned Longitudinal Progress: {planned_lane2_dx:.3f} m along +X")
    print(f"Actual Longitudinal Progress:  {actual_lane2_dx:.3f} m along +X")
    print(f"Planned Line Coordinate:       Z={wp2_target.z:.3f} m")
    print(f"Actual Coordinate at WP2:      X={wp2_reached_pose.x:.3f} m, Z={wp2_reached_pose.z:.3f} m")
    print(f"Lateral Drift Error from Line: {lane2_drift_from_line:.4f} m ({lane2_drift_from_line*100:.2f} cm) (Criterion: < 0.050 m)")
    print(f"Distance Error to WP2:         {wp2_dist:.4f} m ({wp2_dist*100:.2f} cm) (Criterion: <= 0.050 m)")
    assert actual_lane2_dx > 18.0, f"TEST 6 FAILED: Progress along +X insufficient ({actual_lane2_dx:.3f} m)"
    assert wp2_dist <= 0.050, f"TEST 6 FAILED: Distance to WP2 {wp2_dist} > 0.050 m"
    assert lane2_drift_from_line < 0.050, f"TEST 6 FAILED: Lateral drift {lane2_drift_from_line} >= 0.050 m"
    print("TEST 6 RESULT: PASS (Straight drive along Lane 2 with minimal drift verified)")

    # ------------------------------------------------------------
    # TEST 7: AT LEAST 4 COMPLETE LANE TRANSITIONS
    # ------------------------------------------------------------
    print("\n--- TEST 7: AT LEAST 4 COMPLETE LANE TRANSITIONS ---")
    print(f"Planned Transitions: At least 4 consecutive lane transitions")
    print(f"Actual Transitions Logged: Waypoints {transitions_logged}")
    print(f"Number of Completed Transitions: {len(transitions_logged)}")
    for i, wp_t in enumerate(transitions_logged):
        print(f"  Transition {i+1}: at Waypoint {wp_t}")
    assert len(transitions_logged) >= 4, f"TEST 7 FAILED: Only {len(transitions_logged)} transitions logged (required >= 4)"
    print("TEST 7 RESULT: PASS (At least 4 consecutive lane transitions physically executed and verified)")

    print("\n============================================================")
    print("ALL TESTS (TEST 1 THROUGH TEST 7) PASSED WITH 100% SUCCESS!")
    print("PHYSICAL TURN DIRECTIONS AND TOLERANCES FULLY VERIFIED.")
    print("============================================================")

def test_physical_lane_transition():
    run_physical_verification()


if __name__ == "__main__":
    run_physical_verification()

