import math
import sys
import os

# Add raspberry_pi/src to python path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
raspberry_pi_src = os.path.join(project_root, "raspberry_pi", "src")
if raspberry_pi_src not in sys.path:
    sys.path.insert(0, raspberry_pi_src)

from navigation.navigation_types import RobotPose
from navigation.navigator import Navigator
from navigation.differential_drive import DifferentialDriveController
from navigation.heading_controller import HeadingController
from navigation.speed_controller import SpeedController
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
        v_left = -left_motor_vel * self.wheel_radius
        v_right = -right_motor_vel * self.wheel_radius

        v = (v_left + v_right) / 2.0
        omega = (v_left - v_right) / self.wheel_track

        self.x -= v * math.cos(self.heading) * self.dt
        self.z -= v * math.sin(self.heading) * self.dt
        self.heading += omega * self.dt

        while self.heading > math.pi:
            self.heading -= 2.0 * math.pi
        while self.heading < -math.pi:
            self.heading += 2.0 * math.pi

    def get_pose(self):
        return RobotPose(x=self.x, z=self.z, heading=self.heading)


def test_continuous_coverage_behavior():
    """
    Verify exact blade-based continuous coverage:
    1. BLADE_WIDTH = 0.300m, OVERLAP = 0.0 -> LANE_SPACING = 0.300m
    2. Lane spacing is exactly 0.300m within tight engineering tolerance (<= 0.005m)
    3. Turns are exact 90 degrees (89.5° to 90.5°, tolerance <= 0.5°)
    4. Parallel lanes without sideways drift (cross-track error <= 0.008m)
    5. Pure in-place rotation during turns (linear_velocity == 0)
    6. No diagonal drift during lane shifts
    7. Alternating lane directions (-X, +X, -X, +X)
    8. Full telemetry logged for every lane transition
    """
    BLADE_WIDTH = 0.300
    OVERLAP = 0.0
    EXPECTED_LANE_SPACING = BLADE_WIDTH * (1.0 - OVERLAP)  # 0.300 m

    drive_ctrl = DifferentialDriveController(wheel_radius=0.10, wheel_track=0.44)
    heading_ctrl = HeadingController(
        max_angular_speed=0.80,
        heading_tolerance=0.008,  # tight engineering tolerance (~0.46 deg)
        heading_kp=1.50,
        min_angular_speed=0.06,
    )
    speed_ctrl = SpeedController(
        max_speed=0.50,
        min_speed=0.10,
        slowdown_distance=0.60,
        position_tolerance=0.01,
    )

    nav = Navigator(
        differential_drive=drive_ctrl,
        heading_controller=heading_ctrl,
        speed_controller=speed_ctrl,
        position_tolerance=0.01,
    )

    # Verify NO waypoint list is maintained
    assert not hasattr(nav, "waypoints") or nav.waypoints is None, "Navigator must not have waypoint list"

    # Configure bounds with exact blade parameters
    nav.configure_coverage(
        min_x=-9.75,
        max_x=9.75,
        min_z=-9.75,
        max_z=9.75,
        blade_width=BLADE_WIDTH,
        overlap=OVERLAP,
        clearance=0.32,
    )

    assert abs(nav.lane_spacing - 0.300) < 1e-6, f"Lane spacing must be 0.300m, got {nav.lane_spacing}"

    # Initial physical position in Webots world
    sim = PhysicalRobotSimulator(x=9.50, z=-9.50, heading=0.0, dt=0.016)
    start_pose = sim.get_pose()
    nav.start(start_pose)

    lane_records = []
    current_lane_idx = nav.get_lane_index()
    lane_start_x = sim.x
    lane_z_samples = [sim.z]
    turn_telemetries = []

    step = 0
    max_steps = 150000

    print("\n--- Starting Exact Blade-Based Coverage Simulation ---")
    while step < max_steps and len(lane_records) < 4:
        pose = sim.get_pose()
        cmd = nav.update(pose)
        state = nav.get_state()

        # In-place turn verification: linear velocity MUST be zero during rotations
        if state in (NavigationState.TURN_TO_SHIFT, NavigationState.TURN_TO_LANE):
            assert abs(cmd.linear_velocity) < 1e-4, f"Linear velocity must be 0 during turn state {state}"

        l_vel, r_vel = drive_ctrl.calculate_wheel_velocities(cmd.linear_velocity, cmd.angular_velocity)
        sim.step(l_vel, r_vel)

        # Collect events / telemetry
        for evt in nav.pop_events():
            if "TELEMETRY" in evt:
                turn_telemetries.append(evt)
                print(f"[EVENT] {evt}")

        # Detect lane transition
        new_lane_idx = nav.get_lane_index()
        if new_lane_idx != current_lane_idx:
            avg_z = sum(lane_z_samples) / len(lane_z_samples)
            lane_dir = "-X" if current_lane_idx % 2 == 0 else "+X"
            lane_records.append({
                "lane_index": current_lane_idx,
                "direction": lane_dir,
                "start_x": lane_start_x,
                "end_x": pose.x,
                "avg_z": avg_z,
                "z_samples": list(lane_z_samples),
            })
            print(f"LANE {current_lane_idx} FINISHED: dir={lane_dir}, X: {lane_start_x:.2f} -> {pose.x:.2f}, Z={avg_z:.3f}")
            current_lane_idx = new_lane_idx
            lane_start_x = pose.x
            lane_z_samples = [pose.z]
        else:
            if state == NavigationState.DRIVE_LANE:
                lane_z_samples.append(pose.z)

        step += 1

    print(f"\nCompleted {len(lane_records)} lanes in {step} steps ({step * 0.016:.1f} s).")
    for r in lane_records:
        print(f"  Lane {r['lane_index']}: Dir={r['direction']}, StartX={r['start_x']:.2f}, EndX={r['end_x']:.2f}, AvgZ={r['avg_z']:.3f}")

    assert len(lane_records) >= 4, f"Expected at least 4 completed lanes, got {len(lane_records)}"

    # 1. Lane direction alternates (-X, +X, -X, +X)
    assert lane_records[0]["direction"] == "-X", f"Lane 0 must be -X, got {lane_records[0]['direction']}"
    assert lane_records[1]["direction"] == "+X", f"Lane 1 must be +X, got {lane_records[1]['direction']}"
    assert lane_records[2]["direction"] == "-X", f"Lane 2 must be -X, got {lane_records[2]['direction']}"
    assert lane_records[3]["direction"] == "+X", f"Lane 3 must be +X, got {lane_records[3]['direction']}"
    print("PASS: Requirement 1 - Lane direction alternates correctly (-X -> +X -> -X -> +X).")

    # 2. Exact lane spacing = 0.300 m (tight tolerance <= 0.005 m)
    z0 = lane_records[0]["avg_z"]
    z1 = lane_records[1]["avg_z"]
    z2 = lane_records[2]["avg_z"]
    z3 = lane_records[3]["avg_z"]
    dz1 = z1 - z0
    dz2 = z2 - z1
    dz3 = z3 - z2
    print(f"\nExact Spacing Verification (Target: 0.300 m):")
    print(f"  Lane 0->1: {dz1:.4f} m (Error: {abs(dz1 - 0.300)*1000:.1f} mm)")
    print(f"  Lane 1->2: {dz2:.4f} m (Error: {abs(dz2 - 0.300)*1000:.1f} mm)")
    print(f"  Lane 2->3: {dz3:.4f} m (Error: {abs(dz3 - 0.300)*1000:.1f} mm)")

    assert abs(dz1 - 0.300) <= 0.005, f"Lane 0->1 spacing error: expected 0.300m, got {dz1:.4f}m"
    assert abs(dz2 - 0.300) <= 0.005, f"Lane 1->2 spacing error: expected 0.300m, got {dz2:.4f}m"
    assert abs(dz3 - 0.300) <= 0.005, f"Lane 2->3 spacing error: expected 0.300m, got {dz3:.4f}m"
    print("PASS: Requirement 2 - Consecutive lane spacing is exactly 0.300 m within tight tolerance (<= 0.005 m).")

    # 3. Parallel lanes: verify cross-track straightness (no sideways drift)
    for r in lane_records:
        target_z = -9.500 + r["lane_index"] * 0.300
        max_cte = max(abs(z - target_z) for z in r["z_samples"])
        print(f"  Lane {r['lane_index']} Max Cross-Track Error: {max_cte * 1000:.2f} mm")
        assert max_cte <= 0.008, f"Lane {r['lane_index']} cross-track error {max_cte} > 0.008 m"
    print("PASS: Requirement 3 - Mowing lanes are strictly parallel with cross-track error <= 8 mm.")

    # 4. Exact 90-degree turns from telemetry
    assert len(turn_telemetries) >= 3, "Expected telemetry logged for at least 3 transitions"
    for tel in turn_telemetries:
        # Extract TURN_ANGLE and HEADING_ERROR
        for part in tel.split("|"):
            part = part.strip()
            if part.startswith("TURN_ANGLE="):
                angle_deg = float(part.split("=")[1].replace("°", ""))
                print(f"  Logged Turn Angle: {angle_deg:.2f}° (Criterion: 89.50° to 90.50°)")
                assert 89.50 <= angle_deg <= 90.50, f"Turn angle {angle_deg}° not within 90° ± 0.5°"
            if part.startswith("HEADING_ERROR="):
                err_deg = float(part.split("=")[1].replace("°", ""))
                assert err_deg <= 0.50, f"Heading error {err_deg}° exceeds 0.50°"
    print("PASS: Requirement 4 - All turns are geometrically exact 90.0° ± 0.5° (no loose 85°/87° turns).")

    # 5. X boundary triggers deceleration and safe stop (safe_min_x=-9.43, safe_max_x=+9.43)
    safe_min_x = -9.75 + 0.32  # -9.43
    safe_max_x = 9.75 - 0.32   # +9.43
    assert abs(lane_records[0]["end_x"] - safe_min_x) <= 0.02, "Boundary stop X failed at min_x"
    assert abs(lane_records[1]["end_x"] - safe_max_x) <= 0.02, "Boundary stop X failed at max_x"
    print("PASS: Requirement 5 - Safe X boundary triggers deceleration and precision stop.")


if __name__ == "__main__":
    test_continuous_coverage_behavior()
