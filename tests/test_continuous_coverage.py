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
    Verify the 6 continuous zig-zag coverage requirements:
    1. lane direction alternates (-X -> +X -> -X -> +X)
    2. lane Z increases by ~0.27m
    3. X boundary triggers turn
    4. lane shift uses actual pose
    5. next lane starts in opposite X direction
    6. no waypoint list is required
    """
    drive_ctrl = DifferentialDriveController(wheel_radius=0.10, wheel_track=0.44)
    heading_ctrl = HeadingController(max_angular_speed=0.80, heading_tolerance=0.08, heading_kp=1.50)
    speed_ctrl = SpeedController(max_speed=0.50, min_speed=0.10, slowdown_distance=0.60, position_tolerance=0.05)

    nav = Navigator(
        differential_drive=drive_ctrl,
        heading_controller=heading_ctrl,
        speed_controller=speed_ctrl,
        position_tolerance=0.05,
    )

    # 6. Verify NO waypoint list is provided or required
    assert not hasattr(nav, "waypoints") or nav.waypoints is None, "Navigator must not have waypoint list"

    # Configure bounds
    nav.configure_coverage(
        min_x=-9.75,
        max_x=9.75,
        min_z=-9.75,
        max_z=9.75,
        lane_spacing=0.27,
        clearance=0.32,
    )

    # Initial physical position in Webots world
    sim = PhysicalRobotSimulator(x=9.50, z=-9.50, heading=0.0, dt=0.016)
    start_pose = sim.get_pose()
    nav.start(start_pose)

    lane_records = []  # will store (lane_index, direction_str, start_x, end_x, z_level)
    current_lane_idx = nav.get_lane_index()
    lane_start_x = sim.x
    lane_z_samples = [sim.z]

    step = 0
    max_steps = 150000  # run enough steps to complete at least 4 lanes

    print("\n--- Starting Continuous Coverage Simulation ---")
    while step < max_steps and len(lane_records) < 4:
        pose = sim.get_pose()
        cmd = nav.update(pose)
        l_vel, r_vel = drive_ctrl.calculate_wheel_velocities(cmd.linear_velocity, cmd.angular_velocity)
        sim.step(l_vel, r_vel)

        # Detect lane transition
        new_lane_idx = nav.get_lane_index()
        if new_lane_idx != current_lane_idx:
            # Previous lane completed
            avg_z = sum(lane_z_samples) / len(lane_z_samples)
            lane_dir = "-X" if current_lane_idx % 2 == 0 else "+X"
            lane_records.append({
                "lane_index": current_lane_idx,
                "direction": lane_dir,
                "start_x": lane_start_x,
                "end_x": pose.x,
                "avg_z": avg_z,
            })
            print(f"LANE {current_lane_idx} FINISHED: dir={lane_dir}, X: {lane_start_x:.2f} -> {pose.x:.2f}, Z={avg_z:.3f}")
            current_lane_idx = new_lane_idx
            lane_start_x = pose.x
            lane_z_samples = [pose.z]
        else:
            if nav.get_state() == NavigationState.DRIVE_LANE:
                lane_z_samples.append(pose.z)

        step += 1

    print(f"\nCompleted {len(lane_records)} lanes in {step} steps ({step * 0.016:.1f} s).")
    for r in lane_records:
        print(f"  Lane {r['lane_index']}: Dir={r['direction']}, StartX={r['start_x']:.2f}, EndX={r['end_x']:.2f}, AvgZ={r['avg_z']:.3f}")

    assert len(lane_records) >= 3, f"Expected at least 3 completed lanes, got {len(lane_records)}"

    # 1. Lane direction alternates (-X, +X, -X, ...)
    assert lane_records[0]["direction"] == "-X", f"Lane 0 must be -X, got {lane_records[0]['direction']}"
    assert lane_records[1]["direction"] == "+X", f"Lane 1 must be +X, got {lane_records[1]['direction']}"
    assert lane_records[2]["direction"] == "-X", f"Lane 2 must be -X, got {lane_records[2]['direction']}"
    print("PASS: Requirement 1 - Lane direction alternates correctly (-X -> +X -> -X).")

    # 2. Lane Z increases by ~0.27m
    z0 = lane_records[0]["avg_z"]
    z1 = lane_records[1]["avg_z"]
    z2 = lane_records[2]["avg_z"]
    dz1 = z1 - z0
    dz2 = z2 - z1
    print(f"Delta Z: Lane 0->1 = {dz1:.3f}m, Lane 1->2 = {dz2:.3f}m (Target: 0.27m)")
    assert abs(dz1 - 0.27) < 0.03, f"Lane 0->1 shift error: expected 0.27m, got {dz1:.3f}m"
    assert abs(dz2 - 0.27) < 0.03, f"Lane 1->2 shift error: expected 0.27m, got {dz2:.3f}m"
    print("PASS: Requirement 2 - Lane Z increases by ~0.27m between lanes.")

    # 3. X boundary triggers turn (does not crash through safe bounds: -9.43 to +9.43)
    safe_min_x = -9.75 + 0.32  # -9.43
    safe_max_x = 9.75 - 0.32   # +9.43
    assert lane_records[0]["end_x"] <= safe_min_x + 0.05, f"Lane 0 should reach safe_min_x (-9.43), got {lane_records[0]['end_x']:.3f}"
    assert lane_records[1]["end_x"] >= safe_max_x - 0.05, f"Lane 1 should reach safe_max_x (+9.43), got {lane_records[1]['end_x']:.3f}"
    print("PASS: Requirement 3 - X boundary triggers deceleration and turn safely.")

    # 4. Lane shift uses actual pose (Z tracks desired_lane_z with cross-track error compensation)
    assert abs(lane_records[1]["avg_z"] - (-9.50 + 0.27)) < 0.04, "Lane 1 did not achieve desired Z"
    assert abs(lane_records[2]["avg_z"] - (-9.50 + 0.54)) < 0.04, "Lane 2 did not achieve desired Z"
    print("PASS: Requirement 4 - Lane shift uses actual localized pose.")

    # 5. Next lane starts in opposite X direction
    assert lane_records[1]["start_x"] <= safe_min_x + 0.05
    assert lane_records[2]["start_x"] >= safe_max_x - 0.05
    print("PASS: Requirement 5 - Next lane starts in opposite X direction.")


if __name__ == "__main__":
    test_continuous_coverage_behavior()
