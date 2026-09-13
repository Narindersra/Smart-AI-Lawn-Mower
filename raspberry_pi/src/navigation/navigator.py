import math

from .navigation_types import (
    RobotPose,
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
    Continuous behavior-based coverage navigation controller.

    Operates directly on localized pose (X, Z, heading) and lawn boundaries,
    generating a continuous, stable boustrophedon pattern without waypoint lists.
    """

    def __init__(
        self,
        differential_drive,
        heading_controller=None,
        speed_controller=None,
        position_tolerance=0.05,
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
            else SpeedController(position_tolerance=position_tolerance)
        )
        self.position_tolerance = float(position_tolerance)

        self.state_machine = NavigationStateMachine()

        # Coverage geometry parameters derived from blade width
        self.min_x = -9.75
        self.max_x = 9.75
        self.min_z = -9.75
        self.max_z = 9.75
        self.blade_width = 0.30
        self.overlap = 0.0
        self.lane_spacing = self.blade_width * (1.0 - self.overlap)  # exactly 0.300 m
        self.clearance = 0.32
        self.slowdown_distance = 0.60

        # Pure Pursuit geometric path tracking parameters
        # Derived from physical wheel track T = 0.44 m
        self.lookahead_distance = 0.44
        self.max_curvature = 2.5  # rad/m
        self.max_angular_accel = 5.0  # rad/s^2
        self.dt = 0.016
        self.prev_omega = 0.0

        # Coverage quality metrics
        self.max_cross_track_error = 0.0
        self.sum_cross_track_error = 0.0
        self.count_cross_track_error = 0

        # Current lane tracking
        self.lane_index = 0
        self.lane_direction = -1  # -1 for -X, +1 for +X
        self.previous_lane_z = None
        self.desired_lane_z = None
        self.turn_target_heading = None
        self.turn_start_heading = None
        self.turn_phase = "NONE"
        self.actual_spacing = 0.0

        # Complete stop hold logic
        self.stop_hold_counter = 0
        self.stop_hold_cycles = 5

        # Event emission queue
        self._events = []

        # Legacy compatibility attributes
        self.path = None
        self.current_waypoint_index = 0

    def calculate_lookahead_point(self, p1, p2, robot_pos, lookahead_distance=None):
        """
        Calculate forward lookahead point on segment p1 -> p2 using quadratic
        circle-line segment intersection:
            ||P(t) - robot_pos||^2 = L_d^2,  where P(t) = P1 + t * (P2 - P1)
        Returns (x_look, z_look).
        """
        if lookahead_distance is None:
            lookahead_distance = self.lookahead_distance

        x1, z1 = p1
        x2, z2 = p2
        xr, zr = robot_pos
        dx = x2 - x1
        dz = z2 - z1
        seg_len_sq = dx * dx + dz * dz
        if seg_len_sq < 1e-9:
            return (x2, z2)

        # Vector from p1 to robot
        fx = x1 - xr
        fz = z1 - zr

        a = seg_len_sq
        b = 2.0 * (fx * dx + fz * dz)
        c = (fx * fx + fz * fz) - (lookahead_distance * lookahead_distance)

        discriminant = b * b - 4.0 * a * c
        if discriminant >= 0:
            sqrt_disc = math.sqrt(discriminant)
            t1 = (-b - sqrt_disc) / (2.0 * a)
            t2 = (-b + sqrt_disc) / (2.0 * a)

            # Choose forward intersection along segment
            if 0.0 <= t2 <= 1.0:
                return (x1 + t2 * dx, z1 + t2 * dz)
            elif 0.0 <= t1 <= 1.0 and t2 > 1.0:
                return (x2, z2)
            elif t2 > 1.0:
                return (x2, z2)

        # If circle does not intersect or robot is beyond segment,
        # project robot onto segment and look ahead along segment direction
        proj_t = max(0.0, min(1.0, -(fx * dx + fz * dz) / seg_len_sq))
        seg_len = math.sqrt(seg_len_sq)
        look_t = min(1.0, proj_t + (lookahead_distance / seg_len))
        return (x1 + look_t * dx, z1 + look_t * dz)

    def transform_to_robot_frame(self, target_pos, robot_pose):
        """
        Transform target point (x, z) into mower's local coordinate frame.
        Validated Webots chassis: Front = -X, Heading = 0 faces -X.
        Returns:
            x_fwd > 0: ahead of mower
            y_lat > 0: to mower's left
        """
        tx, tz = target_pos
        dx = tx - robot_pose.x
        dz = tz - robot_pose.z
        cos_h = math.cos(robot_pose.heading)
        sin_h = math.sin(robot_pose.heading)

        x_fwd = -(dx * cos_h + dz * sin_h)
        y_lat = dx * sin_h - dz * cos_h
        return x_fwd, y_lat

    def calculate_pure_pursuit_curvature(self, target_pos, robot_pose):
        """
        Calculate Pure Pursuit curvature: kappa = 2 * y_lat / L_d^2
        """
        x_fwd, y_lat = self.transform_to_robot_frame(target_pos, robot_pose)
        ld_actual = math.hypot(x_fwd, y_lat)
        if ld_actual < 1e-4:
            return 0.0, x_fwd, y_lat, 0.0
        kappa = (2.0 * y_lat) / (ld_actual * ld_actual)
        return kappa, x_fwd, y_lat, ld_actual

    def calculate_pure_pursuit_command(
        self,
        p1,
        p2,
        robot_pose,
        base_speed,
        lookahead_distance=None,
    ):
        """
        Continuous Pure Pursuit tracking on segment p1 -> p2:
        1. Find lookahead point via quadratic circle-segment intersection
        2. Transform to robot frame and compute curvature kappa = 2*y_l / Ld^2
        3. Modulate speed based on curvature
        4. Target heading combines segment tangent with cross-track lookahead correction,
           preventing endpoint singularity as robot approaches segment terminus
        5. Apply steering rate limiting
        """
        if lookahead_distance is None:
            lookahead_distance = self.lookahead_distance

        target_pt = self.calculate_lookahead_point(
            p1,
            p2,
            (robot_pose.x, robot_pose.z),
            lookahead_distance=lookahead_distance,
        )
        kappa, x_fwd, y_lat, ld_actual = self.calculate_pure_pursuit_curvature(
            target_pt,
            robot_pose,
        )

        # Smooth speed reduction as curvature increases
        curvature_factor = max(
            0.35,
            1.0 - min(1.0, abs(kappa) / self.max_curvature),
        )
        speed = max(
            self.speed_controller.min_speed,
            base_speed * curvature_factor,
        )

        # Segment tangent in validated Webots coordinates
        dx_seg = p2[0] - p1[0]
        dz_seg = p2[1] - p1[1]
        seg_heading = math.atan2(-dz_seg, -dx_seg)

        # Cross-track error to segment line
        seg_len = math.hypot(dx_seg, dz_seg)
        if seg_len > 1e-6:
            nx = -dz_seg / seg_len
            nz = dx_seg / seg_len
            cte = (robot_pose.x - p1[0]) * nx + (robot_pose.z - p1[1]) * nz
            # Lookahead correction angle
            correction = math.atan2(cte, lookahead_distance)
            target_h = self.heading_controller.normalize_angle(seg_heading - correction)
        else:
            dx = target_pt[0] - robot_pose.x
            dz = target_pt[1] - robot_pose.z
            target_h = math.atan2(-dz, -dx)

        heading_error = self.heading_controller.normalize_angle(
            target_h - robot_pose.heading
        )

        raw_omega = self.heading_controller.calculate_angular_velocity(
            heading_error
        )

        # Rate limiting to prevent angular acceleration jerk
        max_d_omega = self.max_angular_accel * self.dt
        d_omega = raw_omega - self.prev_omega
        if abs(d_omega) > max_d_omega:
            omega = self.prev_omega + math.copysign(max_d_omega, d_omega)
        else:
            omega = raw_omega
        self.prev_omega = omega

        return MotionCommand(linear_velocity=speed, angular_velocity=omega)

    def configure_coverage(
        self,
        min_x=-9.75,
        max_x=9.75,
        min_z=-9.75,
        max_z=9.75,
        blade_width=0.30,
        overlap=0.0,
        lane_spacing=None,
        clearance=0.32,
        slowdown_distance=0.60,
    ):
        """Configure coverage boundaries and geometry mathematically from blade width."""
        self.min_x = float(min_x)
        self.max_x = float(max_x)
        self.min_z = float(min_z)
        self.max_z = float(max_z)
        self.blade_width = float(blade_width)
        self.overlap = float(overlap)
        if lane_spacing is not None:
            self.lane_spacing = float(lane_spacing)
        else:
            self.lane_spacing = self.blade_width * (1.0 - self.overlap)
        self.clearance = float(clearance)
        self.slowdown_distance = float(slowdown_distance)

    @property
    def safe_min_x(self):
        return self.min_x + self.clearance

    @property
    def safe_max_x(self):
        return self.max_x - self.clearance

    def _emit_event(self, event_str):
        self._events.append(event_str)

    def pop_events(self):
        events = list(self._events)
        self._events.clear()
        return events

    def get_state(self):
        return self.state_machine.get_state()

    def get_lane_index(self):
        return self.lane_index

    def get_lane_direction_str(self):
        return "-X" if self.lane_direction == -1 else "+X"

    def get_desired_lane_z(self):
        return self.desired_lane_z

    def get_turn_phase(self):
        return self.turn_phase

    def get_target_heading(self):
        if self.turn_target_heading is not None:
            return self.turn_target_heading
        # In lane driving, return nominal lane heading
        if self.lane_direction == -1:
            return 0.0
        return math.pi

    # Legacy compatibility methods
    def is_complete(self):
        return self.state_machine.get_state() == NavigationState.COVERAGE_COMPLETE

    def stop(self):
        self.state_machine.set_state(NavigationState.STOPPED)
        return MotionCommand(linear_velocity=0.0, angular_velocity=0.0)

    def get_current_waypoint_index(self):
        return self.lane_index

    def get_current_waypoint(self):
        return None

    def set_path(self, path=None, start_pose=None):
        """Legacy compatibility method; sets initial starting pose if provided."""
        if start_pose is not None:
            self.start(start_pose)

    @property
    def lane_transition_phase(self):
        state = self.state_machine.get_state()
        if state in (
            NavigationState.STOP_AT_BOUNDARY,
            NavigationState.TURN_TO_SHIFT,
            NavigationState.SHIFT_LANE,
            NavigationState.STOP_AT_SHIFT,
            NavigationState.TURN_TO_LANE,
        ):
            return state.name
        return None

    def start(self, start_pose=None):
        """Start coverage navigation from robot's initial pose."""
        if start_pose is not None:
            self.desired_lane_z = float(start_pose.z)
            self.previous_lane_z = float(start_pose.z)
            # Determine initial mowing direction toward the farthest safe boundary
            dist_to_min = abs(start_pose.x - self.safe_min_x)
            dist_to_max = abs(self.safe_max_x - start_pose.x)
            self.lane_direction = -1 if dist_to_min >= dist_to_max else +1
        else:
            self.desired_lane_z = self.min_z + self.clearance
            self.previous_lane_z = self.desired_lane_z
            self.lane_direction = -1

        self.lane_index = 0
        self.turn_target_heading = None
        self.turn_start_heading = None
        self.turn_phase = "NONE"
        self.actual_spacing = 0.0
        self.stop_hold_counter = 0

        self.state_machine.set_state(NavigationState.DRIVE_LANE)
        self._emit_event(
            f"DRIVE_LANE index={self.lane_index} dir={self.get_lane_direction_str()}"
        )

    def update(self, pose):
        """
        Execute behavior-based navigation directly from actual localized pose.
        """
        state = self.state_machine.get_state()

        if state == NavigationState.IDLE:
            self.start(pose)
            state = self.state_machine.get_state()

        # ========================================================
        # 1. DRIVE_LANE: Pure-Pursuit continuous geometric lane tracking
        # ========================================================
        if state == NavigationState.DRIVE_LANE:
            if self.lane_direction == -1:
                p_start = (self.safe_max_x, self.desired_lane_z)
                p_end = (self.safe_min_x, self.desired_lane_z)
                dist_to_boundary = pose.x - self.safe_min_x
            else:
                p_start = (self.safe_min_x, self.desired_lane_z)
                p_end = (self.safe_max_x, self.desired_lane_z)
                dist_to_boundary = self.safe_max_x - pose.x

            cmd = self.calculate_pure_pursuit_command(
                p_start,
                p_end,
                pose,
                base_speed=self.speed_controller.max_speed,
            )

            # Track cross-track error metrics
            lane_error = abs(pose.z - self.desired_lane_z)
            self.max_cross_track_error = max(self.max_cross_track_error, lane_error)
            self.sum_cross_track_error += lane_error
            self.count_cross_track_error += 1

            if dist_to_boundary <= self.slowdown_distance:
                self.state_machine.set_state(NavigationState.APPROACH_BOUNDARY)
                self._emit_event("APPROACH_BOUNDARY")

            return cmd

        # ========================================================
        # 2. APPROACH_BOUNDARY: Controlled deceleration to boundary
        # ========================================================
        if state == NavigationState.APPROACH_BOUNDARY:
            if self.lane_direction == -1:
                p_start = (self.safe_max_x, self.desired_lane_z)
                p_end = (self.safe_min_x, self.desired_lane_z)
                dist_to_boundary = pose.x - self.safe_min_x
                boundary_reached = (
                    pose.x <= self.safe_min_x or dist_to_boundary <= 0.01
                )
            else:
                p_start = (self.safe_min_x, self.desired_lane_z)
                p_end = (self.safe_max_x, self.desired_lane_z)
                dist_to_boundary = self.safe_max_x - pose.x
                boundary_reached = (
                    pose.x >= self.safe_max_x or dist_to_boundary <= 0.01
                )

            # Smooth deceleration ramp
            fraction = max(
                0.0, min(1.0, dist_to_boundary / self.slowdown_distance)
            )
            speed = self.speed_controller.min_speed + fraction * (
                self.speed_controller.max_speed
                - self.speed_controller.min_speed
            )

            cmd = self.calculate_pure_pursuit_command(
                p_start,
                p_end,
                pose,
                base_speed=speed,
            )

            lane_error = abs(pose.z - self.desired_lane_z)
            self.max_cross_track_error = max(self.max_cross_track_error, lane_error)

            if boundary_reached:
                self.state_machine.set_state(NavigationState.STOP_AT_BOUNDARY)
                self.stop_hold_counter = 0
                self.prev_omega = 0.0
                self._emit_event("BOUNDARY_REACHED")
                self._emit_event("STOP")
                return MotionCommand(
                    linear_velocity=0.0, angular_velocity=0.0
                )

            return cmd

        # ========================================================
        # 3. STOP_AT_BOUNDARY: Complete physical stop
        # ========================================================
        if state == NavigationState.STOP_AT_BOUNDARY:
            self.stop_hold_counter += 1
            if self.stop_hold_counter >= self.stop_hold_cycles:
                # Check if entire lawn area is covered
                max_usable_z = self.max_z - self.clearance
                next_z = self.desired_lane_z + self.lane_spacing
                if next_z > max_usable_z:
                    if abs(self.desired_lane_z - max_usable_z) < 0.05:
                        self.state_machine.set_state(
                            NavigationState.COVERAGE_COMPLETE
                        )
                        self._emit_event("COVERAGE_COMPLETE")
                        return MotionCommand(
                            linear_velocity=0.0, angular_velocity=0.0
                        )
                    else:
                        next_z = max_usable_z

                self.previous_lane_z = self.desired_lane_z
                self.desired_lane_z = next_z
                self.shift_x = pose.x
                # Target shift heading pointing along +Z: atan2(-1, 0) = -pi/2
                self.turn_target_heading = -math.pi / 2.0
                self.turn_start_heading = pose.heading
                self.turn_phase = "TURN_TO_SHIFT"
                self.state_machine.set_state(NavigationState.TURN_TO_SHIFT)
                self._emit_event("TURN_TO_SHIFT")

            return MotionCommand(
                linear_velocity=0.0, angular_velocity=0.0
            )

        # ========================================================
        # 4. TURN_TO_SHIFT: 90-degree in-place rotation toward +Z
        # ========================================================
        if state == NavigationState.TURN_TO_SHIFT:
            heading_error = self.heading_controller.normalize_angle(
                self.turn_target_heading - pose.heading
            )
            if abs(heading_error) <= self.heading_controller.heading_tolerance:
                self.turn1_angle = abs(
                    self.heading_controller.normalize_angle(
                        pose.heading - self.turn_start_heading
                    )
                )
                self.turn_target_heading = None
                self.turn_phase = "SHIFT"
                self.state_machine.set_state(NavigationState.SHIFT_LANE)
                self._emit_event("SHIFT_LANE")
                return MotionCommand(
                    linear_velocity=0.0, angular_velocity=0.0
                )

            omega = self.heading_controller.calculate_angular_velocity(
                heading_error, in_turn=True
            )
            # Pure in-place rotation: linear velocity MUST be zero
            return MotionCommand(linear_velocity=0.0, angular_velocity=omega)

        # ========================================================
        # 5. SHIFT_LANE: Lateral shift along +Z with zero X drift
        # ========================================================
        if state == NavigationState.SHIFT_LANE:
            dz_remaining = self.desired_lane_z - pose.z
            bound_x = (
                self.safe_min_x
                if self.lane_direction == -1
                else self.safe_max_x
            )
            dx_error = bound_x - pose.x
            v_x = max(-0.08, min(0.08, 0.80 * dx_error))
            v_z = 1.0  # along +Z
            target_h = math.atan2(-v_z, -v_x)

            heading_error = self.heading_controller.normalize_angle(
                target_h - pose.heading
            )
            omega = self.heading_controller.calculate_angular_velocity(
                heading_error
            )

            # Proportional deceleration as robot nears desired_lane_z for millimeter precision stop
            if dz_remaining > 0.08:
                speed = 0.20
            else:
                speed = max(0.04, 0.20 * (dz_remaining / 0.08))

            if dz_remaining <= 0.001 or pose.z >= self.desired_lane_z:
                self.state_machine.set_state(NavigationState.STOP_AT_SHIFT)
                self.stop_hold_counter = 0
                self.prev_omega = 0.0
                self._emit_event("SHIFT_COMPLETE")
                self._emit_event("STOP")
                return MotionCommand(
                    linear_velocity=0.0, angular_velocity=0.0
                )

            return MotionCommand(
                linear_velocity=speed, angular_velocity=omega
            )

        # ========================================================
        # 6. STOP_AT_SHIFT: Complete physical stop at new lane
        # ========================================================
        if state == NavigationState.STOP_AT_SHIFT:
            self.stop_hold_counter += 1
            if self.stop_hold_counter >= self.stop_hold_cycles:
                self.actual_spacing = abs(pose.z - self.previous_lane_z)
                # Reverse lane direction for next lane
                self.lane_direction = -self.lane_direction
                self.lane_index += 1
                # Target lane heading: if dir=+X (1) -> pi; if dir=-X (-1) -> 0.0
                self.turn_target_heading = (
                    math.pi if self.lane_direction == 1 else 0.0
                )
                self.turn_start_heading = pose.heading
                self.turn_phase = "TURN_TO_LANE"
                self.state_machine.set_state(NavigationState.TURN_TO_LANE)
                self._emit_event("TURN_TO_LANE")

            return MotionCommand(
                linear_velocity=0.0, angular_velocity=0.0
            )

        # ========================================================
        # 7. TURN_TO_LANE: 90-degree in-place rotation to face new lane
        # ========================================================
        if state == NavigationState.TURN_TO_LANE:
            heading_error = self.heading_controller.normalize_angle(
                self.turn_target_heading - pose.heading
            )
            if abs(heading_error) <= self.heading_controller.heading_tolerance:
                turn2_angle = abs(
                    self.heading_controller.normalize_angle(
                        pose.heading - self.turn_start_heading
                    )
                )
                h_err2 = abs(heading_error)

                # Format and emit mandatory telemetry
                telemetry = (
                    f"LANE={self.lane_index} | "
                    f"PREV_Z={self.previous_lane_z:.3f} | "
                    f"TARGET_Z={self.desired_lane_z:.3f} | "
                    f"ACTUAL_Z={pose.z:.3f} | "
                    f"SPACING={self.actual_spacing:.3f} | "
                    f"TARGET_H={self.turn_target_heading:.4f} | "
                    f"ACTUAL_H={pose.heading:.4f} | "
                    f"HEADING_ERROR={math.degrees(h_err2):.2f}° | "
                    f"TURN_ANGLE={math.degrees(turn2_angle):.2f}°"
                )
                self._emit_event(f"TELEMETRY {telemetry}")
                self.turn_target_heading = None
                self.turn_phase = "NONE"
                self.state_machine.set_state(NavigationState.DRIVE_LANE)
                self._emit_event(
                    f"DRIVE_LANE index={self.lane_index} dir={self.get_lane_direction_str()}"
                )
                return MotionCommand(
                    linear_velocity=0.0, angular_velocity=0.0
                )

            omega = self.heading_controller.calculate_angular_velocity(
                heading_error, in_turn=True
            )
            # Pure in-place rotation
            return MotionCommand(linear_velocity=0.0, angular_velocity=omega)

        # ========================================================
        # 8. COVERAGE_COMPLETE or STOPPED
        # ========================================================
        return MotionCommand(linear_velocity=0.0, angular_velocity=0.0)