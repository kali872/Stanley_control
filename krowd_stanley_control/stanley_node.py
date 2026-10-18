"""Stanley path tracking against a live Trajectory.

Same contract as krowd_path_tracking's pure_pursuit_node - /planning/trajectory and
/localization/vehicle_state in, /target_angle, /target_speed and /brake out -
so route_publisher_node and vehicle_state_node serve either controller, and
path_tracking.launch.py starts one of them (controller_type). Never both: nothing
else may publish the actuator topics.

See krowd_path_tracking's pure_pursuit_node docstring for the comparison. Stanley passes the
heading error to the steering at unity gain, and this vehicle's heading comes
from an RTK velocity course that carries ~9 deg of error near its 0.5 m/s gate.
Two things here exist because of that:

- The route heading comes from the route's GEOMETRY (stanley.path_heading), not
  from the recorded waypoint yaw, so the mapping drive's heading noise cannot
  reach the steering.
- max_heading_error_deg refuses to drive while the vehicle and the route differ
  by more than that. It is what a start that was not yaw-aligned with the route
  looks like, and what a heading glitch mid-route looks like.
"""

import math

import rclpy
from krowd_control_interfaces.msg import Trajectory, VehicleState
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from mola_slam_mapping.route_io import point_ahead_on_route, project_onto_route

from krowd_stanley_control.disagreement_hold import DisagreementHold
from krowd_stanley_control.tracking_common import (
    TrackedTrajectory,
    end_of_route_reason,
    rate_limited,
    track_trajectory,
    yaw_from_quaternion,
)
from krowd_stanley_control.heading_creep import HeadingCreep, UnverifiedHeadingCreep, creep_permitted
from krowd_stanley_control.recovery import Recovery, pursuit_steering, to_body_frame
from krowd_stanley_control.start_ramp import StartRamp
from krowd_stanley_control.speed_trim import SpeedTrim
from krowd_stanley_control.route_geometry import (
    back_at_route_start,
    remaining_arc_length,
    travelled_arc_length,
    value_at_route_position,
)
from krowd_stanley_control.signal_stop import (
    RedStopDecision,
    SignalStop,
    SignalTracker,
    SignalVerdict,
    parse_frame,
    red_stop_decision,
    zones_from_json,
)
from krowd_stanley_control.external_motion import ExternalMotion
from krowd_stanley_control.pause_stop import PauseStop, PauseVerdict
from krowd_stanley_control.stopline import StoplineEstimator
from krowd_stanley_control.avoidance_handoff import (
    APPROACH,
    HANDED_OFF,

    AvoidanceHandoff,
    HandoffVerdict,
)
import json
from krowd_stanley_control.stanley import (
    curvature_feedforward,
    front_axle_position,
    heading_not_aligned,
    limit_steering_lead,
    path_heading,
    projection_window,
    steering_angle,
    steering_lead_limit,
    value_ahead_on_route,
    wrap_angle,
    yaw_rate_damping,
)
from dataclasses import replace

try:  # The steering encoder message; absent on a machine without the CAN bridge.
    from ros_can_bridge_msgs.msg import EncoderAngleStamped
except ImportError:  # pragma: no cover - depends on the workspace
    EncoderAngleStamped = None

from krowd_stanley_control.vehicle import (
    SteeringActuator,
    steering_table,
    Vehicle,
    implausible_steering_limit_reason,
    speed_command_kmh,
    steering_limit_from_turning_radius,
)


class StanleyNode(Node):
    """Follow the published Trajectory with the Stanley law."""

    def __init__(self):
        super().__init__("stanley_node")

        self.declare_parameter("trajectory_topic", "/planning/trajectory")
        self.declare_parameter("vehicle_state_topic", "/localization/vehicle_state")
        self.declare_parameter("angle_topic", "/target_angle")
        self.declare_parameter("speed_topic", "/target_speed")
        self.declare_parameter("brake_topic", "/brake")
        self.declare_parameter("status_topic", "~/status")
        self.declare_parameter("cross_track_error_topic", "~/cross_track_error")
        self.declare_parameter("heading_error_topic", "~/heading_error")
        self.declare_parameter("target_marker_topic", "~/target_marker")

        # Vehicle geometry: the same parameters, from the same config/vehicle.yaml,
        # as pure_pursuit_node.
        self.declare_parameter("wheelbase_m", 1.340)
        self.declare_parameter("track_width_m", 1.080)
        self.declare_parameter("min_turning_radius_m", 3.4)
        self.declare_parameter("max_steering_angle_deg", 0.0)
        self.declare_parameter("max_steering_rate_deg_s", 60.0)
        self.declare_parameter("rear_axle_offset_m", 0.0)
        self.declare_parameter("steering_sign", 1.0)
        # Road-wheel angle -> /target_angle (see vehicle.SteeringActuator):
        # command = sign * gain * road_wheel + offset, clamped to
        # +/- max_steering_command_deg (the steer MCU rejects anything beyond).
        self.declare_parameter("steering_command_gain", 1.0)
        # Measured command <-> tyre-angle table (vehicle.steering_table). Fewer
        # than 3 rows (the default) = no table: steering_command_gain is used.
        self.declare_parameter("steering_table_encoder_rad", [0.0])
        self.declare_parameter("steering_table_left_tire_deg", [0.0])
        self.declare_parameter("steering_table_right_tire_deg", [0.0])
        self.declare_parameter("steering_command_offset_rad", 0.0)
        self.declare_parameter("max_steering_command_deg", math.degrees(2.5))

        self.declare_parameter("cross_track_gain", 1.0)
        self.declare_parameter("cross_track_softening_kmph", 3.6)
        self.declare_parameter("heading_gain", 1.0)
        self.declare_parameter("heading_window_m", 2.0)
        self.declare_parameter("max_heading_error_deg", 60.0)
        # The ESKF's own verdict on its heading (std_msgs/Bool, latched). It
        # publishes a pose from the moment it starts, but on the heading SAVED
        # from the last session until RTK course (~10 m of straight driving)
        # or the LiDAR map alignment verifies it - 124 deg off on 2026-09-21.
        # With heading_verified_required the node brakes (STOP
        # HEADING_UNVERIFIED) until that Bool is true. sim_overrides.yaml
        # turns it off: the simulator has no ESKF.
        self.declare_parameter("heading_verified_topic", "/ekf/heading_verified")
        self.declare_parameter("heading_verified_required", True)
        self.declare_parameter("heading_source_topic", "/ekf/heading_source")
        self.declare_parameter("heading_status_timeout_sec", 2.0)
        self.declare_parameter("map_heading_consistency_required", False)
        self.declare_parameter(
            "map_heading_odometry_topic", "/mola/map_localization/odometry"
        )
        self.declare_parameter(
            "map_heading_verified_topic", "/mola/map_localization/heading_verified"
        )
        self.declare_parameter("map_heading_max_disagreement_deg", 15.0)
        # A disagreement must LAST this long before it stops the vehicle
        # (disagreement_hold.py): MOLA lags in bends and single 0.1-0.5 s
        # excursions braked the car 14 times on 2026-09-24. Above
        # map_heading_immediate_stop_deg it still stops at once. 0 = old
        # behaviour (stop on the first sample over the limit).
        self.declare_parameter("map_heading_disagreement_hold_sec", 1.0)
        self.declare_parameter("map_heading_immediate_stop_deg", 45.0)
        # MOLA map localization is bursty under real LiDAR load: its pose can
        # be 1.7 s apart and the verified flag more than 2 s apart.  Keep this
        # aligned with the ESKF's 3 s LiDAR-heading liveness window so normal
        # scheduler jitter does not alternate TRACKING and STOP.
        self.declare_parameter("map_heading_timeout_sec", 3.0)
        self.declare_parameter("projection_backward_segments", 5)
        self.declare_parameter("projection_forward_segments", 30)

        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("vehicle_state_timeout_sec", 0.5)
        self.declare_parameter("goal_tolerance_m", 1.0)
        # After STOP_AT_END has latched, resume once the vehicle is back within
        # this distance of the route start (see back_at_route_start). It must
        # exceed the ~5 m of driving before the RTK pose reappears on a restarted
        # drive, plus the axle offset. 0 disables.
        self.declare_parameter("restart_near_start_m", 10.0)
        self.declare_parameter("corridor_half_width_m", 3.0)
        self.declare_parameter("stop_outside_corridor", True)
        # Outside the corridor: drive back to the route instead of only
        # stopping (recovery.py). Off by default; path_tracking.yaml turns it
        # on for the vehicle. There is no lane information off the route, so
        # only with an E-stop operator.
        self.declare_parameter("recovery_enabled", False)
        self.declare_parameter("recovery_speed_kmph", 4.0)
        self.declare_parameter("recovery_lookahead_m", 6.0)
        self.declare_parameter("recovery_done_offset_m", 1.5)
        self.declare_parameter("recovery_done_heading_deg", 30.0)
        self.declare_parameter("recovery_max_offset_m", 8.0)
        self.declare_parameter("recovery_max_sec", 20.0)
        self.declare_parameter("recovery_max_jump_m", 1.0)
        self.declare_parameter("stall_warning_period_sec", 5.0)
        # Speed cap for the first metres after tracking starts or resumes; the
        # route's speed profile has no ramp from standstill. See start_ramp.py.
        # 0 disables.
        # ---- steering-motor lag (2026-09-20 oscillation fix) ------------------
        # Yaw-rate damping (stanley.yaw_rate_damping); 0 disables.
        self.declare_parameter("yaw_rate_damping_gain_s", 0.0)
        # Measured steering angle: ros_can_bridge's /encoder, a command/column-
        # side angle, turned into a road-wheel angle with the steering table.
        # The target may lead it by at most max_steering_command_lead_deg
        # (stanley.limit_steering_lead; 0 disables), and with
        # steering_feedback_required the vehicle is not driven without it.
        # Code defaults are all OFF; path_tracking.yaml switches them on for the
        # vehicle and sim_overrides.yaml off again for the simulator.
        self.declare_parameter("steering_feedback_topic", "/encoder")
        self.declare_parameter("steering_feedback_required", False)
        self.declare_parameter("steering_feedback_timeout_sec", 0.25)
        self.declare_parameter("max_steering_command_lead_deg", 0.0)
        # The lead limit in a bend (stanley.steering_lead_limit), blended by
        # the route's curvature between steering_lead_straight_radius_m and
        # steering_lead_curve_radius_m. Equal to the straight value = one
        # limit everywhere, as before 2026-09-23.
        self.declare_parameter("max_steering_command_lead_curve_deg", 0.0)
        self.declare_parameter("steering_lead_straight_radius_m", 50.0)
        self.declare_parameter("steering_lead_curve_radius_m", 15.0)
        self.declare_parameter("yaw_rate_error_topic", "~/yaw_rate_error")
        self.declare_parameter("desired_steering_topic", "~/desired_steering")
        self.declare_parameter("measured_steering_topic", "~/measured_steering")
        # Outer speed trim around the drive MCU's own loop - see speed_trim.py.
        # 0 gain or 0 max disables it.
        self.declare_parameter("speed_trim_gain_per_s", 0.0)
        self.declare_parameter("speed_trim_max_kmph", 0.0)
        self.declare_parameter("speed_trim_min_speed_kmph", 1.0)
        # Where the route is tighter than this radius the trim is not applied
        # (SpeedTrim hold). 0 = trim everywhere.
        self.declare_parameter("speed_trim_min_radius_m", 0.0)
        # Add (1 - heading_gain) * atan(wheelbase * route curvature) to the
        # steering (stanley.curvature_feedforward). false = plain Stanley.
        self.declare_parameter("curvature_feedforward", False)
        # Red-light stop inside the map's traffic-light zones (signal_stop.py,
        # krowd_mission_manager's rule): 7 km/h in a zone, stop on a red for
        # the zone's direction, go again on the detector's confirmed go; an
        # unknown light or no detector passes. false = no signal handling.
        self.declare_parameter("signal_stop_enabled", False)
        self.declare_parameter("signal_zones_topic", "/planning/signal_zones")
        self.declare_parameter("traffic_json_topic", "/traffic_light/detections_json")
        self.declare_parameter("signal_zone_speed_kmph", 7.0)
        self.declare_parameter("signal_red_confirm_frames", 2)
        self.declare_parameter("signal_go_ttl_sec", 0.25)
        self.declare_parameter("signal_timeout_sec", 0.5)
        # Standing at a red: if no frame has said stop for this long (the light
        # is lost - unknown/off, out of the camera's view), go without the
        # confirmed go. 0 disables (wait for the go forever).
        self.declare_parameter("signal_lost_release_sec", 30.0)
        # Traffic-light zones of the map to ignore, by label ("64-68"): the
        # vehicle drives through them like any road. [""] = none skipped.
        self.declare_parameter("signal_skip_zones", [""])
        # Standing at a red: go when the detector has READ a go (go_straight /
        # go_left, unconfirmed) on this many frames within this window, even if
        # its confirmed flag never comes (2026-10-17: the light flickered
        # red/unknown at 1 Hz and green came as 2+2 frames - never confirmed).
        # 0 frames disables (confirmed go only).
        self.declare_parameter("signal_go_seen_frames", 2)
        # Red without a detected stop line: stop at the zone END waypoint
        # (true; the map's traffic zones end on the stop lines) or where the
        # red was confirmed (false; the mission_manager rule).
        self.declare_parameter("signal_stop_at_zone_end", True)
        self.declare_parameter("signal_go_seen_window_sec", 1.5)
        self.declare_parameter("signal_approach_decel_mps2", 0.5)
        # Live stop line from the lane detector (stopline.py, a copy of
        # krowd_final_control's estimator): on a red the vehicle rolls on at
        # the zone speed and brakes so that its front bumper stops
        # stopline_stop_buffer_m before the line - never past the zone's end.
        # No valid line (none seen, older than stopline_ttl_sec) = brake where
        # the red is seen, as without this. false = never use the stop line.
        self.declare_parameter("stopline_enabled", False)
        self.declare_parameter("stopline_topic", "/glane3d/stopline_markers")
        self.declare_parameter("stopline_ttl_sec", 2.0)
        self.declare_parameter("stopline_stop_buffer_m", 1.0)
        self.declare_parameter("front_axle_to_front_bumper_m", 1.194)
        self.declare_parameter("stopline_lateral_half_band_m", 0.63)
        self.declare_parameter("stopline_min_span_m", 1.0)
        self.declare_parameter("stopline_max_abs_slope", 0.5)
        self.declare_parameter("stopline_max_distance_m", 30.0)
        # Braking distance assumed for the stop: v * latency + v^2 / (2 a).
        self.declare_parameter("signal_brake_latency_sec", 1.0)
        self.declare_parameter("signal_brake_decel_mps2", 3.95)
        # Someone else is driving when the vehicle moves faster than this for
        # this long while we command a stop (external_motion.py): no hand-off then.
        self.declare_parameter("external_motion_speed_mps", 1.5)
        self.declare_parameter("external_motion_sec", 3.0)
        # Right-turn pause in the map's pause zones (pause_stop.py): zone speed
        # up to the stop line / zone end, full stop for pause_hold_sec, then go.
        self.declare_parameter("pause_stop_enabled", False)
        self.declare_parameter("pause_zones_topic", "/planning/pause_zones")
        self.declare_parameter("pause_zone_speed_kmph", 7.0)
        self.declare_parameter("pause_approach_decel_mps2", 0.5)
        self.declare_parameter("pause_hold_sec", 3.0)
        self.declare_parameter("pause_stop_speed_mps", 0.3)
        # Cone-avoidance hand-off in the map's avoidance zones
        # (avoidance_handoff.py): slow to avoidance_approach_speed_kmph by the
        # zone start, stop there, publish /stanley_node/handoff and go silent
        # until avoidance_supervisor_node says /stanley_node/resume.
        self.declare_parameter("avoidance_handoff_enabled", False)
        self.declare_parameter("avoidance_zones_topic", "/planning/avoidance_zones")
        self.declare_parameter("handoff_topic", "/stanley_node/handoff")
        self.declare_parameter("resume_topic", "/stanley_node/resume")
        self.declare_parameter("avoidance_approach_speed_kmph", 5.0)
        self.declare_parameter("avoidance_approach_decel_mps2", 0.5)
        self.declare_parameter("avoidance_stop_speed_mps", 0.3)
        self.declare_parameter("avoidance_stop_settle_sec", 1.0)
        self.declare_parameter("avoidance_heading_tolerance_deg", 5.0)
        self.declare_parameter("avoidance_align_timeout_sec", 10.0)
        # Above this speed the yaw-rate damping gain falls as 1/speed, so the
        # damping loop (gain * speed / wheelbase per unit steer) stays as
        # strong as at this speed. At 30 km/h the fixed 0.15 s gain made that
        # loop ~0.9 with 0.24 s of steering lag and rang at 2-4 Hz (sim,
        # 2026-09-27: wheels +/-0.5 deg, body yaw 0.28 deg). 0 = never scaled.
        self.declare_parameter("yaw_rate_damping_full_speed_kmph", 0.0)
        # Read the feedforward's curvature this many seconds of travel ahead
        # (speed * preview along the route), so the slow steering motor
        # (8-13 deg/s of road wheel) starts turning before the bend. 0 = at
        # the front axle.
        self.declare_parameter("curvature_feedforward_preview_sec", 0.0)
        self.declare_parameter("start_speed_kmph", 5.0)
        self.declare_parameter("start_accel_mps2", 1.0)
        # Before the FIRST vehicle state, with a route loaded: drive straight at
        # this speed so gnss_imu_pose_node can commit its heading (~5 m), for at
        # most this long; then brake. See heading_creep.py. 0 disables (waits).
        self.declare_parameter("heading_creep_speed_kmph", 5.0)
        self.declare_parameter("heading_creep_max_sec", 12.0)
        # ...and only once gnss_imu_pose_node reports NEEDS_MOTION here (no
        # verified ESKF heading, no verified LiDAR alignment). false: creep as
        # soon as a route exists without a vehicle state (the old behaviour).
        self.declare_parameter(
            "heading_acquisition_topic", "/gnss_imu/heading_acquisition"
        )
        self.declare_parameter("heading_creep_requires_request", True)
        # While /ekf/heading_verified is false (heading_verified_required):
        # wait braked for heading_creep_wait_sec (the LiDAR alignment usually
        # verifies it standing still), then creep straight at
        # heading_creep_speed_kmph until it is verified, for at most
        # heading_creep_max_sec / heading_creep_max_m. Off = STOP
        # HEADING_UNVERIFIED as before. See heading_creep.UnverifiedHeadingCreep.
        self.declare_parameter("heading_creep_on_unverified", False)
        self.declare_parameter("heading_creep_wait_sec", 5.0)
        self.declare_parameter("heading_creep_max_m", 15.0)

        self._vehicle = self._build_vehicle()
        # The vehicle's own lock, before _build_actuator narrows it to what the
        # command range can express.
        self._mechanical_steering_lock = self._vehicle.max_steering_angle_rad
        self._actuator = self._build_actuator()
        self._steering_sign = float(self.get_parameter("steering_sign").value)
        if self._steering_sign not in (1.0, -1.0):
            # pure_pursuit_node multiplies by whatever is here, so a sign that
            # drifted into a gain there would silently rescale this law.
            raise RuntimeError(
                f"steering_sign must be +1.0 or -1.0, got {self._steering_sign}; "
                "tune cross_track_gain / heading_gain instead"
            )
        self._max_steering_lead = math.radians(
            float(self.get_parameter("max_steering_command_lead_deg").value)
        )
        curve_lead_deg = float(
            self.get_parameter("max_steering_command_lead_curve_deg").value
        )
        self._max_steering_lead_curve = (
            math.radians(curve_lead_deg)
            if curve_lead_deg > 0.0
            else self._max_steering_lead
        )
        self._lead_straight_radius = float(
            self.get_parameter("steering_lead_straight_radius_m").value
        )
        self._lead_curve_radius = float(
            self.get_parameter("steering_lead_curve_radius_m").value
        )
        if self._max_steering_lead < 0.0 or self._max_steering_lead_curve < 0.0:
            raise RuntimeError(
                "max_steering_command_lead_deg and max_steering_command_lead_curve_deg "
                "must not be negative"
            )
        # Road-wheel limits PER SIDE: what the command range can express on
        # that side, never beyond the vehicle's own lock. _build_actuator has
        # already reduced the vehicle's (symmetric) lock to the smaller side,
        # which everything that needs one number keeps using; the steering
        # command itself is clamped with these.
        lock = self._mechanical_steering_lock
        lowest, highest = self._actuator.road_wheel_limits_rad(self._steering_sign)
        self._steering_min = max(lowest, -lock)
        self._steering_max = min(highest, lock)
        self.get_logger().info(
            f"steering limits: right {math.degrees(-self._steering_min):.1f} deg "
            f"(radius {self._vehicle.wheelbase_m / math.tan(-self._steering_min):.2f} m), "
            f"left {math.degrees(self._steering_max):.1f} deg "
            f"(radius {self._vehicle.wheelbase_m / math.tan(self._steering_max):.2f} m)"
            + (
                ""
                if self._max_steering_lead <= 0.0
                else f"; command lead {math.degrees(self._max_steering_lead):.1f} deg on "
                f"straights (R >= {self._lead_straight_radius:.0f} m) to "
                f"{math.degrees(self._max_steering_lead_curve):.1f} deg in bends "
                f"(R <= {self._lead_curve_radius:.0f} m)"
            )
        )
        for name in (
            "cross_track_gain",
            "heading_gain",
            "heading_window_m",
            "max_heading_error_deg",
        ):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value < 0.0:
                raise RuntimeError(f"{name} must be a non-negative number: {value}")
        self._cross_track_gain = float(self.get_parameter("cross_track_gain").value)
        # km/h at the parameter boundary, m/s below it - as in route_publisher_node.
        self._softening = (
            float(self.get_parameter("cross_track_softening_kmph").value) / 3.6
        )
        if not math.isfinite(self._softening) or self._softening <= 0.0:
            raise RuntimeError(
                "cross_track_softening_kmph must be positive: the law divides by "
                "(softening + speed) and is undefined at a standstill without it"
            )
        self._heading_gain = float(self.get_parameter("heading_gain").value)
        self._damping_full_speed = max(
            float(self.get_parameter("yaw_rate_damping_full_speed_kmph").value), 0.0
        ) / 3.6
        self._feedforward_gain = (
            max(1.0 - self._heading_gain, 0.0)
            if bool(self.get_parameter("curvature_feedforward").value)
            else 0.0
        )
        self._feedforward_preview = max(
            float(self.get_parameter("curvature_feedforward_preview_sec").value), 0.0
        )
        trim_radius = float(self.get_parameter("speed_trim_min_radius_m").value)
        self._trim_max_curvature = 1.0 / trim_radius if trim_radius > 0.0 else math.inf
        self._heading_window = float(self.get_parameter("heading_window_m").value)
        self._max_heading_error = math.radians(
            float(self.get_parameter("max_heading_error_deg").value)
        )
        self._state_timeout_ns = int(
            round(float(self.get_parameter("vehicle_state_timeout_sec").value) * 1e9)
        )
        self._goal_tolerance = float(self.get_parameter("goal_tolerance_m").value)
        self._restart_near_start = float(
            self.get_parameter("restart_near_start_m").value
        )
        self._corridor_half_width = float(
            self.get_parameter("corridor_half_width_m").value
        )
        self._stop_outside_corridor = bool(
            self.get_parameter("stop_outside_corridor").value
        )
        self._recovery_enabled = bool(self.get_parameter("recovery_enabled").value)
        self._recovery_speed = float(self.get_parameter("recovery_speed_kmph").value) / 3.6
        self._recovery_lookahead = float(self.get_parameter("recovery_lookahead_m").value)
        self._recovery = Recovery(
            float(self.get_parameter("recovery_done_offset_m").value),
            math.radians(float(self.get_parameter("recovery_done_heading_deg").value)),
            float(self.get_parameter("recovery_max_offset_m").value),
            float(self.get_parameter("recovery_max_sec").value),
            float(self.get_parameter("recovery_max_jump_m").value),
        )
        if self._recovery_enabled and (
            self._recovery_speed <= 0.0 or self._recovery_lookahead <= 0.0
        ):
            raise RuntimeError("recovery_speed_kmph and recovery_lookahead_m must be positive")
        self._start_ramp = StartRamp(
            float(self.get_parameter("start_speed_kmph").value) / 3.6,
            float(self.get_parameter("start_accel_mps2").value),
        )
        self._speed_trim = SpeedTrim(
            float(self.get_parameter("speed_trim_gain_per_s").value),
            float(self.get_parameter("speed_trim_max_kmph").value) / 3.6,
            float(self.get_parameter("speed_trim_min_speed_kmph").value) / 3.6,
        )
        self._yaw_damping_gain = float(
            self.get_parameter("yaw_rate_damping_gain_s").value
        )
        self._feedback_required = bool(
            self.get_parameter("steering_feedback_required").value
        )
        self._feedback_timeout_ns = int(
            float(self.get_parameter("steering_feedback_timeout_sec").value) * 1e9
        )
        if self._yaw_damping_gain < 0.0:
            raise RuntimeError("yaw_rate_damping_gain_s must not be negative")
        if self._feedback_timeout_ns <= 0:
            raise RuntimeError("steering_feedback_timeout_sec must be positive")
        self._measured_steering: float | None = None
        self._measured_steering_ns: int | None = None
        self._creep_on_unverified = bool(
            self.get_parameter("heading_creep_on_unverified").value
        )
        self._unverified_creep = UnverifiedHeadingCreep(
            float(self.get_parameter("heading_creep_wait_sec").value),
            float(self.get_parameter("heading_creep_speed_kmph").value) / 3.6,
            float(self.get_parameter("heading_creep_max_sec").value),
            float(self.get_parameter("heading_creep_max_m").value),
        )
        self._heading_creep = HeadingCreep(
            float(self.get_parameter("heading_creep_speed_kmph").value) / 3.6,
            float(self.get_parameter("heading_creep_max_sec").value),
        )

        self._angle_pub = self.create_publisher(
            Float32, str(self.get_parameter("angle_topic").value), 10
        )
        self._speed_pub = self.create_publisher(
            Float32, str(self.get_parameter("speed_topic").value), 10
        )
        self._brake_pub = self.create_publisher(
            Bool, str(self.get_parameter("brake_topic").value), 10
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self._error_pub = self.create_publisher(
            Float32, str(self.get_parameter("cross_track_error_topic").value), 10
        )
        self._heading_error_pub = self.create_publisher(
            Float32, str(self.get_parameter("heading_error_topic").value), 10
        )
        self._yaw_rate_error_pub = self.create_publisher(
            Float32, str(self.get_parameter("yaw_rate_error_topic").value), 10
        )
        self._desired_steering_pub = self.create_publisher(
            Float32, str(self.get_parameter("desired_steering_topic").value), 10
        )
        self._measured_steering_pub = self.create_publisher(
            Float32, str(self.get_parameter("measured_steering_topic").value), 10
        )
        self._marker_pub = self.create_publisher(
            Marker, str(self.get_parameter("target_marker_topic").value), 10
        )

        trajectory_qos = QoSProfile(depth=1)
        trajectory_qos.reliability = ReliabilityPolicy.RELIABLE
        trajectory_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Trajectory,
            str(self.get_parameter("trajectory_topic").value),
            self._on_trajectory,
            trajectory_qos,
        )
        self.create_subscription(
            VehicleState,
            str(self.get_parameter("vehicle_state_topic").value),
            self._on_vehicle_state,
            10,
        )
        self._heading_verified: bool | None = None
        self._heading_verified_ns: int | None = None
        self._heading_source: str | None = None
        self._heading_source_ns: int | None = None
        self._heading_verified_required = bool(
            self.get_parameter("heading_verified_required").value
        )
        self._map_heading_consistency_required = bool(
            self.get_parameter("map_heading_consistency_required").value
        )
        self._heading_status_timeout_ns = int(
            float(self.get_parameter("heading_status_timeout_sec").value) * 1e9
        )
        self._map_heading_timeout_ns = int(
            float(self.get_parameter("map_heading_timeout_sec").value) * 1e9
        )
        self._map_heading_max_disagreement = math.radians(float(
            self.get_parameter("map_heading_max_disagreement_deg").value
        ))
        if (
            self._heading_status_timeout_ns <= 0
            or self._map_heading_timeout_ns <= 0
            or not 0.0 < self._map_heading_max_disagreement <= math.pi
        ):
            raise RuntimeError("invalid map heading consistency parameters")
        self._map_heading_hold = DisagreementHold(
            float(self.get_parameter("map_heading_disagreement_hold_sec").value),
            math.radians(float(
                self.get_parameter("map_heading_immediate_stop_deg").value
            )),
        )
        self._map_heading_hold_warn_ns = 0
        self._map_heading_pose: Odometry | None = None
        self._map_heading_pose_ns: int | None = None
        self._map_heading_verified: bool | None = None
        self._map_heading_verified_ns: int | None = None
        self.create_subscription(
            Bool,
            str(self.get_parameter("heading_verified_topic").value),
            self._on_heading_verified,
            trajectory_qos,
        )
        if self._map_heading_consistency_required:
            self.create_subscription(
                String,
                str(self.get_parameter("heading_source_topic").value),
                self._on_heading_source,
                trajectory_qos,
            )
            self.create_subscription(
                Odometry,
                str(self.get_parameter("map_heading_odometry_topic").value),
                self._on_map_heading_pose,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("map_heading_verified_topic").value),
                self._on_map_heading_verified,
                qos_profile_sensor_data,
            )
        feedback_topic = str(self.get_parameter("steering_feedback_topic").value).strip()
        if EncoderAngleStamped is not None and feedback_topic:
            self.create_subscription(
                EncoderAngleStamped, feedback_topic, self._on_steering_feedback, 10
            )
        elif self._feedback_required:
            raise RuntimeError(
                "steering_feedback_required is true but "
                + (
                    "ros_can_bridge_msgs is not available in this workspace"
                    if EncoderAngleStamped is None
                    else "steering_feedback_topic is empty"
                )
            )
        self._heading_state: str | None = None
        self._creep_requires_request = bool(
            self.get_parameter("heading_creep_requires_request").value
        )
        self.create_subscription(
            String,
            str(self.get_parameter("heading_acquisition_topic").value),
            self._on_heading_state,
            trajectory_qos,
        )

        self._signal_enabled = bool(self.get_parameter("signal_stop_enabled").value)
        self._signal = SignalStop(
            float(self.get_parameter("signal_zone_speed_kmph").value) / 3.6,
            float(self.get_parameter("signal_approach_decel_mps2").value),
        )
        self._signal_tracker = SignalTracker(
            int(self.get_parameter("signal_red_confirm_frames").value)
        )
        self._signal_go_ttl = float(self.get_parameter("signal_go_ttl_sec").value)
        self._signal_timeout = float(self.get_parameter("signal_timeout_sec").value)
        self._signal_lost_release = float(self.get_parameter("signal_lost_release_sec").value)
        self._signal_skip = {
            str(label).strip() for label in (self.get_parameter("signal_skip_zones").value or [])
            if str(label).strip()
        }
        self._signal_go_seen_frames = int(self.get_parameter("signal_go_seen_frames").value)
        self._signal_stop_at_zone_end = bool(self.get_parameter("signal_stop_at_zone_end").value)
        self._signal_go_seen_window = float(self.get_parameter("signal_go_seen_window_sec").value)
        self._signal_zones_id: str | None = None
        self._route_s: float | None = None
        self._handoff_enabled = bool(self.get_parameter("avoidance_handoff_enabled").value)
        self._handoff = AvoidanceHandoff(
            approach_speed_mps=float(self.get_parameter("avoidance_approach_speed_kmph").value) / 3.6,
            approach_decel_mps2=float(self.get_parameter("avoidance_approach_decel_mps2").value),
            stop_speed_mps=float(self.get_parameter("avoidance_stop_speed_mps").value),
            stop_settle_s=float(self.get_parameter("avoidance_stop_settle_sec").value),
            heading_tolerance_rad=math.radians(float(self.get_parameter("avoidance_heading_tolerance_deg").value)),
            align_timeout_s=float(self.get_parameter("avoidance_align_timeout_sec").value),
            brake_latency_s=float(self.get_parameter("signal_brake_latency_sec").value),
            brake_decel_mps2=float(self.get_parameter("signal_brake_decel_mps2").value),
        )
        self._avoidance_zones_id: str | None = None
        self._handoff_since_ns: int | None = None
        self._handoff_pub = None
        if self._handoff_enabled:
            self._handoff_pub = self.create_publisher(
                String, str(self.get_parameter("handoff_topic").value), trajectory_qos
            )
            self._handoff_pub.publish(String(data=json.dumps({"active": False})))
            self.create_subscription(
                String,
                str(self.get_parameter("avoidance_zones_topic").value),
                self._on_avoidance_zones,
                trajectory_qos,
            )
            self.create_subscription(
                Bool, str(self.get_parameter("resume_topic").value), self._on_resume, 10
            )
            self.get_logger().info(
                "cone-avoidance hand-off ON: stop at the start of each avoidance zone "
                f"(approach {self._handoff.approach_speed * 3.6:.0f} km/h), hand the vehicle to "
                "avoidance_supervisor_node / cone_avoidance_1005, resume on /stanley_node/resume"
            )
        self._last_brake_cmd: bool | None = None
        self._external = ExternalMotion(
            float(self.get_parameter("external_motion_speed_mps").value),
            float(self.get_parameter("external_motion_sec").value),
        )
        self._external_logged = False
        self._pause_enabled = bool(self.get_parameter("pause_stop_enabled").value)
        self._pause = PauseStop(
            zone_speed_mps=float(self.get_parameter("pause_zone_speed_kmph").value) / 3.6,
            approach_decel_mps2=float(self.get_parameter("pause_approach_decel_mps2").value),
            stop_speed_mps=float(self.get_parameter("pause_stop_speed_mps").value),
            hold_s=float(self.get_parameter("pause_hold_sec").value),
            brake_latency_s=float(self.get_parameter("signal_brake_latency_sec").value),
            brake_decel_mps2=float(self.get_parameter("signal_brake_decel_mps2").value),
        )
        self._pause_zones_id: str | None = None
        if self._pause_enabled:
            self.create_subscription(
                String,
                str(self.get_parameter("pause_zones_topic").value),
                self._on_pause_zones,
                trajectory_qos,
            )
            self.get_logger().info(
                f"right-turn pause ON: {self._pause.zone_speed * 3.6:.0f} km/h in the map's pause zones, "
                "stop at the stop line (or the zone end) for "
                f"{self._pause.hold_s:.1f} s, then go"
            )
        self._red_braking = False
        self._stopline_enabled = self._signal_enabled and bool(
            self.get_parameter("stopline_enabled").value
        )
        self._stopline = StoplineEstimator(
            half_band_m=float(self.get_parameter("stopline_lateral_half_band_m").value),
            min_span_m=float(self.get_parameter("stopline_min_span_m").value),
            max_abs_slope=float(self.get_parameter("stopline_max_abs_slope").value),
            max_distance_m=float(self.get_parameter("stopline_max_distance_m").value),
        )
        self._stopline_ttl = float(self.get_parameter("stopline_ttl_sec").value)
        self._stopline_buffer = float(self.get_parameter("stopline_stop_buffer_m").value)
        self._front_to_bumper = float(self.get_parameter("front_axle_to_front_bumper_m").value)
        self._brake_latency = float(self.get_parameter("signal_brake_latency_sec").value)
        self._brake_decel = float(self.get_parameter("signal_brake_decel_mps2").value)
        if self._stopline_enabled:
            self.create_subscription(
                MarkerArray,
                str(self.get_parameter("stopline_topic").value),
                self._on_stopline,
                5,
            )
        self._route_length_cache: tuple[str, float] | None = None
        if self._signal_enabled:
            self.create_subscription(
                String,
                str(self.get_parameter("signal_zones_topic").value),
                self._on_signal_zones,
                trajectory_qos,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("traffic_json_topic").value),
                self._on_traffic_json,
                10,
            )
        self.get_logger().info(
            "red-light stop "
            + (f"ON: {self._signal.zone_speed * 3.6:.0f} km/h in the map's traffic-light "
               f"zones, stop after {self._signal_tracker.stop_frames} red frames, go on "
               "the detector's confirmed go; unknown light / no detector passes"
               if self._signal_enabled else "off (signal_stop_enabled: false)")
            + (f"; stop line ON: front bumper stops {self._stopline_buffer:.1f} m before the "
               f"detected line (valid {self._stopline_ttl:.1f} s), else where the red is seen"
               if self._stopline_enabled else "; stop line not used")
        )

        self._trajectory: TrackedTrajectory | None = None
        self._vehicle_state: VehicleState | None = None
        self._vehicle_state_ns: int | None = None
        self._last_steering = 0.0
        self._goal_latched = False
        self._latched_trajectory_id: str | None = None
        # Where the goal stop latched, and whether the vehicle has since been
        # driven away from there (see _release_goal_latch_at_route_start).
        self._goal_latch_xy: tuple[float, float] | None = None
        self._left_goal = False
        self._last_control_ns: int | None = None
        self._last_projection_position: float | None = None
        self._last_status = ""

        self._projection_backward = int(
            self.get_parameter("projection_backward_segments").value
        )
        self._projection_forward = int(
            self.get_parameter("projection_forward_segments").value
        )
        if self._projection_backward < 0 or self._projection_forward < 1:
            raise RuntimeError(
                "projection window requires projection_backward_segments >= 0 "
                "and projection_forward_segments >= 1"
            )

        rate = float(self.get_parameter("control_rate_hz").value)
        if rate <= 0.0:
            raise RuntimeError(f"control_rate_hz must be positive: {rate}")
        self._nominal_dt = 1.0 / rate
        self.create_timer(self._nominal_dt, self._control_once)

        # Wall-clock watchdog on the control loop - see pure_pursuit_node for the
        # silent use_sim_time failure it exists to report.
        self._control_cycles = 0
        self._cycles_at_last_check = -1
        self.create_timer(
            float(self.get_parameter("stall_warning_period_sec").value),
            self._warn_if_control_loop_stalled,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )

        self.get_logger().info(
            f"curvature feedforward gain {self._feedforward_gain:.2f} "
            f"(1 - heading_gain; 0 = off), curvature read {self._feedforward_preview:.2f} s "
            "ahead; speed trim held where R < "
            + (f"{1.0 / self._trim_max_curvature:.1f} m"
               if math.isfinite(self._trim_max_curvature) else "- (never)")
        )
        alignment = (
            f"{math.degrees(self._max_heading_error):.0f} deg"
            if self._max_heading_error > 0.0
            else "disabled"
        )
        self.get_logger().info(
            f"stanley_node: wheelbase {self._vehicle.wheelbase_m:.3f} m (cross-track "
            "error at the front axle, "
            f"{self._vehicle.rear_axle_offset_m + self._vehicle.wheelbase_m:.3f} m "
            "ahead of base_link), steering limit "
            f"{math.degrees(self._vehicle.max_steering_angle_rad):.1f} deg (radius "
            f"{self._vehicle.implied_turning_radius_m:.2f} m), sign "
            f"{self._steering_sign:+.0f}, rate limit "
            f"{math.degrees(self._vehicle.max_steering_rate_rad_s):.1f} deg/s, "
            f"k {self._cross_track_gain:.2f} 1/s, softening "
            f"{self._softening * 3.6:.1f} km/h, heading gain "
            f"{self._heading_gain:.2f} over a {self._heading_window:.1f} m window, "
            f"heading alignment limit {alignment}, control at {rate:.1f} Hz"
        )

    def _build_actuator(self) -> SteeringActuator:
        """The road-wheel -> /target_angle mapping, and the limit it imposes.

        The road-wheel limit in use is the smaller of the vehicle's own and
        what the command range can express. The MCU soft limit is +/-2.5 rad.
        """

        table_command, table_road_wheel = steering_table(
            list(self.get_parameter("steering_table_encoder_rad").value),
            list(self.get_parameter("steering_table_left_tire_deg").value),
            list(self.get_parameter("steering_table_right_tire_deg").value),
        )
        actuator = SteeringActuator(
            gain=float(self.get_parameter("steering_command_gain").value),
            offset_rad=float(self.get_parameter("steering_command_offset_rad").value),
            max_command_rad=math.radians(
                float(self.get_parameter("max_steering_command_deg").value)
            ),
            table_command_rad=table_command,
            table_road_wheel_rad=table_road_wheel,
        )
        actuator.validate()
        expressible = actuator.road_wheel_limit_rad
        if expressible < self._vehicle.max_steering_angle_rad:
            self.get_logger().info(
                f"road-wheel steering limit {math.degrees(self._vehicle.max_steering_angle_rad):.1f} deg "
                f"reduced to {math.degrees(expressible):.1f} deg: the command range "
                f"(+/-{math.degrees(actuator.max_command_rad):.0f} deg, "
                + ("measured steering table" if actuator.has_table else f"gain {actuator.gain:.2f}")
                + ") cannot express more - turning radius "
                f"{self._vehicle.wheelbase_m / math.tan(expressible):.2f} m"
            )
            self._vehicle = replace(self._vehicle, max_steering_angle_rad=expressible)
        return actuator

    def _build_vehicle(self) -> Vehicle:
        wheelbase = float(self.get_parameter("wheelbase_m").value)
        min_radius = float(self.get_parameter("min_turning_radius_m").value)
        override_deg = float(self.get_parameter("max_steering_angle_deg").value)
        if override_deg > 0.0:
            max_steering = math.radians(override_deg)
        else:
            max_steering = steering_limit_from_turning_radius(wheelbase, min_radius)

        vehicle = Vehicle(
            wheelbase_m=wheelbase,
            track_width_m=float(self.get_parameter("track_width_m").value),
            min_turning_radius_m=min_radius,
            max_steering_angle_rad=max_steering,
            max_steering_rate_rad_s=math.radians(
                float(self.get_parameter("max_steering_rate_deg_s").value)
            ),
            rear_axle_offset_m=float(self.get_parameter("rear_axle_offset_m").value),
        )
        if vehicle.max_steering_angle_rad <= 0.0:
            raise RuntimeError(
                "steering limit resolved to zero or less; set max_steering_angle_deg "
                "or min_turning_radius_m"
            )
        reason = implausible_steering_limit_reason(vehicle)
        if reason is not None:
            self.get_logger().warn(f"suspicious steering limit: {reason}")
        return vehicle

    def _on_trajectory(self, msg: Trajectory) -> None:
        poses = msg.path.poses
        count = len(poses)
        if count < 2:
            self.get_logger().warn(
                f"ignoring trajectory '{msg.trajectory_id}': {count} poses"
            )
            return

        if len(msg.target_speeds) != count or len(msg.curvatures) != count:
            self.get_logger().error(
                f"ignoring trajectory '{msg.trajectory_id}': {count} poses but "
                f"{len(msg.curvatures)} curvatures and {len(msg.target_speeds)} "
                "target speeds"
            )
            return

        if msg.trajectory_id != self._latched_trajectory_id:
            # A new plan releases a latched goal; a republish of the same one
            # must not.
            self._goal_latched = False

        if self._trajectory is None or msg.trajectory_id != self._trajectory.trajectory_id:
            self._last_projection_position = None
        self._trajectory = track_trajectory(msg, self._corridor_half_width)

    def _project_continuously(self, route, x: float, y: float):
        if self._last_projection_position is None:
            projection = project_onto_route(route, x, y)
        else:
            start, stop = projection_window(
                len(route),
                self._last_projection_position,
                self._projection_backward,
                self._projection_forward,
            )
            projection = project_onto_route(route[start:stop], x, y)
            if projection is not None:
                projection = replace(
                    projection, segment_index=projection.segment_index + start
                )
        if projection is not None:
            self._last_projection_position = projection.route_position
        return projection

    def _on_avoidance_zones(self, msg: String) -> None:
        try:
            trajectory_id, zones = zones_from_json(msg.data)
        except ValueError as error:
            self.get_logger().error(f"ignoring avoidance zones: {error}")
            return
        changed = trajectory_id != self._avoidance_zones_id or zones != self._handoff.zones
        self._avoidance_zones_id = trajectory_id
        self._handoff.set_zones(zones)
        if changed:
            self.get_logger().info(
                f"avoidance zones for '{trajectory_id}': "
                + (", ".join(f"#{z.label} {z.start_s:.0f}-{z.end_s:.0f} m" for z in zones) or "none")
            )

    def _on_resume(self, msg: Bool) -> None:
        if not msg.data or self._handoff.state != HANDED_OFF:
            return
        zone = self._handoff.zone
        self._handoff.resume()
        self._handoff_since_ns = None
        self._start_ramp.reset()
        self._speed_trim.reset()
        self._last_steering = 0.0
        # cone_avoidance may have left the vehicle metres off the route: that
        # is a real offset for the recovery, not a jump to refuse.
        self._recovery.forget_offset()
        # The heading creeps kept counting while stanley was silent (the
        # ESKF can drop its verification while the cone stack drives, 2026-10-08):
        # start them afresh so a re-verification creep is possible after the resume.
        self._unverified_creep.reset()
        self._publish_handoff(None)
        self.get_logger().info(
            f"resumed from cone_avoidance in zone #{zone.label if zone else '?'}: tracking from here"
        )

    def _handoff_step(self, now_ns: int, trajectory, speed: float, heading_error: float) -> HandoffVerdict:
        if (
            not self._handoff_enabled
            or self._avoidance_zones_id != trajectory.trajectory_id
            or self._route_s is None
        ):
            return HandoffVerdict("DRIVE", None, None, False)
        # The vehicle moves against our brake: someone else is driving. Never
        # hand a manually driven vehicle to cone_avoidance (2026-10-17).
        if self._handoff.state != HANDED_OFF and self._external.step(
            now_ns * 1e-9, self._last_brake_cmd, speed
        ):
            if self._handoff.state != "DRIVE" or not self._external_logged:
                self.get_logger().warning(
                    f"vehicle moves at {speed * 3.6:.0f} km/h against our brake: external control, "
                    "cone-avoidance hand-off suppressed"
                )
                self._external_logged = True
            self._handoff.reset()
            return HandoffVerdict("DRIVE", None, None, False)
        if not self._external.active:
            self._external_logged = False
        if self._route_length_cache is None or self._route_length_cache[0] != trajectory.trajectory_id:
            self._route_length_cache = (
                trajectory.trajectory_id,
                travelled_arc_length(trajectory.planar, len(trajectory.planar) - 2, 1.0),
            )
        before = self._handoff.state
        verdict = self._handoff.step(
            now_ns * 1e-9, self._route_s, speed, heading_error,
            self._route_length_cache[1], trajectory.closed_loop,
        )
        if verdict.state == HANDED_OFF and before != HANDED_OFF:
            self._handoff_since_ns = now_ns
            note = self._handoff.last_note
            self.get_logger().warning(
                f"handing the vehicle to cone_avoidance_1005 in zone #{verdict.zone.label} "
                f"(front axle {self._route_s - verdict.zone.start_s:+.2f} m from the zone start)"
                + (f" ({note})" if note else "")
            )
        return verdict

    def _publish_handoff(self, verdict: HandoffVerdict | None) -> None:
        if self._handoff_pub is None:
            return
        if verdict is None:
            payload = {"active": False}
        else:
            payload = {
                "active": True,
                "zone": verdict.zone.label,
                "past_end": bool(verdict.past_end),
                "route_s": round(self._route_s or 0.0, 2),
                "zone_end_s": round(verdict.zone.end_s, 2),
            }
        self._handoff_pub.publish(String(data=json.dumps(payload)))

    def _on_pause_zones(self, msg: String) -> None:
        try:
            trajectory_id, zones = zones_from_json(msg.data)
        except ValueError as error:
            self.get_logger().error(f"ignoring pause zones: {error}")
            return
        changed = trajectory_id != self._pause_zones_id or zones != self._pause.zones
        self._pause_zones_id = trajectory_id
        self._pause.set_zones(zones)
        if changed:
            self.get_logger().info(
                f"pause (right-turn stop) zones for '{trajectory_id}': "
                + (", ".join(f"#{z.label} {z.start_s:.0f}-{z.end_s:.0f} m" for z in zones) or "none")
            )

    def _pause_step(self, now_ns: int, trajectory, speed: float) -> PauseVerdict:
        """The right-turn pause rule for this cycle; nothing when it is off or
        the zones on hand belong to another trajectory."""

        if (
            not self._pause_enabled
            or self._pause_zones_id != trajectory.trajectory_id
            or self._route_s is None
        ):
            return PauseVerdict("DRIVE", None, None, False)
        if self._route_length_cache is None or self._route_length_cache[0] != trajectory.trajectory_id:
            self._route_length_cache = (
                trajectory.trajectory_id,
                travelled_arc_length(trajectory.planar, len(trajectory.planar) - 2, 1.0),
            )
        verdict = self._pause.step(
            now_ns * 1e-9, self._route_s, speed,
            self._stopline_distance(now_ns),
            self._vehicle.wheelbase_m + self._vehicle.rear_axle_offset_m + self._front_to_bumper,
            self._front_to_bumper,
            self._stopline_buffer,
            self._route_length_cache[1], trajectory.closed_loop,
        )
        if verdict.released:
            self._start_ramp.reset()
            self.get_logger().info(
                f"right-turn pause in zone #{verdict.zone.label} done after {verdict.held_s:.1f} s standing - going"
            )
        return verdict

    def _on_signal_zones(self, msg: String) -> None:
        try:
            trajectory_id, zones = zones_from_json(msg.data)
        except ValueError as error:
            self.get_logger().error(f"ignoring signal zones: {error}")
            return
        skipped = [z for z in zones if z.label in self._signal_skip]
        zones = [z for z in zones if z.label not in self._signal_skip]
        changed = trajectory_id != self._signal_zones_id or zones != self._signal.zones
        self._signal_zones_id = trajectory_id
        self._signal.set_zones(zones)
        if changed:
            self.get_logger().info(
                f"traffic-light zones for '{trajectory_id}': "
                + (", ".join(f"#{z.label} {z.direction}" for z in zones) or "none")
                + (f" (skipped by signal_skip_zones: {', '.join('#' + z.label for z in skipped)})"
                   if skipped else "")
            )

    def _on_traffic_json(self, msg: String) -> None:
        try:
            frame = parse_frame(msg.data)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        self._signal_tracker.update(frame, self.get_clock().now().nanoseconds * 1e-9)

    def _on_stopline(self, msg: MarkerArray) -> None:
        """/glane3d/stopline_markers: LINE_STRIP curves in base_link (as
        krowd_final_control's _on_stopline)."""

        stamp_ns = None
        for marker in msg.markers:
            if marker.action == Marker.ADD:
                stamp = marker.header.stamp
                stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
                break
        if stamp_ns is None:
            return
        self._stopline.ingest(msg.markers, stamp_ns, self._route_s)

    def _stopline_distance(self, now_ns: int) -> float | None:
        """base_link -> stop line (m) while a fresh estimate exists."""

        if not self._stopline_enabled:
            return None
        estimate = self._stopline.estimate(now_ns, self._stopline_ttl, self._route_s)
        return estimate.distance_m if estimate.valid else None

    def _stopline_note(self, now_ns: int) -> str:
        if not self._stopline_enabled:
            return ""
        distance = self._stopline_distance(now_ns)
        return " stopline=none" if distance is None else f" stopline={distance:.1f}m"

    def _red_stop(self, now_ns: int, signal: SignalVerdict, speed: float) -> RedStopDecision:
        """Brake for the red now, or roll up to the stop line first. Once
        braking it stays so until the red is released: re-deciding while the
        speed falls would release and re-apply the brake."""

        if self._red_braking:
            return RedStopDecision(True, "holding", None)
        decision = red_stop_decision(
            speed,
            self._route_s if self._route_s is not None else signal.zone.end_s,
            signal.zone.end_s,
            self._stopline_distance(now_ns),
            self._vehicle.wheelbase_m + self._vehicle.rear_axle_offset_m + self._front_to_bumper,
            self._front_to_bumper,
            self._stopline_buffer,
            self._brake_latency,
            self._brake_decel,
            stop_at_zone_end=self._signal_stop_at_zone_end,
        )
        if decision.brake:
            self._red_braking = True
        return decision

    def _signal_verdict(self, now_ns: int, trajectory, projection) -> SignalVerdict:
        """The red-light rule for this cycle; nothing when it is off or the
        zones on hand belong to another trajectory."""

        if not self._signal_enabled or self._signal_zones_id != trajectory.trajectory_id:
            return SignalVerdict(False, None, None)
        if self._route_length_cache is None or self._route_length_cache[0] != trajectory.trajectory_id:
            self._route_length_cache = (
                trajectory.trajectory_id,
                travelled_arc_length(trajectory.planar, len(trajectory.planar) - 2, 1.0),
            )
        now_s = now_ns * 1e-9
        was_stopped = self._signal.stopped_at
        verdict = self._signal.step(
            self._route_s,
            lambda direction: self._signal_tracker.red(direction, now_s, self._signal_timeout),
            lambda direction: self._signal_tracker.go(direction, now_s, self._signal_go_ttl)
            or self._signal_tracker.go_seen(
                direction, now_s, self._signal_go_seen_window, self._signal_go_seen_frames
            ),
            self._route_length_cache[1],
            trajectory.closed_loop,
            lost=self._signal_lost,
        )
        if was_stopped is not None and self._signal.stopped_at is None:
            if self._signal.last_release == "go":
                how = ("confirmed go" if self._signal_tracker.go(was_stopped.direction, now_s, self._signal_go_ttl)
                       else f"go read on {self._signal_go_seen_frames} frames within "
                            f"{self._signal_go_seen_window:.1f} s (unconfirmed)")
            else:
                how = f"light lost for {self._signal_lost_release:.0f} s - going without a go"
            self.get_logger().warning(f"red stop in zone #{was_stopped.label} released: {how}")
        return verdict

    def _signal_lost(self, direction: str) -> bool:
        if self._signal_lost_release <= 0.0:
            return False
        since = self._signal_tracker.since_stop_s(direction, self.get_clock().now().nanoseconds * 1e-9)
        return since is not None and since >= self._signal_lost_release

    def _signal_lost_note(self, direction: str) -> str:
        if self._signal_lost_release <= 0.0:
            return ""
        since = self._signal_tracker.since_stop_s(direction, self.get_clock().now().nanoseconds * 1e-9)
        return "" if since is None else f" lost={since:.0f}/{self._signal_lost_release:.0f}s"

    def _on_steering_feedback(self, msg) -> None:
        """/encoder: a command-side angle -> the road-wheel angle it means."""

        angle = float(msg.angle_rad)
        if not math.isfinite(angle):
            return
        self._measured_steering = self._actuator.road_wheel(angle, self._steering_sign)
        self._measured_steering_ns = self.get_clock().now().nanoseconds

    def _steering_feedback_problem(self, now_ns: int) -> str | None:
        """Why the vehicle may not be driven on the steering feedback; None when
        it may (or when the feedback is not required)."""

        if not self._feedback_required:
            return None
        if self._measured_steering_ns is None:
            return "NO_STEERING_FEEDBACK"
        age_ns = now_ns - self._measured_steering_ns
        if age_ns > self._feedback_timeout_ns:
            return f"STALE_STEERING_FEEDBACK age={age_ns * 1e-9:.2f}s"
        return None

    def _fresh_measured_steering(self, now_ns: int) -> float | None:
        """The measured road-wheel angle while it is fresh, else None - a stale
        angle must not hold the command where the wheels used to be."""

        if (
            self._measured_steering_ns is None
            or now_ns - self._measured_steering_ns > self._feedback_timeout_ns
        ):
            return None
        return self._measured_steering

    def _on_vehicle_state(self, msg: VehicleState) -> None:
        self._vehicle_state = msg
        self._vehicle_state_ns = self.get_clock().now().nanoseconds
        self._heading_creep.vehicle_state_seen()

    def _warn_if_control_loop_stalled(self) -> None:
        if self._control_cycles != self._cycles_at_last_check:
            self._cycles_at_last_check = self._control_cycles
            return
        self._cycles_at_last_check = self._control_cycles

        if bool(self.get_parameter("use_sim_time").value):
            cause = (
                "use_sim_time is true and nothing is advancing /clock - start "
                "the bag (ros2 bag play ... --clock), or drop use_sim_time:=true "
                "if you are not replaying one"
            )
        else:
            cause = "the control timer is not firing"
        self.get_logger().error(
            f"control loop has not run for "
            f"{self.get_parameter('stall_warning_period_sec').value} s "
            f"({self._control_cycles} cycles total): {cause}. No command is "
            "being published on /target_angle, /target_speed or /brake."
        )

    def _control_once(self) -> None:
        self._control_cycles += 1
        now_ns = self.get_clock().now().nanoseconds
        dt = self._nominal_dt
        if self._last_control_ns is not None:
            elapsed = (now_ns - self._last_control_ns) * 1e-9
            if 0.0 < elapsed < 10.0 * self._nominal_dt:
                dt = elapsed
        self._last_control_ns = now_ns

        reason = self._blocking_reason(now_ns)
        if reason is not None:
            if reason == "NO_VEHICLE_STATE":
                self._before_first_vehicle_state(now_ns)
            else:
                self._publish_stop(reason)
            return

        if self._heading_verified_required and self._heading_verified is not True:
            state = "UNKNOWN" if self._heading_verified is None else "FALSE"
            if not self._creep_on_unverified:
                self._publish_stop(f"HEADING_UNVERIFIED state={state}")
                return
            position = self._vehicle_state.odometry.pose.pose.position
            verdict = self._unverified_creep.command(
                now_ns, float(position.x), float(position.y)
            )
            if verdict == UnverifiedHeadingCreep.WAIT:
                self._publish_stop(
                    f"HEADING_UNVERIFIED state={state} waiting "
                    f"{self._unverified_creep.elapsed_s(now_ns):.1f}/"
                    f"{self._unverified_creep.wait_s:.0f}s for the LiDAR alignment"
                )
            elif verdict == UnverifiedHeadingCreep.CREEP:
                self._publish_unverified_creep(now_ns, state)
            else:
                self._publish_stop(
                    "HEADING_NOT_ACQUIRED after "
                    f"{self._unverified_creep.max_duration_s:.0f}s / "
                    f"{self._unverified_creep.distance_m:.1f}m of creeping - "
                    "the ESKF never verified its heading; restart path tracking to retry"
                )
            return
        if self._unverified_creep.active:
            # Verified while creeping: the pose (heading AND position, via the
            # antenna lever arm) has just moved. Project afresh and ramp up
            # from here; the corridor check that follows decides whether the
            # route is still there or recovery has to bring us back.
            self.get_logger().info(
                f"heading verified after {self._unverified_creep.elapsed_s(now_ns):.1f} s "
                f"and {self._unverified_creep.distance_m:.1f} m of creeping - tracking"
            )
            self._unverified_creep.reset()
            self._last_projection_position = None
            self._start_ramp.reset()
        heading_problem = self._map_heading_consistency_problem(now_ns)
        if heading_problem is not None:
            self._publish_stop(heading_problem)
            return

        feedback_problem = self._steering_feedback_problem(now_ns)
        if feedback_problem is not None:
            self._publish_stop(feedback_problem)
            return

        trajectory = self._trajectory
        state = self._vehicle_state
        pose = state.odometry.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        front_x, front_y = front_axle_position(
            float(pose.position.x),
            float(pose.position.y),
            yaw,
            self._vehicle.rear_axle_offset_m,
            self._vehicle.wheelbase_m,
        )

        projection = self._project_continuously(
            trajectory.route, front_x, front_y
        )
        if projection is None:
            self._publish_stop("NO_ROUTE_PROJECTION")
            return
        off_track = self._stop_outside_corridor and not projection.on_track
        if off_track and not self._recovery_enabled:
            self._publish_stop(
                f"OUTSIDE_CORRIDOR cte={projection.signed_offset:+.2f}m "
                f"limit={self._corridor_half_width:.2f}m"
            )
            return

        ahead = trajectory.ahead_of(projection)
        route_yaw = path_heading(
            trajectory.ahead_planar,
            ahead.segment_index,
            ahead.fraction,
            self._heading_window,
        )
        heading_error = wrap_angle(route_yaw - yaw)
        self._error_pub.publish(Float32(data=float(projection.signed_offset)))
        self._heading_error_pub.publish(Float32(data=float(heading_error)))
        self._publish_target_marker(trajectory, projection)

        self._route_s = travelled_arc_length(
            trajectory.planar, projection.segment_index, projection.fraction
        )
        handoff = self._handoff_step(
            now_ns, trajectory, float(state.odometry.twist.twist.linear.x), heading_error
        )
        if handoff.state == HANDED_OFF:
            self._publish_handoff(handoff)
            self._publish_status(
                f"HANDOFF AVOIDANCE zone=#{handoff.zone.label} "
                f"past_end={int(handoff.past_end)} "
                f"t={(now_ns - (self._handoff_since_ns or now_ns)) * 1e-9:.1f}s "
                "- cone_avoidance_1005 is driving, stanley silent"
            )
            return
        if handoff.brake:
            self._publish_stop(
                f"AVOIDANCE_HANDOFF zone=#{handoff.zone.label} stopping at the zone start "
                f"(to_start={handoff.zone.start_s - self._route_s:+.2f}m "
                f"heading_error={math.degrees(heading_error):+.1f}deg) - handing off when still"
            )
            return

        remaining = remaining_arc_length(
            trajectory.planar, projection.segment_index, projection.fraction
        )
        progress = end_of_route_reason(trajectory, remaining, self._goal_tolerance)
        if progress is not None:
            if trajectory.goal_mode == Trajectory.STOP_AT_END:
                if not self._goal_latched:
                    self._goal_latch_xy = (front_x, front_y)
                    self._left_goal = False
                self._goal_latched = True
                self._latched_trajectory_id = trajectory.trajectory_id
            self._publish_stop(progress)
            return

        signal = self._signal_verdict(now_ns, trajectory, projection)
        red_rolling: RedStopDecision | None = None
        if signal.stop:
            decision = self._red_stop(
                now_ns, signal, float(state.odometry.twist.twist.linear.x)
            )
            if decision.brake:
                frame = self._signal_tracker.latest
                self._publish_stop(
                    f"SIGNAL_RED zone=#{signal.zone.label} dir={signal.zone.direction} "
                    f"light={frame.state if frame else 'none'} "
                    f"red_frames={self._signal_tracker.stop_run(signal.zone.direction)}"
                    f"{self._signal_lost_note(signal.zone.direction)} "
                    f"({decision.reason}) - waiting for the confirmed go"
                )
                return
            red_rolling = decision
        else:
            self._red_braking = False

        pause = self._pause_step(now_ns, trajectory, float(state.odometry.twist.twist.linear.x))
        if pause.brake:
            self._publish_stop(
                f"PAUSE zone=#{pause.zone.label} ({pause.reason}) "
                f"held {pause.held_s:.1f}/{self._pause.hold_s:.1f}s - right-turn stop"
            )
            return

        if self._recovery_enabled:
            verdict = self._recover(
                now_ns, dt, trajectory, ahead, projection, pose, yaw, heading_error, off_track
            )
            if verdict is not None:
                return

        if heading_not_aligned(heading_error, self._max_heading_error):
            self._publish_stop(
                f"HEADING_NOT_ALIGNED heading_error="
                f"{math.degrees(heading_error):+.1f}deg limit="
                f"{math.degrees(self._max_heading_error):.0f}deg - align the "
                "vehicle with the route direction"
            )
            return

        speed = float(state.odometry.twist.twist.linear.x)
        desired = steering_angle(
            heading_error,
            projection.signed_offset,
            speed,
            self._cross_track_gain,
            self._softening,
            self._heading_gain,
        )
        # Oppose the yaw the path does not ask for (steering-motor lag).
        measured_yaw_rate = float(state.odometry.twist.twist.angular.z)
        path_curvature = (
            value_at_route_position(
                trajectory.ahead_curvatures, ahead.segment_index, ahead.fraction
            )
            if len(trajectory.ahead_curvatures) == len(trajectory.ahead_route)
            else 0.0
        )
        yaw_rate_error = max(speed, 0.0) * path_curvature - measured_yaw_rate
        damping_gain = self._yaw_damping_gain
        if self._damping_full_speed > 0.0 and speed > self._damping_full_speed:
            damping_gain *= self._damping_full_speed / speed
        yaw_correction = yaw_rate_damping(
            speed, path_curvature, measured_yaw_rate, damping_gain
        )
        feedforward_curvature = path_curvature
        if (
            self._feedforward_preview > 0.0
            and len(trajectory.ahead_curvatures) == len(trajectory.ahead_planar)
        ):
            feedforward_curvature = value_ahead_on_route(
                trajectory.ahead_planar,
                trajectory.ahead_curvatures,
                ahead.segment_index,
                ahead.fraction,
                max(speed, 0.0) * self._feedforward_preview,
            )
        feedforward = curvature_feedforward(
            feedforward_curvature, self._vehicle.wheelbase_m, self._feedforward_gain
        )
        desired = max(
            self._steering_min,
            min(self._steering_max, desired + feedforward + yaw_correction),
        )
        # ...and never command far ahead of where the wheels really are - by
        # more in a bend than on a straight (steering_lead_limit).
        lead = steering_lead_limit(
            path_curvature,
            self._max_steering_lead,
            self._max_steering_lead_curve,
            self._lead_straight_radius,
            self._lead_curve_radius,
        )
        measured_steering = self._fresh_measured_steering(now_ns)
        target = limit_steering_lead(desired, measured_steering, lead)
        steering = rate_limited(
            self._last_steering, target, self._vehicle.max_steering_rate_rad_s, dt
        )
        self._last_steering = steering
        self._yaw_rate_error_pub.publish(Float32(data=float(yaw_rate_error)))
        self._desired_steering_pub.publish(Float32(data=float(desired)))
        if measured_steering is not None:
            self._measured_steering_pub.publish(Float32(data=float(measured_steering)))

        target_speed = value_at_route_position(
            trajectory.target_speeds, projection.segment_index, projection.fraction
        )
        speed_cap = self._start_ramp.cap(speed, front_x, front_y)
        ramping = speed_cap < target_speed
        target_speed = min(target_speed, speed_cap)
        trim_hold = abs(path_curvature) > self._trim_max_curvature
        signal_limited = False
        if handoff.speed_cap_mps is not None and handoff.speed_cap_mps < target_speed:
            target_speed = handoff.speed_cap_mps
            trim_hold = True
        pause_limited = False
        if pause.speed_cap_mps is not None and pause.speed_cap_mps < target_speed:
            target_speed = pause.speed_cap_mps
            trim_hold = True
            pause_limited = True
        # Red, but still rolling up to the stop line: the zone speed.
        signal_cap = self._signal.zone_speed if red_rolling is not None else signal.speed_cap_mps
        if signal_cap is not None:
            # In a zone the command itself is capped (no trim on top); before
            # it only when the approach cap is what limits the speed.
            signal_limited = signal.zone is not None or signal_cap < target_speed
            target_speed = min(target_speed, signal_cap)
            trim_hold = trim_hold or signal_limited
        speed_command = self._speed_trim.command(target_speed, speed, dt, hold=trim_hold)

        command = self._actuator.command(steering, self._steering_sign)
        self._angle_pub.publish(Float32(data=float(command)))
        self._speed_pub.publish(Float32(data=speed_command_kmh(speed_command)))
        self._publish_brake(False)
        self._publish_status(
            f"TRACKING wp={projection.route_position:.1f} "
            f"cte={projection.signed_offset:+.2f}m "
            f"heading_error={math.degrees(heading_error):+.1f}deg "
            f"yaw={math.degrees(measured_yaw_rate):+.1f}deg/s "
            f"yaw_damp={math.degrees(yaw_correction):+.1f}deg "
            f"ff={math.degrees(feedforward):+.1f}deg "
            f"steer={math.degrees(steering):+.1f}deg "
            + (
                "steer_fb=none "
                if measured_steering is None
                else f"steer_fb={math.degrees(measured_steering):+.1f}deg "
            )
            + f"cmd={math.degrees(command):+.0f}deg "
            f"v={speed * 3.6:.1f}->{target_speed * 3.6:.1f}km/h "
            f"cmd_v={speed_command * 3.6:.1f}km/h "
            + ("loop" if trajectory.closed_loop else f"remaining={remaining:.1f}m")
            + (" ramp" if ramping else "")
            + (" trim_hold" if trim_hold else "")
            + (f" avoid_approach=#{handoff.zone.label}" if handoff.state == APPROACH else "")
            + (
                f" pause=#{pause.zone.label} to_stop={pause.room_m:.1f}m"
                if pause.room_m is not None
                else (f" pause_approach=#{pause.zone.label}" if pause_limited and pause.zone is not None else "")
            )
            + (
                f" signal_red=#{signal.zone.label} to_stop={red_rolling.room_m:.1f}m"
                if red_rolling is not None
                else f" signal=#{signal.zone.label}/{signal.zone.direction}"
                + self._stopline_note(now_ns)
                if signal.zone is not None
                else (f" signal_approach=#{signal.approaching.label}"
                      if signal_limited and signal.approaching is not None else "")
            )
        )

    def _release_goal_latch_at_route_start(self, now_ns: int) -> bool:
        """Resume after a goal stop once the vehicle is back at the route start."""

        state, trajectory = self._vehicle_state, self._trajectory
        if (
            self._restart_near_start <= 0.0
            or state is None
            or self._vehicle_state_ns is None
            or trajectory is None
            or now_ns - self._vehicle_state_ns > self._state_timeout_ns
            or now_ns > trajectory.valid_until_ns
            or state.odometry.header.frame_id != trajectory.frame_id
        ):
            return False
        pose = state.odometry.pose.pose
        front_x, front_y = front_axle_position(
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
            self._vehicle.rear_axle_offset_m,
            self._vehicle.wheelbase_m,
        )
        # On a loop whose end lies next to its start (3.3 m on 20260923_113433)
        # the vehicle coasting past the goal is already "at the start": on
        # 2026-09-26 that released the latch, the next cycle projected onto
        # the end again and latched, 20 times a second for two minutes. It
        # must have been driven away from the stop first.
        if self._goal_latch_xy is not None and not self._left_goal:
            if math.hypot(
                front_x - self._goal_latch_xy[0], front_y - self._goal_latch_xy[1]
            ) <= self._restart_near_start:
                return False
            self._left_goal = True
        projection = project_onto_route(trajectory.route, front_x, front_y)
        if projection is None or not back_at_route_start(
            trajectory.planar,
            projection.segment_index,
            projection.fraction,
            front_x,
            front_y,
            self._restart_near_start,
            self._corridor_half_width,
            # Well clear of where a goal stop leaves the vehicle.
            2.0 * self._goal_tolerance,
        ):
            return False
        self._goal_latched = False
        self._latched_trajectory_id = None
        self._goal_latch_xy = None
        # Continue from the start, not from the end the last projection saw.
        self._last_projection_position = projection.route_position
        self.get_logger().info(
            "goal latch released: vehicle is back at the route start "
            f"(wp {projection.route_position:.1f}, "
            f"{projection.signed_offset:+.2f} m off); tracking again"
        )
        return True

    def _blocking_reason(self, now_ns: int) -> str | None:
        """Why control cannot run this cycle; None when it can."""

        if self._goal_latched and not self._release_goal_latch_at_route_start(now_ns):
            return "GOAL_LATCHED"
        if self._vehicle_state is None or self._vehicle_state_ns is None:
            return "NO_VEHICLE_STATE"
        age_ns = now_ns - self._vehicle_state_ns
        if age_ns > self._state_timeout_ns:
            return f"STALE_VEHICLE_STATE age={age_ns * 1e-9:.2f}s"
        if self._trajectory is None:
            return "NO_TRAJECTORY"
        if now_ns > self._trajectory.valid_until_ns:
            expired = (now_ns - self._trajectory.valid_until_ns) * 1e-9
            return f"TRAJECTORY_EXPIRED by={expired:.2f}s"
        state_frame = self._vehicle_state.odometry.header.frame_id
        if state_frame != self._trajectory.frame_id:
            return (
                f"FRAME_MISMATCH trajectory='{self._trajectory.frame_id}' "
                f"vehicle_state='{state_frame}'"
            )
        return None

    def _recover(self, now_ns, dt, trajectory, ahead, projection, pose, yaw, heading_error, off_track):
        """Recovery step. Returns None when normal tracking should carry on
        (on track, or just regained), or the status published otherwise."""

        target = point_ahead_on_route(trajectory.ahead_route, ahead, self._recovery_lookahead)
        target_ahead = False
        bx = by = 0.0
        if target is not None:
            rx = float(pose.position.x) + self._vehicle.rear_axle_offset_m * math.cos(yaw)
            ry = float(pose.position.y) + self._vehicle.rear_axle_offset_m * math.sin(yaw)
            bx, by = to_body_frame(rx, ry, yaw, target.x, target.y)
            target_ahead = bx > 0.5

        # Before the first waypoint the projection is clamped to the route
        # start: drive up to it at the recovery speed without the clock running.
        before_start = (
            not projection.within_longitudinal_extent
            and projection.segment_index == 0
            and projection.fraction <= 0.0
        )
        verdict = self._recovery.step(
            now_ns, projection.signed_offset, heading_error, not off_track, target_ahead,
            hold_timer=before_start,
        )
        if verdict == Recovery.OFF:
            return None
        if verdict == Recovery.DONE:
            self.get_logger().info(
                f"route regained after {self._recovery.last_duration_s:.1f} s: "
                f"cte={projection.signed_offset:+.2f}m heading_error="
                f"{math.degrees(heading_error):+.1f}deg - tracking again"
            )
            self._start_ramp.reset()
            return None
        if verdict == Recovery.FAILED:
            self._publish_stop(
                f"RECOVERY_FAILED cte={projection.signed_offset:+.2f}m "
                f"heading_error={math.degrees(heading_error):+.1f}deg "
                + ("(target behind the vehicle) " if not target_ahead else "")
                + "- move the vehicle back onto the route"
            )
            return verdict

        desired = pursuit_steering(bx, by, self._vehicle.wheelbase_m)
        desired = max(self._steering_min, min(self._steering_max, desired))
        measured_steering = self._fresh_measured_steering(now_ns)
        target_steer = limit_steering_lead(
            desired, measured_steering, self._max_steering_lead_curve
        )
        steering = rate_limited(
            self._last_steering, target_steer, self._vehicle.max_steering_rate_rad_s, dt
        )
        self._last_steering = steering
        self._start_ramp.reset()
        self._speed_trim.reset()
        command = self._actuator.command(steering, self._steering_sign)
        self._publish_brake(False)
        self._angle_pub.publish(Float32(data=float(command)))
        self._speed_pub.publish(Float32(data=speed_command_kmh(self._recovery_speed)))
        self._desired_steering_pub.publish(Float32(data=float(desired)))
        self._publish_status(
            f"RECOVERING wp={projection.route_position:.1f} "
            f"cte={projection.signed_offset:+.2f}m "
            f"heading_error={math.degrees(heading_error):+.1f}deg "
            f"target=({bx:+.1f},{by:+.1f})m steer={math.degrees(steering):+.1f}deg "
            f"cmd={math.degrees(command):+.0f}deg v={self._recovery_speed * 3.6:.1f}km/h "
            f"t={self._recovery.elapsed_s(now_ns):.1f}s"
            + (" before_route_start" if before_start else "")
        )
        return verdict

    def _silent(self, status: str) -> bool:
        """While the vehicle is handed to cone_avoidance nothing may reach the
        actuator topics from here; the status still says what would happen."""

        if self._handoff.state != HANDED_OFF:
            return False
        self._publish_status(f"HANDOFF AVOIDANCE zone=#{self._handoff.zone.label} ({status})")
        return True

    def _publish_brake(self, value: bool) -> None:
        self._brake_pub.publish(Bool(data=bool(value)))
        self._last_brake_cmd = bool(value)

    def _publish_stop(self, reason: str) -> None:
        """Command a stop, holding the last steering angle (see pure_pursuit_node)."""

        if self._silent(f"STOP {reason}"):
            return
        self._start_ramp.reset()
        self._speed_trim.reset()
        self._angle_pub.publish(
            Float32(data=float(self._actuator.command(self._last_steering, self._steering_sign)))
        )
        self._speed_pub.publish(Float32(data=0.0))
        self._publish_brake(True)
        self._publish_status(f"STOP {reason}")

    def _publish_waiting(self, reason: str) -> None:
        """Report that nothing is commanded yet, and command nothing.

        Before the first vehicle state there is nothing to steer against and
        nothing to protect either: the vehicle has not been under this node's
        control. gnss_imu_pose_node publishes its first pose only after ~5 m
        of driving, and that drive is done by hand - a brake held on the CAN
        bus the whole time is exactly what would stop it. ros_can_bridge's
        own speed watchdog keeps the target speed at zero while no command
        arrives. Losing the state later (STALE_VEHICLE_STATE) still brakes.
        """

        self._start_ramp.reset()
        self._publish_status(f"WAITING {reason}")

    def _on_heading_state(self, msg: String) -> None:
        self._heading_state = msg.data

    def _on_heading_verified(self, msg: Bool) -> None:
        self._heading_verified = bool(msg.data)
        self._heading_verified_ns = self.get_clock().now().nanoseconds

    def _on_heading_source(self, msg: String) -> None:
        self._heading_source = msg.data
        self._heading_source_ns = self.get_clock().now().nanoseconds

    def _on_map_heading_pose(self, msg: Odometry) -> None:
        self._map_heading_pose = msg
        self._map_heading_pose_ns = self.get_clock().now().nanoseconds

    def _on_map_heading_verified(self, msg: Bool) -> None:
        self._map_heading_verified = bool(msg.data)
        self._map_heading_verified_ns = self.get_clock().now().nanoseconds

    def _map_heading_consistency_problem(self, now_ns: int) -> str | None:
        if not self._map_heading_consistency_required:
            return None
        if (
            self._heading_verified_ns is None
            or now_ns - self._heading_verified_ns > self._heading_status_timeout_ns
            or self._heading_source_ns is None
            or now_ns - self._heading_source_ns > self._heading_status_timeout_ns
        ):
            return "HEADING_STATUS_STALE"
        # Production ESKF policy is RTK-course-only. Refuse an old launch or
        # stale configuration that still reports a LiDAR-authored heading.
        if self._heading_source != "rtk_course":
            return f"HEADING_SOURCE_INVALID source={self._heading_source}"
        map_available = (
            self._map_heading_verified is True
            and self._map_heading_verified_ns is not None
            and now_ns - self._map_heading_verified_ns <= self._map_heading_timeout_ns
            and self._map_heading_pose is not None
            and self._map_heading_pose_ns is not None
            and now_ns - self._map_heading_pose_ns <= self._map_heading_timeout_ns
        )
        if not map_available:
            return None
        vehicle = self._vehicle_state.odometry
        reference = self._map_heading_pose
        if (
            vehicle.header.frame_id != "map"
            or reference.header.frame_id != "map"
            or vehicle.child_frame_id != reference.child_frame_id
        ):
            return "MAP_HEADING_FRAME_MISMATCH"
        vehicle_stamp_ns = (
            vehicle.header.stamp.sec * 1_000_000_000
            + vehicle.header.stamp.nanosec
        )
        reference_stamp_ns = (
            reference.header.stamp.sec * 1_000_000_000
            + reference.header.stamp.nanosec
        )
        if (
            vehicle_stamp_ns <= 0
            or reference_stamp_ns <= 0
            or abs(vehicle_stamp_ns - reference_stamp_ns)
            > self._map_heading_timeout_ns
        ):
            return "MAP_HEADING_TIME_MISMATCH"
        vehicle_yaw = yaw_from_quaternion(vehicle.pose.pose.orientation)
        reference_yaw = yaw_from_quaternion(reference.pose.pose.orientation)
        if not math.isfinite(vehicle_yaw) or not math.isfinite(reference_yaw):
            return "MAP_HEADING_INVALID"
        error = abs(wrap_angle(vehicle_yaw - reference_yaw))
        hold = getattr(self, "_map_heading_hold", None)
        if hold is None:  # no hold configured: stop on the first sample
            if error > self._map_heading_max_disagreement:
                return f"HEADING_DISAGREEMENT delta={math.degrees(error):.1f}deg"
            return None
        now_s = now_ns * 1e-9
        if hold.update(now_s, error, self._map_heading_max_disagreement):
            return (
                f"HEADING_DISAGREEMENT delta={math.degrees(error):.1f}deg "
                f"for {hold.held_s(now_s):.1f}s"
            )
        if error > self._map_heading_max_disagreement and (
            now_ns - self._map_heading_hold_warn_ns > 500_000_000
        ):
            self._map_heading_hold_warn_ns = now_ns
            self.get_logger().warning(
                f"heading disagreement {math.degrees(error):.1f} deg over "
                f"{math.degrees(self._map_heading_max_disagreement):.0f} for "
                f"{hold.held_s(now_s):.1f}/{hold.hold_s:.1f} s - still driving"
            )
        return None

    def _before_first_vehicle_state(self, now_ns: int) -> None:
        """No pose has ever arrived: creep straight to earn one, or just wait.

        Only with a live trajectory - a creep is a real command to the vehicle,
        so the rest of the stack has to be up. The creep ends with the first
        vehicle state (tracking takes over) or times out into a braked stop.
        """

        trajectory = self._trajectory
        if trajectory is None or now_ns > trajectory.valid_until_ns:
            self._publish_waiting("NO_VEHICLE_STATE")
            return
        if not self._heading_creep.started and not creep_permitted(
            self._heading_state, self._creep_requires_request
        ):
            # Localization is still getting the heading without motion (ESKF,
            # LiDAR): stay put. A creep, once started, runs to its end.
            self._publish_waiting(
                f"NO_VEHICLE_STATE heading={self._heading_state or 'UNKNOWN'}"
            )
            return
        feedback_problem = self._steering_feedback_problem(now_ns)
        if feedback_problem is not None:
            self._publish_stop(f"{feedback_problem} DURING_HEADING_ACQUISITION")
            return
        verdict = self._heading_creep.command(now_ns)
        if verdict == HeadingCreep.CREEP:
            self._publish_creep(now_ns)
        elif verdict == HeadingCreep.TIMEOUT:
            self._publish_stop(
                "HEADING_NOT_ACQUIRED "
                f"after={self._heading_creep.max_duration_s:.0f}s - no vehicle "
                "state arrived while creeping; restart path tracking to retry"
            )
        else:
            self._publish_waiting("NO_VEHICLE_STATE")

    def _publish_unverified_creep(self, now_ns: int, state: str) -> None:
        """Straight ahead (wheels centred on the measured zero) at the creep
        speed while the ESKF heading is unverified. No trim, no ramp, no
        damping: nothing about the pose is trusted yet."""

        if self._silent(f"CREEP HEADING_UNVERIFIED state={state}"):
            return
        self._start_ramp.reset()
        self._speed_trim.reset()
        self._last_steering = 0.0
        self._publish_brake(False)
        self._angle_pub.publish(
            Float32(data=float(self._actuator.command(0.0, self._steering_sign)))
        )
        self._speed_pub.publish(
            Float32(data=speed_command_kmh(self._unverified_creep.speed_mps))
        )
        creep = self._unverified_creep
        self._publish_status(
            f"CREEP HEADING_UNVERIFIED state={state} "
            f"t={creep.elapsed_s(now_ns) - creep.wait_s:.1f}/{creep.max_duration_s:.0f}s "
            f"d={creep.distance_m:.1f}/{creep.max_distance_m:.0f}m "
            f"v={creep.speed_mps * 3.6:.1f}km/h steer=0"
        )

    def _publish_creep(self, now_ns: int) -> None:
        """Straight ahead at the creep speed; the brake is released first so
        ros_can_bridge accepts the speed that follows it."""

        if self._silent("CREEP HEADING_ACQUISITION"):
            return
        self._start_ramp.reset()
        self._last_steering = 0.0
        self._publish_brake(False)
        self._angle_pub.publish(
            Float32(data=float(self._actuator.command(0.0, self._steering_sign)))
        )
        self._speed_pub.publish(Float32(data=speed_command_kmh(self._heading_creep.speed_mps)))
        self._publish_status(
            "CREEP HEADING_ACQUISITION "
            f"elapsed={self._heading_creep.elapsed_s(now_ns):.1f}s "
            f"max={self._heading_creep.max_duration_s:.0f}s "
            f"v={self._heading_creep.speed_mps * 3.6:.1f}km/h steer=0"
        )

    def _publish_status(self, status: str) -> None:
        self._status_pub.publish(String(data=status))
        # Log on a change of state. For STOP that includes its reason, so a stop
        # that turns from NO_VEHICLE_STATE into HEADING_NOT_ALIGNED is not silent.
        parts = status.split(" ", 2)
        head = " ".join(parts[:2]) if parts[0] == "STOP" else parts[0]
        if head != self._last_status:
            self._last_status = head
            self.get_logger().info(status)

    def _publish_target_marker(self, trajectory: TrackedTrajectory, projection) -> None:
        current = trajectory.route[projection.segment_index]
        following = trajectory.route[projection.segment_index + 1]
        fraction = projection.fraction

        marker = Marker()
        marker.header.frame_id = trajectory.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "stanley_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = current.x + fraction * (following.x - current.x)
        marker.pose.position.y = current.y + fraction * (following.y - current.y)
        marker.pose.position.z = current.z + fraction * (following.z - current.z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.6
        # Magenta: the front axle's foot point on the route, where the
        # cross-track error is measured. Distinct from pure_pursuit_node's cyan
        # lookahead and localization_monitor_node's red display-only sphere.
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        self._marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = StanleyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
