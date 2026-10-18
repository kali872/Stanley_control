"""Closed-loop vehicle simulator for the Stanley/Pure Pursuit command topics."""

import math

import rclpy
import rclpy.time
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from krowd_control_interfaces.msg import Trajectory
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker

from krowd_stanley_control.bicycle_sim import (
    BicycleState,
    DelayedCommand,
    achieved_speed,
    step_bicycle,
)
from krowd_stanley_control.sim_sensors import (
    EstimationNoise,
    RtkCourseVerifier,
    SimulatedEncoder,
    prior_heading_pose,
)
from krowd_stanley_control.vehicle import SteeringActuator, steering_table

try:  # the steering encoder message; only the vehicle's CAN bridge defines it
    from ros_can_bridge_msgs.msg import EncoderAngleStamped
except ImportError:  # pragma: no cover
    EncoderAngleStamped = None


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class VehicleDynamicsSimNode(Node):
    def __init__(self):
        super().__init__("vehicle_dynamics_sim_node")
        self.declare_parameter("trajectory_topic", "/planning/trajectory")
        self.declare_parameter("angle_topic", "/simulation/target_angle")
        self.declare_parameter("speed_topic", "/simulation/target_speed")
        self.declare_parameter("brake_topic", "/simulation/brake")
        self.declare_parameter("odometry_topic", "/simulation/odometry")
        self.declare_parameter(
            "vehicle_marker_topic", "/simulation/vehicle_marker"
        )
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")
        # frame_id -> child_frame_id on /tf, so RViz draws the simulated vehicle
        # whichever way the map was launched. A plain localization launch has
        # no pose to publish that edge from (no GNSS in a simulation) and its
        # map_odom_tf_node (base_link_tf_source auto) yields to this one.
        self.declare_parameter("publish_tf", True)
        # > 0: play the ESKF's heading verification - publish
        # /ekf/heading_verified false at start and true after this many
        # seconds - so stanley_node's unverified-heading creep can be watched
        # in the simulator. 0 (default) publishes nothing.
        self.declare_parameter("heading_verified_after_sec", 0.0)
        # How /ekf/heading_verified is played (sim_sensors.RtkCourseVerifier):
        #   rtk_course - like the ESKF: false at start, true after
        #                course_samples straight fixes above course_min_speed.
        #                Until then the reported heading is off by
        #                heading_prior_error_deg (the saved prior) and the
        #                position by what that does through the antenna lever
        #                arm; at verification both snap to the truth.
        #   none       - nothing published, the estimate is right from the start.
        # heading_verified_after_sec > 0 replaces either with a timer.
        self.declare_parameter("heading_verification", "rtk_course")
        self.declare_parameter("heading_prior_error_deg", 5.0)
        self.declare_parameter("gps_lever_arm_m", 2.5038)
        self.declare_parameter("gps_rate_hz", 5.0)
        self.declare_parameter("course_samples", 3)
        self.declare_parameter("course_min_speed_mps", 1.0)
        self.declare_parameter("course_max_yaw_rate", 0.10)
        self.declare_parameter("course_max_gap_sec", 1.0)
        # ESKF estimation error on /simulation/odometry only (TF and the
        # footprint stay on the truth). Measured, see sim_sensors.py; 0 = exact.
        self.declare_parameter("pose_noise_m", 0.006)
        # 2026-09-27: 0.18 -> 0.07. 0.18 was the ESKF's own covariance (an
        # upper bound); the ESKF-vs-RTK position difference on three drives
        # (stanley_0924_191013, stanley_tuning_0926_202254/204115) is 5.4-7.9 cm
        # per axis. 0.18 made the simulated track wander +/-0.3 m on straights.
        self.declare_parameter("pose_drift_m", 0.07)
        self.declare_parameter("pose_drift_tau_sec", 10.0)
        self.declare_parameter("yaw_noise_deg", 0.14)
        self.declare_parameter("yaw_rate_noise_deg_s", 0.58)
        self.declare_parameter("noise_seed", 0)
        # The steering encoder: the simulated command-side angle, as
        # ros_can_bridge publishes /encoder. Empty topic = not published.
        self.declare_parameter("encoder_topic", "/simulation/encoder")
        self.declare_parameter("encoder_rate_hz", 50.0)
        self.declare_parameter("encoder_resolution_rad", 0.01)
        self.declare_parameter("encoder_noise_rad", 0.026)
        # true = every imperfection above and steering_lag_sec /
        # speed_deficit_kmph at 0, heading_verification none.
        self.declare_parameter("ideal", False)
        # Like ros_can_bridge (command_timeout_sec 0.2): no speed command for
        # this long = speed 0. 0 = hold the last command forever.
        self.declare_parameter("command_timeout_sec", 0.3)
        self.declare_parameter("wheelbase_m", 1.340)
        self.declare_parameter("vehicle_length_m", 2.464)
        self.declare_parameter("vehicle_width_m", 1.256)
        self.declare_parameter("steering_command_gain", 8.0)
        self.declare_parameter("steering_command_offset_rad", 0.0)
        # The same table the controllers read from vehicle.yaml, so the
        # simulated wheels turn by what the measured vehicle's would.
        self.declare_parameter("steering_table_encoder_rad", [0.0])
        self.declare_parameter("steering_table_left_tire_deg", [0.0])
        self.declare_parameter("steering_table_right_tire_deg", [0.0])
        self.declare_parameter("max_accel_mps2", 2.5)
        self.declare_parameter("max_decel_mps2", 4.0)
        self.declare_parameter("max_steering_rate_deg_s", 60.0)
        self.declare_parameter("simulation_rate_hz", 50.0)
        # The real vehicle's measured imperfections (stanley_tuning_0922_235531),
        # ON by default so the simulation is not kinder than the car: the
        # steering takes steering_lag_sec to start following /target_angle
        # (then slews at max_steering_rate_deg_s), and the drive MCU holds
        # speed_deficit_kmph less than /target_speed. 0 = the ideal model.
        self.declare_parameter("steering_lag_sec", 0.24)
        # How fast the steering MOTOR turns the road wheels, whatever the
        # command asks (deg/s of road wheel; 0 = only max_steering_rate_deg_s).
        # 2026-09-26, measured while the command led the encoder by > 0.5 rad
        # (the motor running flat out): 1.1-1.7 rad/s at the encoder = 8-13
        # deg/s of road wheel in stanley_tuning_0922_235531, stanley_0924_191013
        # and stanley_tuning_0926_204115. The 40 deg/s used until then was a
        # single 4.7 rad/s spike, and let the simulation turn in far faster
        # than the car (corner exits 0.2-0.3 m out vs 0.7-1.1 m on the car).
        self.declare_parameter("steering_motor_rate_deg_s", 12.0)
        self.declare_parameter("speed_deficit_kmph", 1.7)
        # The TRUE driven track as a latched nav_msgs/Path on driven_path_topic,
        # a point every driven_path_spacing_m; 0 spacing = not published.
        # stanley_dynamics_sim.launch.py sends it to /driven_path, the vehicle
        # RViz config's "Driven Path": its usual publisher, mola_slam_mapping's
        # map_odom_tf_node, builds it from the MOLA LiDAR pose and so never
        # publishes in a simulation.
        self.declare_parameter("driven_path_topic", "/simulation/driven_path")
        self.declare_parameter("driven_path_spacing_m", 0.2)
        self.declare_parameter("driven_path_max_points", 20000)
        # Frame the driven path is published in. RViz stores line points as
        # 32-bit floats: at a UTM northing of 4.13e6 m that is a 0.25 m grid,
        # which drew the path as a zig-zag staircase (2026-09-25). "map" keeps
        # the numbers small; until map <- frame_id is on TF the path goes out
        # in frame_id as before. Empty = frame_id.
        self.declare_parameter("driven_path_frame", "map")
        self.declare_parameter("initial_waypoint_index", 0)
        self.declare_parameter("initial_lateral_offset_m", 0.0)
        self.declare_parameter("initial_heading_offset_deg", 0.0)

        self._wheelbase = float(self.get_parameter("wheelbase_m").value)
        self._command_gain = float(
            self.get_parameter("steering_command_gain").value
        )
        self._command_offset = float(
            self.get_parameter("steering_command_offset_rad").value
        )
        if self._wheelbase <= 0.0 or self._command_gain <= 0.0:
            raise ValueError("wheelbase and steering command gain must be positive")
        table_command, table_road_wheel = steering_table(
            list(self.get_parameter("steering_table_encoder_rad").value),
            list(self.get_parameter("steering_table_left_tire_deg").value),
            list(self.get_parameter("steering_table_right_tire_deg").value),
        )
        self._actuator = SteeringActuator(
            gain=self._command_gain,
            offset_rad=self._command_offset,
            max_command_rad=math.inf,
            table_command_rad=table_command,
            table_road_wheel_rad=table_road_wheel,
        )

        self._max_accel = float(self.get_parameter("max_accel_mps2").value)
        self._max_decel = float(self.get_parameter("max_decel_mps2").value)
        self._max_steering_rate = math.radians(
            float(self.get_parameter("max_steering_rate_deg_s").value)
        )
        motor_rate = 0.0 if bool(self.get_parameter("ideal").value) else float(
            self.get_parameter("steering_motor_rate_deg_s").value
        )
        if motor_rate > 0.0:
            self._max_steering_rate = min(self._max_steering_rate, math.radians(motor_rate))
        rate = float(self.get_parameter("simulation_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("simulation_rate_hz must be positive")

        ideal = bool(self.get_parameter("ideal").value)
        self._ideal = ideal

        def real(name):
            return 0.0 if ideal else float(self.get_parameter(name).value)

        self._state = None
        self._z = 0.0
        self._route_xyz: list[tuple[float, float, float]] = []
        self._route_hint = 0
        self._target_speed = 0.0
        self._target_steering = 0.0
        self._steering_delay = DelayedCommand(
            real("steering_lag_sec")
        )
        self._speed_deficit = real("speed_deficit_kmph") / 3.6
        self._noise = EstimationNoise(
            position_sigma_m=real("pose_noise_m"),
            yaw_sigma_rad=math.radians(real("yaw_noise_deg")),
            yaw_rate_sigma_rad_s=math.radians(real("yaw_rate_noise_deg_s")),
            drift_sigma_m=real("pose_drift_m"),
            drift_tau_s=float(self.get_parameter("pose_drift_tau_sec").value),
            seed=int(self.get_parameter("noise_seed").value),
        )
        self._encoder = SimulatedEncoder(
            resolution_rad=real("encoder_resolution_rad"),
            noise_rad=real("encoder_noise_rad"),
            seed=int(self.get_parameter("noise_seed").value) + 1,
        )
        self._noise_dt = 0.0
        self._brake = True
        self._last_time = None
        self._command_timeout = float(self.get_parameter("command_timeout_sec").value)
        self._last_speed_cmd_ns: int | None = None
        self._yaw_rate = 0.0

        self._publisher = self.create_publisher(
            Odometry, str(self.get_parameter("odometry_topic").value), 10
        )
        self._tf_broadcaster = (
            tf2_ros.TransformBroadcaster(self)
            if bool(self.get_parameter("publish_tf").value)
            else None
        )
        self._heading_verified_after = float(
            self.get_parameter("heading_verified_after_sec").value
        )
        mode = str(self.get_parameter("heading_verification").value).strip().lower()
        if self._heading_verified_after > 0.0:
            mode = "after_sec"
        elif ideal:
            mode = "none"
        if mode not in ("rtk_course", "after_sec", "none"):
            raise ValueError(
                f"heading_verification must be rtk_course or none, not {mode!r}"
            )
        self._heading_mode = mode
        self._heading_verified = mode == "none"
        self._prior_error = (
            0.0 if mode == "none"
            else math.radians(float(self.get_parameter("heading_prior_error_deg").value))
        )
        self._lever_arm = float(self.get_parameter("gps_lever_arm_m").value)
        self._course = RtkCourseVerifier(
            samples=int(self.get_parameter("course_samples").value),
            min_speed_mps=float(self.get_parameter("course_min_speed_mps").value),
            max_yaw_rate_rad_s=float(self.get_parameter("course_max_yaw_rate").value),
            max_gap_s=float(self.get_parameter("course_max_gap_sec").value),
        )
        self._heading_verified_pub = None
        self._heading_start_ns = self.get_clock().now().nanoseconds
        if mode != "none":
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            latched = QoSProfile(depth=1)
            latched.reliability = ReliabilityPolicy.RELIABLE
            latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self._heading_verified_pub = self.create_publisher(
                Bool, "/ekf/heading_verified", latched
            )
            self._heading_verified_pub.publish(Bool(data=False))
            period = (
                0.5 if mode == "after_sec"
                else 1.0 / max(float(self.get_parameter("gps_rate_hz").value), 0.1)
            )
            self.create_timer(period, self._heading_verified_tick)

        self._encoder_pub = None
        encoder_topic = str(self.get_parameter("encoder_topic").value).strip()
        if encoder_topic and EncoderAngleStamped is not None:
            self._encoder_pub = self.create_publisher(
                EncoderAngleStamped, encoder_topic, 10
            )
            encoder_rate = float(self.get_parameter("encoder_rate_hz").value)
            if encoder_rate <= 0.0:
                raise ValueError("encoder_rate_hz must be positive")
            self.create_timer(1.0 / encoder_rate, self._publish_encoder)
        elif encoder_topic:
            self.get_logger().warning(
                "ros_can_bridge_msgs is not available: no simulated encoder, so "
                "stanley_node must run with steering_feedback_required false"
            )
        self._marker_publisher = self.create_publisher(
            Marker, str(self.get_parameter("vehicle_marker_topic").value), 10
        )
        # Until 2026-09-25 the driven track was also drawn as an orange line
        # on this topic (ns "driven_path", id 1); RViz keeps a marker until
        # told otherwise, so remove it from views that still show it.
        stale = Marker()
        stale.header.frame_id = str(self.get_parameter("frame_id").value)
        stale.ns = "driven_path"
        stale.id = 1
        stale.action = Marker.DELETE
        self._marker_publisher.publish(stale)
        self._path_spacing = float(self.get_parameter("driven_path_spacing_m").value)
        self._path_max_points = max(int(self.get_parameter("driven_path_max_points").value), 2)
        self._path_points: list[tuple[float, float, float]] = []
        self._path_publisher = None
        if self._path_spacing > 0.0:
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            path_qos = QoSProfile(depth=1)
            path_qos.reliability = ReliabilityPolicy.RELIABLE
            path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self._path_publisher = self.create_publisher(
                Path, str(self.get_parameter("driven_path_topic").value), path_qos
            )
            self._path_frame = str(self.get_parameter("driven_path_frame").value).strip()
            self._path_transform = None  # (frame, 3x3 rotation, translation)
            self._path_tf_buffer = tf2_ros.Buffer()
            self._path_tf_listener = tf2_ros.TransformListener(self._path_tf_buffer, self)
            self.create_timer(0.2, self._publish_driven_path)
        self.create_subscription(
            Trajectory,
            str(self.get_parameter("trajectory_topic").value),
            self._on_trajectory,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("angle_topic").value),
            self._on_angle,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("speed_topic").value),
            self._on_speed,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("brake_topic").value),
            self._on_brake,
            10,
        )
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "Closed-loop bicycle simulator waiting for /planning/trajectory; "
            "Stanley commands feed /simulation/odometry; vehicle footprint on "
            f"{self.get_parameter('vehicle_marker_topic').value}"
        )

    def _on_trajectory(self, message: Trajectory) -> None:
        if self._state is not None or not message.path.poses:
            return
        index = min(
            max(int(self.get_parameter("initial_waypoint_index").value), 0),
            len(message.path.poses) - 1,
        )
        pose = message.path.poses[index].pose
        route_yaw = yaw_from_quaternion(pose.orientation)
        heading = route_yaw + math.radians(
            float(self.get_parameter("initial_heading_offset_deg").value)
        )
        lateral = float(self.get_parameter("initial_lateral_offset_m").value)
        self._state = BicycleState(
            x=pose.position.x - lateral * math.sin(route_yaw),
            y=pose.position.y + lateral * math.cos(route_yaw),
            yaw=heading,
            speed_mps=0.0,
            steering_rad=0.0,
        )
        self._z = pose.position.z
        # The route's own height, so the vehicle follows a hill instead of
        # staying at the start's height (20260927_211309 climbs 6.7 m: the
        # vehicle vanished under the point cloud, 2026-09-27).
        self._route_xyz = [
            (p.pose.position.x, p.pose.position.y, p.pose.position.z)
            for p in message.path.poses
        ]
        self._route_hint = index
        self._last_time = self.get_clock().now()
        self.get_logger().info(
            f"Simulator initialized at waypoint {index}: "
            f"({self._state.x:.2f}, {self._state.y:.2f}), "
            f"yaw {math.degrees(self._state.yaw):.1f} deg; steering lag "
            f"{self._steering_delay.delay_s:.2f} s, wheels slew at "
            f"{math.degrees(self._max_steering_rate):.0f} deg/s, speed deficit "
            f"{self._speed_deficit * 3.6:.1f} km/h; "
            + ("IDEAL sensors" if self._ideal else
               f"pose noise {self._noise.position_sigma * 100:.1f} cm + drift, "
               f"yaw noise {math.degrees(self._noise.yaw_sigma):.2f} deg, "
               f"encoder {self._encoder.resolution:.3f} rad steps")
            + f"; heading {self._heading_mode}"
            + ("" if self._heading_verified else
               f" (prior off by {math.degrees(self._prior_error):+.1f} deg until verified)")
        )
        self._publish()

    def _on_angle(self, message: Float32) -> None:
        command = float(message.data)
        if math.isfinite(command):
            self._target_steering = self._actuator.road_wheel(command)
            self._steering_delay.push(
                self.get_clock().now().nanoseconds * 1e-9, self._target_steering
            )

    def _on_speed(self, message: Float32) -> None:
        speed_kmph = float(message.data)
        if math.isfinite(speed_kmph):
            self._target_speed = max(speed_kmph, 0.0) / 3.6
            self._last_speed_cmd_ns = self.get_clock().now().nanoseconds

    def _on_brake(self, message: Bool) -> None:
        self._brake = bool(message.data)

    def _tick(self) -> None:
        if self._state is None:
            return
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if not 0.0 < dt <= 0.2:
            return
        stale = (
            self._command_timeout > 0.0
            and (self._last_speed_cmd_ns is None
                 or (now.nanoseconds - self._last_speed_cmd_ns) * 1e-9 > self._command_timeout)
        )
        target_speed = (
            0.0 if (self._brake or stale) else achieved_speed(self._target_speed, self._speed_deficit)
        )
        self._state, self._yaw_rate = step_bicycle(
            self._state,
            target_speed,
            self._steering_delay.value(now.nanoseconds * 1e-9),
            dt,
            self._wheelbase,
            self._max_accel,
            self._max_decel,
            self._max_steering_rate,
        )
        self._noise_dt = dt
        self._z = self._ground_z(self._state.x, self._state.y)
        self._record_driven_path()
        self._publish()

    def _heading_verified_tick(self) -> None:
        if self._heading_verified:
            return
        now_ns = self.get_clock().now().nanoseconds
        elapsed = (now_ns - self._heading_start_ns) * 1e-9
        if self._heading_mode == "after_sec":
            verified = elapsed >= self._heading_verified_after
        elif self._state is None:
            return
        else:  # rtk_course: one GPS fix
            verified = self._course.update(
                now_ns * 1e-9, self._state.speed_mps, self._yaw_rate
            )
        if verified:
            self._heading_verified = True
            self._heading_verified_pub.publish(Bool(data=True))
            self.get_logger().info(
                f"simulated ESKF heading verified ({self._heading_mode}) after "
                f"{elapsed:.1f} s: the pose snaps "
                f"{math.degrees(self._prior_error):+.1f} deg / "
                f"{abs(2.0 * self._lever_arm * math.sin(0.5 * self._prior_error)):.2f} m "
                "to the truth"
            )

    def _ground_z(self, x: float, y: float) -> float:
        """Route height at the point of the route nearest (x, y), searched
        around the last match so a loop's end cannot capture its start."""

        pts = self._route_xyz
        if len(pts) < 2:
            return self._z
        lo = max(self._route_hint - 10, 0)
        hi = min(self._route_hint + 40, len(pts) - 2)
        best = None
        for i in range(lo, hi + 1):
            (x0, y0, z0), (x1, y1, z1) = pts[i], pts[i + 1]
            dx, dy = x1 - x0, y1 - y0
            length2 = dx * dx + dy * dy
            t = 0.0 if length2 <= 0.0 else min(max(((x - x0) * dx + (y - y0) * dy) / length2, 0.0), 1.0)
            d2 = (x0 + t * dx - x) ** 2 + (y0 + t * dy - y) ** 2
            if best is None or d2 < best[0]:
                best = (d2, i, z0 + t * (z1 - z0))
        self._route_hint = best[1]
        return best[2]

    def _record_driven_path(self) -> None:
        if self._path_publisher is None:
            return
        here = (self._state.x, self._state.y, self._z)
        last = self._path_points[-1] if self._path_points else None
        if last is None or math.hypot(here[0] - last[0], here[1] - last[1]) >= self._path_spacing:
            self._path_points.append(here)
            if len(self._path_points) > self._path_max_points:
                del self._path_points[: len(self._path_points) - self._path_max_points]

    def _driven_path_transform(self):
        """(frame, rotation rows, translation) taking frame_id points into
        driven_path_frame, looked up once (the map <- utm edge is static)."""

        source = str(self.get_parameter("frame_id").value)
        if not self._path_frame or self._path_frame == source:
            return None
        if self._path_transform is None:
            try:
                t = self._path_tf_buffer.lookup_transform(
                    self._path_frame, source, rclpy.time.Time()
                ).transform
            except Exception:  # not on TF (yet): publish in frame_id
                return None
            x, y, z, w = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
            rotation = (
                (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
            )
            translation = (t.translation.x, t.translation.y, t.translation.z)
            self._path_transform = (self._path_frame, rotation, translation)
            self.get_logger().info(
                f"driven path published in '{self._path_frame}' (small coordinates "
                "for RViz) instead of " + repr(source)
            )
        return self._path_transform

    def _publish_driven_path(self) -> None:
        if len(self._path_points) < 2:
            return
        stamp = self.get_clock().now().to_msg()
        transform = self._driven_path_transform()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = (
            transform[0] if transform else str(self.get_parameter("frame_id").value)
        )
        for x, y, ground in self._path_points:
            z = ground + 0.05
            if transform:
                r, t = transform[1], transform[2]
                px = r[0][0] * x + r[0][1] * y + r[0][2] * z + t[0]
                py = r[1][0] * x + r[1][1] * y + r[1][2] * z + t[1]
                pz = r[2][0] * x + r[2][1] * y + r[2][2] * z + t[2]
            else:
                px, py, pz = x, y, z
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = pz
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._path_publisher.publish(path)

    def _publish_encoder(self) -> None:
        steering = self._state.steering_rad if self._state is not None else 0.0
        message = EncoderAngleStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "steering"
        message.angle_rad = float(self._encoder.read(self._actuator.command(steering)))
        self._encoder_pub.publish(message)

    def _estimate(self):
        """(x, y, yaw, yaw_rate) as the ESKF would report the true state."""

        x, y, yaw = self._state.x, self._state.y, self._state.yaw
        if not self._heading_verified and self._prior_error != 0.0:
            x, y, yaw = prior_heading_pose(x, y, yaw, self._prior_error, self._lever_arm)
        noise = self._noise.sample(self._noise_dt)
        self._noise_dt = 0.0
        return (
            x + noise.dx,
            y + noise.dy,
            yaw + noise.dyaw,
            self._yaw_rate + noise.dyaw_rate,
        )

    def _publish(self) -> None:
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("frame_id").value)
        message.child_frame_id = str(self.get_parameter("child_frame_id").value)
        est_x, est_y, est_yaw, est_yaw_rate = self._estimate()
        message.pose.pose.position.x = est_x
        message.pose.pose.position.y = est_y
        message.pose.pose.position.z = self._z
        message.pose.pose.orientation.z = math.sin(0.5 * est_yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * est_yaw)
        message.twist.twist.linear.x = self._state.speed_mps
        message.twist.twist.angular.z = est_yaw_rate
        self._publisher.publish(message)

        # TF and the footprint: the TRUE vehicle, so RViz shows where the car
        # is while Stanley acts on the estimate.
        truth = message.pose.pose.__class__()
        truth.position.x = self._state.x
        truth.position.y = self._state.y
        truth.position.z = self._z
        truth.orientation.z = math.sin(0.5 * self._state.yaw)
        truth.orientation.w = math.cos(0.5 * self._state.yaw)
        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = message.child_frame_id
            transform.transform.translation.x = truth.position.x
            transform.transform.translation.y = truth.position.y
            transform.transform.translation.z = truth.position.z
            transform.transform.rotation = truth.orientation
            self._tf_broadcaster.sendTransform(transform)

        # Publish the physical vehicle footprint directly. The localization
        # monitor also emits a small point marker, but that is easy to lose
        # under a dense point cloud and does not show vehicle heading.
        # Attached to base_link when this node publishes that TF: RViz
        # resolves TF in doubles, whereas a marker pose at UTM magnitude lands
        # on its 0.25 m float grid and the box jumps.
        marker = Marker()
        marker.header = message.header
        marker.ns = "dynamics_vehicle"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = True
        if self._tf_broadcaster is not None:
            marker.header.frame_id = message.child_frame_id
            marker.pose.orientation.w = 1.0
            marker.pose.position.z = 0.45
        else:
            marker.pose = truth
            marker.pose.position.z += 0.45
        marker.scale.x = float(self.get_parameter("vehicle_length_m").value)
        marker.scale.y = float(self.get_parameter("vehicle_width_m").value)
        marker.scale.z = 0.9
        marker.color.r = 0.05
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 1.0
        self._marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleDynamicsSimNode()
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
