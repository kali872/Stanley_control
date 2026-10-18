"""Vehicle geometry: what the car IS, as opposed to how the controller is tuned.

Single source of truth for the whole stack (config/vehicle.yaml). These numbers
used to live inside krowd_embedded_mpc/mpc_algorithm_embedded.py, which meant
every consumer either imported the MPC or copied the value; wheel_odom_bicycle_node
still carries its own 1.340 default for exactly that reason. Putting a different
vehicle on this code should be one YAML edit, not a grep.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Vehicle:
    """Measured geometry plus the steering limit derived from it."""

    wheelbase_m: float
    track_width_m: float
    min_turning_radius_m: float
    max_steering_angle_rad: float
    max_steering_rate_rad_s: float
    # Where the rear axle sits relative to base_link, along base_link's +x.
    # Pure Pursuit's geometry is derived at the REAR AXLE: steering at a point
    # that is not the rear axle biases the whole controller, and the bias grows
    # with the offset. 0.0 declares "base_link is the rear axle" - which is an
    # assumption worth actually measuring on the vehicle, not inheriting.
    rear_axle_offset_m: float

    @property
    def max_curvature(self) -> float:
        return math.tan(self.max_steering_angle_rad) / self.wheelbase_m

    @property
    def implied_turning_radius_m(self) -> float:
        """The turning radius max_steering_angle_rad actually commands."""

        return self.wheelbase_m / math.tan(self.max_steering_angle_rad)


@dataclass(frozen=True)
class SteeringActuator:
    """How a road-wheel angle becomes the /target_angle the steer MCU takes.

    The steer Teensy's angle is its encoder count over a fixed gear ratio - a
    steering-COLUMN-side angle, not the road wheel's. Measured on the 0912_2
    drive (encoder against the road-wheel angle the IMU yaw rate and RTK speed
    imply, kinematic bicycle): about 8.0 command radians per road-wheel radian,
    consistent from 15 to 80 deg of command. The MCU also REJECTS a command
    beyond max_command_rad (ANGLE_LIMIT_RAD, 1.7453) rather than clamping it,
    so the command is clamped here first.
    """

    gain: float
    offset_rad: float
    max_command_rad: float
    # Optional measured table (see steering_table): command rad <-> road-wheel
    # rad, both strictly increasing through (0, 0). When present it replaces
    # `gain`, which is one number for a relation that is neither linear nor
    # symmetric: the 2026-09-19 tyre-angle measurement gives about 6.7 command
    # rad per road-wheel rad (7.2 to the right, 6.2 to the left), so gain 8.0
    # turned the wheels ~20 % further than the controller asked.
    table_command_rad: tuple = ()
    table_road_wheel_rad: tuple = ()

    @property
    def has_table(self) -> bool:
        return len(self.table_command_rad) >= 3

    def validate(self) -> None:
        if self.gain <= 0.0:
            raise ValueError(f"steering command gain must be positive: {self.gain}")
        if self.max_command_rad <= abs(self.offset_rad):
            raise ValueError(
                f"steering command limit {self.max_command_rad} must exceed the "
                f"offset {self.offset_rad}"
            )
        if not self.has_table:
            return
        command, wheel = self.table_command_rad, self.table_road_wheel_rad
        if len(command) != len(wheel):
            raise ValueError("steering table columns differ in length")
        if any(b <= a for a, b in zip(command, command[1:])) or any(
            b <= a for a, b in zip(wheel, wheel[1:])
        ):
            raise ValueError("steering table must be strictly increasing in both columns")
        if 0.0 not in command or abs(wheel[command.index(0.0)]) > 1e-9:
            raise ValueError("steering table must contain the straight-ahead point (0, 0)")
        # Commands are clamped to +/-max_command_rad RAW; centred on the offset
        # that is an asymmetric range, and the table must cover both ends.
        lowest = -self.max_command_rad - self.offset_rad
        highest = self.max_command_rad - self.offset_rad
        if lowest < command[0] - 1e-9 or highest > command[-1] + 1e-9:
            raise ValueError(
                f"steering table covers [{command[0]}, {command[-1]}] rad (centred) "
                f"but commands reach [{lowest:.3f}, {highest:.3f}]"
            )

    def command(self, road_wheel_rad: float, sign: float = 1.0) -> float:
        if self.has_table:
            raw = _interpolate(
                self.table_road_wheel_rad, self.table_command_rad, sign * road_wheel_rad
            ) + self.offset_rad
        else:
            raw = sign * self.gain * road_wheel_rad + self.offset_rad
        return max(-self.max_command_rad, min(self.max_command_rad, raw))

    def road_wheel(self, command_rad: float, sign: float = 1.0) -> float:
        """The road-wheel angle a command produces: command()'s inverse."""

        centred = command_rad - self.offset_rad
        if self.has_table:
            return sign * _interpolate(
                self.table_command_rad, self.table_road_wheel_rad, centred
            )
        return sign * centred / self.gain

    def road_wheel_limits_rad(self, sign: float = 1.0) -> tuple[float, float]:
        """(most negative, most positive) road-wheel angle the command range can
        express - the RIGHT and LEFT limit for sign +1.

        Separate sides since 2026-09-21: the measured table is not symmetric
        (99 deg of command is 13.7 deg to the right but 15.7 deg to the left),
        and clamping both sides to the smaller one threw the difference away -
        on a left hairpin that is a 5.5 m turning radius where 4.8 m was
        available. Without a table both sides are road_wheel_limit_rad.
        """

        # The RAW command range is +/-max_command_rad; road_wheel() takes the
        # offset off, so with a non-zero offset the two sides differ even
        # without a table (2026-09-23: straight ahead is -0.21 rad on this
        # session's encoder zero, so the right has 1.52 rad of travel and the
        # left 1.94).
        ends = (
            self.road_wheel(-self.max_command_rad, sign),
            self.road_wheel(self.max_command_rad, sign),
        )
        return min(ends), max(ends)

    @property
    def road_wheel_limit_rad(self) -> float:
        """The largest road-wheel angle the command range can still express.

        With a table the two sides differ (the left tyre turns further than
        the right one); the smaller side is the limit, so a command is never
        clamped on one side only.
        """

        lowest, highest = self.road_wheel_limits_rad()
        return min(abs(lowest), abs(highest))


def _interpolate(xs, ys, x: float) -> float:
    """Piecewise-linear y(x), clamped to the table's ends."""

    if x <= xs[0]:
        return float(ys[0])
    if x >= xs[-1]:
        return float(ys[-1])
    for index in range(1, len(xs)):
        if x <= xs[index]:
            fraction = (x - xs[index - 1]) / (xs[index] - xs[index - 1])
            return float(ys[index - 1] + fraction * (ys[index] - ys[index - 1]))
    return float(ys[-1])


def bicycle_angle_rad(left_deg: float, right_deg: float) -> float:
    """Single-track (bicycle) steering angle of an Ackermann pair of tyre angles:
    cot(delta) = (cot(left) + cot(right)) / 2 - the same reduction
    krowd_final_control.steering_calibration uses on the same measurement."""

    cots = [
        1.0 / math.tan(math.radians(angle))
        for angle in (left_deg, right_deg)
        if abs(angle) > 1e-9
    ]
    if not cots:
        return 0.0
    if min(cots) < 0.0 < max(cots):
        raise ValueError(f"left/right tyre angles disagree in sign: {left_deg}, {right_deg}")
    return math.atan(len(cots) / sum(cots))


def steering_table(encoder_rad, left_tire_deg, right_tire_deg) -> tuple[tuple, tuple]:
    """(command rad, road-wheel rad) columns from the team's tyre-angle table.

    Fewer than 3 rows means "no table" (the ROS parameter default) and returns
    two empty tuples, so SteeringActuator falls back to its gain.
    """

    rows = sorted(zip(encoder_rad, left_tire_deg, right_tire_deg))
    if len(rows) < 3:
        return (), ()
    if not (len(encoder_rad) == len(left_tire_deg) == len(right_tire_deg)):
        raise ValueError("steering table columns differ in length")
    return (
        tuple(float(row[0]) for row in rows),
        tuple(bicycle_angle_rad(row[1], row[2]) for row in rows),
    )


MPS_TO_KMH = 3.6


def speed_command_kmh(speed_mps: float) -> float:
    """The /target_speed value for a speed in m/s: km/h, never negative.

    ros_can_bridge multiplies /target_speed by 100 into CAN 0x110 and the drive
    MCU reads that as km/h x 100 (mcu_code/*: "실제 속도(km/h) x 100"). The
    controllers here work in m/s (the Trajectory, nav_msgs, the tests), so the
    conversion happens once, at the topic. Until 2026-09-19 the m/s value was
    published as is and the vehicle ran at 1/3.6 of the route speed. The MPC
    stack's can_adapter sends km/h the same way (speed_scale_mps_to_kmh 3.6).
    """

    return max(float(speed_mps), 0.0) * MPS_TO_KMH


def steering_limit_from_turning_radius(
    wheelbase_m: float, min_turning_radius_m: float
) -> float:
    """The road-wheel angle that produces `min_turning_radius_m`.

    The bicycle-model inverse of R = L / tan(delta). Note this is the angle of
    the (virtual, centred) FRONT ROAD WHEEL, not a steering-wheel angle: on a
    car with a steering ratio the two differ by that ratio and are not
    interchangeable anywhere in this package.
    """

    if wheelbase_m <= 0.0:
        raise ValueError(f"wheelbase_m must be positive: {wheelbase_m}")
    if min_turning_radius_m <= 0.0:
        raise ValueError(
            f"min_turning_radius_m must be positive: {min_turning_radius_m}"
        )
    return math.atan(wheelbase_m / min_turning_radius_m)


def implausible_steering_limit_reason(vehicle: Vehicle) -> str | None:
    """Why this steering limit looks like a unit mistake; None when it is sane.

    The one mistake this is here to catch: commanding a STEERING-WHEEL angle
    through an interface that means a ROAD-WHEEL angle. /target_angle is
    radians of road-wheel angle (ros_can_bridge encodes it as rad*100 on CAN
    0x120) and a steering-wheel figure put there is larger by the steering
    ratio, which reads as a physically impossible limit rather than as a wrong
    unit. A warning, not an error: an unmeasured MIN_TURNING_RADIUS is just as
    likely to be the thing that is wrong.
    """

    if vehicle.max_steering_angle_rad >= 0.5 * math.pi:
        return "steering limit is at or beyond 90 deg, which commands no radius at all"
    radius = vehicle.implied_turning_radius_m
    if radius < 0.5 * vehicle.track_width_m:
        return (
            f"steering limit {math.degrees(vehicle.max_steering_angle_rad):.1f} deg "
            f"implies a {radius:.2f} m turning radius, inside the vehicle's own "
            f"half track width ({0.5 * vehicle.track_width_m:.2f} m). This is the "
            "signature of a steering-WHEEL angle used where a road-WHEEL angle is "
            "meant; /target_angle is a road-wheel angle in radians"
        )
    return None
