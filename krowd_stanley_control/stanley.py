"""The Stanley steering law itself, with no ROS types in sight.

Pure functions of numbers, unit-tested against
hand-computed values. Steering angles follow REP-103 - positive is a LEFT turn -
while RouteProjection.signed_offset is positive when a point lies LEFT of the
route, so a vehicle left of the route must steer negative (right).
"""

import bisect
import math


def projection_window(
    point_count: int,
    previous_route_position: float,
    backward_segments: int,
    forward_segments: int,
) -> tuple[int, int]:
    """Return point-slice bounds for a projection continuous with the last one.

    A nearest-point search over an entire route can jump from the beginning to
    the end when the recorded drive returns near its start.  At control rate a
    vehicle can only advance a few segments, so subsequent searches stay in a
    bounded part of the polyline.  The returned stop is exclusive and always
    leaves at least one complete segment.
    """

    if point_count < 2:
        raise ValueError("a projection window needs at least two route points")
    if backward_segments < 0 or forward_segments < 1:
        raise ValueError("projection window must allow >=0 back and >=1 forward")
    segment = min(max(int(math.floor(previous_route_position)), 0), point_count - 2)
    start = max(0, segment - backward_segments)
    stop = min(point_count, segment + forward_segments + 2)
    return start, stop


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to (-pi, pi]."""

    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def front_axle_position(
    x: float, y: float, yaw: float, rear_axle_offset_m: float, wheelbase_m: float
) -> tuple[float, float]:
    """Slide a base_link position forward to the front axle along the body x axis.

    Stanley measures the cross-track error at the FRONT axle - the point the
    steered wheels move - and its convergence result is derived there.
    rear_axle_offset_m is Vehicle's: the rear axle relative to base_link along +x.
    """

    distance = rear_axle_offset_m + wheelbase_m
    return (x + distance * math.cos(yaw), y + distance * math.sin(yaw))


def _point_at_arc(
    points: list[tuple[float, float]], cumulative: list[float], arc_m: float
) -> tuple[float, float]:
    arc_m = min(max(arc_m, 0.0), cumulative[-1])
    index = min(max(bisect.bisect_right(cumulative, arc_m) - 1, 0), len(points) - 2)
    length = cumulative[index + 1] - cumulative[index]
    fraction = (arc_m - cumulative[index]) / length if length > 0.0 else 0.0
    start, end = points[index], points[index + 1]
    return (
        start[0] + fraction * (end[0] - start[0]),
        start[1] + fraction * (end[1] - start[1]),
    )


def path_heading(
    points: list[tuple[float, float]],
    segment_index: int,
    fraction: float,
    window_m: float,
) -> float:
    """Direction of the route at a projected position, from its GEOMETRY.

    The chord between the points window_m / 2 behind and ahead along the arc,
    clamped to the route's ends. Deliberately not the recorded waypoint yaw:
    that is the mapping drive's own heading estimate, noise included, and
    Stanley passes the heading error to the steering at unity gain. With ~1 m
    waypoint spacing a single segment's direction also steps at every waypoint;
    the window averages that out. window_m <= 0 uses the projected segment alone.
    """

    if len(points) < 2:
        raise ValueError("a route needs at least two points to have a heading")
    index = min(max(segment_index, 0), len(points) - 2)
    clamped = min(max(fraction, 0.0), 1.0)

    if window_m > 0.0:
        cumulative = [0.0]
        for start, end in zip(points, points[1:]):
            cumulative.append(
                cumulative[-1] + math.hypot(end[0] - start[0], end[1] - start[1])
            )
        here = cumulative[index] + clamped * (cumulative[index + 1] - cumulative[index])
        behind = _point_at_arc(points, cumulative, here - 0.5 * window_m)
        ahead = _point_at_arc(points, cumulative, here + 0.5 * window_m)
        if math.hypot(ahead[0] - behind[0], ahead[1] - behind[1]) > 1e-6:
            return math.atan2(ahead[1] - behind[1], ahead[0] - behind[0])

    start, end = points[index], points[index + 1]
    return math.atan2(end[1] - start[1], end[0] - start[0])


def cross_track_steering(
    signed_offset_m: float, speed_mps: float, gain: float, softening_mps: float
) -> float:
    """Stanley's second term, atan(k e / (k_soft + v)), steering back to the route.

    signed_offset_m uses RouteProjection's convention (positive = left of the
    route), so the term is negated: left of the route steers right. The
    softening speed keeps the term finite at a standstill, where plain
    atan(k e / v) is not, and stops a crawling vehicle being told to turn hard
    for a small offset. Reversing is not modelled; negative speed counts as zero.
    """

    if softening_mps <= 0.0:
        raise ValueError(f"softening speed must be positive: {softening_mps}")
    return -math.atan2(gain * signed_offset_m, softening_mps + max(speed_mps, 0.0))


def steering_angle(
    heading_error_rad: float,
    signed_offset_m: float,
    speed_mps: float,
    cross_track_gain: float,
    softening_mps: float,
    heading_gain: float = 1.0,
) -> float:
    """delta = heading_gain * (route yaw - vehicle yaw) + atan(k e / (k_soft + v)).

    heading_gain is 1.0 in Stanley's law. It is exposed because this vehicle's
    heading comes from an RTK velocity course that is noisy at low speed, and at
    unity gain that noise reaches the steering undiminished.
    """

    return heading_gain * wrap_angle(heading_error_rad) + cross_track_steering(
        signed_offset_m, speed_mps, cross_track_gain, softening_mps
    )


def heading_not_aligned(heading_error_rad: float, limit_rad: float) -> bool:
    """True when the vehicle and route directions differ by more than limit_rad.

    limit_rad <= 0 disables the check.
    """

    return limit_rad > 0.0 and abs(wrap_angle(heading_error_rad)) > limit_rad


def yaw_rate_damping(
    speed_mps: float,
    path_curvature_per_m: float,
    measured_yaw_rate_rad_s: float,
    gain_s: float,
) -> float:
    """Steering correction (rad, road wheel) from the yaw-rate error.

        gain_s * (speed * curvature - measured yaw rate)

    The basic law only sees where the vehicle IS; with a steering motor that
    lags its command the vehicle is already turning back by the time the
    offset changes sign, and the law adds to that turn - the wide weaving of
    the 2026-09-20 drives. This term opposes the rotation the path does not ask
    for: on a straight, a vehicle yawing left at 10 deg/s gets 1.5 deg to the
    right at gain 0.15 s. On a curve driven correctly it is zero, because the
    path's own yaw rate is subtracted first. Stanley (the vehicle) used the
    same term - Thrun et al. 2006, section 9.2.

    Curvature and yaw rate are both positive to the LEFT. Reversing is not
    modelled: a negative speed counts as zero, like cross_track_steering.
    gain_s <= 0 disables the term.
    """

    if gain_s <= 0.0:
        return 0.0
    desired = max(speed_mps, 0.0) * path_curvature_per_m
    return gain_s * (desired - measured_yaw_rate_rad_s)


def curvature_feedforward(
    path_curvature_per_m: float, wheelbase_m: float, gain: float
) -> float:
    """gain * atan(wheelbase * curvature): the steering a bend of that curvature
    needs, scaled.

    Stanley's heading term alone supplies the bend's steering only at
    heading_gain 1.0. At 0.6 the missing 40 % has to come from the cross-track
    term, i.e. from a standing offset to the OUTSIDE of the bend: 1.9-2.5 m in
    the 4 m-radius corners of stanley_0924_191013, matching this model within
    0.1-0.2 m. Adding (1 - heading_gain) of the bend's steering from the
    route's own curvature (geometry, no sensor noise) removes that offset in
    the kinematic model without making the heading loop any faster.
    Positive curvature = left bend = positive (left) steering. gain <= 0 = 0.
    """

    if gain <= 0.0:
        return 0.0
    return gain * math.atan(wheelbase_m * path_curvature_per_m)


def value_ahead_on_route(
    points: list[tuple[float, float]],
    values: list[float],
    segment_index: int,
    fraction: float,
    ahead_m: float,
) -> float:
    """A per-waypoint value `ahead_m` along the route from a projected position
    (linear between waypoints, clamped to the route's end)."""

    if len(values) != len(points) or len(points) < 2:
        raise ValueError("one value per route point, at least two points")
    index = min(max(segment_index, 0), len(points) - 2)
    t = min(max(fraction, 0.0), 1.0)
    remaining = max(ahead_m, 0.0)
    position = t
    while True:
        (x0, y0), (x1, y1) = points[index], points[index + 1]
        length = math.hypot(x1 - x0, y1 - y0)
        left = (1.0 - position) * length
        if remaining <= left or index == len(points) - 2:
            if length > 0.0:
                position = min(position + remaining / length, 1.0)
            return values[index] + position * (values[index + 1] - values[index])
        remaining -= left
        index += 1
        position = 0.0


def steering_lead_limit(
    path_curvature_per_m: float,
    straight_lead_rad: float,
    curve_lead_rad: float,
    straight_radius_m: float,
    curve_radius_m: float,
) -> float:
    """How far the command may lead the wheels HERE: straight_lead_rad on a
    straight, curve_lead_rad in a bend of curve_radius_m, linear in curvature
    between the two radii.

    One number cannot serve both: a tight limit is what stops the command
    stacking up a swing the wheels have not made yet (the weaving of
    2026-09-20), but in a corner the controller needs the whole steering range
    at once, and a tight limit there is what let the vehicle run wide out of
    the 4.4 m bends on 2026-09-22 while the steering was still winding on.
    Blended by the route's own curvature, so the limit has opened up BEFORE the
    corner arrives.

    Same shape as the speed profile's curve cap (speed_profile.curve_radius_m)
    and the old Pure Pursuit lookahead: a weight of 0 at straight_radius_m and
    1 at curve_radius_m, clamped outside. Equal leads, or a non-positive
    radius pair, give the straight value everywhere.
    """

    if curve_lead_rad == straight_lead_rad or straight_radius_m <= 0.0 or curve_radius_m <= 0.0:
        return straight_lead_rad
    straight_k = 1.0 / straight_radius_m
    curve_k = 1.0 / curve_radius_m
    if curve_k <= straight_k:
        return straight_lead_rad
    weight = (abs(path_curvature_per_m) - straight_k) / (curve_k - straight_k)
    weight = min(1.0, max(0.0, weight))
    return straight_lead_rad + weight * (curve_lead_rad - straight_lead_rad)


def limit_steering_lead(
    target_rad: float, measured_rad: float | None, max_lead_rad: float
) -> float:
    """`target_rad` kept within `max_lead_rad` of the MEASURED road-wheel angle.

    The command may only run so far ahead of where the wheels really are.
    Without this, while the motor is still working through a left command the
    controller can already have stacked up a large right one, and the wheels
    then swing through the centre at full rate - overshoot built in before the
    vehicle has responded at all.

    max_lead_rad <= 0 or no measurement (None) passes the target through.
    """

    if max_lead_rad <= 0.0 or measured_rad is None:
        return target_rad
    return max(measured_rad - max_lead_rad, min(measured_rad + max_lead_rad, target_rad))
