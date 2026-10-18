import math

import pytest

from krowd_stanley_control.stanley import (
    cross_track_steering,
    front_axle_position,
    heading_not_aligned,
    path_heading,
    projection_window,
    steering_angle,
    wrap_angle,
)


WHEELBASE = 1.340
GAIN = 1.0
SOFTENING = 1.0


def test_projection_window_prevents_a_jump_to_a_nearby_route_end():
    start, stop = projection_window(508, 2.4, 5, 30)
    assert start == 0
    assert stop == 34
    assert 493 not in range(start, stop)


def test_projection_window_moves_with_progress_and_clamps_at_the_end():
    assert projection_window(508, 250.8, 5, 30) == (245, 282)
    assert projection_window(508, 506.9, 5, 30) == (501, 508)


def test_on_the_route_and_aligned_commands_no_steering():
    assert steering_angle(0.0, 0.0, 3.0, GAIN, SOFTENING) == pytest.approx(0.0)


def test_left_of_the_route_steers_right_by_the_stanley_law():
    """signed_offset is positive LEFT of the route; REP-103 steering is positive
    LEFT. A vehicle left of the route must therefore steer right."""

    speed = 2.0
    left = steering_angle(0.0, 0.5, speed, GAIN, SOFTENING)
    right = steering_angle(0.0, -0.5, speed, GAIN, SOFTENING)

    assert left == pytest.approx(-math.atan(GAIN * 0.5 / (SOFTENING + speed)))
    assert left < 0.0
    assert right == pytest.approx(-left)


def test_heading_error_passes_through_at_the_heading_gain():
    error = math.radians(10.0)
    assert steering_angle(error, 0.0, 3.0, GAIN, SOFTENING) == pytest.approx(error)
    assert steering_angle(
        error, 0.0, 3.0, GAIN, SOFTENING, heading_gain=0.5
    ) == pytest.approx(0.5 * error)


def test_cross_track_term_is_finite_at_a_standstill():
    """Plain atan(k e / v) is undefined at v = 0; the softening speed is why this is not."""

    commanded = cross_track_steering(1.0, 0.0, GAIN, SOFTENING)
    assert commanded == pytest.approx(-math.atan(GAIN / SOFTENING))


def test_cross_track_term_weakens_as_speed_rises():
    slow = abs(cross_track_steering(1.0, 1.0, GAIN, SOFTENING))
    fast = abs(cross_track_steering(1.0, 5.0, GAIN, SOFTENING))
    assert fast < slow


def test_reverse_speed_is_treated_as_a_standstill():
    assert cross_track_steering(1.0, -3.0, GAIN, SOFTENING) == pytest.approx(
        cross_track_steering(1.0, 0.0, GAIN, SOFTENING)
    )


def test_softening_speed_must_be_positive():
    for bad in (0.0, -1.0):
        with pytest.raises(ValueError, match="softening"):
            cross_track_steering(1.0, 1.0, GAIN, bad)


def test_heading_error_wraps_across_pi():
    # Route at +179 deg, vehicle at -179 deg: 2 deg apart, not 358.
    assert wrap_angle(math.radians(179.0) - math.radians(-179.0)) == pytest.approx(
        math.radians(-2.0)
    )
    assert steering_angle(
        math.radians(358.0), 0.0, 3.0, GAIN, SOFTENING
    ) == pytest.approx(math.radians(-2.0))


def test_front_axle_sits_a_wheelbase_ahead_of_the_rear_axle():
    x, y = front_axle_position(10.0, 5.0, math.pi / 2.0, 0.0, WHEELBASE)
    assert x == pytest.approx(10.0)
    assert y == pytest.approx(5.0 + WHEELBASE)
    # Rear axle 0.3 m behind base_link: the front axle is WHEELBASE - 0.3 ahead of it.
    x, _ = front_axle_position(0.0, 0.0, 0.0, -0.3, WHEELBASE)
    assert x == pytest.approx(WHEELBASE - 0.3)


def test_path_heading_on_a_straight_route(straight_route):
    assert path_heading(straight_route(10), 4, 0.5, 2.0) == pytest.approx(0.0)


def test_path_heading_follows_the_route_geometry_through_an_arc(circular_route):
    """The chord over a window centred on a vertex is the tangent there."""

    radius = 10.0
    route = circular_route(radius, 20)
    step = 1.0 / radius
    chord = 2.0 * radius * math.sin(0.5 * step)
    assert path_heading(route, 7, 0.0, 2.0 * chord) == pytest.approx(7 * step, abs=1e-9)


def test_path_heading_window_is_clamped_at_the_route_start(circular_route):
    radius = 10.0
    route = circular_route(radius, 20)
    step = 1.0 / radius
    chord = 2.0 * radius * math.sin(0.5 * step)
    # Nothing lies behind waypoint 0, so the chord runs from it to waypoint 1.
    assert path_heading(route, 0, 0.0, 2.0 * chord) == pytest.approx(
        0.5 * step, abs=1e-9
    )


def test_path_heading_without_a_window_uses_the_projected_segment(circular_route):
    radius = 10.0
    route = circular_route(radius, 20)
    step = 1.0 / radius
    # Segment 7 joins vertices 7 and 8; its direction is midway between their tangents.
    assert path_heading(route, 7, 0.3, 0.0) == pytest.approx(7.5 * step, abs=1e-9)


def test_heading_alignment_gate():
    limit = math.radians(60.0)
    assert not heading_not_aligned(math.radians(59.0), limit)
    assert heading_not_aligned(math.radians(-61.0), limit)
    assert not heading_not_aligned(math.radians(170.0), 0.0)  # 0 disables


def test_heading_alignment_gate_uses_the_wrapped_error():
    # A 359 deg raw difference is a 1 deg misalignment, not a reversed vehicle.
    assert not heading_not_aligned(math.radians(359.0), math.radians(60.0))
    # 180 deg is a vehicle facing the route backwards.
    assert heading_not_aligned(math.pi, math.radians(60.0))


def test_the_offset_sign_matches_route_io_projection():
    """Guard the one convention the law cannot check by itself."""

    route_io = pytest.importorskip("mola_slam_mapping.route_io")
    visualization = pytest.importorskip("mola_slam_mapping.route_visualization")
    route = [
        visualization.make_route_point(float(index), 0.0, 0.0, 0.0, 3.0)
        for index in range(5)
    ]

    projection = route_io.project_onto_route(route, 2.0, 0.5)  # left of a +x route

    assert projection.signed_offset == pytest.approx(0.5)
    assert steering_angle(0.0, projection.signed_offset, 2.0, GAIN, SOFTENING) < 0.0


# ---- steering-motor lag (2026-09-20 oscillation fix) --------------------------


def test_yaw_damping_opposes_a_rotation_the_straight_does_not_ask_for():
    from krowd_stanley_control.stanley import yaw_rate_damping

    # Straight, yawing LEFT at 10 deg/s: 1.5 deg to the RIGHT at 0.15 s.
    correction = yaw_rate_damping(5.0 / 3.6, 0.0, math.radians(10.0), 0.15)
    assert math.degrees(correction) == pytest.approx(-1.5)
    # ...and the mirror image.
    correction = yaw_rate_damping(5.0 / 3.6, 0.0, math.radians(-10.0), 0.15)
    assert math.degrees(correction) == pytest.approx(+1.5)


def test_yaw_damping_is_zero_on_a_curve_driven_correctly():
    from krowd_stanley_control.stanley import yaw_rate_damping

    speed, curvature = 3.0, 1.0 / 20.0
    assert yaw_rate_damping(speed, curvature, speed * curvature, 0.15) == pytest.approx(0.0)
    # Turning less than the curve asks for steers further INTO it.
    assert yaw_rate_damping(speed, curvature, 0.0, 0.15) > 0.0
    assert yaw_rate_damping(speed, -curvature, 0.0, 0.15) < 0.0


def test_yaw_damping_off_and_reverse_speed():
    from krowd_stanley_control.stanley import yaw_rate_damping

    assert yaw_rate_damping(3.0, 0.05, 0.4, 0.0) == 0.0
    assert yaw_rate_damping(3.0, 0.05, 0.4, -1.0) == 0.0
    # A negative speed counts as standing still: only the measured rate remains.
    assert yaw_rate_damping(-2.0, 0.05, 0.1, 0.15) == pytest.approx(-0.015)


def test_the_steering_target_may_lead_the_wheels_by_the_limit_only():
    from krowd_stanley_control.stanley import limit_steering_lead

    lead = math.radians(3.0)
    measured = math.radians(-8.0)  # the wheels are still turned right
    # The controller already wants 10 deg left: only 3 deg ahead of the wheels.
    assert limit_steering_lead(math.radians(10.0), measured, lead) == pytest.approx(
        math.radians(-5.0)
    )
    assert limit_steering_lead(math.radians(-20.0), measured, lead) == pytest.approx(
        math.radians(-11.0)
    )
    # Inside the band the target is untouched.
    assert limit_steering_lead(math.radians(-7.0), measured, lead) == pytest.approx(
        math.radians(-7.0)
    )


def test_the_lead_limit_is_off_without_a_limit_or_a_measurement():
    from krowd_stanley_control.stanley import limit_steering_lead

    target = math.radians(10.0)
    assert limit_steering_lead(target, math.radians(-8.0), 0.0) == target
    assert limit_steering_lead(target, None, math.radians(3.0)) == target


def test_the_lead_limit_opens_up_with_the_route_curvature():
    from krowd_stanley_control.stanley import steering_lead_limit

    straight, curve = math.radians(5.0), math.radians(12.0)

    def lead(radius_m):
        return math.degrees(steering_lead_limit(1.0 / radius_m, straight, curve, 50.0, 15.0))

    assert lead(1e9) == pytest.approx(5.0)   # dead straight
    assert lead(50.0) == pytest.approx(5.0)  # at the straight radius
    assert lead(30.0) == pytest.approx(7.0)  # halfway in curvature (1/30 is midway 1/50..1/15)
    assert lead(15.0) == pytest.approx(12.0)
    assert lead(4.4) == pytest.approx(12.0)  # clamped, never beyond the curve value
    # A right-hand bend counts the same as a left-hand one.
    assert steering_lead_limit(-1.0 / 15.0, straight, curve, 50.0, 15.0) == pytest.approx(curve)


def test_the_lead_limit_is_one_value_when_not_split():
    from krowd_stanley_control.stanley import steering_lead_limit

    one = math.radians(5.0)
    assert steering_lead_limit(1.0 / 5.0, one, one, 50.0, 15.0) == one
    assert steering_lead_limit(1.0 / 5.0, one, math.radians(12.0), 0.0, 15.0) == one
    # Radii the wrong way round (curve wider than straight) are not a split either.
    assert steering_lead_limit(1.0 / 5.0, one, math.radians(12.0), 15.0, 50.0) == one


def test_curvature_feedforward_supplies_the_missing_part_of_the_bend():
    import math
    from krowd_stanley_control.stanley import curvature_feedforward

    k = 1 / 4.0
    need = math.atan(1.34 * k)
    assert curvature_feedforward(k, 1.34, 0.4) == pytest.approx(0.4 * need)
    assert curvature_feedforward(-k, 1.34, 0.4) == pytest.approx(-0.4 * need)
    assert curvature_feedforward(k, 1.34, 0.0) == 0.0
    assert curvature_feedforward(0.0, 1.34, 0.4) == 0.0


def test_feedforward_plus_heading_gain_tracks_a_circle_without_offset():
    import math
    from krowd_stanley_control.stanley import curvature_feedforward, steering_angle

    # Kinematic steady state on a circle: the heading term sees the bend's own
    # steering angle as heading error, so with the feedforward the cross-track
    # term has nothing left to supply.
    k = 1 / 4.0
    need = math.atan(1.34 * k)
    delta = steering_angle(need, 0.0, 2.0, 0.3, 3.0, 0.6) + curvature_feedforward(k, 1.34, 0.4)
    assert delta == pytest.approx(need)


def test_value_ahead_on_route_walks_the_arc():
    from krowd_stanley_control.stanley import value_ahead_on_route

    points = [(float(i), 0.0) for i in range(6)]
    values = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]
    assert value_ahead_on_route(points, values, 1, 0.5, 0.0) == pytest.approx(0.0)
    assert value_ahead_on_route(points, values, 1, 0.5, 1.0) == pytest.approx(0.5)
    assert value_ahead_on_route(points, values, 1, 0.5, 2.0) == pytest.approx(1.0)
    assert value_ahead_on_route(points, values, 1, 0.5, 50.0) == pytest.approx(1.0)
